// Used work from:
//IFRA-Cranfield (2023) Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher. 

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>   // for getMinMax3D
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_ros/transforms.hpp>
#include <cmath>

const double tau = 2 * M_PI;

static constexpr const char* EE_LINK = "Grabber_Base_v1_1";

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
    : move_group(node, "ur5_manipulator"),
      gripper(node, "robotic_gripper"),
      planning_scene_interface(),
      logger(rclcpp::get_logger("PickAndPlace")),
      node_(node),
      tf_buffer(node_->get_clock()),
      tf_listener(tf_buffer)

    {
        move_group.setPoseReferenceFrame("world");
        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
        cloud_sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points",                      // your topic
            rclcpp::SensorDataQoS(),               // correct QoS for cameras
            std::bind(&PickAndPlace::cloudCallback, this, std::placeholders::_1)
        );
        filtered_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/points_filtered", 10);
        joint_state_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            last_joint_state = *msg;
        });


        
    }

    double getGripperPosition()
    {
        for (size_t i = 0; i < last_joint_state.name.size(); ++i)
        {
            if (last_joint_state.name[i] == "gripper_open_joint")
                return last_joint_state.position[i];
        }
        return -1.0;
    }

    void close_gripper_until_contact()
    {
        const double step = 0.01;        // small closing step
        const double max_close = 0.8;    // fully closed
        const double stall_eps = 0.005;  // motion threshold
        const int stall_cycles = 5;      // how many stalls = contact

        double target = 0.0;
        double last_pos = getGripperPosition();
        int stalled = 0;

        while (rclcpp::ok() && target < max_close)
        {
            target += step;
            gripper.setJointValueTarget("gripper_open_joint", target);
            gripper.setJointValueTarget("gripper_right_joint", target); // mirrors motion
            gripper.move();

            rclcpp::sleep_for(std::chrono::milliseconds(200));

            double current_pos = getGripperPosition();

            if (std::abs(current_pos - last_pos) < stall_eps)
            {
                stalled++;
                if (stalled >= stall_cycles)
                {
                    RCLCPP_INFO(logger, "Gripper contact detected — stopping close");
                    return;
                }
            }
            else
            {
                stalled = 0;
            }

            last_pos = current_pos;
        }

        RCLCPP_INFO(logger, "Gripper fully closed (no contact)");
    }


    void close_gripper()
    {
        gripper.setJointValueTarget("gripper_open_joint", 0.05);
        gripper.setJointValueTarget("gripper_right_joint", 0.05);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("gripper_open_joint", 0.0);
        gripper.setJointValueTarget("gripper_right_joint", 0.0);
        gripper.move();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Ends up being a 10 stage filter process for pointcloud

        // Pt1) Transform cloud into base_link 
        sensor_msgs::msg::PointCloud2 cloud_base;

        try
        {
            auto tf = tf_buffer.lookupTransform(
                "world",                 // target frame
                msg->header.frame_id,        // camera frame
                tf2::TimePointZero);

            pcl_ros::transformPointCloud("world", tf, *msg, cloud_base);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(logger, "TF transform failed: %s", ex.what());
            return;
        }

        // Convert to PCL for actual use
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_base, *raw);

        if (raw->points.empty())
            return;

        // DEBUG: cloud bounds for testing (changing camera/urdf later)
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*raw, min_pt, max_pt);

        RCLCPP_INFO(logger,
            "Cloud bounds: X[%.2f %.2f] Y[%.2f %.2f] Z[%.2f %.2f]",
            min_pt.x, max_pt.x,
            min_pt.y, max_pt.y,
            min_pt.z, max_pt.z);

        // Pt2) Remove only impossible points (floor / ceiling)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &p : raw->points)
        {
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
                p.z > -1.0 && p.z < 0.2)   // real base_link heights
            {
                cloud->points.push_back(p);
            }
        }

        if (cloud->points.empty())
        {
            RCLCPP_WARN(logger, "No points after basic Z filtering");
            return;
        }

        // Pt3) Detect horizontal table plane
        // Does not assume flat world just the plane of object rest

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        // Force horizontal plane
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(10.0 * M_PI / 180.0);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_WARN(logger, "No table plane detected");
            return;
        }

        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];

        // Safety (avoid divide by zero, had some trouble with that)
        if (std::abs(c) < 0.8)
        {
            RCLCPP_WARN(logger, "Detected plane not horizontal enough");
            return;
        }

        // Pt4) Remove table from points
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*objects);

        if (objects->points.empty())
        {
            RCLCPP_WARN(logger, "No points after table removal");
            return;
        }

        // Pt5) Keep only thin objects just above table 
        pcl::PointCloud<pcl::PointXYZ>::Ptr above_table(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &p : objects->points)
        {
            double table_z = -(a * p.x + b * p.y + d) / c;

            // USB sits ~5–60 mm above table
            if (p.z > table_z + 0.001 && p.z < table_z + 0.06)
                above_table->points.push_back(p);
        }

        if (above_table->points.empty())
        {
            RCLCPP_WARN(logger, "No objects above table");
            return;
        }

        RCLCPP_INFO(logger, "Points above table: %zu", above_table->points.size());

        // Pt6) Downsample the voxel filter to get rid of some noise
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(above_table);
        voxel.setLeafSize(0.004f, 0.004f, 0.004f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel.filter(*cloud_filtered);

        if (cloud_filtered->points.empty())
        {
            RCLCPP_WARN(logger, "No points after voxel filtering");
            return;
        }

        // Pt7) Clustering (USB-scale tuned, will need refacotring for a new application)
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(above_table);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        // USB is small → tight tolerance + small clusters
        ec.setClusterTolerance(0.012);     // 1.5 cm
        ec.setMinClusterSize(10);          // USB often 50–150 points
        ec.setMaxClusterSize(5000);        // reject table chunks
        ec.setSearchMethod(tree);
        ec.setInputCloud(above_table);
        ec.extract(cluster_indices);

        if (cluster_indices.empty())
        {
            RCLCPP_WARN(logger, "No clusters found");
            return;
        }

        // ---------- 8) Choose USB-sized cluster ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        double best_score = 1e9;

        pcl::PointXYZ best_cmin, best_cmax;
        bool have_best = false;

        for (auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            cluster->reserve(indices.indices.size());

            for (int idx : indices.indices)
                cluster->points.push_back(above_table->points[idx]);

            pcl::PointXYZ cmin, cmax;
            pcl::getMinMax3D(*cluster, cmin, cmax);

            double dx = cmax.x - cmin.x;
            double dy = cmax.y - cmin.y;
            double dz = cmax.z - cmin.z;

            // Debug every cluster (DO NOT DELETE OR CHANGE)
            RCLCPP_INFO(logger,
                "Cluster pts=%zu  dx=%.3f dy=%.3f dz=%.3f",
                cluster->points.size(), dx, dy, dz);

            // USB physical gate (~~ real dimensions for reusability)
            if (dx > 0.015 && dx < 0.08 &&      // length 3–8 cm
                dy > 0.005 && dy < 0.04 &&     // width 0.5–4 cm
                dz < 0.03)                     // height < 3 cm
            {
                // Prefer longestwrist_3_link thin object (USB shape)
                double score = std::abs(dx - 0.05) + dz;  // favor ~5 cm length, thin

                if (score < best_score)
                {
                    best_score = score;
                    *best_cluster = *cluster;
                    best_cmin = cmin;
                    best_cmax = cmax;
                    have_best = true;
                }
            }
        }

        if (best_cluster->points.empty())
        {
            RCLCPP_WARN(logger, "No USB-sized cluster found");
            return;
        }


        // ---------- 9) Publish debug cloud ----------
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*best_cluster, filtered_msg);
        filtered_msg.header.frame_id = "world";
        filtered_msg.header.stamp = msg->header.stamp;
        filtered_pub->publish(filtered_msg);

        // ---------- 10) Compute grasp pose ----------
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*best_cluster, centroid);

        detected_pose.position.x = centroid[0];
        detected_pose.position.y = centroid[1];
        detected_pose.position.z = centroid[2] + 0.08;   // approach height

        // Determine USB orientation in table plane
        double dx = best_cmax.x - best_cmin.x;
        double dy = best_cmax.y - best_cmin.y;

        double yaw;

        if (dx > dy)
        {
            // USB long axis along X -> grasp across Y
            yaw = M_PI;
        }
        else
        {
            // USB long axis along Y -> grasp across X
            yaw = 0.0;
        }

        tf2::Quaternion q;
        q.setRPY(-M_PI, 0, 0);   // top-down grasp
        detected_pose.orientation = tf2::toMsg(q);

        object_detected = true;

        RCLCPP_INFO(logger,
            "USB detected at (%.3f, %.3f, %.3f) yaw=%.2f",
            centroid[0], centroid[1], centroid[2], yaw);
    }

    void pick()
    {
        move_group.setMaxVelocityScalingFactor(0.4);
        move_group.setMaxAccelerationScalingFactor(0.4);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(true);
        move_group.setGoalTolerance(0.003);
        move_group.setEndEffectorLink("Grabber_Base_v1_1");

        move_group.clearPoseTargets();

        // ---------- 1) Approach pose (above USB) ----------
        geometry_msgs::msg::Pose approach = detected_pose;
        approach.position.z += 0.12;   // 6 cm above object
        approach.orientation = detected_pose.orientation;
        RCLCPP_INFO(logger, "Target pose: x=%.3f y=%.3f z=%.3f", detected_pose.position.x, detected_pose.position.y, detected_pose.position.z);
        // IMPORTANT: target the GRIPPER BASE, not tool0, not wrist
        RCLCPP_INFO(logger,
            "Target quaternion: x=%.3f y=%.3f z=%.3f w=%.3f",
            approach.orientation.x,
            approach.orientation.y,
            approach.orientation.z,
            approach.orientation.w);
        move_group.setPoseTarget(approach, "Grabber_Base_v1_1");

        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        bool success = (move_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
        moveit::core::MoveItErrorCode result = move_group.plan(plan1);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            switch (result.val) {
                case moveit::core::MoveItErrorCode::PLANNING_FAILED: RCLCPP_WARN(logger, "Planning failed"); break;
                case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION: RCLCPP_WARN(logger, "Start state in collision"); break;
                case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION: RCLCPP_WARN(logger, "Goal in collision"); break;
                case moveit::core::MoveItErrorCode::NO_IK_SOLUTION: RCLCPP_WARN(logger, "No IK solution"); break;
                default: RCLCPP_WARN(logger, "Unknown error code: %d", result.val); break;
            }
            return;
        }

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "Failed to plan approach!");
            return;
        }

        RCLCPP_INFO(logger, "Approach plan: %s", success ? "SUCCESS" : "FAILED");

        if (!success)
        {
            RCLCPP_ERROR(logger, "Failed to plan approach");
            return;
        }


        move_group.move();
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        open_gripper();
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // ---------- 2) Descend to grasp ----------
        geometry_msgs::msg::Pose grasp = approach;
        grasp.position.z = detected_pose.position.z;   // 1.2 cm above object

        geometry_msgs::msg::Pose grasp_pose = detected_pose;
        grasp_pose.position.z += 0.12; // 12 cm above for pre-grasp


        move_group.clearPoseTargets();
        move_group.setPoseTarget(grasp_pose, EE_LINK); // "Grabber_Base_v1_1"

        moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
        auto grasp_result = move_group.plan(grasp_plan);

        if (grasp_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "Approach planning failed");
            return;
        }

        move_group.move();

        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // ---------- 3) Close gripper ----------
        close_gripper_until_contact();
        rclcpp::sleep_for(std::chrono::seconds(1));

        attachObject();

        move_group.clearPoseTargets();

        // ---------- 4) Lift object ----------
        geometry_msgs::msg::Pose lift = grasp;
        lift.position.z += 0.10;   // lift 10 cm

        move_group.setPoseTarget(lift, "Grabber_Base_v1_1");

        moveit::planning_interface::MoveGroupInterface::Plan plan3;
        success = (move_group.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
            move_group.move();

            
    }


    void place()
    {
        move_group.clearPoseTargets();

        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        //Change to 200 later after demo to increase success rate 
        //and maximize speed
        move_group.setPlanningTime(200.0); 
        
        //Good to prevent the robot from fully failing instantly
        move_group.allowReplanning(true);  
        move_group.setGoalTolerance(0.03); 

        geometry_msgs::msg::Pose place_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-2.551, -0.058, 3.116);
        place_pose.orientation = tf2::toMsg(orientation);
        place_pose.position.x = 0.121;
        place_pose.position.y = 0.468;
        place_pose.position.z = 0.218;

        move_group.setPoseTarget(place_pose, "Grabber_Base_v1_1");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Place plan: %s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Motion execution completed.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning failed!");
        }
    }

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "ARM";  // Name of the Gazebo robot
        request->link1_name = "Grabber_Base_v1_1"; // Link of the gripper
        request->model2_name = "USB"; // Name of the Cube
        request->link2_name = "link_0";    //Link of the cube

        while (!attach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
        }

        auto future = attach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "ARM";  // Name of the Gazebo robot
        request->link1_name = "Grabber_Base_v1_1"; // Link of the gripper
        request->model2_name = "USB"; //wrist_3_link Name of the Cube
        request->link2_name = "link_0";    //Link of the cube

        //Wait for the detach to go through so that the cube doesn't fling
        while (!detach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
        }

        auto future = detach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
        }
    }

    // void attachusb()
    // {
    //     auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    //     request->model1_name = "control_console"; //Name of the Computer
    //     request->link1_name = "link"; //computer Link
    //     request->model2_name = "USB"; //USB name
    //     request->link2_name = "link_0"; //USB link

    //     while (!attach_client->wait_for_service(std::chrono::seconds(1)))
    //     {
    //         RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
    //     }

    //     auto future = attach_client->async_send_request(request);
    //     if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    //     {
    //         RCLCPP_INFO(logger, "Object attached successfully.");
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(logger, "Failed to attach object.");
    //     }
    // }

    bool object_detected = false;


private:
    //Defining moveit groups to use moveit for pathfinding
    //Colling that link attach ptrs to simulat physical grabbing

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    rclcpp::Logger logger;

    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    geometry_msgs::msg::Pose detected_pose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub;

    sensor_msgs::msg::JointState last_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

    
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
};


int main(int argc, char **argv)
{
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_node");

    PickAndPlace pick_and_place(node);

    RCLCPP_INFO(node->get_logger(), "Waiting for perception...");
    
    rclcpp::Time start = node->now();

    while (rclcpp::ok() && !pick_and_place.object_detected)
    {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(50));

        // Optional timeout safety
        if ((node->now() - start).seconds() > 10.0)
        {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting for detection");
            break;
        }
    }

    // Give Octomap a moment to integrate filtered cloud
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Pick Execution

    pick_and_place.pick();

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Not very good/depricated for modern applications
    // Attach collision object
    // pick_and_place.attachCollisionObject();
    // rclcpp::sleep_for(std::chrono::seconds(1));

    // Place Execution
    pick_and_place.place();
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Open gripper
    pick_and_place.open_gripper();
    // rclcpp::sleep_for(std::chrono::seconds(1));
    // pick_and_place.detachCollisionObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // pick_and_place.attachusb();
    // rclcpp::sleep_for(std::chrono::seconds(1));

    pick_and_place.detachObject();
    rclcpp::sleep_for(std::chrono::seconds(1));


    // Spegni ROS2
    rclcpp::shutdown();
    return 0;
}