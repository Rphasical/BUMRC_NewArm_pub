from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():

    ld = LaunchDescription()

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------
    pkg_share = get_package_share_directory("arm_description")
    controller_yaml = os.path.join(pkg_share, "launch", "controller.yaml")

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    with_rviz = DeclareLaunchArgument("with_rviz", default_value="true")
    ld.add_action(with_rviz)

    # --------------------------------------------------
    # MoveIt Config (HARDWARE MODE)
    # --------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("ARM", package_name="arm_gripper_moveit_config")
        .robot_description(
            file_path="config/ARM.urdf.xacro",
            mappings={
                "sim_gazebo": "false",   # IMPORTANT
            },
        )
        .robot_description_semantic(file_path="config/ARM.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --------------------------------------------------
    # ros2_control (REAL HARDWARE)
    # --------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controller_yaml,
        ],
        output="screen",
    )
    ld.add_action(ros2_control_node)

    # --------------------------------------------------
    # Controller Spawners
    # --------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    end_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "end_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    ld.add_action(joint_state_broadcaster)
    ld.add_action(arm_controller)
    ld.add_action(end_controller)

    # --------------------------------------------------
    # RViz
    # --------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory("arm_gripper_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz")),
        output="screen",
    )
    ld.add_action(rviz)

    # --------------------------------------------------
    # Move Group
    # --------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ld.add_action(move_group)

    return ld