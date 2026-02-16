from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    world_file = os.path.join(pkg_share, "worlds", "pick_place_world2.world")
    controller_yaml = os.path.join(pkg_share, "launch", "controller.yaml")

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    with_rviz = DeclareLaunchArgument("with_rviz", default_value="true")
    with_octomap = DeclareLaunchArgument("with_octomap", default_value="false")
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")

    ld.add_action(with_rviz)
    ld.add_action(with_octomap)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)

    # --------------------------------------------------
    # MoveIt Config
    # --------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("ARM", package_name="arm_gripper_moveit_config")
        .robot_description(
            file_path="config/ARM.urdf.xacro",
            mappings={
                "sim_gazebo": "true",
                "simulation_controllers": controller_yaml,
            },
        )
        .robot_description_semantic(file_path="config/ARM.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    # --------------------------------------------------
    # Gazebo
    # --------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "paused": "false",
            "use_sim_time": "true",
            "gui": "true",
        }.items(),
    )
    ld.add_action(gazebo)

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --------------------------------------------------
    # ros2_control
    # --------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controller_yaml,
            {"use_sim_time": True},
        ],
        output="screen",
    )
    ld.add_action(ros2_control_node)

    # --------------------------------------------------
    # Spawn robot
    # --------------------------------------------------
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "ARM",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    ld.add_action(TimerAction(period=3.0, actions=[spawn]))

    # --------------------------------------------------
    # Controllers (after spawn)
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

    

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=spawn,
                on_start=[
                    TimerAction(period=2.0, actions=[joint_state_broadcaster]),
                    TimerAction(period=4.0, actions=[arm_controller]),
                    TimerAction(period=6.0, actions=[end_controller]),
                ],
            )
        )
    )

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
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz")),
        output="screen",
    )
    ld.add_action(rviz)

    # --------------------------------------------------
    # Move Group
    # --------------------------------------------------
    mg_params = moveit_config.to_dict()
    mg_params.update({"use_sim_time": True})

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ld.add_action(move_group)

    return ld
