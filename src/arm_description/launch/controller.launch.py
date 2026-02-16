from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    robot_xacro = os.path.join(pkg_share, 'urdf', 'ARM.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'launch', 'urdf_clean.rviz')
    rviz_config = os.path.join(pkg_share, 'rviz',  'rvizconfig.rviz')
    empty_world = os.path.join(gazebo_ros_dir, 'worlds', 'empty.world')
    controller_yaml = os.path.join(pkg_share, 'launch', 'controller.yaml')
    custom_world = os.path.join(pkg_share, 'worlds', 'newarm_world2.world')

    sim_gazebo_arg = DeclareLaunchArgument('sim_gazebo', default_value='true')
    simulation_controllers_arg = DeclareLaunchArgument('simulation_controllers', default_value=controller_yaml)
    sim_ignition_arg = DeclareLaunchArgument('sim_ignition', default_value='false')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='')

    robot_description = {
        'robot_description': Command([
            'xacro ', robot_xacro,
            ' sim_gazebo:=', LaunchConfiguration('sim_gazebo'),
            ' simulation_controllers:=', LaunchConfiguration('simulation_controllers'),
            ' sim_ignition:=', LaunchConfiguration('sim_ignition'),
            ' tf_prefix:=', LaunchConfiguration('tf_prefix')
        ])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': empty_world,
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true'
        }.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_yaml, {'use_sim_time': True}],
        output='screen'
    )

    # --- Spawn robot AFTER ros2_control_node is up ---
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ARM'],
        output='screen'
    )
    delayed_spawn = TimerAction(
        period=3.0,  # small delay to ensure ros2_control_node started
        actions=[spawn_robot]
    )

    # --- Controllers spawn AFTER robot is spawned ---
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    delayed_controllers_1 = TimerAction(period=1.0, actions=[joint_state_broadcaster])
    delayed_controllers_2 = TimerAction(period=2.0, actions=[arm_controller])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        sim_gazebo_arg,
        simulation_controllers_arg,
        sim_ignition_arg,
        tf_prefix_arg,
        gazebo,
        rsp,
        ros2_control_node,
        delayed_spawn,
        delayed_controllers_1,
        delayed_controllers_2,
        rviz_node
    ])
