#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# ADDED: ExecuteProcess and TimerAction to handle the simulation position reset
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Toggle between simulation/fake and live execution
    is_live = LaunchConfiguration('is_live')
    robot_ip = LaunchConfiguration('robot_ip')

    declare_is_live = DeclareLaunchArgument(
        'is_live',
        default_value='false',
        description='Set to "true" to connect to the real robot, "false" for fake/simulation RViz.'
    )

    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.213',
        description='IP address of the physical xArm (only used if is_live:=true)'
    )

    # Hardcoded geometry specifications for your 6.5-inch tool
    base_geometry_args = {
        'add_other_geometry': 'true',
        'geometry_type': 'cylinder',
        'geometry_height': '0.1651',   # 6.5 inches
        'geometry_radius': '0.01905',  # ~3/4 inch radius
        'geometry_mass': '0.1',
    }

    xarm_moveit_config_dir = get_package_share_directory('xarm_moveit_config')

    # --- SIMULATION BRANCH ---
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_moveit_config_dir, 'launch', 'xarm6_moveit_fake.launch.py')
        ),
        launch_arguments=base_geometry_args.items(),
        condition=UnlessCondition(is_live)
    )

    # --- REAL ROBOT BRANCH ---
    real_robot_args = dict(base_geometry_args)
    real_robot_args['robot_ip'] = robot_ip

    launch_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_moveit_config_dir, 'launch', 'xarm6_moveit_realmove.launch.py')
        ),
        launch_arguments=real_robot_args.items(),
        condition=IfCondition(is_live)
    )

    # --- ADDED: AUTOMATED SIMULATION POSTURE OVERRIDE ---
    # Fired via CLI process execution. Directly targets ros2_control without safety block intervention.
    teleport_arm_clear = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/xarm6_traj_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            "{joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], "
            "points: [{positions: [1.5324, -0.0244, -0.0436, 3.1329, 2.2742, -0.2129], time_from_start: {sec: 0, nanosec: 500000000}}]}"
        ],
        output='screen'
    )

    # Wraps the teleport command in a 5-second timer to give mock controllers time to initialize.
    # CRITICAL: 'condition=UnlessCondition(is_live)' guarantees this NEVER triggers on the physical robot.
    delay_teleport = TimerAction(
        period=5.0,
        actions=[teleport_arm_clear],
        condition=UnlessCondition(is_live)
    )

    return LaunchDescription([
        declare_is_live,
        declare_robot_ip,
        launch_sim,
        launch_real,
        delay_teleport # ADDED: include the delayed action to the stack
    ])
