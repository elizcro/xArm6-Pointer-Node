#!/usr/bin/env python3
#
# pointer_node.launch.py
#
# Launches ONLY the xarm6_pointer node, supplying it with the xArm6 MoveIt
# configuration (robot_description, semantic, kinematics, joint limits, ...)
# built by the xArm-specific MoveItConfigsBuilder from uf_ros_lib.
#
# This does NOT start move_group or RViz. Start the xArm MoveIt stack first
# in another terminal, then run this:
#
#   # Terminal 1 (simulation / RViz):
#   ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py
#   # Terminal 2:
#   ros2 launch xarm6_pointer pointer_node.launch.py
#
#   # For the real robot:
#   # Terminal 1:
#   ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.213
#   # Terminal 2:
#   ros2 launch xarm6_pointer pointer_node.launch.py controllers_name:=controllers
#
# NOTE: The MoveItConfigsBuilder keyword arguments below match the xArm
# convention (dof, robot_type). If your installed xarm_ros2 version expects a
# different signature, see the "Fallback launch" section of the README.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    dof = int(LaunchConfiguration('dof').perform(context))
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controllers_name = LaunchConfiguration('controllers_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    standoff_distance = float(LaunchConfiguration('standoff_distance').perform(context))
    min_reach = float(LaunchConfiguration('min_reach').perform(context))
    max_reach = float(LaunchConfiguration('max_reach').perform(context))
    vel_scale = float(LaunchConfiguration('vel_scale').perform(context))
    acc_scale = float(LaunchConfiguration('acc_scale').perform(context))
    goal_pos_tol = float(LaunchConfiguration('goal_pos_tol').perform(context))
    goal_orient_tol = float(LaunchConfiguration('goal_orient_tol').perform(context))
    planning_time = float(LaunchConfiguration('planning_time').perform(context))
    position_only = LaunchConfiguration('position_only').perform(context).lower() == 'true'
    roll_samples = int(LaunchConfiguration('roll_samples').perform(context))
    ee_link = LaunchConfiguration('ee_link').perform(context)
    tool_length = float(LaunchConfiguration('tool_length').perform(context))
    tool_radius = float(LaunchConfiguration('tool_radius').perform(context))
    pedestal_size_x = float(LaunchConfiguration('pedestal_size_x').perform(context))
    pedestal_size_y = float(LaunchConfiguration('pedestal_size_y').perform(context))
    pedestal_size_z = float(LaunchConfiguration('pedestal_size_z').perform(context))
    pedestal_offset_x = float(LaunchConfiguration('pedestal_offset_x').perform(context))
    pedestal_offset_y = float(LaunchConfiguration('pedestal_offset_y').perform(context))
    
    # Build the xArm6 MoveIt configuration with our tool included via the
    # xArm's built-in add_other_geometry mechanism. The xacros handle
    # creating the tool link, its collision geometry, AND updating the SRDF
    # so collision pairs are properly disabled — no runtime attachment.
    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        dof=dof,
        robot_type=robot_type,
        add_other_geometry='true',
        geometry_type='cylinder',
        geometry_height=tool_length,    # m, along the tool axis
        geometry_radius=tool_radius,    # m, conservative bounding radius
        geometry_mass=0.1,              # placeholder; not used for planning
    ).to_moveit_configs()

    pointer_node = Node(
        package='xarm6_pointer',
        executable='point_at_target',
        name='xarm6_pointer',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim_time,
                'planning_group': 'xarm6',
                'planning_frame': 'link_base',
                'ee_link': ee_link,
                'target_topic': 'target_point',
                'standoff_distance': standoff_distance,
                'min_reach': min_reach,
                'max_reach': max_reach,
                'vel_scale': vel_scale,
                'acc_scale': acc_scale,
                'planning_time': planning_time,
                'planning_attempts': 10.0,
                'goal_pos_tol': goal_pos_tol,
                'goal_orient_tol': goal_orient_tol,
                'position_only': position_only,
                'roll_samples': float(roll_samples),
                'pedestal_size_x': pedestal_size_x,
                'pedestal_size_y': pedestal_size_y,
                'pedestal_size_z': pedestal_size_z,
                'pedestal_offset_x': pedestal_offset_x,
                'pedestal_offset_y': pedestal_offset_y,
            },
        ],
    )

    return [pointer_node]


def generate_launch_description():
    return LaunchDescription([
        # Robot / MoveIt config selection
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('robot_type', default_value='xarm'),
        DeclareLaunchArgument('controllers_name', default_value='fake_controllers',
                              description="'fake_controllers' for sim, 'controllers' for real arm"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Pointing geometry + safety envelope
        DeclareLaunchArgument('standoff_distance', default_value='0.15',
                              description='Metres the tool stops short of the target'),
        DeclareLaunchArgument('min_reach', default_value='0.20',
                              description='Min allowed EE distance from base origin (m)'),
        DeclareLaunchArgument('max_reach', default_value='0.65',
                              description='Max allowed EE distance from base origin (m); xArm6 reach ~0.70'),
        DeclareLaunchArgument('vel_scale', default_value='0.1',
                              description='Velocity scaling 0..1 (start slow!)'),
        DeclareLaunchArgument('acc_scale', default_value='0.1',
                              description='Acceleration scaling 0..1'),
        DeclareLaunchArgument('goal_pos_tol', default_value='0.01',
                              description='Goal position tolerance (m)'),
        DeclareLaunchArgument('goal_orient_tol', default_value='0.2',
                              description='Goal orientation tolerance (rad); roll is free for pointing'),
        DeclareLaunchArgument('planning_time', default_value='2.0',
                              description='Max planning time per roll sample (s)'),
        DeclareLaunchArgument('position_only', default_value='false',
                              description='Diagnostic: plan to EE position only, ignore orientation'),
        DeclareLaunchArgument('roll_samples', default_value='12',
                              description='Number of roll angles to try about the pointing axis (12 = every 30 deg)'),
        DeclareLaunchArgument('ee_link', default_value='other_geometry_link',
                              description='Tip link name used for pointing. Updated after add_other_geometry is in URDF.'),
        DeclareLaunchArgument('tool_length', default_value='0.1651',
                              description='Length of the mounted tool along link_eef +Z (m). Default 6.5 in = 0.1651 m'),
        DeclareLaunchArgument('tool_radius', default_value='0.01905',
                              description='cylinder radius for the sprayer tool (m)'),
        DeclareLaunchArgument('pedestal_size_x', default_value = '0.4636',
                              description = 'Pedestal X dimension in m (default 18.25 in = 0.4636 m)'),
        DeclareLaunchArgument('pedestal_size_y', default_value = '0.4572',
                              description = 'Pedestal Y dimension in m (default 18.00 in = 0.4572 m)'),
        DeclareLaunchArgument('pedestal_size_z', default_value='1.0033',
                              description='Pedestal Z dimension (height) in m (default 39.50 in = 1.0033 m)'),
        DeclareLaunchArgument('pedestal_offset_x', default_value='0.0',
                              description='Pedestal center offset from arm base in X (m)'),
        DeclareLaunchArgument('pedestal_offset_y', default_value='0.1651',
                              description='Pedestal center offset from arm base in Y (m). Arm 15.5 in 18 in edge'),

        OpaqueFunction(function=launch_setup),
    ])
