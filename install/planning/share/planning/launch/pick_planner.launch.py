#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/target_pose',
        description='Topic to receive target pose messages'
    )

    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='tmr_arm',
        description='MoveIt planning group name for TM12M'
    )

    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='flange',
        description='End effector link name'
    )

    pre_grasp_offset_arg = DeclareLaunchArgument(
        'pre_grasp_offset',
        default_value='0.15',
        description='Pre-grasp offset distance in meters'
    )

    post_grasp_offset_arg = DeclareLaunchArgument(
        'post_grasp_offset',
        default_value='0.20',
        description='Post-grasp lift offset in meters'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base',
        description='Robot base frame name'
    )

    # Pick planner node
    pick_planner_node = Node(
        package='planning',
        executable='pick_planner',
        name='pick_planner',
        output='screen',
        parameters=[{
            'robot_name': 'tm12',
            'planning_group': LaunchConfiguration('planning_group'),
            'end_effector_link': LaunchConfiguration('end_effector_link'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'pre_grasp_offset': LaunchConfiguration('pre_grasp_offset'),
            'post_grasp_offset': LaunchConfiguration('post_grasp_offset'),
            'base_frame': LaunchConfiguration('base_frame'),
            'use_sim_time': True,
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        pose_topic_arg,
        planning_group_arg,
        end_effector_link_arg,
        pre_grasp_offset_arg,
        post_grasp_offset_arg,
        base_frame_arg,
        pick_planner_node,
    ])
