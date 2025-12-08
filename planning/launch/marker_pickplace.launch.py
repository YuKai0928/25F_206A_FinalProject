#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Continuous Pick-and-Place System (using Aruco marker detection) Launch File
    
    Launches all necessary nodes for vision-guided pick-and-place of glass slides:
    - RealSense camera
    - ArUco marker detection
    - Camera-to-flange static transform
    - Gripper controller
    - Pick-and-place controller
    - Slide Detector
    """
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    
    # Gripper serial port
    gripper_port_arg = DeclareLaunchArgument(
        'gripper_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino gripper controller'
    )
    gripper_port = LaunchConfiguration('gripper_port')
    
    # Slide indices to detect
    slide_indices_arg = DeclareLaunchArgument(
        'slide_indices',
        default_value='[8, 18]',
        description='List of slide indices to detect (slot positions)'
    )
    slide_indices = LaunchConfiguration('slide_indices')
    
    # =========================================================================
    # CAMERA: RealSense D435i
    # =========================================================================
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'rgb_camera.color_profile': '1280x720x30',
        }.items(),
    )
    
    # =========================================================================
    # PLANNING: Camera Static Transform (Flange -> Camera)
    # =========================================================================
    
    camera_tf_node = Node(
        package='planning',
        executable='camera_tf',
        name='camera_tf',
        output='screen',
    )
    
    # =========================================================================
    # PERCEPTION: ArUco Marker Detection
    # =========================================================================
    
    marker_detect_node = Node(
        package='realsense_cv',
        executable='marker_node',
        name='marker_node',
        output='screen',
    )
    
    # =========================================================================
    # ACTUATION: Gripper Controller
    # =========================================================================
    
    gripper_node = Node(
        package='actuation',
        executable='gripper',
        name='gripper',
        output='screen',
        parameters=[{
            'serial_port': gripper_port,
        }]
    )
    
    # =========================================================================
    # PLANNING: Pick-and-Place Controller
    # =========================================================================
    
    pick_and_place_node = Node(
        package='planning',
        executable='pick_and_place',
        name='pick_and_place',
        output='screen',
    )
    
    # =========================================================================
    # PERCEPTION: Slide Detector
    # =========================================================================
    
    slide_detector_node = Node(
        package='realsense_cv',
        executable='slide_detector',
        name='slide_detector',
        output='screen',
        parameters=[{
            'slide_indices': slide_indices,
        }]
    )
    
    # =========================================================================
    # EVENT HANDLERS
    # =========================================================================
    
    # Shutdown all nodes if any critical node exits
    shutdown_on_any_exit = RegisterEventHandler(
        OnProcessExit(
            on_exit=[EmitEvent(event=Shutdown(reason='SOMETHING BONKED'))]
        )
    )
    
    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    
    return LaunchDescription([
        # Arguments
        gripper_port_arg,
        slide_indices_arg,
        
        # Nodes
        realsense_launch,
        camera_tf_node,
        marker_detect_node,
        gripper_node,
        pick_and_place_node,
        slide_detector_node,
        
        # Event handlers
        shutdown_on_any_exit,
    ])
