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
    Continuous Pick-and-Place System (using GSAM AI detection) Launch File
    
    Launches all necessary nodes for vision-guided pick-and-place of glass slides:
    - RealSense camera
    - GSAM AI slide detection (service-based)
    - Camera-to-flange static transform
    - Gripper controller
    - Pick-and-place controller (with detection_mode='gsam')
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
    
    # Alignment method (direct for GSAM recommended)
    alignment_method_arg = DeclareLaunchArgument(
        'alignment_method',
        default_value='direct',
        description='Gripper alignment method: direct or perpendicular'
    )
    alignment_method = LaunchConfiguration('alignment_method')

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
    # PERCEPTION: GSAM AI Slide Detection (Service)
    # =========================================================================
    
    gsam_detect_node = Node(
        package='realsense_cv',
        executable='gsam_slide_detect',
        name='gsam_slide_detect',
        output='screen',
        parameters=[{
            'text_prompt': 'colored box.',
            'grounding_model': 'IDEA-Research/grounding-dino-tiny',
            # Update these paths to match your setup:
            'sam2_checkpoint': '/home/bryan/final_project_ws/src/perception/realsense_cv/models/sam2.1_hiera_small.pt',
            'sam2_model_config': 'configs/sam2.1/sam2.1_hiera_s.yaml',
        }]
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
    # PLANNING: Pick-and-Place Controller (GSAM MODE)
    # =========================================================================
    
    pick_and_place_node = Node(
        package='planning',
        executable='pick_and_place',
        name='pick_and_place',
        output='screen',
        parameters=[{
            'detection_mode': 'gsam',  # KEY PARAMETER: Use GSAM detection
            'alignment_method': alignment_method,  # Use direct for GSAM
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
        alignment_method_arg,

        # Nodes
        realsense_launch,
        camera_tf_node,
        gsam_detect_node,
        gripper_node,
        pick_and_place_node,

        # Event handlers
        shutdown_on_any_exit,
    ])
