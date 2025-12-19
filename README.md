# PRISM: Precision Relocation via Intelligent Slide Manipulation


PRISM is ROS 2-based robotic system for automated detection and manipulation of glass slides using a Techman TM12 robot arm. This project was developed for **EECS 206A: Introduction to Robotics** (Fall 2025) at UC Berkeley, in collaboration with **Ember Robotics** as part of the Industry Project.

## 📋 Table of Contents

- [Overview](#overview)
- [Project Goals](#project-goals)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Key Components](#key-components)
- [Detection Modes](#detection-modes)
- [Performance](#performance)
- [Video Demo](#video-demo)
- [Citations](#citations)

## 🎯 Overview

This project implements a vision-guided pick-and-place system for fragile glass laboratory slides. The system demonstrates high precision manipulation through:

1. **Robust Object Detection**: Two detection modes (ArUco markers and AI-based GSAM) detect slides from RGB images
2. **Accurate Motion Planning**: Inverse kinematics (IK) and MoveIt2 trajectory planning
3. **Precise Actuation**: Arduino-controlled gripper with serial communication

The robot autonomously:
- **Detects** glass slides in a storage box (up to 25 slots, typically 2-3 slides in experiments)
- **Plans** collision-free trajectories using IK and motion planning
- **Picks** slides from the storage box with precise alignment
- **Places** slides on an orange target plate with gentle motion

### Key Challenge: Transparent Glass Slides

Glass slides are transparent (25mm × 75mm × 1mm standard lab slides), making traditional depth sensing unreliable. Our system overcomes this by:
- **Edge detection** (Canny) to identify geometric features
- **ArUco markers** for marker-based localization
- **AI vision models** (Grounding DINO + SAM2) for markerless detection

## 🎓 Project Goals

This project was developed for **EECS 206A: Introduction to Robotics** (Fall 2025) at UC Berkeley, in collaboration with **Ember Robotics** as part of the Industry Track. The goal is to design and implement a vision-guided pick-and-place system using a Techman robotic arm that can:

- Manipulate fragile glass lab slides with high precision
- Perform robust object detection in challenging conditions (transparent objects)
- Execute accurate motion planning and trajectory execution
- Provide precise actuation control for delicate operations

## 🏗️ System Architecture

The system is organized into four main ROS 2 packages:

```
┌─────────────────┐
│   Perception    │  ← Computer vision (ArUco/GSAM detection)
├─────────────────┤
│    Planning     │  ← Motion planning, IK, pick-and-place logic
├─────────────────┤
│   Actuation     │  ← Gripper control (Arduino serial)
├─────────────────┤
│Planning Interfaces│  ← Service definitions
└─────────────────┘
```

### Data Flow

```
Camera (RealSense D435i) 
    ↓
Perception Node (ArUco/GSAM)
    ↓
TF Frames (slide_XX)
    ↓
Slide Detector
    ↓
Pick-and-Place Planner
    ↓
IK Solver → MoveIt2 → TM12 Robot Arm
    ↓
Gripper Controller → Arduino
```

## 🔧 Hardware Requirements

- **Robot Arm**: Techman TM12 (TM12M)
- **Camera**: Intel RealSense D435i (RGB-D)
- **Gripper**: Custom Arduino-controlled gripper
- **Storage Box**: Holds up to 25 glass slides (25mm × 75mm × 1mm), positioned sideways (side facing up for grasping). Yellow box were used in our experiment
- **Target Plate**: Orange plate with slots for glass slides
- **Computer**: Ubuntu 22.04 with ROS 2 Humble (or compatible)

### Experimental Setup

- **Typical Configuration**: 2-3 slides per box (due to gripper size limitations)
- **Slide Orientation**: Slides are placed sideways in the storage box with the side facing up for easier grasping
- **Placement Pattern**: Slides are placed on the orange target plate with even spacing

## 📦 Software Dependencies

### ROS 2 Packages
- `ros-humble-realsense2-camera` - RealSense camera driver
- `moveit2` - Motion planning framework
- `tf2_ros` - Transform library
- `cv_bridge` - OpenCV-ROS bridge

### Python Packages
- `opencv-python` - Computer vision
- `numpy` - Numerical operations
- `torch` - PyTorch (for GSAM models)
- `transformers` - Hugging Face transformers (for Grounding DINO)
- `supervision` - Computer vision utilities
- `pyserial` - Serial communication with Arduino

### AI Models (GSAM Mode)
- **Grounding DINO**: `IDEA-Research/grounding-dino-tiny` (from Hugging Face)
- **SAM2**: `sam2.1_hiera_small.pt` (from [Grounded-SAM-2](https://github.com/IDEA-Research/Grounded-SAM-2))

## 📁 Repository Structure

```
25F_206A_FinalProject/
├── perception/                    # Computer vision package
│   ├── realsense_cv/
│   │   └── realsense_cv/
│   │       ├── gsam_slide_detect.py    # GSAM AI detection node
│   │       ├── marker_detect.py         # ArUco marker detection
│   │       ├── slide_detector.py        # TF-based slide detection helper
│   │       ├── edge_detect.py           # Edge detection utilities
│   │       └── ...           # Other files are for testing during development
│   ├── aruco_marker/              # ArUco marker images
│   └── perception_note.md         # Perception design notes
│
├── planning/                      # Motion planning package
│   ├── planning/
│   │   ├── pick_and_place.py      # Main pick-and-place controller
│   │   ├── ik.py                  # Inverse kinematics solver
│   │   ├── camera_static_transform.py  # Camera-to-flange static TF
│   │   └── simple_move.py         # Simple motion utility (not used)
│   └── launch/
│       ├── gsam_pickplace.launch.py    # Launch file for GSAM mode
│       └── marker_pickplace.launch.py  # Launch file for marker mode
│
├── actuation/                     # Gripper control package
│   └── actuation/
│       └── gripper_server.py      # Arduino serial gripper controller
│
└── planning_interfaces/           # Service definitions
    └── srv/
        ├── PickPlaceService.srv      # Not used for production (testing only)
        ├── ContinuousPickPlace.srv   # Main service for continuous pick place
        └── MoveToTarget.srv          # Service that move flange to a target pose
```

## 🚀 Installation

### 1. Clone the Repository

```bash
cd ~/your_workspace/src
git clone <repository-url> 25F_206A_FinalProject
```

### 2. Install ROS 2 Dependencies

```bash
sudo apt install ros-humble-realsense2-camera \
                 ros-humble-moveit \
                 ros-humble-tf2-ros \
                 ros-humble-cv-bridge
```

### 3. Install Python Dependencies

```bash
pip install opencv-python numpy torch transformers supervision pyserial
```

### 4. Build the Workspace

```bash
cd ~/your_workspace
colcon build --symlink-install
source install/setup.bash
```

### 5. Download AI Models (GSAM Mode Only)

For GSAM detection mode, download the SAM2 model checkpoint:

```bash
# Create models directory
mkdir -p ~/your_workspace/src/perception/realsense_cv/models

# Download SAM2.1 Hiera Small model
# (Download from Grounded-SAM-2 repository or provide direct link)
# Place at: ~/your_workspace/src/perception/realsense_cv/models/sam2.1_hiera_small.pt
```

The Grounding DINO model will be automatically downloaded from Hugging Face on first use.

## 💻 Usage

### Launch the System

#### Establish connection to the TM12 robot arm
```bash
ros2 launch tm12_moveit_config tm12_run_move_group.launch.py robot_ip:=172.16.8.2
```

#### Option 1: GSAM AI Detection Mode

```bash
ros2 launch planning gsam_pickplace.launch.py
```

#### Option 2: ArUco Marker Detection Mode

```bash
ros2 launch planning marker_pickplace.launch.py
```

### Call Continuous Pick-and-Place Service

Once the system is running, call the continuous pick-and-place service (Make sure you tune the values!):

```bash
ros2 service call /continuous_pick_place planning_interfaces/srv/ContinuousPickPlace "
{
  pick_scan_pose: {
    header: {frame_id: 'base'},
    pose: {
      position: {x: 0.514, y: 0.019, z: 0.314},
      orientation: {x: -0.506, y: 0.863, z: -0.002, w: -0.010}
    }
  },
  place_scan_pose: {
    header: {frame_id: 'base'},
    pose: {
      position: {x: 0.640, y: -0.740, z: 0.359},
      orientation: {x: 0.974, y: -0.224, z: 0.005, w: 0.008}
    }
  },
  pick_distance: 0.18,
  retreat_distance: 0.1,
  place_distance: 0.19,
  place_rotation_y_deg: 35.0
}
"
```

### Individual Services

These services can be helpful when trying to find the correct pose and distance for Continuous Pick-and-Place:

- **Single Pick-and-Place**: `/pick_and_place` (service: `PickPlaceService`)
- **Move to Target**: `/move_to_target` (service: `MoveToTarget`)
- **Gripper Control**: `/gripper/control` (service: `std_srvs/SetBool`) ("True" is close gripper)

## 🔍 Key Components

### 1. Perception Package (`realsense_cv`)

#### `gsam_slide_detect.py`
- **Purpose**: AI-based slide detection using Grounding DINO + SAM2
- **Input**: RGB images from RealSense camera
- **Output**: TF frames (`slide_XX`) for each detected slide
- **Key Features**:
  - Text-prompted object detection (e.g., "colored box.")
  - Instance segmentation with SAM2
  - Edge detection for slot localization
  - Automatic slot indexing (1-25)

#### `marker_detect.py`
- **Purpose**: ArUco marker-based slide localization
- **Input**: RGB images with ArUco markers
- **Output**: TF frames for marker positions and derived slide positions
- **Key Features**:
  - Pose estimation using solvePnP
  - Z-axis locking for table alignment
  - Pseudo slide frame generation

#### `slide_detector.py`
- **Purpose**: Helper node to monitor TF frames and return slide poses
- **Features**:
  - Tracks which slides have been picked
  - Converts TF frames to `PoseStamped` messages
  - Supports both marker and GSAM modes

### 2. Planning Package (`planning`)

#### `pick_and_place.py`
- **Purpose**: Main controller for pick-and-place operations
- **Key Features**:
  - Continuous pick-and-place until no slides remain
  - Two detection modes: `marker` and `gsam`
  - Two alignment methods: `direct` and `perpendicular`
  - IK-based motion planning
  - Configurable velocity scaling
  - Handles slides at any orientation and orientation changes during operation

#### `ik.py`
- **Purpose**: Inverse kinematics solver using MoveIt2 services
- **Features**:
  - IK computation for TM12 robot
  - Forward kinematics (FK)
  - Motion planning with joint constraints
  - Velocity scaling support

#### `camera_static_transform.py`
- **Purpose**: Publishes static transform from robot flange to camera
- **Transform**: `flange` → `camera_link`

### 3. Actuation Package (`actuation`)

#### `gripper_server.py`
- **Purpose**: Controls Arduino-based gripper via serial communication
- **Commands**:
  - `'C'` - Close gripper
  - `'O'` - Open gripper
- **Interface**: ROS 2 service `/gripper/control`

## 🎨 Detection Modes

### Marker Mode (`detection_mode: 'marker'`)

**How it works:**
1. ArUco markers are placed on the slide storage box
2. Camera detects markers and estimates pose
3. Slide positions are computed relative to marker positions
4. TF frames are published for each slide slot

**Pros:**
- Fast and reliable
- No AI model dependencies
- Works well in controlled lighting

**Cons:**
- Requires physical markers
- Less flexible (marker positions fixed)

### GSAM Mode (`detection_mode: 'gsam'`)

**How it works:**
1. Grounding DINO detects objects matching text prompt (e.g., "colored box.")
2. SAM2 generates segmentation masks
3. Edge detection identifies slide slots within the box
4. Kernel-based scanning detects individual slides
5. TF frames are published for detected slides

**Pros:**
- No markers required
- More flexible and generalizable
- Can detect slides in various configurations

**Cons:**
- Requires GPU for real-time performance
- More complex setup (model downloads)
- Slower than marker mode

## ⚡ Performance

The system demonstrates robust performance in real-world scenarios:

- **Continuous Operation**: Successfully performs continuous pick-and-place operations without failures
- **High Adaptability**: 
  - Handles glass slides placed at any orientation
  - Adapts to orientation changes during operation
  - Robust to variations in slide placement
- **Precision**: Accurate manipulation of fragile 1mm-thick glass slides
- **Reliability**: Stable operation with both marker-based and AI-based detection modes

### Experimental Results

- **Typical Setup**: 2-3 slides per storage box (due to gripper size constraints)
- **Success Rate**: High reliability in continuous pick-and-place cycles
- **Orientation Handling**: Successfully handles slides at various orientations and adapts to changes

## 🎥 Video Demo

https://youtu.be/4Mbxl2Z15pg

## 🔧 Configuration

### Key Parameters

#### GSAM Detection Node (`gsam_slide_detect.py`)
- `text_prompt`: Text description for object detection (default: `"colored box."`)
- `grounding_model`: Hugging Face model ID (default: `"IDEA-Research/grounding-dino-tiny"`)
- `sam2_checkpoint`: Path to SAM2 model file
- `force_cpu`: Force CPU mode (default: `False`)

#### Pick-and-Place Node (`pick_and_place.py`)
- `detection_mode`: `'marker'` or `'gsam'` (default: `'marker'`)
- `alignment_method`: `'direct'` or `'perpendicular'` (default: `'perpendicular'`)
- `z_velocity_scale`: Vertical motion speed (default: `0.2`)
- `xy_velocity_scale`: Horizontal motion speed (default: `0.4`)

#### Gripper Node (`gripper_server.py`)
- `serial_port`: Arduino serial port (default: `'/dev/ttyUSB0'`)
- `baud_rate`: Serial baud rate (default: `115200`)


## 📚 Citations

### AI Models

This project uses the following open-source models:

1. **Grounded-SAM-2** (Grounding DINO + SAM2)
   - Repository: [IDEA-Research/Grounded-SAM-2](https://github.com/IDEA-Research/Grounded-SAM-2)
   - Grounding DINO: [IDEA-Research/grounding-dino-tiny](https://huggingface.co/IDEA-Research/grounding-dino-tiny)
   - SAM2: Segment Anything Model 2 by Meta AI

2. **ArUco Markers**
   - OpenCV ArUco library for marker detection and pose estimation

## 👥 Authors

**Team Members:**
- David Chen
- Ryan Chung
- Yu-Wei Chang
- Bryan Chang
- Kain Hung

**Course**: EECS 206A: Introduction to Robotics (Fall 2025), UC Berkeley  
**Industry Partner**: Ember Robotics

## 📄 License

See individual package.xml files for license information.

---

**Note**: This project was developed as part of the EECS 206A course at UC Berkeley in collaboration with Ember Robotics. For more detailed information about the project, please refer to the [project presentation slides](https://docs.google.com/presentation/d/1IEKUIgCwdznhzMRzVUyWATOjx524NDUfZrOHgApcxFs/edit?usp=sharing).
