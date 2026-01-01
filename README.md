# ROS 2 Mobile Manipulator

## Overview

This repository contains a complete ROS 2 implementation of a mobile manipulator system featuring a differential-drive mobile base with an integrated 2-DOF robotic arm and RGB camera. The system is designed for simulation in Gazebo Harmonic and provides a modular framework for robotics research and development.

### System Components

- **Mobile Base**: Differential-drive platform with velocity control
- **Manipulator**: 2-DOF robotic arm with revolute joints (0-90° range)
- **Perception**: Forward-facing RGB camera with optical frame
- **Simulation**: Full Gazebo Harmonic integration with physics modeling
- **Control**: ROS 2 Control framework with position controllers

### Key Features

- Modular URDF/Xacro robot description architecture
- ROS 2 Control integration for real-time joint control
- Gazebo-ROS 2 bridge for seamless communication
- Transform (TF) tree publishing for spatial relationships
- RViz visualization support
- Configurable joint dynamics (friction and damping)
- Camera sensor integration with standard ROS interfaces

## Prerequisites

- **ROS 2**: Jazzy (tested) - Humble may also work but is untested
- **Gazebo**: Harmonic
- **Operating System**: Ubuntu 24.04 (for Jazzy) or Ubuntu 22.04 (for Humble)
- **Required ROS 2 Packages**:
  - `ros-jazzy-gazebo-ros-pkgs` (or `ros-humble-*` for Humble)
  - `ros-jazzy-ros2-control`
  - `ros-jazzy-ros2-controllers`
  - `ros-jazzy-xacro`
  - `ros-jazzy-joint-state-publisher`

## Repository Structure

## Repository Structure

```
ros2_mobile_manipulator/
├── robot_description/
│   ├── urdf/                 # URDF and Xacro files
│   ├── launch/               # RViz launch files
│   └── rviz/                 # RViz configuration
├── robot_bringup/
│   ├── launch/               # Main launch files
│   ├── world/                # Gazebo world files
│   ├── models/               # Gazebo models
│   └── bridge_config/        # Gazebo-ROS bridge configuration
└── README.md
```

## Installation

### Clone the Repository

```bash
cd ~/ros2_ws/src
git clone <repository-url> ros2_mobile_manipulator
```

### Build the Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select robot_description robot_bringup
source install/setup.bash
```

## Usage

### Launch the Complete Simulation

Start the Gazebo simulation with the mobile manipulator:

```bash
ros2 launch robot_bringup robot_gazebo.launch.py
```

This command initializes:
- Gazebo Harmonic simulator with the robot world
- Mobile manipulator robot model
- ROS 2 Control controllers
- Gazebo-ROS 2 bridge nodes
- Joint state publishers and TF broadcasters

### Control Interface

#### Mobile Base Control

Control the differential-drive base using velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

#### Arm Joint Control

Control individual arm joints using position commands:

```bash
# Forearm joint (0 to 1.57 rad)
ros2 topic pub /forearm_joint_cmd_pose std_msgs/msg/Float64 "data: 1.0"

# Hand joint (0 to 1.57 rad)
ros2 topic pub /hand_joint_cmd_pose std_msgs/msg/Float64 "data: 0.5"
```

#### Camera Visualization

To view the camera feed in RViz:

1. Launch RViz: `rviz2`
2. Add an **Image** display
3. Set the topic to `/camera/image_raw`

## Technical Specifications

### Mobile Base

- **Type**: Differential drive
- **Control Interface**: Velocity commands (`/cmd_vel`)
- **Sensors**: Wheel encoders for odometry

### Manipulator Arm

#### Links

| Link | Geometry | Dimensions | Mass (kg) | Color |
|------|----------|------------|-----------|-------|
| arm_base_link | Box | 0.1 × 0.1 × 0.02 m | 0.5 | Orange |
| forearm_link | Cylinder | r=0.02 m, l=0.3 m | 0.3 | Yellow |
| hand_link | Cylinder | r=0.02 m, l=0.3 m | 0.3 | Orange |

#### Joints

| Joint | Type | Range | Effort (N⋅m) | Velocity (rad/s) |
|-------|------|-------|--------------|------------------|
| arm_base → forearm | Revolute | 0 → π/2 rad | 100 | 100 |
| forearm → hand | Revolute | 0 → π/2 rad | 100 | 100 |

Both joints include friction and damping dynamics for realistic simulation.

#### Controllers

- **Joint State Publisher**: Publishes current joint positions
- **Position Controllers**: Individual controllers per joint
  - Forearm: P-gain = 5.0
  - Hand: P-gain = 3.0

### Camera

- **Type**: RGB camera
- **Topic**: `/camera/image_raw`
- **Frame**: Camera optical frame with proper orientation

## Development

### Architecture Overview

The system follows a modular ROS 2 architecture:

1. **Robot Description** (`robot_description`): Contains all URDF/Xacro files defining the robot's physical and visual properties
2. **Bringup** (`robot_bringup`): Launch files and configuration for simulation and control
3. **Control Layer**: ROS 2 Control framework manages joint controllers
4. **Bridge Layer**: Gazebo-ROS bridge handles communication between simulator and ROS 2

### Development Workflow

1. **Modeling**: Define robot geometry and properties in URDF/Xacro
2. **Visualization**: Test visual appearance in RViz
3. **Physics**: Add collision and inertia properties
4. **Simulation**: Integrate with Gazebo for physics simulation
5. **Control**: Implement controllers and bridge topics
6. **Testing**: Validate functionality in complete system

## Future Enhancements

- [ ] Gripper integration for manipulation tasks
- [ ] MoveIt2 integration for inverse kinematics and motion planning
- [ ] LIDAR sensor for obstacle detection
- [ ] Nav2 integration for autonomous navigation
- [ ] SLAM capabilities for mapping and localization
- [ ] Perception pipeline for object detection and recognition
- [ ] Behavior trees for task planning
- [ ] Multi-robot coordination

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

This project was developed as part of the ROS 2 For Beginners Level 2 course and has been extended with additional features including camera integration, improved URDF architecture, and Gazebo Harmonic compatibility.
