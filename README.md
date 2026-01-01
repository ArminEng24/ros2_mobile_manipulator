# ROS 2 Mobile Manipulator — **Hercules**
A ROS 2 + Gazebo Harmonic simulation of a differential‑drive mobile robot equipped with a 2‑DOF manipulator arm and an onboard RGB camera.

This project was built as the final project for the **ROS2 For Beginners Level 2** course and extended with additional features such as a camera, improved URDF structure, and Harmonic compatibility.

---

## 🚀 Overview

**Hercules** combines:

- A differential‑drive mobile base  
- A 2‑DOF robotic arm (forearm + hand, each rotating 0 → 90° around the Y‑axis)  
- A forward‑facing RGB camera  
- A complete Gazebo Harmonic simulation  
- Modular URDF/Xacro robot description  
- ROS 2 Control, TF, RViz, and ROS ↔ Gazebo bridging  

This project demonstrates a full ROS 2 robot pipeline: modeling → simulation → control → visualization.

---

## ✨ Features

- Differential‑drive mobile base  
- 2‑DOF manipulator arm  
  - Forearm joint: 0 → 90°  
  - Hand joint: 0 → 90°  
  - Revolute joints with friction + damping  
- Onboard RGB camera with optical frame  
- URDF/Xacro modular robot description  
- Gazebo Harmonic simulation  
- ROS 2 Control integration  
- JointState + TF publishing  
- RViz visualization  
- ROS ↔ Gazebo bridge for motion control  
- Teleoperation for base and arm  

---

## 📁 Repository Structure

```
ros2_mobile_manipulator/
├── my_robot_description/     # URDF, Xacro, meshes, materials
├── my_robot_gazebo/          # Gazebo plugins, world files, configs
├── my_robot_bringup/         # Launch files for RViz + Gazebo
├── my_robot_bridge/          # ROS ↔ Gazebo bridge YAML
└── README.md
```

---

## 🔧 Build Instructions

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## ▶️ Launch the Simulation

```bash
ros2 launch my_robot_bringup simulation.launch.py
```

This starts:

- Gazebo Harmonic  
- The Hercules robot  
- ROS 2 controllers  
- Gazebo ↔ ROS 2 bridges  
- RViz (optional depending on your launch file)

---

## 🎮 Control the Robot

### Move the mobile base

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### Move the arm joints

```bash
ros2 topic pub /forearm_joint_cmd_pose std_msgs/msg/Float64 "data: 1.0"
ros2 topic pub /hand_joint_cmd_pose std_msgs/msg/Float64 "data: 0.5"
```

### View the camera feed in RViz

Add an **Image** display and set:

```
/camera/image_raw
```

---

## 🦾 Arm Specifications (Course Project Requirements)

### Links

- **arm_base_link**  
  - Box: `0.1 × 0.1 × 0.02`  
  - Color: orange  
  - Mass: 0.5  

- **forearm_link**  
  - Cylinder: radius `0.02`, length `0.3`  
  - Color: yellow  
  - Mass: 0.3  

- **hand_link**  
  - Cylinder: radius `0.02`, length `0.3`  
  - Color: orange  
  - Mass: 0.3  

### Joints

- **arm_base → forearm**  
  - Revolute  
  - Limits: `0 → π/2`  
  - Effort: 100  
  - Velocity: 100  
  - Dynamics: friction + damping  

- **forearm → hand**  
  - Same specs as above  

### Gazebo Plugins

- `JointStatePublisher`  
- `JointPositionController` (one per joint)  
  - p_gain: 5.0 (forearm), 3.0 (hand)

---

## 🧠 How the Project Was Built (Course Steps)

### Step 1 — Build the arm
- Create `arm.xacro` and `standalone_arm.urdf.xacro`
- Add visual links + joints
- Test in RViz

### Step 2 — Add physics
- Add collision + inertia  
- Add joint dynamics  
- Test in Gazebo

### Step 3 — Add control
- Add JointStatePublisher  
- Add JointPositionController  
- Bridge topics  
- Publish commands from the terminal

### Step 4 — Integrate with mobile base
- Attach arm to `base_link`  
- Launch full robot in Gazebo  
- Test base + arm + camera together

---

## 🖼️ Screenshots

*(Add your RViz and Gazebo screenshots here.)*

---

## 🛠️ Future Work

- Add a gripper  
- Add MoveIt2 for IK + motion planning  
- Add LIDAR or depth camera  
- Add Navigation2 for autonomous navigation  
- Add behavior trees or mission scripts  
- Add SLAM  
- Add perception pipelines  

---

## 📄 License

Choose one:

- MIT  

---

## 🙌 Credits

This project was built as part of the  
**ROS2 For Beginners Level 2 — Final Project**,  
and extended with additional features (camera, improved URDF, Harmonic support).
