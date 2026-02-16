# BUMRC_NewArm  

**Software repository for the ARM robot of the BU Mars Rover**  
*Includes simulation, MoveIt 2 integration, linkattacher isntall, and vision explanation.

---

## Table of Contents

- [Project Overview](#project-overview)  
- [Repository Structure](#repository-structure)  
- [Requirements](#requirements)  
- [Workspace Setup](#workspace-setup)  
- [Build and Source](#build-and-source)  
- [Simulation Launch](#simulation-launch)  
- [Verification and Testing](#verification-and-testing)  
- [Hardware Integration](#hardware-integration)  
- [Additional Notes](#additional-notes)  
- [Licensing](#licensing)  

---

## Project Overview

Full ROS2 workspace containing the ARM robot description (URDF, meshes), Gazebo simulation launch files, and MoveIt 2 configuration for motion planning and execution. Supports both simulation and hardware testing.

---

## Repository Structure

- src/
- ├── arm_description/
- │ ├── urdf/
- │ ├── meshes/
- │ └── launch/
- └── arm_gripper_moveit_config/
- ├── config/
- └── launch/

- **arm_description**: Contains robot URDF, meshes, materials, and Gazebo integration.  
- **arm_gripper_moveit_config**: MoveIt 2 planning and execution configuration for robot and gripper.

---

## Requirements

- **OS**: Ubuntu 22.04 LTS  
- **ROS2**: Humble/Iron  
- **Gazebo**: 11+  
- **Python**: 3.10+  
- **Build tool**: colcon  

**ROS packages**:
```bash
sudo apt install ros-humble-moveit ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control



