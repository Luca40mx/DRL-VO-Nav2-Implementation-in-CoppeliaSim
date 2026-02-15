
# DRL-VO Nav2 Implementation in CoppeliaSim

This repository contains a **Nav2-based implementation** of the **DRL-VO navigation control policy** integrated into the **CoppeliaSim environment**, developed for the *Mobile Robotics* course at the University of Verona.


## Overview

This workspace contains:
- **DRL-VO Navigation**: Implementation of the DRL-VO control policy for navigating through crowded dynamic scenes
- **Limo Robot Support**: ROS 2 packages for the Agilex Limo robot platform
- **Nav2 Integration**: Custom Nav2 controller plugin integrating DRL-VO with the Nav2 stack
- **CoppeliaSim Integration**: Simulation scenes and Lua scripts for CoppeliaSim

Based on the paper: ["DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles"](https://doi.org/10.1109/TRO.2023.3257549) (IEEE T-RO 2023)

## System Requirements

- **OS**: Ubuntu 22.04
- **ROS**: ROS 2 Humble Desktop
- **Python**: 3.10
- **CoppeliaSim**: V4.10.0 or later


## Installation

### **1. Install ROS2 (Humble) and Navigation 2**
- [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Nav2 Installation Guide](https://docs.nav2.org/getting_started/index.html)



### 2. Install CoppeliaSim

- [CoppeliaSim](https://www.coppeliarobotics.com/downloads) 

Then extract and move to desired location

### 3. Clone and Build Workspace

```bash
git clone <repository-url> .

# Install Python dependencies
cd ~/nav2_ws/src/drl_vo_nav
pip install -r requirements.txt

# Build the workspace
cd ~/nav2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Running the Navigation System

#### Terminal 1: Start CoppeliaSim

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch CoppeliaSim
cd /path/to/CoppeliaSim/folder
./coppeliaSim
```

In CoppeliaSim GUI:
1. File → Open scene and go to src/drl_vo_nav/nav2py_drl_vo/scenes/scene_to_use_Exam.ttt
2. Click "Play" button to start simulation
 

#### Terminal 2: Launch Nav2 with DRL-VO Controller

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash

# Launch the navigation stack
ros2 launch nav2py_drl_vo_controller bringup_drl.launch.py
```
At this point Rviz2 should open automatically with bringup_drl.launch.py.
**Inside RViz2:** Use the "2D Goal Pose" tool to set navigation goal.



## Configuration

Navigation parameters can be modified in:
- `src/drl_vo_nav/nav2py_drl_vo/tb3_drl_vo_nav2_params.yaml`
- Individual package config files in `src/drl_vo_nav/nav2py_drl_vo/nav2py_drl_vo_controller/config/`

#
### CoppeliaSim Connection

Ensure CoppeliaSim is running and publishing topics:
```bash
ros2 topic list
# Should see: /scan, /odom, /tf, etc.
```


## References

- [DRL-VO Paper](https://doi.org/10.1109/TRO.2023.3257549)
- [DRL-VO Repository](https://github.com/TempleRAIL/drl_vo_nav)
- [Limo ROS 2 Repository](https://github.com/agilexrobotics/limo_ros2)
- [Nav2 Documentation](https://navigation.ros.org/)
- [CoppeliaSim Documentation](https://www.coppeliarobotics.com/helpFiles/)

