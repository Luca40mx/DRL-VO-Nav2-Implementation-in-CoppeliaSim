# nav2py_drl_vo
# DRL-VO Nav2 Implementation in CoppeliaSim

This repository contains a **Nav2-based implementation** of the **DRL-VO navigation control policy** integrated into the **CoppeliaSim environment**, developed for the *Mobile Robotics* course at the University of Verona.

The implementation is based on the paper:

**DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles**  
https://doi.org/10.1109/TRO.2023.3257549  
https://arxiv.org/pdf/2301.06512.pdf  

---

## References

- **Original DRL-VO repository**  
  https://github.com/TempleRAIL/drl_vo_nav  

- **ROS 2 (Humble) + Gazebo implementation**  
  https://github.com/TempleRAIL/drl_vo_nav/tree/humble/nav2py_drl_vo  

---

This repository adapts the original work to a **ROS 2 Nav2 framework running in CoppeliaSim**, maintaining compatibility with the original DRL-VO architecture while enabling its use within a different simulation environment.



## Requirements:
* Ubuntu 22.04
* ROS2-Humble
* Python 3.10.12

## 📌 Installation

### **1. Install ROS2 (Humble) and Navigation 2**
- [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Nav2 Installation Guide](https://docs.nav2.org/getting_started/index.html)

### **2. Clone this repository into your workspace**
```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src
git clone -b humble https://github.com/TempleRAIL/drl_vo_nav.git
```

### **3. Build the workspace**
```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build
```

### **4. Source the workspace**
```bash
source ~/nav2_ws/install/setup.bash
```

### **5. Run the Nav2 demo launch**
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:=$(pwd)/src/nav2py_drl_vo/tb3_drl_vo_nav2_params.yaml
```


