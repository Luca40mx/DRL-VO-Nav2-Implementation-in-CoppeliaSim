# Nav2 DRL-VO Navigation Workspace

A ROS 2 Humble workspace integrating the DRL-VO (Deep Reinforcement Learning - Velocity Obstacles) navigation controller with Nav2 for autonomous robot navigation in dynamic environments using the Limo robot platform and CoppeliaSim simulation.

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
- **GPU**: CUDA-compatible GPU (optional but recommended for training)

## Installation

### 1. Install ROS 2 Humble

```bash
# Follow official ROS 2 Humble installation guide
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rqt-robot-steering \
  ros-humble-teleop-twist-keyboard
```

### 2. Install CoppeliaSim

```bash
# Download CoppeliaSim from:
# https://www.coppeliarobotics.com/downloads

# Extract and move to desired location
cd ~/Downloads
wget https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz
tar -xf CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz
sudo mv CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04 /opt/coppeliasim
```

### 3. Clone and Build Workspace

```bash
# Create workspace directory (if not already cloned)
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src

# If this is a fresh clone:
# git clone <repository-url> .

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
cd /opt/coppeliasim
./coppeliaSim.sh

# In CoppeliaSim GUI:
# 1. Open: src/drl_vo_nav/nav2py_drl_vo/scenes/limo.ttt (or your custom scene)
# 2. Click "Play" button to start simulation
```

#### Terminal 2: Launch Nav2 with DRL-VO Controller

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash

# Launch the navigation stack
ros2 launch nav2py_drl_vo_controller bringup_drl.launch.py
```

#### Terminal 3: Send Navigation Goals

**Option A: Using ROS 2 Action (CLI)**
```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash

# Send a goal pose
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Option B: Using RViz2**
```bash
# RViz2 should automatically launch with bringup_drl.launch.py
# Use the "2D Goal Pose" tool to set navigation goals interactively
```

**Option C: Using Keyboard Teleop (Manual Control)**
```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Simplified Launch (CoppeliaSim Only)

For testing with just CoppeliaSim integration:

```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash

ros2 launch nav2py_drl_vo_controller limo_coppelia_simple.launch.py
```

## Workspace Structure

```
nav2_ws/
├── src/
│   ├── drl_vo_nav/                    # DRL-VO navigation implementation
│   │   └── nav2py_drl_vo/
│   │       ├── nav2py/                # Python Nav2 utilities
│   │       ├── nav2py_drl_vo_controller/  # Nav2 DRL-VO plugin
│   │       └── scenes/                # CoppeliaSim scenes
│   │           ├── limo.ttt           # Main simulation scene
│   │           ├── limo_ros2_bridge.lua  # ROS 2 bridge script
│   │           └── [custom_scene].lua # Your custom scene (upload here)
│   │
│   └── limo_ros2/                     # Limo robot packages
│       ├── limo_base/                 # Base driver
│       ├── limo_car/                  # Gazebo simulation
│       ├── limo_description/          # Robot URDF models
│       └── limo_msgs/                 # Custom messages
│
├── build/                             # Build artifacts (git-ignored)
├── install/                           # Install space (git-ignored)
├── log/                              # Build logs (git-ignored)
└── README.md                         # This file
```

## CoppeliaSim Scene Upload

To add your custom CoppeliaSim scene:

1. Place your `.ttt` scene file in: `src/drl_vo_nav/nav2py_drl_vo/scenes/`
2. If you have a custom Lua script, place it in the same directory
3. Update the launch file to reference your scene if needed

## Configuration

Navigation parameters can be modified in:
- `src/drl_vo_nav/nav2py_drl_vo/tb3_drl_vo_nav2_params.yaml`
- Individual package config files in `src/drl_vo_nav/nav2py_drl_vo/nav2py_drl_vo_controller/config/`

## Troubleshooting

### Build Issues

```bash
# Clean build
cd ~/nav2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Python Dependencies

```bash
# Reinstall Python packages
cd ~/nav2_ws/src/drl_vo_nav
pip install --upgrade -r requirements.txt
```

### CoppeliaSim Connection

Ensure CoppeliaSim is running and publishing topics:
```bash
ros2 topic list
# Should see: /scan, /odom, /tf, etc.
```

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
# View generated frames.pdf
```

## Development

### Building Individual Packages

```bash
cd ~/nav2_ws
colcon build --packages-select nav2py_drl_vo_controller
source install/setup.bash
```

### Running Tests

```bash
cd ~/nav2_ws
colcon test
colcon test-result --verbose
```

## References

- [DRL-VO Paper](https://doi.org/10.1109/TRO.2023.3257549)
- [DRL-VO Repository](https://github.com/TempleRAIL/drl_vo_nav)
- [Limo ROS 2 Repository](https://github.com/agilexrobotics/limo_ros2)
- [Nav2 Documentation](https://navigation.ros.org/)
- [CoppeliaSim Documentation](https://www.coppeliarobotics.com/helpFiles/)

## License

- DRL-VO: See [LICENSE](src/drl_vo_nav/LICENSE)
- Limo ROS2: Check individual package licenses

## Citation

If you use this work, please cite:

```bibtex
@article{wang2023drlvo,
  title={DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles},
  author={Wang, Guangyi and Miao, Congcong and Wang, Mingyu and Li, Hao and Dames, Philip},
  journal={IEEE Transactions on Robotics},
  year={2023},
  publisher={IEEE}
}
```

## Contact

For issues and questions:
- Open an issue on GitHub
- Check original DRL-VO repository for algorithm-specific questions
