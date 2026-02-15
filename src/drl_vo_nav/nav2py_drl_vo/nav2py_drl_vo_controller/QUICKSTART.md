# Quick Start: Testing CoppeliaSim Integration

## Prerequisites
- ROS2 Humble installed
- Workspace built and sourced
- CoppeliaSim running OR test publisher ready

## 5-Minute Test (No CoppeliaSim Required)

### Terminal 1: Test Publisher
```bash
cd /home/luca/nav2_ws
source install/setup.bash
python3 src/drl_vo_nav/nav2py_drl_vo/nav2py_drl_vo_controller/nav2py_drl_vo_controller/test_obstacle_integration.py
```

**Expected output**:
```
Publishing test obstacles to /dynamic_obstacles at 10 Hz
Published: pos=(10.00, 3.00), vel=(-1.50, 0.00)
...
```

### Terminal 2: Verify Topic
```bash
source install/setup.bash
ros2 topic echo /dynamic_obstacles --once
```

**Expected output**:
```json
data: '{"obstacles": [{"id": 1, "x": 10.0, "y": 3.0, ...}]}'
```

### Terminal 3: Launch Controller
```bash
source install/setup.bash
ros2 launch nav2py_drl_vo_controller bringup_drl.launch.py
```

**Expected logs**:
```
[nav2py_drl_vo_controller] nav2py_drl_vo_controller initialized
[dynamic_obstacle_subscriber] DynamicObstacleSubscriber initialised
[dynamic_obstacle_subscriber] Obstacles: 1 | Sample: id=1, pos=(...)
[dynamic_obstacle_subscriber] Ped velocity map: 45/12800 non-zero cells
```

**Success criteria**: ✅ Non-zero ped_map cells logged

## With CoppeliaSim

1. Launch CoppeliaSim with your scene
2. Run your Lua script (should publish to `/dynamic_obstacles`)
3. Launch DRL-VO controller (Terminal 3 above)
4. Send navigation goal (if Nav2 stack running)
5. Observe robot avoiding dynamic obstacles

## Troubleshooting

**"Ped velocity map: 0/12800 non-zero cells"**
→ Obstacles outside [0,20]×[-10,+10] robot frame or robot pose not updated

**No logs from dynamic_obstacle_subscriber**
→ Check `ros2 node list`, verify ROS2 thread started

**See full troubleshooting guide**: [README_INTEGRATION.md](README_INTEGRATION.md#troubleshooting)
