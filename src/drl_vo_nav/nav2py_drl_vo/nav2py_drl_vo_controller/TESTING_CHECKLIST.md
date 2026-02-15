# DRL-VO CoppeliaSim Integration Testing Checklist

## Prerequisites
- [ ] ROS2 workspace built: `colcon build --packages-select nav2py_drl_vo_controller`
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] CoppeliaSim running with Lua script OR test publisher ready

---

## Test 1: Topic Verification

### Check topic exists:
```bash
ros2 topic list | grep dynamic_obstacles
```
**Expected output**: `/dynamic_obstacles`

### Monitor raw messages:
```bash
ros2 topic echo /dynamic_obstacles --once
```
**Expected**: JSON with `obstacles` array containing `id, x, y, z, vx, vy` fields

### Check publishing rate:
```bash
ros2 topic hz /dynamic_obstacles
```
**Expected**: ~10 Hz from CoppeliaSim script

---

## Test 2: Standalone Test Publisher

### Run test publisher:
```bash
cd ~/nav2_ws
source install/setup.bash
python3 src/drl_vo_nav/nav2py_drl_vo/nav2py_drl_vo_controller/nav2py_drl_vo_controller/test_obstacle_integration.py
```

**Expected logs**:
```
Published: pos=(10.00, 3.00), vel=(-1.50, 0.00)
Published: pos=(9.98, 3.05), vel=(-1.52, 0.03)
...
```

---

## Test 3: Subscriber Initialization

### Launch the controller:
```bash
# Adjust launch command based on your setup:
ros2 launch nav2py_drl_vo <your_launch_file>.launch.py

# Or run directly if it's a node:
ros2 run nav2py_drl_vo_controller <node_name>
```

**Expected logs** (on startup):
```
[nav2py_drl_vo_controller] nav2py_drl_vo_controller initialized
[dynamic_obstacle_subscriber] DynamicObstacleSubscriber initialised — listening on /dynamic_obstacles
[nav2py_drl_vo_controller] ROS2 DynamicObstacleSubscriber started in background thread
```

**Error to watch for**:
```
[ERROR] Failed to start DynamicObstacleSubscriber: ...
[ERROR] ROS2 spin error: ...
```
If this appears, check that `rclpy.init()` isn't being called twice.

---

## Test 4: Obstacle Reception

While controller is running, check logs for:

**From subscriber** (every ~50th message, ~5 seconds at 10 Hz):
```
[dynamic_obstacle_subscriber] Obstacles: 1 | Sample: id=1, pos=(10.00,3.00), vel=(-1.50,0.00)
```

**If this is missing**:
- Verify topic name matches: `ros2 topic list`
- Check if subscriber thread started: look for thread crash errors
- Check: `ros2 node list` — is `dynamic_obstacle_subscriber` listed?

---

## Test 5: Grid Generation

**Expected logs** (when obstacles are within [0,20]×[-10,+10] range):
```
[dynamic_obstacle_subscriber] Ped velocity map: 2/12800 non-zero cells, max_vel=1.500 m/s
[nav2py_drl_vo_controller] [MAIN] Using ped_map with 2 non-zero cells
```

**If always 0 non-zero cells**:
1. Check robot pose is being updated:
   - Look for `_data_callback` logs showing `Robot position: x=... y=...`
2. Check obstacle coordinates:
   - Are obstacles behind robot (rel_x < 0)? → Filtered out (correct behavior)
   - Are obstacles too far (rel_x > 20 or |rel_y| > 10)? → Outside grid range
3. Verify `update_robot_pose()` is being called:
   - Look for the robot pose extraction in `_data_callback` logs

---

## Test 6: Coordinate Transform Verification

### Manual sanity check:
- Robot at (0, 0, 0°), obstacle at global (5, 2) → relative (5, 2) → r=20, c=32
- Robot at (5, 0, 0°), obstacle at global (5, 0) → relative (0, 0) → r=0, c=40
- Robot at (0, 0, 90°), obstacle at global (0, 5) → relative (5, 0) → r=20, c=40

### Temporary debug (add to `get_ped_velocity_map()` if needed):
```python
# TEMPORARY DEBUG — remove after verification
if robot_pose is not None and len(obstacles) > 0:
    self.get_logger().warn(
        f'[DEBUG] Robot=({robot_pose[0]:.2f},{robot_pose[1]:.2f},{robot_pose[2]:.2f}rad) '
        f'Obs_global=({ox:.2f},{oy:.2f}) Obs_rel=({rel_x:.2f},{rel_y:.2f}) '
        f'Grid=(r={r},c={c})')
```

---

## Test 7: End-to-End Navigation (if Nav2 stack available)

1. Place virtual obstacles in robot's path in CoppeliaSim
2. Send navigation goal
3. Observe robot behavior:
   - [ ] Robot sees obstacles (check non-zero ped_map)
   - [ ] Robot adjusts trajectory to avoid them
   - [ ] Different behavior than pure LiDAR mode (zeros ped_map)

**Note**: This requires full Nav2 setup + map + localization.

---

## Test 8: Edge Cases

| Scenario | Expected Behavior |
|---|---|
| No obstacles published | `get_ped_velocity_map()` returns `np.zeros(12800)` |
| Obstacle at (25, 0) | Filtered out (x > 20) |
| Obstacle at (-5, 0) | Filtered out (x < 0, behind robot) |
| Obstacle at (10, 15) | Filtered out (y > 10) |
| Obstacle at exact boundary (20.0, 0) | Clamped to grid (r=79, c=40) |
| 20 obstacles | All processed, no performance issues |
| Malformed JSON | Error logged, no crash |
| Empty obstacles array | Returns zeros, no crash |

---

## Debugging Guide

### Issue: Subscriber never receives data
- Check: `ros2 node list` → is `dynamic_obstacle_subscriber` listed?
- Check: Thread started? Look for `_spin_ros()` errors in logs
- Check: Topic name typo? Use `ros2 topic info /dynamic_obstacles`

### Issue: ped_map always zeros
1. Add print in `_obstacle_callback`: is it being called?
2. Add print in `update_robot_pose()`: is robot pose set?
3. Add print showing obstacle global vs relative coords
4. Check if obstacles are outside [0,20]×[-10,+10] range after transform

### Issue: Robot doesn't avoid obstacles
- Verify ped_map has non-zero values (use logs from Test 5)
- Check neural network is loaded correctly (separate issue, not integration)
- Compare behavior with/without obstacles to confirm network uses ped_map

---

## Files Modified

| File | Change |
|---|---|
| `dynamic_obstacle_subscriber.py` | **NEW** — ROS2 node, JSON parser, grid builder |
| `__main__.py` | ROS2 thread init, robot pose extraction, ped_map routing |
| `planner.py` | No changes (already handles `ped_map=None` fallback) |
| `test_obstacle_integration.py` | **NEW** — Test publisher script |

## ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/dynamic_obstacles` | `std_msgs/String` | CoppeliaSim → Subscriber | JSON obstacle data |

## Expected JSON Format
```json
{
  "obstacles": [
    {"id": 1, "x": 1.5, "y": 2.3, "z": 0.25, "vx": 0.3, "vy": -0.1},
    {"id": 2, "x": -1.2, "y": 0.8, "z": 0.25, "vx": -0.2, "vy": 0.4}
  ]
}
```

## Success Criteria

Integration is working correctly if:
- ✅ Subscriber receives obstacle messages (Test 2, 4)
- ✅ Grid generation produces non-zero velocities when obstacles present (Test 5)
- ✅ Coordinate transform is correct (Test 6)
- ✅ No runtime errors or thread crashes
- ✅ Fallback to zeros works when no obstacles

**Full validation requires trained neural network and Nav2 stack (out of scope for integration testing).**
