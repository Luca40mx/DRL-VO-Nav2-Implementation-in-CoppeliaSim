# CoppeliaSim Dynamic Obstacles Integration

## Overview

This integration allows the DRL-VO Nav2 controller to receive dynamic obstacle data from CoppeliaSim (or any other simulator) via a ROS2 topic, replacing the original ZED camera + YOLOv3 + MHT tracker pipeline.

## Architecture

### Original System (Hardware)
```
ZED Camera → YOLOv3 → MHT Tracker → Kinematic Maps → DRL-VO Network
```

### New System (Simulation)
```
CoppeliaSim Lua Script → /dynamic_obstacles (JSON) → DynamicObstacleSubscriber → Kinematic Maps → DRL-VO Network
```

## Components Modified

| File | Changes | Purpose |
|---|---|---|
| `dynamic_obstacle_subscriber.py` | **NEW** | ROS2 subscriber that reads `/dynamic_obstacles` topic and generates 80×80×2 velocity grid |
| `__main__.py` | Modified | Initializes subscriber in background thread, replaces broken `/ped_vel_map` subscription |
| `planner.py` | None | Already handles `ped_map=None` fallback correctly |

## Data Flow

1. **CoppeliaSim publishes** (10 Hz):
   ```json
   {
     "obstacles": [
       {"id": 1, "x": 5.2, "y": 1.3, "z": 0.25, "vx": 0.5, "vy": -0.2}
     ]
   }
   ```

2. **DynamicObstacleSubscriber receives** and stores in thread-safe list

3. **Controller updates robot pose** (from Nav2 odometry) via `update_robot_pose(x, y, theta)`

4. **Grid generation** (on each control cycle):
   - Transform obstacles from global → robot frame
   - Bin into 80×80 grid (0.25m resolution, [0,20]m × [-10,+10]m range)
   - Flatten to shape (12800,): first 6400 = vx channel, last 6400 = vy channel

5. **DRL-VO network** uses `ped_map` alongside LiDAR scan to compute velocity command

## Coordinate System

### Grid Layout (matches original `cnn_data_pub.py`)
- **X axis (rows)**: [0, 20] meters forward from robot
- **Y axis (columns)**: [-10, +10] meters, left(+) / right(-), **inverted** in grid
- **Robot position**: (x=0, y=0) → bottom-center of grid (row 0, col 40)
- **Bin size**: 0.25 m/cell

### Example Mappings
| Obstacle Position (robot frame) | Grid Index | Notes |
|---|---|---|
| (5m, 0m) | r=20, c=40 | 5m ahead, center |
| (0m, 5m) | r=0, c=20 | At robot, 5m left |
| (10m, -10m) | r=40, c=79 | 10m ahead, 10m right (edge) |
| (-5m, 0m) | Rejected | Behind robot (outside [0,20] range) |
| (25m, 0m) | Rejected | Too far ahead |

## ROS2 Topic Interface

### Subscribed Topic
- **Name**: `/dynamic_obstacles`
- **Type**: `std_msgs/msg/String`
- **Format**: JSON with schema:
  ```json
  {
    "obstacles": [
      {
        "id": "<int>",
        "x": "<float — global x position (m)>",
        "y": "<float — global y position (m)>",
        "z": "<float — height (ignored)>",
        "vx": "<float — global x velocity (m/s)>",
        "vy": "<float — global y velocity (m/s)>"
      }
    ]
  }
  ```

### Published Topics
None (subscriber only)

## Testing

See [TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) for detailed testing procedures.

### Quick Test

1. **Source workspace**:
   ```bash
   source install/setup.bash
   ```

2. **Run test publisher** (terminal 1):
   ```bash
   python3 src/drl_vo_nav/nav2py_drl_vo/nav2py_drl_vo_controller/nav2py_drl_vo_controller/test_obstacle_integration.py
   ```

3. **Launch controller** (terminal 2):
   ```bash
   ros2 launch nav2py_drl_vo_controller bringup_drl.launch.py
   ```

4. **Check logs** for:
   ```
   [dynamic_obstacle_subscriber] Obstacles: 1 | Sample: id=1, pos=(...)
   [dynamic_obstacle_subscriber] Ped velocity map: 45/12800 non-zero cells
   [nav2py_drl_vo_controller] [MAIN] Using ped_map with 45 non-zero cells
   ```

## CoppeliaSim Setup

Your Lua script must publish to `/dynamic_obstacles` in the correct format. Example:

```lua
function sysCall_actuation()
    local json = require('dkjson')
    local obstacles = {}

    for i = 1, #cubes do
        local pos = sim.getObjectPosition(cubes[i].handle, -1)  -- global frame
        local vel = cubes[i].velocity  -- computed by your movement logic

        table.insert(obstacles, {
            id = i,
            x = pos[1],
            y = pos[2],
            z = pos[3],
            vx = vel[1],
            vy = vel[2]
        })
    end

    local msgData = {data = json.encode({obstacles = obstacles})}
    simROS2.publish(obstaclesPub, msgData)
end
```

## Troubleshooting

### No obstacles received
- Check topic: `ros2 topic echo /dynamic_obstacles --once`
- Check subscriber init: look for `[dynamic_obstacle_subscriber] DynamicObstacleSubscriber initialised` in logs
- Check ROS2 thread: if missing, check for `ROS2 spin error` in logs

### ped_map always zeros
- Verify robot pose updated: add debug log in `update_robot_pose()`
- Check obstacle coordinates: are they within [0,20]×[-10,+10] robot-relative range?
- Check frame: CoppeliaSim must publish in **global frame**, controller transforms to robot frame

### Build errors
- Ensure ROS2 Humble sourced: `source /opt/ros/humble/setup.bash`
- Clean rebuild: `rm -rf build/ install/ && colcon build --packages-select nav2py_drl_vo_controller`

## Performance

- **Subscriber thread**: ~0.1% CPU (daemon thread, 10 Hz callback)
- **Grid generation**: ~0.5ms per cycle (80×80 grid, pure NumPy)
- **Memory**: ~100 KB (grid arrays + obstacle list)

## Limitations

- **No depth filtering**: All obstacles at any z-height are projected onto 2D grid
- **No obstacle size**: Each obstacle treated as point mass
- **No occlusion**: Obstacles behind walls are not filtered (requires LiDAR fusion)
- **Global frame assumption**: CoppeliaSim must publish in same global frame as Nav2 localization

## Future Improvements

- Add `/dynamic_obstacles_markers` visualization topic (MarkerArray) for RViz
- Add obstacle size (`radius` field) and inflate grid cells accordingly
- Add LiDAR-based obstacle filtering (remove obstacles behind walls)
- Add configurable grid parameters (range, resolution) via ROS2 params
- Add performance metrics publisher (callback latency, grid generation time)

## References

- Original DRL-VO paper: [arXiv:2301.06512](https://arxiv.org/abs/2301.06512)
- Original ROS1 code: [github.com/TempleRAIL/drl_vo_nav](https://github.com/TempleRAIL/drl_vo_nav)
- Grid generation: `cnn_data_pub.py` lines 53-76
- Coordinate transform: `track_ped_pub.py` lines 65-74
