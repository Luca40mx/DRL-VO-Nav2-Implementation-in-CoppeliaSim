"""
MODIFIED: integrated CoppeliaSim obstacles
Standalone ROS2 node that subscribes to /dynamic_obstacles (std_msgs/String JSON)
and converts dynamic obstacle data into a 2-channel velocity map compatible with
the DRL-VO neural network input (flat np.float32 array of shape 12800).

Topic format expected:
{
  "obstacles": [
    {"id": 1, "x": 1.5, "y": 2.3, "z": 0.25, "vx": 0.3, "vy": -0.1},
    ...
  ]
}

Grid specification (matches original cnn_data_pub.py):
  - 2 channels: vx (first 6400) and vy (last 6400)
  - 80×80 grid, 0.25 m/bin
  - X axis (rows): [0, 20] m forward from robot
  - Y axis (cols): [-10, +10] m left/right, column index inverted
  - Robot sits at (x=0, y=0) → row 0, column 40

Flattening order: np.concatenate([vx_grid.flatten(), vy_grid.flatten()])
  which is equivalent to shape (2, 80, 80) C-order flatten, matching
  the NN reshape: ped_pos.reshape(-1, 2, 80, 80) in custom_cnn_full.py.
"""

import json
import threading
from typing import Tuple, Optional

import numpy as np
from rclpy.node import Node
from std_msgs.msg import String


class DynamicObstacleSubscriber(Node):
    """ROS2 node that converts CoppeliaSim /dynamic_obstacles into a ped velocity map."""

    # ---- Grid constants (must match DRL-VO training / cnn_data_pub.py) -----
    GRID_SIZE: int = 80           # cells per axis
    BIN_SIZE: float = 0.25        # metres per cell
    X_MIN: float = 0.0            # metres — robot position (forward start)
    X_MAX: float = 20.0           # metres — max forward distance
    Y_MIN: float = -10.0          # metres — right side
    Y_MAX: float = 10.0           # metres — left side
    MAP_SIZE: int = GRID_SIZE * GRID_SIZE    # 6400
    TOTAL_SIZE: int = 2 * MAP_SIZE           # 12800

    def __init__(self):
        super().__init__('dynamic_obstacle_subscriber')

        # Thread-safe lock for shared data
        self._lock = threading.Lock()

        # Latest obstacle list (raw JSON dicts)
        self._obstacles: list = []

        # Robot pose in global frame: (x, y, theta)
        # Set by the controller via update_robot_pose()
        self._robot_pose: Optional[Tuple[float, float, float]] = None

        # ROS2 subscription
        self.subscription = self.create_subscription(
            String,
            '/dynamic_obstacles',
            self._obstacle_callback,
            10
        )

        self._diag_done = False
        self._msg_count = 0  # ADDED: message counter for periodic logging
        self.get_logger().info(
            'DynamicObstacleSubscriber initialised — '
            'listening on /dynamic_obstacles')

    # --------------------------------------------------------------------- #
    #  Callback (runs on the ROS2 spin thread)
    # --------------------------------------------------------------------- #
    def _obstacle_callback(self, msg: String) -> None:
        """Parse incoming JSON obstacle data."""
        try:
            data = json.loads(msg.data)
            obstacles = data.get('obstacles', [])

            with self._lock:
                self._obstacles = obstacles

            # One-time diagnostic
            if not self._diag_done and len(obstacles) > 0:
                self.get_logger().info(
                    f'✓ First obstacle batch received: {len(obstacles)} obstacles')
                self._diag_done = True

            # ADDED: Periodic logging (every 50th message to avoid spam)
            self._msg_count += 1
            if len(obstacles) > 0 and self._msg_count % 50 == 0:
                sample = obstacles[0]
                self.get_logger().info(
                    f'Obstacles: {len(obstacles)} | Sample: '
                    f'id={sample.get("id")}, '
                    f'pos=({sample.get("x", 0):.2f},{sample.get("y", 0):.2f}), '
                    f'vel=({sample.get("vx", 0):.2f},{sample.get("vy", 0):.2f})')

            self.get_logger().debug(
                f'Updated {len(obstacles)} obstacles')

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Error parsing obstacles: {e}')

    # --------------------------------------------------------------------- #
    #  Public API (called from the controller / main thread)
    # --------------------------------------------------------------------- #
    def update_robot_pose(self, pose: Tuple[float, float, float]) -> None:
        """
        Update the robot pose used for global → robot-frame conversion.

        Args:
            pose: (x, y, theta) in the global/map frame.
        """
        with self._lock:
            self._robot_pose = pose
        # DEBUG: Periodic pose logging (first call + every 50th)
        if not hasattr(self, '_pose_update_count'):
            self._pose_update_count = 0
        self._pose_update_count += 1
        if self._pose_update_count == 1 or self._pose_update_count % 50 == 0:
            self.get_logger().info(
                f'[POSE] Robot at x={pose[0]:.2f}, y={pose[1]:.2f}, '
                f'theta={pose[2]:.2f} rad ({pose[2]*57.3:.1f}°)')

    def get_ped_velocity_map(self) -> np.ndarray:
        """
        Build a 2-channel velocity map on an 80×80 grid.

        Grid coordinate system (matches original cnn_data_pub.py):
          - X axis → rows (r):  [0, 20] m forward from robot.
                r = int(floor(x / 0.25)),  clamped from 80 → 79.
          - Y axis → cols (c):  [-10, +10] m left(+)/right(-), INVERTED index.
                c = int(floor(-(y - 10) / 0.25)),  clamped from 80 → 79.
          - Robot is at (x=0, y=0) → r=0, c=40 (bottom-centre of grid).

        Test cases (robot at origin, facing +X):
          Obstacle at ( 5,  0)  → r=20, c=40  (centre-front)
          Obstacle at ( 0,  5)  → r= 0, c=20  (left, at robot)
          Obstacle at (20,  0)  → r=79, c=40  (far front, clamped 80→79)
          Obstacle at (10,-10)  → r=40, c=79  (right edge, clamped 80→79)
          Obstacle at ( 0, 10)  → r= 0, c= 0  (far left edge)

        Returns:
            Flat np.float32 array of shape (12800,).
            Layout: [vx_channel (6400), vy_channel (6400)].
        """
        vx_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)
        vy_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)

        # DEBUG: Call counter for periodic verbose logging (first 5, then every 20th)
        if not hasattr(self, '_map_call_count'):
            self._map_call_count = 0
        self._map_call_count += 1
        _dbg_verbose = (self._map_call_count <= 5 or self._map_call_count % 20 == 0)

        # ---- snapshot under lock ----------------------------------------
        with self._lock:
            obstacles = list(self._obstacles)
            robot_pose = self._robot_pose

        if not obstacles:
            return np.zeros(self.TOTAL_SIZE, dtype=np.float32)

        # DEBUG: Transform verification counters
        _dbg_in_range = 0
        _dbg_out_range = 0

        for _i, obs in enumerate(obstacles):
            try:
                ox = float(obs['x'])
                oy = float(obs['y'])
                ovx = float(obs['vx'])
                ovy = float(obs['vy'])
            except (KeyError, TypeError, ValueError):
                continue  # skip malformed entries

            # -- coordinate conversion (global → robot frame) -------------
            if robot_pose is not None:
                rx, ry, rtheta = robot_pose
                dx = ox - rx
                dy = oy - ry
                cos_t = np.cos(-rtheta)
                sin_t = np.sin(-rtheta)

                # Position in robot frame
                rel_x = dx * cos_t - dy * sin_t
                rel_y = dx * sin_t + dy * cos_t

                # Velocity in robot frame
                vel_x = ovx * cos_t - ovy * sin_t
                vel_y = ovx * sin_t + ovy * cos_t
            else:
                # No robot pose available — assume data already in robot frame
                rel_x, rel_y = ox, oy
                vel_x, vel_y = ovx, ovy

            # DEBUG: Log first obstacle's full transform for diagnostics
            if _i == 0 and _dbg_verbose:
                _in = (self.X_MIN <= rel_x <= self.X_MAX
                       and self.Y_MIN <= rel_y <= self.Y_MAX)
                self.get_logger().info(
                    f'[TRANSFORM] Obs0: global=({ox:.2f},{oy:.2f}), '
                    f'robot_rel=({rel_x:.2f},{rel_y:.2f}), '
                    f'vel_rel=({vel_x:.2f},{vel_y:.2f}), '
                    f'in_range={_in}, pose_set={robot_pose is not None}')

            # -- range check (forward-only grid, matching cnn_data_pub.py) -
            if not (self.X_MIN <= rel_x <= self.X_MAX
                    and self.Y_MIN <= rel_y <= self.Y_MAX):
                _dbg_out_range += 1  # DEBUG
                continue
            _dbg_in_range += 1  # DEBUG

            # -- grid indices (matching cnn_data_pub.py lines 69-72) -------
            r = int(np.floor(rel_x / self.BIN_SIZE))              # row from x
            c = int(np.floor(-(rel_y - self.Y_MAX) / self.BIN_SIZE))  # col from y (inverted)

            # Boundary clamping (matches original: if r==80: r=79)
            if r >= self.GRID_SIZE:
                r = self.GRID_SIZE - 1
            if c >= self.GRID_SIZE:
                c = self.GRID_SIZE - 1

            # Assignment, not accumulation (matches original behaviour)
            vx_grid[r, c] = vel_x
            vy_grid[r, c] = vel_y

        # DEBUG: Summary of obstacle mapping
        if _dbg_verbose and len(obstacles) > 0:
            self.get_logger().info(
                f'[GRID] Mapped {_dbg_in_range}/{len(obstacles)} obstacles to grid, '
                f'{_dbg_out_range} rejected (outside '
                f'[{self.X_MIN},{self.X_MAX}]x[{self.Y_MIN},{self.Y_MAX}])')

        # Flatten: first vx channel, then vy channel  (C-order of shape (2,80,80))
        ped_map = np.concatenate([vx_grid.flatten(), vy_grid.flatten()])

        # ADDED: Log grid statistics when obstacles are present
        non_zero_count = np.count_nonzero(ped_map)
        if non_zero_count > 0:
            self.get_logger().info(
                f'[GRID] ✓ {non_zero_count}/{self.TOTAL_SIZE} non-zero cells, '
                f'max_vel={np.max(np.abs(ped_map)):.3f} m/s')
        else:
            # DEBUG: Critical — grid is all zeros despite having obstacles
            if _dbg_verbose:
                self.get_logger().warn(
                    f'[GRID] ✗ Grid is ALL ZEROS! '
                    f'{len(obstacles)} obstacles received, '
                    f'robot_pose={robot_pose is not None}')

        return ped_map
