#!/usr/bin/env python3
"""
fake_perception_node.py — Replicates CoppeliaSim cube motion and produces
a pedestrian velocity map (12 800 floats) for the DRL-VO controller.

FORMAT CONTRACT (must match upstream TempleRAIL/drl_vo_nav training pipeline):
  • Array shape before flatten:  (2, 80, 80)   — channel-first
        channel 0 = vx  (velocity in robot-local x / forward)
        channel 1 = vy  (velocity in robot-local y / left)
  • Grid convention:
        row  index r  ←  local_x (forward),  0 m … 20 m,  cell = 0.25 m
        col  index c  ←  local_y (lateral), +10 m … −10 m, cell = 0.25 m
        c = floor((10 − local_y) / 0.25)
  • Flatten order: C-order  →  first 6400 = all vx, next 6400 = all vy

BUG FIXES vs previous version (2026-02-08):
  1. Channel layout:  was (80,80,2) interleaved  →  now (2,80,80) channel-first
  2. Grid params:     was 8 m×8 m centered, 0.10 m cell
                      →  now 20 m×20 m forward-looking, 0.25 m cell
  3. Direction field: was ignored (X-only motion)
                      →  now uses per-cube 'direction' angle for 2-D motion
  4. cube_count:      was hardcoded 10  →  now read from JSON
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import std_msgs.msg
import numpy as np
import math
import json
import os
from tf_transformations import euler_from_quaternion


class FakePerceptionNode(Node):
    def __init__(self):
        super().__init__('fake_perception_node')

        # ── CoppeliaSim motion parameters (must match the Lua scene script) ──
        self.move_amplitude = 0.5   # metres
        self.move_speed     = 0.5   # Hz

        # ── Grid parameters (must match training: cnn_data_pub.py) ──────────
        self.grid_size  = 80
        self.cell_size  = 0.25          # 20 m / 80 = 0.25 m per cell
        self.x_max      = 20.0          # forward range [0, 20] m
        self.y_half     = 10.0          # lateral range [−10, +10] m

        # ── Runtime state ────────────────────────────────────────────────────
        self.cubes_init = None
        self.robot_pose = None
        self.sim_time        = 0.0
        self._have_time      = False   # True once ANY time source fires
        self._use_clock      = False   # True once /clock delivers data
        self._odom_time_base = None    # first odom header stamp (for elapsed-time fallback)

        self.load_cube_poses()

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',  self.odom_callback,  10)
        self.create_subscription(Clock,    '/clock', self.clock_callback, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.ped_map_pub = self.create_publisher(
            Float32MultiArray, '/ped_vel_map', 10)
        self.debug_cloud_pub = self.create_publisher(
            PointCloud2, '/debug_cube_positions', 10)
        self.debug_marker_pub = self.create_publisher(
            MarkerArray, '/debug_cube_velocities', 10)

        # ── Timer: 50 Hz ─────────────────────────────────────────────────────
        self.timer = self.create_timer(0.02, self.publish_velocity_map)

        self.get_logger().info(
            'FakePerceptionNode started — grid 20×20 m, cell 0.25 m, '
            'channel-first (2,80,80)')

    # ─────────────────────────── helpers ──────────────────────────────────────

    def load_cube_poses(self):
        """Load initial cube poses written by the CoppeliaSim Lua script."""
        json_path = '/tmp/cube_init_poses.json'
        import time
        for _ in range(50):                       # wait up to 5 s
            if os.path.exists(json_path):
                break
            time.sleep(0.1)

        if not os.path.exists(json_path):
            self.get_logger().error(f'JSON not found: {json_path}')
            return

        with open(json_path, 'r') as f:
            data = json.load(f)

        self.cubes_init = data['cubes']
        self.get_logger().info(
            f'Loaded {len(self.cubes_init)} cubes from {json_path}')

    def clock_callback(self, msg):
        """Store simulation time published by CoppeliaSim on /clock."""
        t = msg.clock.sec + msg.clock.nanosec / 1e9
        self.sim_time   = t
        self._have_time = True
        self._use_clock = True

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot_pose = (pos.x, pos.y, yaw)

        # ── Fallback time source: elapsed time from odom header ──────────
        # CoppeliaSim's Lua bridge sets  header.stamp = simROS2.getTime()
        # which may be wall-clock.  We record the first stamp and compute
        # the delta, so elapsed ≈ sim.getSimulationTime() at 1× speed.
        # This is ONLY used when /clock has no publisher.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp > 0.0:
            if self._odom_time_base is None:
                self._odom_time_base = stamp
                self.get_logger().info(
                    f'Odom time base set: {stamp:.3f} s  '
                    f'(/clock available: {self._use_clock})')
            if not self._use_clock:
                self.sim_time   = stamp - self._odom_time_base
                self._have_time = True

    # ─────────────────────── main publish loop ────────────────────────────────

    def publish_velocity_map(self):
        # --- guard clauses ---------------------------------------------------
        if self.cubes_init is None:
            self.get_logger().warn('Cubes not loaded yet',
                                   throttle_duration_sec=5.0)
            return
        if self.robot_pose is None:
            self.get_logger().warn('Waiting for /odom …',
                                   throttle_duration_sec=5.0)
            return
        if not self._have_time:
            self.get_logger().warn(
                'No time source yet — waiting for /clock or /odom …',
                throttle_duration_sec=5.0)
            return

        t = self.sim_time
        robot_x, robot_y, robot_theta = self.robot_pose
        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)

        # FIX #1: channel-first grid matching training pipeline
        #   shape = (2, 80, 80)  →  channel 0 = vx, channel 1 = vy
        velocity_grid = np.zeros((2, self.grid_size, self.grid_size),
                                 dtype=np.float32)

        # Debug data
        cube_world_pts  = []   # (x, y, z) in odom frame
        cube_local_info = []   # (local_x, local_y, local_vx, local_vy)
        n_in_grid = 0

        for cube in self.cubes_init:
            # ── FIX #3: use per-cube 'direction' for 2-D sinusoidal motion ──
            direction = cube['direction']
            phase_arg = 2.0 * math.pi * self.move_speed * t + cube['phase']
            offset    = self.move_amplitude * math.sin(phase_arg)

            cube_x = cube['x'] + offset * math.cos(direction)
            cube_y = cube['y'] + offset * math.sin(direction)

            # Analytical velocity (d/dt of offset) decomposed along direction
            d_offset = (self.move_amplitude * 2.0 * math.pi
                        * self.move_speed * math.cos(phase_arg))
            cube_vx_global = d_offset * math.cos(direction)
            cube_vy_global = d_offset * math.sin(direction)

            cube_world_pts.append([cube_x, cube_y, 0.25])

            # ── Transform position: world → robot-local ─────────────────────
            #   Uses R(−θ) · (p_world − p_robot), equivalent to the
            #   inverse homogeneous transform used in track_ped_pub.py
            dx = cube_x - robot_x
            dy = cube_y - robot_y
            local_x =  dx * cos_t + dy * sin_t
            local_y = -dx * sin_t + dy * cos_t

            # ── Transform velocity: world → robot-local ─────────────────────
            local_vx =  cube_vx_global * cos_t + cube_vy_global * sin_t
            local_vy = -cube_vx_global * sin_t + cube_vy_global * cos_t

            cube_local_info.append((local_x, local_y, local_vx, local_vy))

            # ── FIX #2: grid mapping matching training (cnn_data_pub.py) ─────
            #   row  r  ← forward distance x ∈ [0, 20 m]
            #   col  c  ← lateral  y flipped: c = floor((10 − y) / 0.25)
            if local_x < 0.0 or local_x > self.x_max:
                continue
            if abs(local_y) > self.y_half:
                continue

            r = int(math.floor(local_x / self.cell_size))
            c = int(math.floor((self.y_half - local_y) / self.cell_size))

            # Clamp boundary (matches training: if r==80: r=79)
            if r >= self.grid_size:
                r = self.grid_size - 1
            if c >= self.grid_size:
                c = self.grid_size - 1

            # Write velocity — last-write-wins (same as training)
            velocity_grid[0, r, c] = local_vx
            velocity_grid[1, r, c] = local_vy
            n_in_grid += 1

        # ── Flatten: C-order on (2, 80, 80) → first 6400 = vx, next = vy ────
        ped_map = velocity_grid.flatten()            # 12 800 floats

        msg = Float32MultiArray()
        msg.data = ped_map.tolist()
        self.ped_map_pub.publish(msg)

        # ── Debug visualisations ─────────────────────────────────────────────
        self._publish_debug_cloud(cube_world_pts)
        self._publish_debug_markers(cube_local_info)

        self.get_logger().info(
            f'ped_vel_map: {n_in_grid}/{len(self.cubes_init)} cubes in grid, '
            f'sim_t={t:.2f}s, '
            f'max|vx|={np.max(np.abs(velocity_grid[0])):.3f} '
            f'max|vy|={np.max(np.abs(velocity_grid[1])):.3f}',
            throttle_duration_sec=2.0)

    # ─────────────────── debug publishers ─────────────────────────────────────

    def _publish_debug_cloud(self, world_pts):
        """Publish cube world positions as PointCloud2 for RViz overlay."""
        if not world_pts:
            return
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'
        fields = [
            point_cloud2.PointField(
                name='x', offset=0,
                datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(
                name='y', offset=4,
                datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(
                name='z', offset=8,
                datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]
        self.debug_cloud_pub.publish(
            point_cloud2.create_cloud(header, fields, world_pts))

    def _publish_debug_markers(self, local_info):
        """Publish velocity arrows in robot frame for visual verification."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (lx, ly, lvx, lvy) in enumerate(local_info):
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'base_footprint'
            m.ns     = 'cube_vel'
            m.id     = i
            m.type   = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = 0.05          # shaft diameter
            m.scale.y = 0.10          # head diameter
            m.scale.z = 0.0
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.9
            m.lifetime.sec = 0
            m.lifetime.nanosec = 100_000_000   # 100 ms
            # start = cube position, end = position + velocity vector
            from geometry_msgs.msg import Point as GPoint
            p_start = GPoint(x=lx, y=ly, z=0.25)
            p_end   = GPoint(x=lx + lvx, y=ly + lvy, z=0.25)
            m.points = [p_start, p_end]
            ma.markers.append(m)
        # Delete old markers that no longer exist
        for j in range(len(local_info), len(local_info) + 5):
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'base_footprint'
            m.ns     = 'cube_vel'
            m.id     = j
            m.action = Marker.DELETE
            ma.markers.append(m)
        self.debug_marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FakePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
