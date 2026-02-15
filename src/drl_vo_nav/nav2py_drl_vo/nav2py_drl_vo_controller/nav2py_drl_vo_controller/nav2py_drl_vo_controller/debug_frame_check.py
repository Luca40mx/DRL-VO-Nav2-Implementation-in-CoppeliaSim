#!/usr/bin/env python3
"""
DEBUG: Coordinate frame verification script.

Subscribes to /dynamic_obstacles and /odom to verify that obstacle positions
are in the correct frame and within the DRL-VO grid range after transform.

Usage:
    python3 debug_frame_check.py

This script is temporary and should be removed after debugging.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json
import numpy as np


class FrameDebugger(Node):
    def __init__(self):
        super().__init__('frame_debugger')

        self.obstacle_sub = self.create_subscription(
            String, '/dynamic_obstacles', self.obstacle_callback, 10
        )

        # CoppeliaSim typically publishes /odom (nav_msgs/Odometry)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.robot_pose = None
        self.obstacles = []
        self._call_count = 0

        self.get_logger().info('Frame debugger started')
        self.get_logger().info('Listening to /dynamic_obstacles and /odom')
        self.get_logger().info(
            '(If robot pose never appears, check: '
            'ros2 topic list | grep -E "odom|pose")')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = np.arctan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))

        self.robot_pose = (x, y, theta)
        self._call_count += 1
        if self._call_count <= 3 or self._call_count % 50 == 0:
            self.get_logger().info(
                f'[ROBOT] x={x:.2f}, y={y:.2f}, '
                f'theta={theta:.2f} rad ({theta*57.3:.1f}°)')

    def obstacle_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacles = data.get('obstacles', [])

            if not self.obstacles:
                return

            self.get_logger().info(
                f'\n--- {len(self.obstacles)} obstacles received ---')

            for i, obs in enumerate(self.obstacles[:5]):
                obs_x = obs['x']
                obs_y = obs['y']
                obs_vx = obs.get('vx', 0)
                obs_vy = obs.get('vy', 0)

                if self.robot_pose:
                    rx, ry, rtheta = self.robot_pose
                    dx = obs_x - rx
                    dy = obs_y - ry
                    cos_t = np.cos(-rtheta)
                    sin_t = np.sin(-rtheta)

                    rel_x = dx * cos_t - dy * sin_t
                    rel_y = dx * sin_t + dy * cos_t
                    vel_x = obs_vx * cos_t - obs_vy * sin_t
                    vel_y = obs_vx * sin_t + obs_vy * cos_t

                    in_x = 0 <= rel_x <= 20
                    in_y = -10 <= rel_y <= 10
                    in_range = in_x and in_y

                    # Grid indices (if in range)
                    if in_range:
                        r = int(np.floor(rel_x / 0.25))
                        c = int(np.floor(-(rel_y - 10.0) / 0.25))
                        if r >= 80:
                            r = 79
                        if c >= 80:
                            c = 79
                        grid_str = f'grid=({r},{c})'
                    else:
                        grid_str = 'OUT OF GRID'
                        if not in_x:
                            grid_str += f' [x={rel_x:.1f} outside [0,20]]'
                        if not in_y:
                            grid_str += f' [y={rel_y:.1f} outside [-10,10]]'

                    self.get_logger().info(
                        f'  Obs {obs.get("id", i)}: '
                        f'global=({obs_x:.2f},{obs_y:.2f}), '
                        f'rel=({rel_x:.2f},{rel_y:.2f}), '
                        f'vel_rel=({vel_x:.2f},{vel_y:.2f}), '
                        f'{grid_str}')
                else:
                    self.get_logger().info(
                        f'  Obs {obs.get("id", i)}: '
                        f'global=({obs_x:.2f},{obs_y:.2f}) '
                        f'[NO ROBOT POSE YET]')

            if not self.robot_pose:
                self.get_logger().warn(
                    'Robot pose not available! Is /odom being published? '
                    'Try: ros2 topic echo /odom --once')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main():
    rclpy.init()
    node = FrameDebugger()

    print('\n' + '=' * 60)
    print('  Frame Debugger - Verify obstacle coordinates')
    print('=' * 60)
    print('This script will show:')
    print('  1. Robot pose in global frame (from /odom)')
    print('  2. Obstacle positions in global frame')
    print('  3. Obstacle positions relative to robot')
    print('  4. Whether obstacles land in grid [0,20]x[-10,+10]')
    print('  5. Grid cell indices (r, c) for in-range obstacles')
    print('\nPress Ctrl+C to stop\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
