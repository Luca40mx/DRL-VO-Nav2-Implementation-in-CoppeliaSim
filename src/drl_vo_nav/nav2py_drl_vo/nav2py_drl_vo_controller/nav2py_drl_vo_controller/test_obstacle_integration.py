#!/usr/bin/env python3
"""
Test script to verify CoppeliaSim obstacle integration with DRL-VO.

Publishes dummy obstacle data to /dynamic_obstacles and monitors reception.
Run this alongside the DRL-VO controller to verify the integration pipeline.

Usage:
    python3 test_obstacle_integration.py
    # or, after colcon build + source:
    ros2 run nav2py_drl_vo_controller test_obstacle_integration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import numpy as np


class ObstacleTestPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_test_publisher')
        self.publisher = self.create_publisher(String, '/dynamic_obstacles', 10)
        self.get_logger().info('Test publisher initialized on /dynamic_obstacles')

    def publish_test_obstacles(self):
        """Publish a single obstacle moving in a circle for testing."""
        t = time.time()

        # Create an obstacle moving in a circle (3m radius, centred 10m ahead)
        angle = t * 0.5          # 0.5 rad/s rotation
        radius = 3.0
        x_center = 10.0          # 10m in front (within [0, 20] grid range)

        obstacle = {
            "id": 1,
            "x": x_center + radius * np.cos(angle),
            "y": radius * np.sin(angle),
            "z": 0.25,
            "vx": -radius * 0.5 * np.sin(angle),     # tangential velocity
            "vy":  radius * 0.5 * np.cos(angle),
        }

        msg = String()
        msg.data = json.dumps({"obstacles": [obstacle]})
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published: pos=({obstacle["x"]:.2f}, {obstacle["y"]:.2f}), '
            f'vel=({obstacle["vx"]:.2f}, {obstacle["vy"]:.2f})')

    def publish_static_obstacle(self, x: float = 5.0, y: float = 0.0):
        """Publish a stationary obstacle at a fixed position (for debugging)."""
        obstacle = {
            "id": 99,
            "x": x, "y": y, "z": 0.25,
            "vx": 0.0, "vy": 0.0,
        }
        msg = String()
        msg.data = json.dumps({"obstacles": [obstacle]})
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published static obstacle at ({x:.2f}, {y:.2f})')

    def publish_multi_obstacles(self, n: int = 5):
        """Publish N obstacles spread across the grid for coverage testing."""
        t = time.time()
        obstacles = []
        for i in range(n):
            x = 2.0 + i * 3.5          # 2, 5.5, 9, 12.5, 16
            y = -6.0 + i * 3.0         # -6, -3, 0, 3, 6
            vx = 0.3 * np.sin(t + i)
            vy = 0.3 * np.cos(t + i)
            obstacles.append({
                "id": i + 1,
                "x": x, "y": y, "z": 0.25,
                "vx": vx, "vy": vy,
            })

        msg = String()
        msg.data = json.dumps({"obstacles": obstacles})
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published {n} obstacles across grid')


def main():
    rclpy.init()
    publisher = ObstacleTestPublisher()

    print("\n" + "=" * 60)
    print("  DRL-VO Obstacle Integration Test Publisher")
    print("=" * 60)
    print("Publishing test obstacles to /dynamic_obstacles at 10 Hz")
    print("Run the DRL-VO controller in another terminal and check logs")
    print("")
    print("Modes:")
    print("  1 = Single obstacle in circle  (default)")
    print("  2 = Static obstacle at (5, 0)")
    print("  3 = 5 obstacles spread across grid")
    print("")
    print("Press Ctrl+C to stop\n")

    mode = 1

    try:
        while rclpy.ok():
            if mode == 1:
                publisher.publish_test_obstacles()
            elif mode == 2:
                publisher.publish_static_obstacle()
            elif mode == 3:
                publisher.publish_multi_obstacles()

            rclpy.spin_once(publisher, timeout_sec=0.1)
            time.sleep(0.1)   # 10 Hz
    except KeyboardInterrupt:
        print("\nTest publisher stopped")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
