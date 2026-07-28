#!/usr/bin/env python3
"""
Circle path controller for a mobile robot in ROS 2.

Drives the robot in one full circle, then stops.
  radius        (ROS 2 param, default 0.5 m)
  linear_speed  (ROS 2 param, default 0.2 m/s)

angular_velocity = linear_speed / radius
duration         = 2 * pi * radius / linear_speed
"""
#ros2 run my_robot_bringup twist_relay.py
#source ~/ros2-drone-gnss-ins-sim/install/setup.bash
#ros2 run teleop_twist_keyboard teleop_twist_keyboard

import math

import rclpy
import rclpy.parameter
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped


class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

        self.declare_parameter('radius', 0.5)
        self.declare_parameter('linear_speed', 0.2)

        radius       = self.get_parameter('radius').get_parameter_value().double_value
        linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        self.linear_speed  = linear_speed
        self.angular_speed = linear_speed / radius                  # rad/s
        self.duration      = 2.0 * math.pi * radius / linear_speed  # seconds

        self.start_time = None
        self.done       = False

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        self.timer = self.create_timer(0.02, self.control_loop)   # 50 Hz

        self.get_logger().info(
            f"CircleMover: radius={radius} m, "
            f"linear={linear_speed} m/s, "
            f"angular={self.angular_speed:.3f} rad/s, "
            f"expected duration={self.duration:.2f} s"
        )

    def publish_cmd(self, vx: float, wz: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x  = vx
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

    def control_loop(self):
        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now
            self.get_logger().info("Starting circle motion.")

        if self.done:
            self.publish_cmd(0.0, 0.0)
            return

        elapsed = (now - self.start_time).nanoseconds * 1e-9  # seconds

        if elapsed >= self.duration:
            self.publish_cmd(0.0, 0.0)
            self.done = True
            self.get_logger().info("Circle completed. Stopped.")
            return

        self.publish_cmd(self.linear_speed, self.angular_speed)


def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
