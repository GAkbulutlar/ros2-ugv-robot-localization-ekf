#!/usr/bin/env python3
"""
Relay: converts geometry_msgs/Twist  →  geometry_msgs/TwistStamped
Subscribes:  /cmd_vel               (from teleop_twist_keyboard)
Publishes:   /diff_drive_controller/cmd_vel  (TwistStamped, needed by ros2_control)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
        ])

        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_twist, 10)

        self.get_logger().info(
            'twist_relay ready: /cmd_vel (Twist) → '
            '/diff_drive_controller/cmd_vel (TwistStamped)'
        )

    def on_twist(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_footprint'
        out.twist           = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
