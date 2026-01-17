#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist


class PointToCmdVel(Node):
    def __init__(self):
        super().__init__('point_to_cmd_vel')

        # ===== PARAMETRY =====
        self.declare_parameter('image_height', 512)
        self.declare_parameter('forward_speed', 0.2)

        self.image_height = self.get_parameter('image_height').value
        self.forward_speed = self.get_parameter('forward_speed').value

        self.center_y = self.image_height / 2

        # Subskrypcja punktu
        self.point_sub = self.create_subscription(
            Point,
            '/point',
            self.point_callback,
            10
        )

        # Publisher cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info(
            f'PointToCmdVel started | center_y={self.center_y}'
        )

    def point_callback(self, msg: Point):
        twist = Twist()

        if msg.y < self.center_y:
            # Punkt powyżej środka → jedź
            twist.linear.x = self.forward_speed
            self.get_logger().info(
                f'Point ABOVE center ({msg.y:.1f}) → MOVE'
            )
        else:
            # Punkt poniżej środka → stop
            twist.linear.x = 0.0
            self.get_logger().info(
                f'Point BELOW center ({msg.y:.1f}) → STOP'
            )

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PointToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

