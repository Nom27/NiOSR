#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class MinimalSub(Node):
    def __init__(self):
        super().__init__('minimal_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info("Dostałem wiadomość na topicu image_raw!")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

