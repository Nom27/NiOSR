#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_point_publisher')
        self.publisher_ = self.create_publisher(Point, '/point', 10)
        self.publish_point()

    def publish_point(self):
        point_msg = Point()
        point_msg.x = 1.0
        point_msg.y = 2.0
        point_msg.z = 3.0
        self.publisher_.publish(point_msg)
        self.get_logger().info("Opublikowałem Point na /point")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

