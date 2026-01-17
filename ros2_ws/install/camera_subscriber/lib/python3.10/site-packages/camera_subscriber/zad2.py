#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Bridge do konwersji ROS Image -> OpenCV
        self.bridge = CvBridge()

        # Subskrypcja obrazu
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10
        )

        self.get_logger().info('Camera subscriber node started')

    def listener_callback(self, image_msg):
        # Konwersja obrazu ROS na OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # Wyświetlenie obrazu
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)

    # Sprzątanie
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

