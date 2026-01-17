#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cv2
import numpy as np


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.window_name = "camera"
        self.bridge = CvBridge()

        # ===== PARAMETR =====
        self.declare_parameter('square_size', 200)
        self.square_size = self.get_parameter('square_size').value

        # Subskrypcja obrazu
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10
        )

        # Publisher Point
        self.point_publisher = self.create_publisher(
            Point,
            '/point',
            10
        )

        self.point = None

        self.get_logger().info(
            f'Camera subscriber started | square_size={self.square_size}'
        )

    def listener_callback(self, image_data):
        # Pusta klatka (jak w przykładzie z instrukcji)
        cv_image = np.zeros((512, 700, 3), np.uint8)

        # Rysowanie kwadratu o zadanej długości boku
        if self.point is not None:
            cv2.rectangle(
                cv_image,
                self.point,
                (self.point[0] + self.square_size,
                 self.point[1] + self.square_size),
                (0, 255, 0),
                3
            )

        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(25)
        cv2.setMouseCallback(self.window_name, self.draw_rectangle)

    def draw_rectangle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)

            # Publikacja Point
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = 0.0

            self.point_publisher.publish(msg)

            self.get_logger().info(
                f'Published Point: x={msg.x}, y={msg.y}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

