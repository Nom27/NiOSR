#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
	def __init__(self):
		super().__init__('camera_subscriber_oop')
		self.bridge = CvBridge()

		# Subscriber to /image_raw topic	
		self.subscription = self.create_subscription(
			Image,
			'image_raw',
			self.listener_callback,
			10
		)
		self.subscription # avoid unused variable warning
		self.get_logger().info('CameraSubscriber OOP node started')

	def listener_callback(self, msg: Image):
		# Convert ROS Image -> OpenCV image
		cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		cv2.imshow('camera', cv_image)
		cv2.waitKey(1)


def main(args=None):
	rclpy.init(args=args)
	node = CameraSubscriber()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()



if __name__ == '__main__':
	main()
