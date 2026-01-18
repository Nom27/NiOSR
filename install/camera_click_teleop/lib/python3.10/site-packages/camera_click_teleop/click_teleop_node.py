#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ClickTeleopNode(Node):
    def __init__(self):
        super().__init__('click_teleop_node')

        self.get_logger().info("ClickTeleopNode running with camera subscriber.")

        # ROS <-> OpenCV converter
        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',          # CHANGE THIS if your topic name is different
            self.image_callback,
            10
        )

        # Create OpenCV window + mouse callback
        cv2.namedWindow("Camera View")
        cv2.setMouseCallback("Camera View", self.on_mouse_click)

        self.latest_frame = None

    # ---- Mouse Callback ----
    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.latest_frame is None:
                return

            height = self.latest_frame.shape[0]
            center_y = height // 2

            if y < center_y:
                self.get_logger().info(f"Clicked UP (y={y}) → ABOVE center")
            else:
                self.get_logger().info(f"Clicked DOWN (y={y}) → BELOW center")

    # ---- Image Callback ----
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame

            cv2.imshow("Camera View", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = ClickTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
