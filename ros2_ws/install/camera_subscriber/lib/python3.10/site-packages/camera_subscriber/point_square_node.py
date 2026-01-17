#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.window_name = "camera"
        self.subscription = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        self.subscription # prevent unused variable warning
        self.point = None

    def listener_callback(self, image_data):
        cv_image = np.zeros((512,700,3), np.uint8)
        if(self.point is not None):
            cv2.rectangle(cv_image,self.point,(self.point[0]+200,self.point[1]+200),(0,55,0),3)
        cv2.imshow(self.window_name,cv_image)
        cv2.waitKey(25)
        cv2.setMouseCallback(self.window_name, self.draw_rectangle)

    def draw_rectangle(self, event, x, y, flags, param):
        # if event == cv2.EVENT_LBUTTONDOWN: # check if mouse event is click
            self.point = (x,y)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
