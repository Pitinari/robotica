#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FeatureViewer(Node):
    def __init__(self):
        super().__init__('feature_viewer')
        
        self.bridge = CvBridge()
        
        # Create subscribers
        self.rect_sub = self.create_subscription(
            Image, '/stereo/left/rect', self.rect_callback, 10)
        self.features_sub = self.create_subscription(
            Image, '/features/image', self.features_callback, 10)
        
        self.rect_img = None
        self.features_img = None
        
        self.get_logger().info('Feature viewer started')
    
    def rect_callback(self, msg):
        try:
            self.rect_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing rectified image: {e}')
    
    def features_callback(self, msg):
        try:
            self.features_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing features image: {e}')
    
    def display_images(self):
        if self.features_img is not None:
            # The features image now contains both left and right images with features
            # Just display it directly
            cv2.imshow('Stereo Vision - ORB Feature Extraction', self.features_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    node = FeatureViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
