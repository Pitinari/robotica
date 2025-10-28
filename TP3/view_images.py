#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        
        self.bridge = CvBridge()
        
        # Create subscribers
        self.left_sub = self.create_subscription(
            Image, '/cam0/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Image, '/cam1/image_raw', self.right_callback, 10)
        self.left_rect_sub = self.create_subscription(
            Image, '/stereo/left/rect', self.left_rect_callback, 10)
        self.right_rect_sub = self.create_subscription(
            Image, '/stereo/right/rect', self.right_rect_callback, 10)
        
        self.left_img = None
        self.right_img = None
        self.left_rect_img = None
        self.right_rect_img = None
        
        self.get_logger().info('Image viewer started')
    
    def left_callback(self, msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing left image: {e}')
    
    def right_callback(self, msg):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing right image: {e}')
    
    def left_rect_callback(self, msg):
        try:
            self.left_rect_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing left rectified image: {e}')
    
    def right_rect_callback(self, msg):
        try:
            self.right_rect_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing right rectified image: {e}')
    
    def display_images(self):
        # Only display when we have both raw images
        if self.left_img is not None and self.right_img is not None:
            # Create a combined view
            if self.left_img.shape[0] != self.right_img.shape[0]:
                # Resize to match heights
                h = min(self.left_img.shape[0], self.right_img.shape[0])
                left_resized = cv2.resize(self.left_img, (int(self.left_img.shape[1] * h / self.left_img.shape[0]), h))
                right_resized = cv2.resize(self.right_img, (int(self.right_img.shape[1] * h / self.right_img.shape[0]), h))
            else:
                left_resized = self.left_img.copy()
                right_resized = self.right_img.copy()
            
            # Combine images side by side
            combined = np.hstack([left_resized, right_resized])
            
            # Add text labels
            cv2.putText(combined, 'Left (Raw)', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(combined, 'Right (Raw)', (left_resized.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display
            cv2.imshow('Stereo Images (Raw)', combined)
            cv2.waitKey(1)
        
        # Only display when we have both rectified images
        if self.left_rect_img is not None and self.right_rect_img is not None:
            # Create a combined view for rectified images
            if self.left_rect_img.shape[0] != self.right_rect_img.shape[0]:
                # Resize to match heights
                h = min(self.left_rect_img.shape[0], self.right_rect_img.shape[0])
                left_rect_resized = cv2.resize(self.left_rect_img, (int(self.left_rect_img.shape[1] * h / self.left_rect_img.shape[0]), h))
                right_rect_resized = cv2.resize(self.right_rect_img, (int(self.right_rect_img.shape[1] * h / self.right_rect_img.shape[0]), h))
            else:
                left_rect_resized = self.left_rect_img.copy()
                right_rect_resized = self.right_rect_img.copy()
            
            # Combine images side by side
            combined_rect = np.hstack([left_rect_resized, right_rect_resized])
            
            # Add text labels
            cv2.putText(combined_rect, 'Left (Rectified)', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(combined_rect, 'Right (Rectified)', (left_rect_resized.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Display
            cv2.imshow('Stereo Images (Rectified)', combined_rect)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    node = ImageViewer()
    
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
