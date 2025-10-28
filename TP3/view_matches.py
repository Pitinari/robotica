#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MatchesViewer(Node):
    def __init__(self):
        super().__init__('matches_viewer')
        
        self.bridge = CvBridge()
        
        # Create subscribers
        self.all_matches_sub = self.create_subscription(
            Image, '/matches/all', self.all_matches_callback, 10)
        self.filtered_matches_sub = self.create_subscription(
            Image, '/matches/filtered', self.filtered_matches_callback, 10)
        
        self.all_matches_img = None
        self.filtered_matches_img = None
        
        self.get_logger().info('Matches viewer started')
    
    def all_matches_callback(self, msg):
        try:
            self.all_matches_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing all matches image: {e}')
    
    def filtered_matches_callback(self, msg):
        try:
            self.filtered_matches_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f'Error processing filtered matches image: {e}')
    
    def display_images(self):
        if self.all_matches_img is not None and self.filtered_matches_img is not None:
            # Create a combined view
            if self.all_matches_img.shape[0] != self.filtered_matches_img.shape[0]:
                # Resize to match heights
                h = min(self.all_matches_img.shape[0], self.filtered_matches_img.shape[0])
                all_resized = cv2.resize(self.all_matches_img, (int(self.all_matches_img.shape[1] * h / self.all_matches_img.shape[0]), h))
                filtered_resized = cv2.resize(self.filtered_matches_img, (int(self.filtered_matches_img.shape[1] * h / self.filtered_matches_img.shape[0]), h))
            else:
                all_resized = self.all_matches_img.copy()
                filtered_resized = self.filtered_matches_img.copy()
            
            # Combine images vertically
            combined = np.vstack([all_resized, filtered_resized])
            
            # Add separator line
            cv2.line(combined, (0, all_resized.shape[0]), (combined.shape[1], all_resized.shape[0]), (255, 255, 255), 2)
            
            # Display
            cv2.imshow('Feature Matching - All vs Filtered Matches', combined)
            cv2.waitKey(1)
        elif self.all_matches_img is not None:
            # Show only all matches
            cv2.imshow('Feature Matching - All Matches', self.all_matches_img)
            cv2.waitKey(1)
        elif self.filtered_matches_img is not None:
            # Show only filtered matches
            cv2.imshow('Feature Matching - Filtered Matches', self.filtered_matches_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    node = MatchesViewer()
    
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
