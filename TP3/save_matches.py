#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class MatchesSaver(Node):
    def __init__(self):
        super().__init__('matches_saver')
        
        self.bridge = CvBridge()
        self.save_count = 0
        self.max_saves = 5  # Save 5 images for the report
        
        # Create output directory
        self.output_dir = "/tmp/matches_images"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create subscribers
        self.all_matches_sub = self.create_subscription(
            Image, '/matches/all', self.all_matches_callback, 10)
        self.filtered_matches_sub = self.create_subscription(
            Image, '/matches/filtered', self.filtered_matches_callback, 10)
        
        self.all_matches_img = None
        self.filtered_matches_img = None
        
        self.get_logger().info('Matches saver started - will save images to ' + self.output_dir)
    
    def all_matches_callback(self, msg):
        try:
            self.all_matches_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.save_images()
        except Exception as e:
            self.get_logger().error(f'Error processing all matches image: {e}')
    
    def filtered_matches_callback(self, msg):
        try:
            self.filtered_matches_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.save_images()
        except Exception as e:
            self.get_logger().error(f'Error processing filtered matches image: {e}')
    
    def save_images(self):
        if self.all_matches_img is not None and self.filtered_matches_img is not None and self.save_count < self.max_saves:
            timestamp = int(time.time() * 1000)
            
            # Save all matches
            all_path = os.path.join(self.output_dir, f"all_matches_{self.save_count}_{timestamp}.png")
            cv2.imwrite(all_path, self.all_matches_img)
            
            # Save filtered matches
            filtered_path = os.path.join(self.output_dir, f"filtered_matches_{self.save_count}_{timestamp}.png")
            cv2.imwrite(filtered_path, self.filtered_matches_img)
            
            # Create combined image
            combined = cv2.vconcat([self.all_matches_img, self.filtered_matches_img])
            combined_path = os.path.join(self.output_dir, f"combined_matches_{self.save_count}_{timestamp}.png")
            cv2.imwrite(combined_path, combined)
            
            self.save_count += 1
            self.get_logger().info(f'Saved match images {self.save_count}/{self.max_saves}')
            
            if self.save_count >= self.max_saves:
                self.get_logger().info('Finished saving images. Check ' + self.output_dir)
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    node = MatchesSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
