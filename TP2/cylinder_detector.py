#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion


class CylinderDetector(Node):
    # Known cylinder radius in the world (meters)
    CYLINDER_RADIUS = 0.5
    
    def __init__(self):
        super().__init__('cylinder_detector')
        
        # Robot's current position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Keep track of last detected cylinder count to reduce logging
        self.last_cylinder_count = 0
        self.scan_count = 0
        self.max_markers_published = 0
        
        # Tracking cylinders over time for stability
        self.tracked_cylinders = []
        self.frame_count = 0
        self.max_frames_missing = 10  # Keep cylinder for 10 frames even if not detected
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.cylinder_centers_pub = self.create_publisher(
            PoseArray,
            '/cylinder_centers',
            10
        )
        
        self.cylinder_markers_pub = self.create_publisher(
            MarkerArray,
            '/cylinder_markers',
            10
        )
        
        self.get_logger().info('Cylinder Detector Node Started')
    
    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angles to get theta
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)
    
    def scan_callback(self, msg):
        """Process laser scan data to detect cylinders."""
        # Convert LaserScan to list of polar coordinates with range filtering
        ranges = []
        angle = msg.angle_min
        
        for i, r in enumerate(msg.ranges):
            # Filter out invalid ranges using lidar's min/max range parameters
            if r > msg.range_max or r < msg.range_min:
                r = None
            
            # Convert to our format
            if r is not None:
                ranges.append({
                    'range': r,
                    'angle': angle  # Keep in robot frame for now
                })
            else:
                ranges.append({
                    'range': None,
                    'angle': angle
                })
            angle += msg.angle_increment
        
        # Detect cylinders
        cylinders = self.__split_by_cylinder(ranges)
        
        # Update tracked cylinders for temporal consistency
        self.frame_count += 1
        cylinders = self.__update_tracked_cylinders(cylinders)
        
        # Publish results (always publish, even if empty, to clear old markers)
        self.publish_cylinder_centers(cylinders, msg.header)
        self.publish_cylinder_markers(cylinders, msg.header)
        
        # Log detected cylinders only when count changes
        self.scan_count += 1
        if len(cylinders) != self.last_cylinder_count:
            self.get_logger().info(f'Detected {len(cylinders)} cylinders')
            self.last_cylinder_count = len(cylinders)
    
    def publish_cylinder_centers(self, cylinders, header):
        """Publish cylinder centers as PoseArray."""
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.header.frame_id = 'odom'
        
        for cylinder in cylinders:
            pose = Pose()
            pose.position.x = cylinder['x']
            pose.position.y = cylinder['y']
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.cylinder_centers_pub.publish(pose_array)
    
    def publish_cylinder_markers(self, cylinders, header):
        """Publish cylinder markers for visualization in RViz."""
        marker_array = MarkerArray()
        
        for i, cylinder in enumerate(cylinders):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = 'odom'
            marker.ns = 'cylinders'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = cylinder['x']
            marker.pose.position.y = cylinder['y']
            marker.pose.position.z = 0.5  # Half the height
            marker.pose.orientation.w = 1.0
            
            # Scale (cylinder dimensions) - use the known cylinder radius
            diameter = self.CYLINDER_RADIUS * 2.0
            marker.scale.x = diameter  # diameter
            marker.scale.y = diameter  # diameter
            marker.scale.z = 1.0  # height
            
            # Color (green with transparency)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            # No lifetime - markers persist until explicitly deleted or updated
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            
            marker_array.markers.append(marker)
            
            # Add text label showing the radius
            text_marker = Marker()
            text_marker.header = header
            text_marker.header.frame_id = 'odom'
            text_marker.ns = 'cylinder_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = cylinder['x']
            text_marker.pose.position.y = cylinder['y']
            text_marker.pose.position.z = 1.2  # Above the cylinder
            text_marker.pose.orientation.w = 1.0
            
            text_marker.text = f"r={self.CYLINDER_RADIUS:.2f}m"
            text_marker.scale.z = 0.2  # Text size
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # No lifetime - markers persist
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 0
            
            marker_array.markers.append(text_marker)
        
        # Delete any old markers that are no longer needed
        # (if we detected fewer cylinders than before)
        for i in range(len(cylinders), self.max_markers_published):
            # Delete old cylinder marker
            delete_marker = Marker()
            delete_marker.header = header
            delete_marker.header.frame_id = 'odom'
            delete_marker.ns = 'cylinders'
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)
            
            # Delete old text marker
            delete_text = Marker()
            delete_text.header = header
            delete_text.header.frame_id = 'odom'
            delete_text.ns = 'cylinder_labels'
            delete_text.id = i
            delete_text.action = Marker.DELETE
            marker_array.markers.append(delete_text)
        
        # Update max markers count
        self.max_markers_published = max(len(cylinders), self.max_markers_published)
        
        self.cylinder_markers_pub.publish(marker_array)
    
    def __update_tracked_cylinders(self, detected_cylinders):
        """
        Update tracked cylinders with new detections for temporal stability.
        Matches new detections with existing tracked cylinders and maintains them
        even if temporarily not detected.
        """
        matching_threshold = 0.4  # 40cm - if cylinder is within this distance, it's the same one
        
        # Mark all currently detected cylinders in the tracking list
        matched_indices = set()
        
        for detected in detected_cylinders:
            best_match_idx = None
            best_distance = float('inf')
            
            # Find closest tracked cylinder
            for i, tracked in enumerate(self.tracked_cylinders):
                dist = math.sqrt((detected['x'] - tracked['x'])**2 + 
                               (detected['y'] - tracked['y'])**2)
                if dist < matching_threshold and dist < best_distance:
                    best_distance = dist
                    best_match_idx = i
            
            if best_match_idx is not None:
                # Update existing tracked cylinder with new detection
                # Use exponential moving average for smooth updates
                alpha = 0.3  # Smoothing factor
                self.tracked_cylinders[best_match_idx]['x'] = (
                    alpha * detected['x'] + 
                    (1 - alpha) * self.tracked_cylinders[best_match_idx]['x']
                )
                self.tracked_cylinders[best_match_idx]['y'] = (
                    alpha * detected['y'] + 
                    (1 - alpha) * self.tracked_cylinders[best_match_idx]['y']
                )
                self.tracked_cylinders[best_match_idx]['last_seen'] = self.frame_count
                matched_indices.add(best_match_idx)
            else:
                # New cylinder - add to tracking
                detected['last_seen'] = self.frame_count
                self.tracked_cylinders.append(detected)
        
        # Remove old cylinders that haven't been seen for too long
        self.tracked_cylinders = [
            cyl for cyl in self.tracked_cylinders 
            if self.frame_count - cyl['last_seen'] <= self.max_frames_missing
        ]
        
        # Return all tracked cylinders (including ones not seen this frame)
        return self.tracked_cylinders
    
    def __polar_to_xy(self, polar):
        """Convert polar coordinates to cartesian in world frame."""
        # Transform from robot frame to world frame
        angle_world = polar['angle'] + self.theta
        x = self.x + polar['range'] * math.cos(angle_world)
        y = self.y + polar['range'] * math.sin(angle_world)
        return {'x': x, 'y': y}
    
    
    def __calculate_arc_span(self, cluster, circle):
        """
        Calculate the angular span of the arc formed by the points around the circle center.
        Returns span in degrees.
        """
        if len(cluster) < 2:
            return 0
        
        # Calculate angles of all points relative to circle center
        angles = []
        for point in cluster:
            dx = point['x'] - circle['x']
            dy = point['y'] - circle['y']
            angle = math.atan2(dy, dx)
            angles.append(angle)
        
        # Sort angles
        angles.sort()
        
        # Calculate span (handle wraparound at -π/π boundary)
        max_gap = 0
        for i in range(len(angles)):
            next_i = (i + 1) % len(angles)
            gap = angles[next_i] - angles[i]
            
            # Handle wraparound
            if gap < 0:
                gap += 2 * math.pi
            
            max_gap = max(max_gap, gap)
        
        # Convert to degrees
        span_rad = 2 * math.pi - max_gap
        span_deg = math.degrees(span_rad)
        
        return span_deg
    
    def __calculate_circle_center_3_points(self, p1, p2, p3, radius):
        """
        Calculate the center of a circle with fixed radius passing through 3 points.
        Uses the circumcenter formula but with known radius.
        
        Args:
            p1, p2, p3: Three points on the circle
            radius: Known radius of the cylinder (0.5m)
            
        Returns: {'x': center_x, 'y': center_y, 'radius': radius} or None
        """
        x1, y1 = p1['x'], p1['y']
        x2, y2 = p2['x'], p2['y']
        x3, y3 = p3['x'], p3['y']
        
        # Calculate the determinant
        D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
        
        # Check if points are collinear (determinant is zero)
        if abs(D) < 1e-10:
            return None
        
        # Calculate center coordinates using circumcenter formula
        ux = ((x1**2 + y1**2) * (y2 - y3) + 
              (x2**2 + y2**2) * (y3 - y1) + 
              (x3**2 + y3**2) * (y1 - y2)) / D
        
        uy = ((x1**2 + y1**2) * (x3 - x2) + 
              (x2**2 + y2**2) * (x1 - x3) + 
              (x3**2 + y3**2) * (x2 - x1)) / D
        
        # Validate that the calculated center gives the expected radius
        calc_radius = math.sqrt((x1 - ux)**2 + (y1 - uy)**2)
        
        # Accept if radius is close to expected (within 20cm tolerance)
        if abs(calc_radius - radius) < 0.2:
            return {'x': ux, 'y': uy, 'radius': radius}
        else:
            return None
    
    def __find_circle_center_simple(self, points, radius):
        """
        Find the center of a circle using only first, middle, and last points.
        Also checks if the cluster span is too long (likely a wall).
        
        Args:
            points: List of points on the circle arc
            radius: Known radius of the cylinder (0.5m)
            
        Returns: {'x': center_x, 'y': center_y, 'radius': radius} or None
        """
        if len(points) < 3:
            return None
        
        # Check if cluster span is too long (likely a wall, not a cylinder)
        first = points[0]
        last = points[-1]
        span_distance = math.sqrt((last['x'] - first['x'])**2 + (last['y'] - first['y'])**2)
        
        # If span is longer than 1.5m, it's probably a wall, not a cylinder
        if span_distance > 1.5:
            return None
        
        # Use only first, middle, and last points
        middle = points[len(points) // 2]
        return self.__calculate_circle_center_3_points(first, middle, last, radius)
    
    
    def __split_by_cylinder(self, ranges):
        """Split scan ranges into individual cylinders and find their centers."""
        # Group consecutive valid readings into clusters
        clusters = []
        current_cluster = []
        
        # Distance threshold to determine if points belong to same cylinder
        # If consecutive points are more than this distance apart, they're different objects
        max_point_distance = 0.2  # 50cm threshold
        
        for i, range_data in enumerate(ranges):
            if range_data['range'] is None:
                # Gap detected - save current cluster if it exists
                if len(current_cluster) > 0:
                    clusters.append(current_cluster)
                    current_cluster = []
            else:
                # Valid reading
                point = self.__polar_to_xy(range_data)
                
                if len(current_cluster) == 0:
                    # Start new cluster
                    current_cluster.append(point)
                else:
                    # Check if this point is close to the previous point
                    last_point = current_cluster[-1]
                    distance = math.sqrt(
                        (point['x'] - last_point['x'])**2 + 
                        (point['y'] - last_point['y'])**2
                    )
                    
                    if distance < max_point_distance:
                        # Same cylinder
                        current_cluster.append(point)
                    else:
                        # Different cylinder - save current and start new
                        clusters.append(current_cluster)
                        current_cluster = [point]
        
        # Don't forget the last cluster
        if len(current_cluster) > 0:
            clusters.append(current_cluster)
        
        # Calculate center of each cluster (cylinder) using fixed-radius optimization
        cylinders = []
        for cluster in clusters:
            # Only consider clusters with enough points (filter noise)
            if len(cluster) >= 3:  # Need at least 3 points
                # Use simple first-middle-last approach with span check
                circle = self.__find_circle_center_simple(cluster, self.CYLINDER_RADIUS)
                
                if circle is not None:
                    cylinders.append({
                        'x': circle['x'], 
                        'y': circle['y'],
                        'radius': self.CYLINDER_RADIUS
                    })
        
        return cylinders


def main(args=None):
    rclpy.init(args=args)
    node = CylinderDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

