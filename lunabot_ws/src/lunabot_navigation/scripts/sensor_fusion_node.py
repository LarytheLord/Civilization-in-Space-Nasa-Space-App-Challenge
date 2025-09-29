#!/usr/bin/env python3
"""
LunaBot Sensor Fusion Node
Combines LiDAR, camera, and IMU data for enhanced perception and navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R

# ROS2 message types
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker

class SensorFusionNode(Node):
    """
    Advanced sensor fusion for lunar navigation
    Combines multiple sensor modalities for robust perception
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, sensor_qos)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, sensor_qos)
        
        # Publishers
        self.fused_obstacles_pub = self.create_publisher(
            MarkerArray, '/fused_obstacles', 10)
        self.enhanced_scan_pub = self.create_publisher(
            LaserScan, '/enhanced_scan', 10)
        self.terrain_analysis_pub = self.create_publisher(
            Float32MultiArray, '/terrain_analysis', 10)
        self.pose_correction_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/pose_correction', 10)
        
        # Data storage
        self.latest_scan = None
        self.latest_image = None
        self.latest_imu = None
        
        # Fusion parameters
        self.obstacle_confidence_threshold = 0.7
        self.terrain_roughness_threshold = 0.3
        self.imu_drift_correction_factor = 0.1
        
        # Kalman filter for pose estimation
        self.pose_estimate = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.pose_covariance = np.eye(6) * 0.1
        
        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)
        
        self.get_logger().info("🌙 LunaBot Sensor Fusion Node initialized")
    
    def laser_callback(self, msg):
        """Process LiDAR scan data"""
        self.latest_scan = msg
        
        # Enhance scan with obstacle classification
        enhanced_scan = self.enhance_laser_scan(msg)
        if enhanced_scan:
            self.enhanced_scan_pub.publish(enhanced_scan)
    
    def camera_callback(self, msg):
        """Process camera image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            
            # Perform visual analysis
            self.analyze_visual_features(cv_image)
            
        except Exception as e:
            self.get_logger().error(f"Camera processing error: {e}")
    
    def imu_callback(self, msg):
        """Process IMU data"""
        self.latest_imu = msg
        
        # Extract orientation and angular velocity
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        
        # Update pose estimate with IMU data
        self.update_pose_with_imu(orientation, angular_velocity, linear_acceleration)
    
    def enhance_laser_scan(self, scan_msg):
        """Enhance laser scan with additional processing"""
        if not scan_msg.ranges:
            return None
        
        enhanced_scan = LaserScan()
        enhanced_scan.header = scan_msg.header
        enhanced_scan.angle_min = scan_msg.angle_min
        enhanced_scan.angle_max = scan_msg.angle_max
        enhanced_scan.angle_increment = scan_msg.angle_increment
        enhanced_scan.time_increment = scan_msg.time_increment
        enhanced_scan.scan_time = scan_msg.scan_time
        enhanced_scan.range_min = scan_msg.range_min
        enhanced_scan.range_max = scan_msg.range_max
        
        # Filter and enhance ranges
        enhanced_ranges = []
        for i, range_val in enumerate(scan_msg.ranges):
            if np.isnan(range_val) or np.isinf(range_val):
                enhanced_ranges.append(scan_msg.range_max)
            else:
                # Apply noise filtering
                filtered_range = self.filter_range_noise(range_val, i)
                enhanced_ranges.append(filtered_range)
        
        enhanced_scan.ranges = enhanced_ranges
        enhanced_scan.intensities = scan_msg.intensities
        
        return enhanced_scan
    
    def filter_range_noise(self, range_val, index):
        """Apply noise filtering to range measurements"""
        # Simple moving average filter for lunar dust interference
        if hasattr(self, 'range_history'):
            if len(self.range_history) > index:
                history = self.range_history[index]
                history.append(range_val)
                if len(history) > 5:
                    history.pop(0)
                return np.median(history)
        else:
            self.range_history = [[] for _ in range(360)]
        
        return range_val
    
    def analyze_visual_features(self, image):
        """Analyze visual features for obstacle detection and terrain assessment"""
        if image is None:
            return
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Edge detection for obstacles
        edges = cv2.Canny(gray, 50, 150)
        
        # Contour detection
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Analyze terrain texture
        terrain_roughness = self.calculate_terrain_roughness(gray)
        
        # Publish terrain analysis
        terrain_msg = Float32MultiArray()
        terrain_msg.data = [terrain_roughness, len(contours), np.mean(gray)]
        self.terrain_analysis_pub.publish(terrain_msg)
    
    def calculate_terrain_roughness(self, gray_image):
        """Calculate terrain roughness from image texture"""
        # Use Laplacian variance as roughness measure
        laplacian_var = cv2.Laplacian(gray_image, cv2.CV_64F).var()
        normalized_roughness = min(laplacian_var / 1000.0, 1.0)
        return normalized_roughness
    
    def update_pose_with_imu(self, orientation, angular_velocity, linear_acceleration):
        """Update pose estimate using IMU data with Kalman filtering"""
        # Extract quaternion
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert to Euler angles
        r = R.from_quat(quat)
        euler = r.as_euler('xyz', degrees=False)
        
        # Update pose estimate (simplified Kalman filter)
        dt = 0.1  # 10 Hz update rate
        
        # Prediction step
        self.pose_estimate[3:6] = euler  # Update orientation
        
        # Add process noise
        process_noise = np.eye(6) * 0.01
        self.pose_covariance += process_noise
        
        # Measurement update (if we have other sensors)
        if self.latest_scan is not None:
            # Use scan matching for position correction
            position_correction = self.estimate_position_from_scan()
            if position_correction is not None:
                # Update position estimate
                K = self.pose_covariance[:3, :3] @ np.linalg.inv(
                    self.pose_covariance[:3, :3] + np.eye(3) * 0.1)
                self.pose_estimate[:3] += K @ (position_correction - self.pose_estimate[:3])
                self.pose_covariance[:3, :3] = (np.eye(3) - K) @ self.pose_covariance[:3, :3]
    
    def estimate_position_from_scan(self):
        """Estimate position correction from laser scan data"""
        # Simplified scan matching - in practice, use more sophisticated algorithms
        if self.latest_scan is None:
            return None
        
        # This is a placeholder - implement proper scan matching
        return np.array([0.0, 0.0, 0.0])
    
    def fusion_callback(self):
        """Main fusion processing callback"""
        if not all([self.latest_scan, self.latest_image, self.latest_imu]):
            return
        
        # Perform multi-modal obstacle detection
        obstacles = self.detect_fused_obstacles()
        
        # Publish fused obstacle markers
        if obstacles:
            marker_array = self.create_obstacle_markers(obstacles)
            self.fused_obstacles_pub.publish(marker_array)
        
        # Publish pose correction if significant drift detected
        if self.detect_pose_drift():
            pose_correction = self.create_pose_correction_msg()
            self.pose_correction_pub.publish(pose_correction)
    
    def detect_fused_obstacles(self):
        """Detect obstacles using fused sensor data"""
        obstacles = []
        
        if self.latest_scan is None:
            return obstacles
        
        # Process laser scan for obstacles
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            len(ranges)
        )
        
        # Find potential obstacles (close range readings)
        obstacle_indices = np.where(
            (ranges < 2.0) & (ranges > self.latest_scan.range_min)
        )[0]
        
        for idx in obstacle_indices:
            angle = angles[idx]
            distance = ranges[idx]
            
            # Convert to Cartesian coordinates
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            
            # Confidence based on multiple sensor agreement
            confidence = self.calculate_obstacle_confidence(x, y, distance)
            
            if confidence > self.obstacle_confidence_threshold:
                obstacles.append({
                    'x': x,
                    'y': y,
                    'distance': distance,
                    'confidence': confidence
                })
        
        return obstacles
    
    def calculate_obstacle_confidence(self, x, y, distance):
        """Calculate obstacle confidence using multi-sensor data"""
        base_confidence = 0.5
        
        # Increase confidence if visual features support obstacle presence
        if self.latest_image is not None:
            # Simplified visual confirmation
            base_confidence += 0.2
        
        # Adjust based on IMU stability (less confidence if robot is shaking)
        if self.latest_imu is not None:
            accel_magnitude = np.sqrt(
                self.latest_imu.linear_acceleration.x**2 +
                self.latest_imu.linear_acceleration.y**2 +
                self.latest_imu.linear_acceleration.z**2
            )
            if accel_magnitude > 12.0:  # High acceleration indicates instability
                base_confidence -= 0.1
        
        # Distance-based confidence (closer obstacles more reliable)
        distance_factor = max(0.1, 1.0 - distance / 5.0)
        
        return min(1.0, base_confidence * distance_factor)
    
    def create_obstacle_markers(self, obstacles):
        """Create visualization markers for detected obstacles"""
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "fused_obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = obstacle['x']
            marker.pose.position.y = obstacle['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Scale based on confidence
            scale = 0.1 + 0.3 * obstacle['confidence']
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            # Color based on confidence (red = high confidence)
            marker.color.r = obstacle['confidence']
            marker.color.g = 1.0 - obstacle['confidence']
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
        
        return marker_array
    
    def detect_pose_drift(self):
        """Detect if pose estimate has significant drift"""
        # Simplified drift detection
        if self.latest_imu is None:
            return False
        
        # Check for high angular velocity indicating potential drift
        angular_vel_magnitude = np.sqrt(
            self.latest_imu.angular_velocity.x**2 +
            self.latest_imu.angular_velocity.y**2 +
            self.latest_imu.angular_velocity.z**2
        )
        
        return angular_vel_magnitude > 0.5
    
    def create_pose_correction_msg(self):
        """Create pose correction message"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set pose from current estimate
        pose_msg.pose.pose.position.x = self.pose_estimate[0]
        pose_msg.pose.pose.position.y = self.pose_estimate[1]
        pose_msg.pose.pose.position.z = self.pose_estimate[2]
        
        # Convert Euler to quaternion
        r = R.from_euler('xyz', self.pose_estimate[3:6])
        quat = r.as_quat()
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]
        
        # Set covariance
        pose_msg.pose.covariance = self.pose_covariance.flatten().tolist()
        
        return pose_msg

def main(args=None):
    rclpy.init(args=args)
    
    sensor_fusion_node = SensorFusionNode()
    
    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
