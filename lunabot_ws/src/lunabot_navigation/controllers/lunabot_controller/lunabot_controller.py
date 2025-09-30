#!/usr/bin/env python3
"""
LunaBot Webots Controller
Interfaces between Webots simulation and ROS2 for LunaBot robot
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from controller import Robot, Motor, Lidar, Camera, InertialUnit, GPS, Compass

# ROS2 message types
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import LaserScan, Image, Imu, NavSatFix
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LunaBotController(Node):
    """
    Webots controller for LunaBot robot
    Handles sensor data publishing and motor control
    """
    
    def __init__(self):
        super().__init__('lunabot_webots_controller')
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize motors
        self.left_front_motor = self.robot.getDevice('left_front_wheel_motor')
        self.right_front_motor = self.robot.getDevice('right_front_wheel_motor')
        self.left_rear_motor = self.robot.getDevice('left_rear_wheel_motor')
        self.right_rear_motor = self.robot.getDevice('right_rear_wheel_motor')
        
        # Set motor positions to infinity for velocity control
        self.left_front_motor.setPosition(float('inf'))
        self.right_front_motor.setPosition(float('inf'))
        self.left_rear_motor.setPosition(float('inf'))
        self.right_rear_motor.setPosition(float('inf'))
        
        # Initialize sensors
        self.lidar = self.robot.getDevice('lidar')
        self.camera = self.robot.getDevice('camera')
        self.imu = self.robot.getDevice('imu')
        self.gps = self.robot.getDevice('gps')
        self.compass = self.robot.getDevice('compass')
        
        # Enable sensors
        self.lidar.enable(self.timestep)
        self.camera.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)
        
        # Robot parameters
        self.wheel_radius = 0.15  # meters
        self.wheel_separation = 0.7  # meters (distance between left and right wheels)
        
        # Current velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.robot.getTime()
        
        # ROS2 Publishers
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # ROS2 Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for main control loop
        self.timer = self.create_timer(self.timestep / 1000.0, self.control_loop)
        
        self.get_logger().info("🤖 LunaBot Webots Controller initialized")
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        
        # Convert to wheel velocities
        left_velocity = (self.linear_velocity - self.angular_velocity * self.wheel_separation / 2.0) / self.wheel_radius
        right_velocity = (self.linear_velocity + self.angular_velocity * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Set motor velocities
        self.left_front_motor.setVelocity(left_velocity)
        self.left_rear_motor.setVelocity(left_velocity)
        self.right_front_motor.setVelocity(right_velocity)
        self.right_rear_motor.setVelocity(right_velocity)
    
    def control_loop(self):
        """Main control loop"""
        # Step the simulation
        if self.robot.step(self.timestep) == -1:
            return
        
        # Update odometry
        self.update_odometry()
        
        # Publish sensor data
        self.publish_laser_scan()
        self.publish_camera_image()
        self.publish_imu_data()
        self.publish_odometry()
        self.publish_gps_data()
        
        # Publish transforms
        self.publish_transforms()
    
    def update_odometry(self):
        """Update robot odometry"""
        current_time = self.robot.getTime()
        dt = current_time - self.last_time
        
        if dt > 0:
            # Simple odometry calculation
            dx = self.linear_velocity * math.cos(self.theta) * dt
            dy = self.linear_velocity * math.sin(self.theta) * dt
            dtheta = self.angular_velocity * dt
            
            self.x += dx
            self.y += dy
            self.theta += dtheta
            
            # Normalize theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.last_time = current_time
    
    def publish_laser_scan(self):
        """Publish LiDAR scan data"""
        if self.lidar.getNumberOfPoints() == 0:
            return
        
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "lidar_link"
        
        # LiDAR parameters
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = 2 * math.pi / self.lidar.getNumberOfPoints()
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = self.timestep / 1000.0
        scan_msg.range_min = self.lidar.getMinRange()
        scan_msg.range_max = self.lidar.getMaxRange()
        
        # Get range data
        ranges = []
        for i in range(self.lidar.getNumberOfPoints()):
            range_value = self.lidar.getRangeImage()[i]
            if range_value == float('inf'):
                ranges.append(scan_msg.range_max)
            else:
                ranges.append(range_value)
        
        scan_msg.ranges = ranges
        scan_msg.intensities = []  # Not provided by Webots LiDAR
        
        self.laser_pub.publish(scan_msg)
    
    def publish_camera_image(self):
        """Publish camera image data"""
        # Note: This is a simplified implementation
        # Full implementation would convert Webots image to ROS Image message
        image_msg = Image()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_link"
        image_msg.width = self.camera.getWidth()
        image_msg.height = self.camera.getHeight()
        image_msg.encoding = "rgb8"
        image_msg.step = image_msg.width * 3
        
        # Get image data (simplified)
        # In a full implementation, you would convert the Webots image format
        image_data = self.camera.getImage()
        if image_data:
            # Convert image data to ROS format
            # This is a placeholder - actual implementation needed
            image_msg.data = []
        
        self.camera_pub.publish(image_msg)
    
    def publish_imu_data(self):
        """Publish IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Get IMU data
        roll_pitch_yaw = self.imu.getRollPitchYaw()
        
        # Convert to quaternion
        roll = roll_pitch_yaw[0]
        pitch = roll_pitch_yaw[1]
        yaw = roll_pitch_yaw[2]
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        imu_msg.orientation.w = cy * cp * cr + sy * sp * sr
        imu_msg.orientation.x = cy * cp * sr - sy * sp * cr
        imu_msg.orientation.y = sy * cp * sr + cy * sp * cr
        imu_msg.orientation.z = sy * cp * cr - cy * sp * sr
        
        # Angular velocity (simplified)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = self.angular_velocity
        
        # Linear acceleration (simplified)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = -1.62  # Lunar gravity
        
        self.imu_pub.publish(imu_msg)
    
    def publish_odometry(self):
        """Publish odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity
        
        # Covariance (simplified)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        
        self.odom_pub.publish(odom_msg)
    
    def publish_gps_data(self):
        """Publish GPS data"""
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"
        
        # Get GPS values
        gps_values = self.gps.getValues()
        if len(gps_values) >= 3:
            # Convert Webots coordinates to GPS coordinates (simplified)
            gps_msg.latitude = gps_values[0]  # This would need proper conversion
            gps_msg.longitude = gps_values[1]
            gps_msg.altitude = gps_values[2]
            
            gps_msg.status.status = 0  # STATUS_FIX
            gps_msg.status.service = 1  # SERVICE_GPS
        else:
            gps_msg.status.status = -1  # STATUS_NO_FIX
        
        self.gps_pub.publish(gps_msg)
    
    def publish_transforms(self):
        """Publish TF transforms"""
        current_time = self.get_clock().now()
        
        # Odom to base_footprint transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    controller = LunaBotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()