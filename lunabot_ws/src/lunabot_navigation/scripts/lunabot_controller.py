#!/usr/bin/env python3
"""
LunaBot Webots Controller
Interface between Webots simulation and ROS2 system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, Imu, Image, CameraInfo
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from webots_ros2_driver_webots.controller import Node as WebotsNode
from webots_ros2_driver_webots.utils import is_wsl
from webots_ros2_driver_webots import Ros2Interface
from webots_ros2_driver_webots.utils import set_robot_client_id
from webots_ros2_driver_webots import WebotsNode
import tf_transformations
import numpy as np
import math
from datetime import datetime


class LunaBotWebotsController(Node):
    """
    Webots controller for LunaBot that interfaces between Webots simulation and ROS2
    """
    
    def __init__(self):
        super().__init__('lunabot_webots_controller')
        
        # Initialize Webots robot instance
        self.get_logger().info("Initializing LunaBot Webots Controller...")
        
        # Robot components from Webots
        self.robot = None  # Will be set by Webots
        self.timestep = int(16)  # 16ms timestep for Webots
        
        # Initialize motors and sensors
        self.left_front_motor = None
        self.right_front_motor = None
        self.left_rear_motor = None
        self.right_rear_motor = None
        self.lidar = None
        self.camera = None
        self.imu = None
        self.gps = None
        self.compass = None
        
        # Motor properties
        self.max_motor_velocity = 10.0
        self.wheel_radius = 0.15  # From URDF
        self.wheel_separation = 0.66  # From URDF + wheel_ygap
        
        # Odometry tracking
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        
        # ROS2 publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.lidar_pub = self.create_publisher(
            LaserScan, '/scan', 10)
        
        self.camera_pub = self.create_publisher(
            Image, '/camera/image_raw', 10)
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10)
        
        self.imu_pub = self.create_publisher(
            Imu, '/imu', 10)
        
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10)
        
        self.tf_pub = self.create_publisher(
            TFMessage, '/tf', 10)
        
        # Setup timer for main control loop
        self.control_timer = self.create_timer(0.016, self.control_loop)  # 16ms = ~60Hz
        
        # Initialize robot components
        self.initialize_robot_components()
        
        self.get_logger().info("LunaBot Webots Controller initialized successfully")

    def initialize_robot_components(self):
        """
        Initialize all Webots robot components
        """
        self.get_logger().info("Initializing Webots robot components...")
        
        # Initialize motors
        try:
            self.left_front_motor = self.robot.getDevice('left_front_wheel_motor')
            self.right_front_motor = self.robot.getDevice('right_front_wheel_motor')
            self.left_rear_motor = self.robot.getDevice('left_rear_wheel_motor')
            self.right_rear_motor = self.robot.getDevice('right_rear_wheel_motor')
            
            # Set motor positions to infinity for velocity control
            self.left_front_motor.setPosition(float('inf'))
            self.right_front_motor.setPosition(float('inf'))
            self.left_rear_motor.setPosition(float('inf'))
            self.right_rear_motor.setPosition(float('inf'))
            
            # Set initial motor velocities to 0
            self.left_front_motor.setVelocity(0.0)
            self.right_front_motor.setVelocity(0.0)
            self.left_rear_motor.setVelocity(0.0)
            self.right_rear_motor.setVelocity(0.0)
            
            self.get_logger().info("Motors initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motors: {e}")
        
        # Initialize sensors
        try:
            # LiDAR sensor
            self.lidar = self.robot.getDevice('lidar')
            if self.lidar:
                self.lidar.enable(self.timestep)
                self.get_logger().info("LiDAR sensor initialized")
            else:
                self.get_logger().error("LiDAR sensor not found in Webots")
            
            # Camera sensor
            self.camera = self.robot.getDevice('camera')
            if self.camera:
                self.camera.enable(self.timestep)
                self.get_logger().info("Camera sensor initialized")
            else:
                self.get_logger().error("Camera sensor not found in Webots")
            
            # IMU sensor
            self.imu = self.robot.getDevice('imu')
            if self.imu:
                self.imu.enable(self.timestep)
                self.get_logger().info("IMU sensor initialized")
            else:
                self.get_logger().error("IMU sensor not found in Webots")
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensors: {e}")

    def cmd_vel_callback(self, msg):
        """
        Handle velocity commands from ROS2
        """
        # Convert differential drive velocities to wheel velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Calculate wheel velocities for differential drive
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Limit velocities to max
        left_wheel_vel = max(-self.max_motor_velocity, min(self.max_motor_velocity, left_wheel_vel))
        right_wheel_vel = max(-self.max_motor_velocity, min(self.max_motor_velocity, right_wheel_vel))
        
        # Set motor velocities
        if self.left_front_motor and self.left_rear_motor:
            self.left_front_motor.setVelocity(left_wheel_vel)
            self.left_rear_motor.setVelocity(left_wheel_vel)
        
        if self.right_front_motor and self.right_rear_motor:
            self.right_front_motor.setVelocity(right_wheel_vel)
            self.right_rear_motor.setVelocity(right_wheel_vel)

    def control_loop(self):
        """
        Main control loop - called every timestep
        """
        # Step the Webots simulation
        self.robot.step(self.timestep)
        
        # Publish sensor data
        self.publish_lidar_data()
        self.publish_camera_data()
        self.publish_imu_data()
        self.publish_odometry()
        self.publish_tf()

    def publish_lidar_data(self):
        """
        Publish LiDAR scan data
        """
        if self.lidar:
            try:
                # Get range data from LiDAR
                ranges = self.lidar.getRangeImage()
                if ranges:
                    # Create LaserScan message
                    scan_msg = LaserScan()
                    scan_msg.header.stamp = self.get_clock().now().to_msg()
                    scan_msg.header.frame_id = 'lidar_link'
                    
                    # Set LiDAR parameters based on Webots config
                    scan_msg.angle_min = -math.pi
                    scan_msg.angle_max = math.pi
                    scan_msg.angle_increment = (2 * math.pi) / len(ranges)
                    scan_msg.time_increment = 0.0
                    scan_msg.scan_time = 0.1  # 10Hz
                    scan_msg.range_min = 0.1
                    scan_msg.range_max = 30.0
                    
                    # Convert Webots ranges to LaserScan ranges
                    ranges_converted = [float(r) if not math.isinf(r) else 30.0 for r in ranges]
                    scan_msg.ranges = ranges_converted
                    scan_msg.intensities = []  # Empty for simplicity
                    
                    self.lidar_pub.publish(scan_msg)
            except Exception as e:
                self.get_logger().warn(f"Error publishing LiDAR data: {e}")

    def publish_camera_data(self):
        """
        Publish camera image data
        """
        if self.camera:
            try:
                # Get camera image
                image = self.camera.getImage()
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                
                if image:
                    # Create Image message
                    img_msg = Image()
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = 'camera_link'
                    img_msg.width = width
                    img_msg.height = height
                    img_msg.encoding = 'rgb8'
                    img_msg.is_bigendian = 0
                    img_msg.step = width * 3  # 3 bytes per pixel (RGB)
                    
                    # Convert Webots image data to bytes
                    img_data = []
                    for x in range(width):
                        for y in range(height):
                            r = self.camera.imageGetRed(image, width, x, y)
                            g = self.camera.imageGetGreen(image, width, x, y)
                            b = self.camera.imageGetBlue(image, width, x, y)
                            img_data.extend([r, g, b])
                    
                    img_msg.data = bytes(img_data)
                    self.camera_pub.publish(img_msg)
                    
                    # Publish camera info as well
                    self.publish_camera_info(width, height)
            except Exception as e:
                self.get_logger().warn(f"Error publishing camera data: {e}")

    def publish_camera_info(self, width, height):
        """
        Publish camera info
        """
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'camera_link'
        info_msg.width = width
        info_msg.height = height
        
        # Set default camera parameters (these should be calibrated properly)
        info_msg.k = [width / 2, 0.0, width /  2,  # fx, 0, cx
                      0.0, width / 2, height / 2,  # 0, fy, cy
                      0.0, 0.0, 1.0]              # 0, 0, 1
        info_msg.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
        info_msg.p = [width / 2, 0.0, width / 2, 0.0,
                      0.0, width / 2, height / 2, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        
        self.camera_info_pub.publish(info_msg)

    def publish_imu_data(self):
        """
        Publish IMU data
        """
        if self.imu:
            try:
                # Get IMU data from Webots
                accelerometer_values = self.imu.getValues()  # Acceleration values
                # Note: Webots IMU might need more processing to get proper IMU data
                # This is a simplified implementation
                
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'
                
                # Set linear acceleration (simplified)
                imu_msg.linear_acceleration.x = float(accelerometer_values[0])
                imu_msg.linear_acceleration.y = float(accelerometer_values[1])
                imu_msg.linear_acceleration.z = float(accelerometer_values[2])
                
                # Set orientation (simplified - in a real system you'd integrate angular velocities)
                imu_msg.orientation.w = 1.0  # Default orientation (no rotation)
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                
                # For now, set angular velocity to 0 (in a real system you'd get from gyroscope)
                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = 0.0
                
                self.imu_pub.publish(imu_msg)
            except Exception as e:
                self.get_logger().warn(f"Error publishing IMU data: {e}")

    def publish_odometry(self):
        """
        Publish odometry data (simplified)
        """
        # This is a simplified odometry publisher - in a real system you'd integrate 
        # velocity or use position sensors
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Set position (simplified - in reality you'd use position sensors from Webots)
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Set velocities (simplified)
        # In a real system, you'd derive this from wheel encoders or Webots velocity data
        odom_msg.twist.twist.linear.x = 0.0  # Would come from wheel velocities
        odom_msg.twist.twist.angular.z = 0.0  # Would come from robot turning rate
        
        self.odom_pub.publish(odom_msg)

    def publish_tf(self):
        """
        Publish transform data
        """
        tf_msg = TFMessage()
        
        # Create transform from odom to base_footprint
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = self.get_clock().now().to_msg()
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_footprint'
        odom_to_base.transform.translation.x = self.x_pos
        odom_to_base.transform.translation.y = self.y_pos
        odom_to_base.transform.translation.z = 0.0
        
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_to_base.transform.rotation.x = quat[0]
        odom_to_base.transform.rotation.y = quat[1]
        odom_to_base.transform.rotation.z = quat[2]
        odom_to_base.transform.rotation.w = quat[3]
        
        tf_msg.transforms.append(odom_to_base)
        
        # Create transform from base_footprint to base_link
        base_to_link = TransformStamped()
        base_to_link.header.stamp = self.get_clock().now().to_msg()
        base_to_link.header.frame_id = 'base_footprint'
        base_to_link.child_frame_id = 'base_link'
        base_to_link.transform.translation.x = 0.0
        base_to_link.transform.translation.y = 0.0
        base_to_link.transform.translation.z = 0.0  # Wheel radius offset
        base_to_link.transform.rotation.w = 1.0
        base_to_link.transform.rotation.x = 0.0
        base_to_link.transform.rotation.y = 0.0
        base_to_link.transform.rotation.z = 0.0
        
        tf_msg.transforms.append(base_to_link)
        
        # Create transform from base_link to lidar_link
        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = self.get_clock().now().to_msg()
        base_to_lidar.header.frame_id = 'base_link'
        base_to_lidar.child_frame_id = 'lidar_link'
        base_to_lidar.transform.translation.x = 0.0
        base_to_lidar.transform.translation.y = 0.0
        base_to_lidar.transform.translation.z = 0.15  # Height of LiDAR above base
        base_to_lidar.transform.rotation.w = 1.0
        base_to_lidar.transform.rotation.x = 0.0
        base_to_lidar.transform.rotation.y = 0.0
        base_to_lidar.transform.rotation.z = 0.0
        
        tf_msg.transforms.append(base_to_lidar)
        
        # Create transform from base_link to camera_link
        base_to_camera = TransformStamped()
        base_to_camera.header.stamp = self.get_clock().now().to_msg()
        base_to_camera.header.frame_id = 'base_link'
        base_to_camera.child_frame_id = 'camera_link'
        base_to_camera.transform.translation.x = 0.4  # Forward position of camera
        base_to_camera.transform.translation.y = 0.0
        base_to_camera.transform.translation.z = 0.05  # Height of camera above base
        base_to_camera.transform.rotation.w = 1.0
        base_to_camera.transform.rotation.x = 0.0
        base_to_camera.transform.rotation.y = 0.0
        base_to_camera.transform.rotation.z = 0.0
        
        tf_msg.transforms.append(base_to_camera)
        
        # Create transform from base_link to imu_link
        base_to_imu = TransformStamped()
        base_to_imu.header.stamp = self.get_clock().now().to_msg()
        base_to_imu.header.frame_id = 'base_link'
        base_to_imu.child_frame_id = 'imu_link'
        base_to_imu.transform.translation.x = 0.0
        base_to_imu.transform.translation.y = 0.0
        base_to_imu.transform.translation.z = 0.0
        base_to_imu.transform.rotation.w = 1.0
        base_to_imu.transform.rotation.x = 0.0
        base_to_imu.transform.rotation.y = 0.0
        base_to_imu.transform.rotation.z = 0.0
        
        tf_msg.transforms.append(base_to_imu)
        
        self.tf_pub.publish(tf_msg)


def main(args=None):
    """
    Main function to run the LunaBot Webots Controller
    """
    print("Starting LunaBot Webots Controller...")
    
    # Initialize Webots ROS2 interface
    rclpy.init(args=args)
    
    # Create controller node
    controller = LunaBotWebotsController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()