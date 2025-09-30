#!/usr/bin/env python3
"""
LunaBot Advanced Navigation Controller
Provides advanced navigation capabilities with adaptive path planning and obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import time
from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Tuple
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R

# ROS2 message types
from std_msgs.msg import String, Bool, Header
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose, FollowPath
from nav2_msgs.srv import GetCostmap, ClearEntireCostmap
from action_msgs.msg import GoalStatus

class NavigationState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    REPLANNING = "replanning"
    EMERGENCY_STOP = "emergency_stop"
    RETURNING_HOME = "returning_home"

class NavigationMode(Enum):
    NORMAL = "normal"
    CAUTIOUS = "cautious"
    AGGRESSIVE = "aggressive"
    LUNAR_OPTIMIZED = "lunar_optimized"

class Waypoint:
    """Represents a navigation waypoint with metadata"""
    def __init__(self, x: float, y: float, z: float = 0.0, 
                 tolerance: float = 0.5, priority: int = 1):
        self.x = x
        self.y = y
        self.z = z
        self.tolerance = tolerance
        self.priority = priority
        self.attempts = 0
        self.last_attempt = None
        self.status = "pending"  # pending, active, completed, failed

class NavigationController(Node):
    """
    Advanced navigation controller for LunaBot
    Provides intelligent path planning, obstacle avoidance, and adaptive navigation
    """
    
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Navigation state
        self.current_state = NavigationState.IDLE
        self.navigation_mode = NavigationMode.LUNAR_OPTIMIZED
        self.current_goal = None
        self.waypoint_queue = []
        self.home_position = (0.0, 0.0, 0.0)
        
        # Navigation parameters
        self.max_linear_velocity = 0.5  # m/s (reduced for lunar gravity)
        self.max_angular_velocity = 0.8  # rad/s
        self.obstacle_detection_range = 2.0  # meters
        self.emergency_stop_distance = 0.5  # meters
        self.path_replanning_threshold = 3  # failed attempts before replanning
        
        # Lunar-specific parameters
        self.lunar_gravity_factor = 1.62 / 9.81  # Moon gravity / Earth gravity
        self.dust_accumulation_factor = 0.1
        self.traction_coefficient = 0.6  # Reduced traction on lunar surface
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Current robot pose
        self.current_pose = None
        self.current_velocity = None
        
        # Obstacle tracking
        self.detected_obstacles = []
        self.dynamic_obstacles = []
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, sensor_qos)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.mission_command_sub = self.create_subscription(
            String, '/mission_command', self.mission_command_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.navigation_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.path_markers_pub = self.create_publisher(MarkerArray, '/navigation_path', 10)
        self.obstacle_markers_pub = self.create_publisher(MarkerArray, '/detected_obstacles', 10)
        
        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        
        # Service clients
        self.get_costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        
        # Timers
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)  # 10 Hz
        self.status_timer = self.create_timer(2.0, self.publish_status)
        self.safety_timer = self.create_timer(0.05, self.safety_check)  # 20 Hz safety check
        
        # Navigation metrics
        self.navigation_metrics = {
            'total_distance': 0.0,
            'successful_navigations': 0,
            'failed_navigations': 0,
            'obstacle_avoidance_events': 0,
            'replanning_events': 0,
            'average_speed': 0.0,
            'navigation_efficiency': 0.0
        }
        
        self.get_logger().info("🧭 LunaBot Navigation Controller initialized")
        self.get_logger().info(f"Navigation mode: {self.navigation_mode.value}")
        
        # Wait for action servers
        self.wait_for_action_servers()
    
    def wait_for_action_servers(self):
        """Wait for navigation action servers to become available"""
        self.get_logger().info("Waiting for navigation action servers...")
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("NavigateToPose action server not available")
        else:
            self.get_logger().info("NavigateToPose action server ready")
        
        if not self.follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("FollowPath action server not available")
        else:
            self.get_logger().info("FollowPath action server ready")
    
    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.detect_obstacles(msg)
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
        # Update navigation metrics
        if self.current_velocity:
            linear_speed = np.sqrt(
                self.current_velocity.linear.x**2 + 
                self.current_velocity.linear.y**2
            )
            self.update_speed_metrics(linear_speed)
    
    def costmap_callback(self, msg):
        """Process global costmap updates"""
        # Analyze costmap for navigation planning
        self.analyze_costmap(msg)
    
    def mission_command_callback(self, msg):
        """Handle mission commands"""
        command = msg.data.lower()
        
        if command.startswith("goto"):
            # Parse goto command: "goto x y" or "goto x y z"
            parts = command.split()
            if len(parts) >= 3:
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3]) if len(parts) > 3 else 0.0
                    self.navigate_to_position(x, y, z)
                except ValueError:
                    self.get_logger().error(f"Invalid goto command: {command}")
        
        elif command == "emergency_stop":
            self.emergency_stop()
        elif command == "return_home":
            self.return_home()
        elif command == "resume":
            self.resume_navigation()
        elif command == "clear_costmap":
            self.clear_costmap()
        elif command.startswith("set_mode"):
            # Parse mode command: "set_mode cautious"
            parts = command.split()
            if len(parts) == 2:
                self.set_navigation_mode(parts[1])
        
        self.get_logger().info(f"Processed mission command: {command}")
    
    def navigate_to_position(self, x: float, y: float, z: float = 0.0, tolerance: float = 0.5):
        """Navigate to specified position"""
        waypoint = Waypoint(x, y, z, tolerance)
        self.add_waypoint(waypoint)
        
        if self.current_state == NavigationState.IDLE:
            self.start_navigation()
    
    def add_waypoint(self, waypoint: Waypoint):
        """Add waypoint to navigation queue"""
        self.waypoint_queue.append(waypoint)
        self.get_logger().info(f"Added waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})")
    
    def start_navigation(self):
        """Start navigation to next waypoint"""
        if not self.waypoint_queue:
            self.get_logger().info("No waypoints in queue")
            return
        
        if not self.nav_to_pose_client.server_is_ready():
            self.get_logger().warn("Navigation server not ready")
            return
        
        # Get next waypoint
        waypoint = self.waypoint_queue[0]
        waypoint.status = "active"
        waypoint.last_attempt = datetime.now()
        waypoint.attempts += 1
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint.x
        goal_msg.pose.pose.position.y = waypoint.y
        goal_msg.pose.pose.position.z = waypoint.z
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation
        
        # Adjust goal based on navigation mode
        goal_msg = self.adjust_goal_for_mode(goal_msg, waypoint)
        
        # Send goal
        self.current_goal = goal_msg
        self.current_state = NavigationState.NAVIGATING
        
        future = self.nav_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.navigation_feedback_callback
        )
        future.add_done_callback(self.navigation_goal_response_callback)
        
        self.get_logger().info(f"Navigating to waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})")
    
    def adjust_goal_for_mode(self, goal_msg, waypoint: Waypoint):
        """Adjust navigation goal based on current navigation mode"""
        if self.navigation_mode == NavigationMode.CAUTIOUS:
            # Reduce speed and increase tolerance
            waypoint.tolerance *= 1.5
        elif self.navigation_mode == NavigationMode.LUNAR_OPTIMIZED:
            # Apply lunar-specific adjustments
            # Account for reduced gravity and traction
            pass
        
        return goal_msg
    
    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            self.handle_navigation_failure()
            return
        
        self.get_logger().info("Navigation goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        
        # Monitor navigation progress
        if hasattr(feedback, 'distance_remaining'):
            distance = feedback.distance_remaining
            if distance < self.emergency_stop_distance and self.detected_obstacles:
                self.get_logger().warn("Close to obstacles, initiating emergency stop")
                self.emergency_stop()
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.handle_navigation_success()
        else:
            self.handle_navigation_failure()
    
    def handle_navigation_success(self):
        """Handle successful navigation"""
        if self.waypoint_queue:
            waypoint = self.waypoint_queue.pop(0)
            waypoint.status = "completed"
            
            self.navigation_metrics['successful_navigations'] += 1
            self.get_logger().info(f"Successfully reached waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})")
        
        # Continue to next waypoint or return to idle
        if self.waypoint_queue:
            self.start_navigation()
        else:
            self.current_state = NavigationState.IDLE
            self.get_logger().info("All waypoints completed")
    
    def handle_navigation_failure(self):
        """Handle navigation failure"""
        if not self.waypoint_queue:
            return
        
        waypoint = self.waypoint_queue[0]
        self.navigation_metrics['failed_navigations'] += 1
        
        if waypoint.attempts < self.path_replanning_threshold:
            # Retry navigation
            self.get_logger().warn(f"Navigation failed, retrying ({waypoint.attempts}/{self.path_replanning_threshold})")
            self.current_state = NavigationState.REPLANNING
            
            # Wait before retry
            self.create_timer(2.0, lambda: self.start_navigation())
        else:
            # Give up on this waypoint
            waypoint.status = "failed"
            self.waypoint_queue.pop(0)
            self.get_logger().error(f"Failed to reach waypoint after {waypoint.attempts} attempts")
            
            # Continue to next waypoint
            if self.waypoint_queue:
                self.start_navigation()
            else:
                self.current_state = NavigationState.IDLE
    
    def detect_obstacles(self, scan_msg):
        """Detect obstacles from laser scan"""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Find obstacles within detection range
        valid_indices = np.where(
            (ranges < self.obstacle_detection_range) & 
            (ranges > scan_msg.range_min) & 
            ~np.isnan(ranges) & 
            ~np.isinf(ranges)
        )[0]
        
        obstacles = []
        for idx in valid_indices:
            angle = angles[idx]
            distance = ranges[idx]
            
            # Convert to Cartesian coordinates
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            
            obstacles.append({
                'x': x,
                'y': y,
                'distance': distance,
                'angle': angle,
                'timestamp': time.time()
            })
        
        self.detected_obstacles = obstacles
        
        # Check for emergency stop conditions
        emergency_obstacles = [obs for obs in obstacles if obs['distance'] < self.emergency_stop_distance]
        if emergency_obstacles and self.current_state == NavigationState.NAVIGATING:
            self.get_logger().warn(f"Emergency obstacles detected: {len(emergency_obstacles)}")
            self.initiate_obstacle_avoidance()
    
    def initiate_obstacle_avoidance(self):
        """Initiate obstacle avoidance maneuver"""
        self.current_state = NavigationState.OBSTACLE_AVOIDANCE
        self.navigation_metrics['obstacle_avoidance_events'] += 1
        
        # Simple obstacle avoidance - stop and replan
        self.stop_robot()
        
        # Clear costmap and replan
        self.clear_costmap()
        
        # Wait before replanning
        self.create_timer(1.0, self.replan_path)
    
    def replan_path(self):
        """Replan path around obstacles"""
        self.current_state = NavigationState.REPLANNING
        self.navigation_metrics['replanning_events'] += 1
        
        # For now, simply retry navigation
        # In a more advanced implementation, this would use advanced path planning
        if self.waypoint_queue:
            self.start_navigation()
    
    def emergency_stop(self):
        """Execute emergency stop"""
        self.current_state = NavigationState.EMERGENCY_STOP
        self.stop_robot()
        
        # Cancel current navigation goal
        if hasattr(self, 'current_goal_handle'):
            self.nav_to_pose_client.cancel_goal_async(self.current_goal_handle)
        
        self.get_logger().error("🚨 EMERGENCY STOP ACTIVATED")
    
    def stop_robot(self):
        """Stop robot movement"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.angular.z = 0.0
        
        # Send stop command multiple times to ensure it's received
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(0.1)
    
    def return_home(self):
        """Return to home position"""
        self.get_logger().info("Returning to home position")
        self.current_state = NavigationState.RETURNING_HOME
        
        # Clear current waypoints and add home position
        self.waypoint_queue.clear()
        home_waypoint = Waypoint(
            self.home_position[0], 
            self.home_position[1], 
            self.home_position[2]
        )
        self.add_waypoint(home_waypoint)
        self.start_navigation()
    
    def resume_navigation(self):
        """Resume navigation after emergency stop"""
        if self.current_state == NavigationState.EMERGENCY_STOP:
            self.current_state = NavigationState.IDLE
            self.get_logger().info("Navigation resumed")
            
            # Continue with waypoints if any
            if self.waypoint_queue:
                self.start_navigation()
    
    def set_navigation_mode(self, mode_name: str):
        """Set navigation mode"""
        try:
            self.navigation_mode = NavigationMode(mode_name.lower())
            self.get_logger().info(f"Navigation mode set to: {self.navigation_mode.value}")
            
            # Adjust parameters based on mode
            self.adjust_parameters_for_mode()
        except ValueError:
            self.get_logger().error(f"Invalid navigation mode: {mode_name}")
    
    def adjust_parameters_for_mode(self):
        """Adjust navigation parameters based on current mode"""
        if self.navigation_mode == NavigationMode.CAUTIOUS:
            self.max_linear_velocity = 0.3
            self.max_angular_velocity = 0.5
            self.obstacle_detection_range = 3.0
        elif self.navigation_mode == NavigationMode.AGGRESSIVE:
            self.max_linear_velocity = 0.8
            self.max_angular_velocity = 1.2
            self.obstacle_detection_range = 1.5
        elif self.navigation_mode == NavigationMode.LUNAR_OPTIMIZED:
            # Optimized for lunar conditions
            self.max_linear_velocity = 0.5 * self.traction_coefficient
            self.max_angular_velocity = 0.8 * self.traction_coefficient
            self.obstacle_detection_range = 2.5
    
    def clear_costmap(self):
        """Clear global costmap"""
        if self.clear_costmap_client.service_is_ready():
            request = ClearEntireCostmap.Request()
            future = self.clear_costmap_client.call_async(request)
            self.get_logger().info("Clearing global costmap")
    
    def analyze_costmap(self, costmap_msg):
        """Analyze costmap for navigation planning"""
        # Extract costmap data
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        resolution = costmap_msg.info.resolution
        
        # Analyze for high-cost areas that might indicate obstacles
        data = np.array(costmap_msg.data).reshape((height, width))
        high_cost_areas = np.where(data > 80)  # Threshold for high cost
        
        if len(high_cost_areas[0]) > 0:
            self.get_logger().debug(f"Detected {len(high_cost_areas[0])} high-cost cells in costmap")
    
    def update_speed_metrics(self, current_speed: float):
        """Update speed and efficiency metrics"""
        # Simple moving average for speed
        if not hasattr(self, '_speed_history'):
            self._speed_history = []
        
        self._speed_history.append(current_speed)
        if len(self._speed_history) > 100:  # Keep last 100 readings
            self._speed_history.pop(0)
        
        self.navigation_metrics['average_speed'] = np.mean(self._speed_history)
    
    def navigation_loop(self):
        """Main navigation control loop"""
        # Update current pose from TF if available
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            # Update current pose from transform
            self.current_pose_from_tf = transform.transform
            
        except Exception as e:
            # TF not available, rely on odometry
            pass
        
        # State-specific processing
        if self.current_state == NavigationState.OBSTACLE_AVOIDANCE:
            self.process_obstacle_avoidance()
        elif self.current_state == NavigationState.REPLANNING:
            self.process_replanning()
    
    def process_obstacle_avoidance(self):
        """Process obstacle avoidance state"""
        # Check if obstacles are still present
        current_obstacles = [obs for obs in self.detected_obstacles 
                           if obs['distance'] < self.emergency_stop_distance]
        
        if not current_obstacles:
            # Obstacles cleared, resume navigation
            self.get_logger().info("Obstacles cleared, resuming navigation")
            self.current_state = NavigationState.NAVIGATING
    
    def process_replanning(self):
        """Process path replanning state"""
        # Wait for replanning to complete
        pass
    
    def safety_check(self):
        """Perform high-frequency safety checks"""
        if self.current_state == NavigationState.NAVIGATING:
            # Check for immediate obstacles
            immediate_obstacles = [obs for obs in self.detected_obstacles 
                                 if obs['distance'] < self.emergency_stop_distance]
            
            if immediate_obstacles:
                self.emergency_stop()
    
    def publish_status(self):
        """Publish navigation status"""
        status_data = {
            'timestamp': datetime.now().isoformat(),
            'state': self.current_state.value,
            'mode': self.navigation_mode.value,
            'waypoints_remaining': len(self.waypoint_queue),
            'current_goal': {
                'x': self.current_goal.pose.pose.position.x if self.current_goal else None,
                'y': self.current_goal.pose.pose.position.y if self.current_goal else None
            } if self.current_goal else None,
            'detected_obstacles': len(self.detected_obstacles),
            'metrics': self.navigation_metrics,
            'parameters': {
                'max_linear_velocity': self.max_linear_velocity,
                'max_angular_velocity': self.max_angular_velocity,
                'obstacle_detection_range': self.obstacle_detection_range
            }
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.navigation_status_pub.publish(status_msg)
        
        # Publish visualization markers
        self.publish_path_markers()
        self.publish_obstacle_markers()
    
    def publish_path_markers(self):
        """Publish path visualization markers"""
        marker_array = MarkerArray()
        
        # Waypoint markers
        for i, waypoint in enumerate(self.waypoint_queue):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = waypoint.x
            marker.pose.position.y = waypoint.y
            marker.pose.position.z = waypoint.z + 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color based on status
            if waypoint.status == "active":
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            elif waypoint.status == "completed":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:  # pending
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 0.8
            marker.lifetime.sec = 10
            
            marker_array.markers.append(marker)
        
        self.path_markers_pub.publish(marker_array)
    
    def publish_obstacle_markers(self):
        """Publish obstacle visualization markers"""
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(self.detected_obstacles):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = obstacle['x']
            marker.pose.position.y = obstacle['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Color based on distance
            if obstacle['distance'] < self.emergency_stop_distance:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.7
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        self.obstacle_markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    navigation_controller = NavigationController()
    
    try:
        rclpy.spin(navigation_controller)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()