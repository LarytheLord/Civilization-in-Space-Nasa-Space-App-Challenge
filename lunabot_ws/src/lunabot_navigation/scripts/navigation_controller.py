#!/usr/bin/env python3
"""
LunaBot Navigation Controller
Advanced path planning and obstacle avoidance for lunar environments
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from enum import Enum
from typing import List, Tuple, Optional

# ROS2 message types
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class NavigationState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    EMERGENCY_STOP = "emergency_stop"
    PATROL = "patrol"
    MAINTENANCE = "maintenance"

class LunarObstacle:
    """Represents an obstacle in the lunar environment"""
    def __init__(self, x: float, y: float, radius: float, confidence: float):
        self.x = x
        self.y = y
        self.radius = radius
        self.confidence = confidence
        self.type = "unknown"
        self.danger_level = 0.5

class NavigationController(Node):
    """
    Advanced navigation controller for LunaBot
    Handles path planning, obstacle avoidance, and mission execution
    """
    
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Navigation state
        self.current_state = NavigationState.IDLE
        self.current_goal = None
        self.patrol_waypoints = []
        self.current_waypoint_index = 0
        
        # Obstacle tracking
        self.detected_obstacles = []
        self.emergency_obstacles = []
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, sensor_qos)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.mission_cmd_sub = self.create_subscription(
            String, '/mission_command', self.mission_command_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.obstacle_markers_pub = self.create_publisher(
            MarkerArray, '/detected_obstacles', 10)
        self.navigation_status_pub = self.create_publisher(
            String, '/navigation_status', 10)
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10)
        
        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Navigation parameters
        self.max_linear_velocity = 0.5  # Reduced for lunar terrain
        self.max_angular_velocity = 0.8
        self.obstacle_detection_range = 3.0
        self.emergency_stop_distance = 0.8
        self.path_planning_resolution = 0.1
        
        # Lunar-specific parameters
        self.lunar_gravity = 1.62  # m/s²
        self.dust_factor = 0.8  # Reduced traction due to lunar dust
        self.communication_delay = 2.6  # Earth-Moon communication delay (seconds)
        
        # Initialize patrol waypoints (example coordinates)
        self.initialize_patrol_waypoints()
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Mission timer for autonomous operations
        self.mission_timer = self.create_timer(1.0, self.mission_loop)
        
        self.get_logger().info("🚀 LunaBot Navigation Controller initialized")
    
    def initialize_patrol_waypoints(self):
        """Initialize default patrol waypoints for habitat monitoring"""
        self.patrol_waypoints = [
            {'x': 3.0, 'y': 3.0, 'name': 'Habitat Module 1 Check'},
            {'x': -3.0, 'y': 3.0, 'name': 'Habitat Module 2 Check'},
            {'x': 0.0, 'y': 8.0, 'name': 'Communication Antenna'},
            {'x': 8.0, 'y': 5.0, 'name': 'Equipment Storage'},
            {'x': 10.0, 'y': 0.0, 'name': 'Solar Panel Inspection'},
            {'x': 0.0, 'y': 0.0, 'name': 'Central Hub Return'}
        ]
    
    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.detect_obstacles_from_scan(msg)
        self.check_emergency_conditions(msg)
    
    def costmap_callback(self, msg):
        """Process global costmap updates"""
        # Update path planning based on costmap changes
        if self.current_state == NavigationState.NAVIGATING and self.current_goal:
            self.replan_if_necessary()
    
    def mission_command_callback(self, msg):
        """Handle mission commands"""
        command = msg.data.lower()
        
        if command == "start_patrol":
            self.start_patrol_mission()
        elif command == "emergency_stop":
            self.emergency_stop()
        elif command == "resume":
            self.resume_navigation()
        elif command == "return_home":
            self.return_to_base()
        elif command.startswith("goto"):
            # Parse goto command: "goto x y"
            parts = command.split()
            if len(parts) == 3:
                try:
                    x, y = float(parts[1]), float(parts[2])
                    self.navigate_to_point(x, y)
                except ValueError:
                    self.get_logger().error(f"Invalid goto command: {command}")
        
        self.get_logger().info(f"Received mission command: {command}")
    
    def detect_obstacles_from_scan(self, scan_msg):
        """Detect and classify obstacles from laser scan"""
        self.detected_obstacles.clear()
        
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter valid ranges
        valid_indices = np.where(
            (ranges >= scan_msg.range_min) & 
            (ranges <= self.obstacle_detection_range) &
            ~np.isnan(ranges) & ~np.isinf(ranges)
        )[0]
        
        # Group nearby points into obstacles
        obstacle_clusters = self.cluster_obstacle_points(ranges[valid_indices], angles[valid_indices])
        
        for cluster in obstacle_clusters:
            obstacle = self.analyze_obstacle_cluster(cluster)
            if obstacle:
                self.detected_obstacles.append(obstacle)
        
        # Publish obstacle markers
        self.publish_obstacle_markers()
    
    def cluster_obstacle_points(self, ranges, angles):
        """Cluster nearby obstacle points"""
        clusters = []
        current_cluster = []
        
        for i, (range_val, angle) in enumerate(zip(ranges, angles)):
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            point = (x, y, range_val, angle)
            
            if not current_cluster:
                current_cluster = [point]
            else:
                # Check distance to last point in cluster
                last_x, last_y, _, _ = current_cluster[-1]
                distance = np.sqrt((x - last_x)**2 + (y - last_y)**2)
                
                if distance < 0.5:  # Points within 50cm are same obstacle
                    current_cluster.append(point)
                else:
                    if len(current_cluster) >= 3:  # Minimum points for valid obstacle
                        clusters.append(current_cluster)
                    current_cluster = [point]
        
        # Add final cluster
        if len(current_cluster) >= 3:
            clusters.append(current_cluster)
        
        return clusters
    
    def analyze_obstacle_cluster(self, cluster):
        """Analyze obstacle cluster to determine properties"""
        if len(cluster) < 3:
            return None
        
        # Calculate centroid
        x_coords = [point[0] for point in cluster]
        y_coords = [point[1] for point in cluster]
        ranges = [point[2] for point in cluster]
        
        center_x = np.mean(x_coords)
        center_y = np.mean(y_coords)
        avg_range = np.mean(ranges)
        
        # Estimate radius
        max_distance = 0
        for x, y, _, _ in cluster:
            dist = np.sqrt((x - center_x)**2 + (y - center_y)**2)
            max_distance = max(max_distance, dist)
        
        radius = max(0.2, max_distance + 0.1)  # Minimum 20cm radius
        
        # Calculate confidence based on point density and consistency
        confidence = min(1.0, len(cluster) / 10.0)
        
        # Create obstacle
        obstacle = LunarObstacle(center_x, center_y, radius, confidence)
        
        # Classify obstacle type based on characteristics
        obstacle.type = self.classify_obstacle(cluster, avg_range)
        obstacle.danger_level = self.assess_danger_level(obstacle)
        
        return obstacle
    
    def classify_obstacle(self, cluster, avg_range):
        """Classify obstacle type based on scan characteristics"""
        cluster_size = len(cluster)
        
        if avg_range > 2.0:
            return "distant_object"
        elif cluster_size > 15:
            return "large_structure"  # Habitat module, equipment
        elif cluster_size > 8:
            return "medium_obstacle"  # Rocks, equipment
        else:
            return "small_debris"  # Small rocks, debris
    
    def assess_danger_level(self, obstacle):
        """Assess danger level of obstacle"""
        distance = np.sqrt(obstacle.x**2 + obstacle.y**2)
        
        # Base danger on distance and size
        distance_factor = max(0.1, 1.0 - distance / 3.0)
        size_factor = min(1.0, obstacle.radius / 0.5)
        
        danger_level = (distance_factor + size_factor) / 2.0
        
        # Increase danger for certain obstacle types
        if obstacle.type == "large_structure":
            danger_level *= 1.2
        elif obstacle.type == "small_debris":
            danger_level *= 0.8
        
        return min(1.0, danger_level)
    
    def check_emergency_conditions(self, scan_msg):
        """Check for emergency stop conditions"""
        self.emergency_obstacles.clear()
        
        # Check for very close obstacles
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Look for obstacles in front of robot
        front_indices = np.where(
            (angles >= -np.pi/4) & (angles <= np.pi/4) &
            (ranges < self.emergency_stop_distance) &
            (ranges >= scan_msg.range_min)
        )[0]
        
        if len(front_indices) > 0:
            self.get_logger().warn(f"Emergency obstacle detected at {np.min(ranges[front_indices]):.2f}m")
            self.emergency_stop()
    
    def emergency_stop(self):
        """Execute emergency stop"""
        self.current_state = NavigationState.EMERGENCY_STOP
        
        # Stop robot immediately
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Cancel current navigation goal
        if self.nav_to_pose_client.server_is_ready():
            self.nav_to_pose_client.cancel_all_goals()
        
        # Publish emergency status
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
        
        status_msg = String()
        status_msg.data = "EMERGENCY_STOP: Obstacle detected in critical zone"
        self.navigation_status_pub.publish(status_msg)
        
        self.get_logger().error("🚨 EMERGENCY STOP ACTIVATED")
    
    def resume_navigation(self):
        """Resume navigation after emergency stop"""
        if self.current_state == NavigationState.EMERGENCY_STOP:
            # Check if path is clear
            if self.is_path_clear():
                self.current_state = NavigationState.IDLE
                
                emergency_msg = Bool()
                emergency_msg.data = False
                self.emergency_stop_pub.publish(emergency_msg)
                
                self.get_logger().info("✅ Navigation resumed")
            else:
                self.get_logger().warn("⚠️ Path still blocked, cannot resume")
    
    def is_path_clear(self):
        """Check if immediate path is clear of obstacles"""
        for obstacle in self.detected_obstacles:
            distance = np.sqrt(obstacle.x**2 + obstacle.y**2)
            if distance < self.emergency_stop_distance * 1.5:
                return False
        return True
    
    def start_patrol_mission(self):
        """Start autonomous patrol mission"""
        if self.current_state in [NavigationState.IDLE, NavigationState.EMERGENCY_STOP]:
            self.current_state = NavigationState.PATROL
            self.current_waypoint_index = 0
            self.navigate_to_next_waypoint()
            
            self.get_logger().info("🛰️ Starting patrol mission")
    
    def navigate_to_next_waypoint(self):
        """Navigate to next waypoint in patrol"""
        if self.current_waypoint_index < len(self.patrol_waypoints):
            waypoint = self.patrol_waypoints[self.current_waypoint_index]
            self.navigate_to_point(waypoint['x'], waypoint['y'])
            
            self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}: {waypoint['name']}")
        else:
            # Patrol complete, return to start
            self.current_waypoint_index = 0
            self.navigate_to_next_waypoint()
    
    def navigate_to_point(self, x: float, y: float):
        """Navigate to specific point"""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available")
            return
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.current_goal = (x, y)
        self.current_state = NavigationState.NAVIGATING
        
        # Send goal
        future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self.navigation_feedback_callback)
        future.add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            self.current_state = NavigationState.IDLE
            return
        
        self.get_logger().info("Navigation goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Log progress or handle feedback as needed
        pass
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Navigation goal reached")
            
            if self.current_state == NavigationState.PATROL:
                # Move to next waypoint
                self.current_waypoint_index += 1
                # Add delay for inspection/maintenance
                self.create_timer(3.0, self.navigate_to_next_waypoint, one_shot=True)
            else:
                self.current_state = NavigationState.IDLE
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
            self.current_state = NavigationState.IDLE
    
    def return_to_base(self):
        """Return to base/charging station"""
        self.navigate_to_point(0.0, 0.0)  # Assume base is at origin
        self.get_logger().info("🏠 Returning to base")
    
    def replan_if_necessary(self):
        """Replan path if obstacles detected in current path"""
        # This would integrate with path planning algorithms
        # For now, just log the need for replanning
        if len(self.detected_obstacles) > 0:
            self.get_logger().info("🔄 Obstacles detected, considering replanning")
    
    def publish_obstacle_markers(self):
        """Publish visualization markers for detected obstacles"""
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(self.detected_obstacles):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detected_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position and size
            marker.pose.position.x = obstacle.x
            marker.pose.position.y = obstacle.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = obstacle.radius * 2
            marker.scale.y = obstacle.radius * 2
            marker.scale.z = 0.5
            
            # Color based on danger level
            marker.color.r = obstacle.danger_level
            marker.color.g = 1.0 - obstacle.danger_level
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker.lifetime.sec = 2
            marker_array.markers.append(marker)
        
        self.obstacle_markers_pub.publish(marker_array)
    
    def control_loop(self):
        """Main control loop"""
        # Publish current status
        status_msg = String()
        status_msg.data = f"State: {self.current_state.value}, Obstacles: {len(self.detected_obstacles)}"
        self.navigation_status_pub.publish(status_msg)
        
        # Handle state-specific behaviors
        if self.current_state == NavigationState.OBSTACLE_AVOIDANCE:
            self.handle_obstacle_avoidance()
    
    def mission_loop(self):
        """Mission-level autonomous behavior"""
        # Implement autonomous decision making
        if self.current_state == NavigationState.IDLE:
            # Could start patrol or other autonomous behaviors
            pass
    
    def handle_obstacle_avoidance(self):
        """Handle local obstacle avoidance"""
        # Implement dynamic window approach or similar
        # This is a simplified version
        
        if not self.detected_obstacles:
            self.current_state = NavigationState.NAVIGATING
            return
        
        # Find best direction to avoid obstacles
        best_direction = self.calculate_avoidance_direction()
        
        if best_direction is not None:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2  # Slow forward movement
            cmd_vel.angular.z = best_direction
            self.cmd_vel_pub.publish(cmd_vel)
        else:
            # No safe direction, stop
            self.emergency_stop()
    
    def calculate_avoidance_direction(self):
        """Calculate best direction for obstacle avoidance"""
        # Simplified potential field approach
        total_force_x = 0.0
        total_force_y = 0.0
        
        for obstacle in self.detected_obstacles:
            distance = np.sqrt(obstacle.x**2 + obstacle.y**2)
            if distance < 0.1:
                continue
            
            # Repulsive force
            force_magnitude = 1.0 / (distance**2)
            force_x = -force_magnitude * obstacle.x / distance
            force_y = -force_magnitude * obstacle.y / distance
            
            total_force_x += force_x
            total_force_y += force_y
        
        # Convert to angular velocity
        if abs(total_force_y) > 0.1:
            return np.sign(total_force_y) * 0.5
        
        return None

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
