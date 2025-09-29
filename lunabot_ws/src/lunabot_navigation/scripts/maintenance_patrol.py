#!/usr/bin/env python3
"""
LunaBot Maintenance Patrol System
Autonomous patrol and maintenance task execution for lunar habitat
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import time
from datetime import datetime, timedelta
from enum import Enum
from typing import Dict, List, Optional, Tuple

# ROS2 message types
from std_msgs.msg import String, Bool, Header
from geometry_msgs.msg import PoseStamped, Point, Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.action import NavigateToPose

class PatrolState(Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    INSPECTING = "inspecting"
    MAINTENANCE = "maintenance"
    REPORTING = "reporting"
    EMERGENCY_RESPONSE = "emergency_response"

class MaintenanceTask:
    """Represents a maintenance task"""
    def __init__(self, task_id: str, name: str, location: Tuple[float, float], 
                 priority: int, frequency_hours: int):
        self.task_id = task_id
        self.name = name
        self.location = location
        self.priority = priority  # 1-5, 5 being highest
        self.frequency_hours = frequency_hours
        self.last_completed = None
        self.status = "pending"
        self.inspection_data = {}
        self.anomalies_detected = []
    
    def is_due(self) -> bool:
        """Check if task is due for execution"""
        if self.last_completed is None:
            return True
        
        time_since_last = datetime.now() - self.last_completed
        return time_since_last.total_seconds() > (self.frequency_hours * 3600)
    
    def get_urgency_score(self) -> float:
        """Calculate urgency score based on priority and time overdue"""
        base_score = self.priority
        
        if self.last_completed is None:
            return base_score * 2.0
        
        time_since_last = datetime.now() - self.last_completed
        hours_overdue = max(0, time_since_last.total_seconds() / 3600 - self.frequency_hours)
        
        return base_score + (hours_overdue / 24.0)  # Add 1 point per day overdue

class InspectionPoint:
    """Represents a specific inspection point"""
    def __init__(self, point_id: str, name: str, location: Tuple[float, float], 
                 inspection_type: str):
        self.point_id = point_id
        self.name = name
        self.location = location
        self.inspection_type = inspection_type  # visual, sensor, structural
        self.last_inspection = None
        self.baseline_data = {}
        self.current_data = {}
        self.anomaly_threshold = 0.1

class MaintenancePatrol(Node):
    """
    Autonomous maintenance patrol system
    Performs scheduled inspections, detects anomalies, and executes maintenance tasks
    """
    
    def __init__(self):
        super().__init__('maintenance_patrol')
        
        # Patrol state
        self.current_state = PatrolState.IDLE
        self.current_task = None
        self.patrol_route = []
        self.current_route_index = 0
        
        # Initialize maintenance tasks and inspection points
        self.maintenance_tasks = self.initialize_maintenance_tasks()
        self.inspection_points = self.initialize_inspection_points()
        
        # Mission data
        self.patrol_log = []
        self.anomaly_reports = []
        self.maintenance_history = []
        
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
        self.habitat_alerts_sub = self.create_subscription(
            String, '/habitat_alerts', self.habitat_alerts_callback, 10)
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        self.patrol_command_sub = self.create_subscription(
            String, '/patrol_command', self.patrol_command_callback, 10)
        
        # Publishers
        self.patrol_status_pub = self.create_publisher(String, '/patrol_status', 10)
        self.maintenance_report_pub = self.create_publisher(String, '/maintenance_report', 10)
        self.anomaly_alert_pub = self.create_publisher(String, '/anomaly_alert', 10)
        self.task_markers_pub = self.create_publisher(MarkerArray, '/maintenance_tasks', 10)
        self.patrol_path_pub = self.create_publisher(Path, '/patrol_path', 10)
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timers
        self.patrol_timer = self.create_timer(1.0, self.patrol_loop)
        self.task_scheduler_timer = self.create_timer(60.0, self.schedule_tasks)  # Check every minute
        self.status_publisher_timer = self.create_timer(5.0, self.publish_status)
        
        # Initialize patrol route
        self.generate_patrol_route()
        
        self.get_logger().info("🔧 LunaBot Maintenance Patrol System initialized")
        self.get_logger().info(f"Loaded {len(self.maintenance_tasks)} maintenance tasks")
        self.get_logger().info(f"Loaded {len(self.inspection_points)} inspection points")
    
    def initialize_maintenance_tasks(self) -> Dict[str, MaintenanceTask]:
        """Initialize maintenance task definitions"""
        tasks = {}
        
        # Critical infrastructure tasks
        tasks['life_support_check'] = MaintenanceTask(
            'life_support_check', 'Life Support System Check', (0.0, 5.0), 5, 6)
        tasks['power_system_inspect'] = MaintenanceTask(
            'power_system_inspect', 'Power System Inspection', (10.0, 0.0), 5, 12)
        tasks['airlock_maintenance'] = MaintenanceTask(
            'airlock_maintenance', 'Airlock Seal Inspection', (0.0, -5.0), 4, 24)
        
        # Habitat integrity tasks
        tasks['module1_inspection'] = MaintenanceTask(
            'module1_inspection', 'Habitat Module 1 Inspection', (5.0, 0.0), 3, 48)
        tasks['module2_inspection'] = MaintenanceTask(
            'module2_inspection', 'Habitat Module 2 Inspection', (-5.0, 0.0), 3, 48)
        tasks['corridor_check'] = MaintenanceTask(
            'corridor_check', 'Corridor Structural Check', (0.0, 0.0), 3, 72)
        
        # Equipment maintenance
        tasks['antenna_alignment'] = MaintenanceTask(
            'antenna_alignment', 'Communication Antenna Check', (0.0, 8.0), 4, 168)
        tasks['solar_panel_clean'] = MaintenanceTask(
            'solar_panel_clean', 'Solar Panel Cleaning', (10.0, 0.0), 3, 336)
        tasks['storage_inventory'] = MaintenanceTask(
            'storage_inventory', 'Equipment Storage Inventory', (8.0, 5.0), 2, 720)
        
        # Environmental monitoring
        tasks['dust_accumulation'] = MaintenanceTask(
            'dust_accumulation', 'Lunar Dust Accumulation Check', (0.0, 0.0), 2, 24)
        tasks['radiation_survey'] = MaintenanceTask(
            'radiation_survey', 'Radiation Level Survey', (0.0, 0.0), 4, 168)
        
        return tasks
    
    def initialize_inspection_points(self) -> Dict[str, InspectionPoint]:
        """Initialize inspection point definitions"""
        points = {}
        
        # Structural inspection points
        points['module1_hull'] = InspectionPoint(
            'module1_hull', 'Module 1 Hull Integrity', (5.0, 0.0), 'structural')
        points['module2_hull'] = InspectionPoint(
            'module2_hull', 'Module 2 Hull Integrity', (-5.0, 0.0), 'structural')
        points['corridor_joints'] = InspectionPoint(
            'corridor_joints', 'Corridor Connection Joints', (0.0, 0.0), 'structural')
        
        # Equipment inspection points
        points['antenna_base'] = InspectionPoint(
            'antenna_base', 'Antenna Base Mount', (0.0, 8.0), 'visual')
        points['solar_panel_surface'] = InspectionPoint(
            'solar_panel_surface', 'Solar Panel Surface', (10.0, 0.0), 'visual')
        points['airlock_seals'] = InspectionPoint(
            'airlock_seals', 'Airlock Door Seals', (0.0, -5.0), 'visual')
        
        # Environmental monitoring points
        points['dust_sensors'] = InspectionPoint(
            'dust_sensors', 'Dust Accumulation Sensors', (0.0, 0.0), 'sensor')
        points['pressure_sensors'] = InspectionPoint(
            'pressure_sensors', 'Pressure Monitoring Points', (0.0, 0.0), 'sensor')
        
        return points
    
    def generate_patrol_route(self):
        """Generate optimal patrol route covering all inspection points"""
        # Simple route generation - in practice, use TSP solver
        locations = []
        
        # Add all maintenance task locations
        for task in self.maintenance_tasks.values():
            locations.append(task.location)
        
        # Add inspection points
        for point in self.inspection_points.values():
            locations.append(point.location)
        
        # Remove duplicates and sort by distance from origin
        unique_locations = list(set(locations))
        unique_locations.sort(key=lambda loc: np.sqrt(loc[0]**2 + loc[1]**2))
        
        self.patrol_route = unique_locations
        
        # Publish patrol path
        self.publish_patrol_path()
    
    def publish_patrol_path(self):
        """Publish patrol path for visualization"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, location in enumerate(self.patrol_route):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = location[0]
            pose.pose.position.y = location[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.patrol_path_pub.publish(path_msg)
    
    def laser_callback(self, msg):
        """Process laser scan for inspection data"""
        if self.current_state == PatrolState.INSPECTING:
            self.analyze_structural_data(msg)
    
    def camera_callback(self, msg):
        """Process camera data for visual inspection"""
        if self.current_state == PatrolState.INSPECTING:
            self.analyze_visual_data(msg)
    
    def habitat_alerts_callback(self, msg):
        """Handle habitat alerts for emergency response"""
        try:
            alerts = json.loads(msg.data)
            emergency_alerts = [a for a in alerts if a.get('level') == 'EMERGENCY']
            
            if emergency_alerts:
                self.handle_emergency_response(emergency_alerts)
        except json.JSONDecodeError:
            pass
    
    def diagnostics_callback(self, msg):
        """Process system diagnostics"""
        # Analyze diagnostics for maintenance needs
        for status in msg.status:
            if status.level >= 2:  # WARN or ERROR
                self.create_maintenance_task_from_diagnostic(status)
    
    def patrol_command_callback(self, msg):
        """Handle patrol commands"""
        command = msg.data.lower()
        
        if command == "start_patrol":
            self.start_patrol()
        elif command == "stop_patrol":
            self.stop_patrol()
        elif command == "emergency_return":
            self.emergency_return()
        elif command == "force_maintenance":
            self.force_maintenance_check()
        elif command.startswith("inspect"):
            # Parse inspect command: "inspect point_id"
            parts = command.split()
            if len(parts) == 2:
                self.force_inspection(parts[1])
        
        self.get_logger().info(f"Received patrol command: {command}")
    
    def patrol_loop(self):
        """Main patrol control loop"""
        if self.current_state == PatrolState.PATROLLING:
            self.execute_patrol()
        elif self.current_state == PatrolState.INSPECTING:
            self.execute_inspection()
        elif self.current_state == PatrolState.MAINTENANCE:
            self.execute_maintenance()
        elif self.current_state == PatrolState.REPORTING:
            self.execute_reporting()
    
    def schedule_tasks(self):
        """Schedule maintenance tasks based on priority and due dates"""
        if self.current_state != PatrolState.IDLE:
            return
        
        # Find highest priority due task
        due_tasks = [task for task in self.maintenance_tasks.values() if task.is_due()]
        
        if due_tasks:
            # Sort by urgency score
            due_tasks.sort(key=lambda t: t.get_urgency_score(), reverse=True)
            highest_priority_task = due_tasks[0]
            
            self.get_logger().info(f"Scheduling high priority task: {highest_priority_task.name}")
            self.current_task = highest_priority_task
            self.navigate_to_task_location(highest_priority_task)
    
    def start_patrol(self):
        """Start patrol mission"""
        if self.current_state == PatrolState.IDLE:
            self.current_state = PatrolState.PATROLLING
            self.current_route_index = 0
            self.get_logger().info("🚁 Starting maintenance patrol")
    
    def stop_patrol(self):
        """Stop patrol mission"""
        self.current_state = PatrolState.IDLE
        self.current_task = None
        self.get_logger().info("⏹️ Patrol stopped")
    
    def execute_patrol(self):
        """Execute patrol route"""
        if self.current_route_index >= len(self.patrol_route):
            # Patrol complete, return to start
            self.current_route_index = 0
            self.get_logger().info("✅ Patrol route completed, restarting")
        
        current_location = self.patrol_route[self.current_route_index]
        self.navigate_to_location(current_location)
    
    def navigate_to_location(self, location: Tuple[float, float]):
        """Navigate to specific location"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Navigation server not available")
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = location[0]
        goal_msg.pose.pose.position.y = location[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_complete_callback)
    
    def navigate_to_task_location(self, task: MaintenanceTask):
        """Navigate to maintenance task location"""
        self.current_state = PatrolState.PATROLLING
        self.navigate_to_location(task.location)
    
    def navigation_complete_callback(self, future):
        """Handle navigation completion"""
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            if self.current_state == PatrolState.PATROLLING:
                # Start inspection at current location
                self.start_inspection_at_location()
            
            self.current_route_index += 1
        else:
            self.get_logger().warn(f"Navigation failed: {result.status}")
    
    def start_inspection_at_location(self):
        """Start inspection at current location"""
        self.current_state = PatrolState.INSPECTING
        
        # Find inspection points at current location
        current_location = self.patrol_route[self.current_route_index]
        nearby_points = self.find_nearby_inspection_points(current_location)
        
        if nearby_points:
            self.get_logger().info(f"Starting inspection of {len(nearby_points)} points")
            self.inspect_points(nearby_points)
        
        # Check for maintenance tasks at this location
        nearby_tasks = self.find_nearby_maintenance_tasks(current_location)
        if nearby_tasks:
            self.execute_maintenance_tasks(nearby_tasks)
    
    def find_nearby_inspection_points(self, location: Tuple[float, float]) -> List[InspectionPoint]:
        """Find inspection points near given location"""
        nearby_points = []
        threshold = 1.0  # 1 meter threshold
        
        for point in self.inspection_points.values():
            distance = np.sqrt((point.location[0] - location[0])**2 + 
                             (point.location[1] - location[1])**2)
            if distance <= threshold:
                nearby_points.append(point)
        
        return nearby_points
    
    def find_nearby_maintenance_tasks(self, location: Tuple[float, float]) -> List[MaintenanceTask]:
        """Find maintenance tasks near given location"""
        nearby_tasks = []
        threshold = 1.0  # 1 meter threshold
        
        for task in self.maintenance_tasks.values():
            distance = np.sqrt((task.location[0] - location[0])**2 + 
                             (task.location[1] - location[1])**2)
            if distance <= threshold and task.is_due():
                nearby_tasks.append(task)
        
        return nearby_tasks
    
    def inspect_points(self, points: List[InspectionPoint]):
        """Inspect given points"""
        for point in points:
            self.get_logger().info(f"Inspecting: {point.name}")
            
            # Perform inspection based on type
            if point.inspection_type == 'visual':
                self.perform_visual_inspection(point)
            elif point.inspection_type == 'structural':
                self.perform_structural_inspection(point)
            elif point.inspection_type == 'sensor':
                self.perform_sensor_inspection(point)
            
            point.last_inspection = datetime.now()
        
        # Return to patrol after inspection
        self.current_state = PatrolState.PATROLLING
    
    def perform_visual_inspection(self, point: InspectionPoint):
        """Perform visual inspection using camera"""
        # Simulate visual inspection
        inspection_data = {
            'timestamp': datetime.now().isoformat(),
            'point_id': point.point_id,
            'type': 'visual',
            'dust_accumulation': np.random.uniform(0, 0.3),
            'surface_damage': np.random.uniform(0, 0.1),
            'color_change': np.random.uniform(0, 0.05)
        }
        
        point.current_data = inspection_data
        
        # Check for anomalies
        if inspection_data['dust_accumulation'] > 0.2:
            self.report_anomaly(point, "High dust accumulation detected")
        
        if inspection_data['surface_damage'] > 0.05:
            self.report_anomaly(point, "Surface damage detected")
    
    def perform_structural_inspection(self, point: InspectionPoint):
        """Perform structural inspection using LiDAR"""
        # Simulate structural inspection
        inspection_data = {
            'timestamp': datetime.now().isoformat(),
            'point_id': point.point_id,
            'type': 'structural',
            'dimensional_variance': np.random.uniform(0, 0.02),
            'surface_roughness': np.random.uniform(0, 0.01),
            'deformation': np.random.uniform(0, 0.005)
        }
        
        point.current_data = inspection_data
        
        # Check for structural anomalies
        if inspection_data['dimensional_variance'] > 0.01:
            self.report_anomaly(point, "Structural deformation detected")
    
    def perform_sensor_inspection(self, point: InspectionPoint):
        """Perform sensor-based inspection"""
        # Simulate sensor inspection
        inspection_data = {
            'timestamp': datetime.now().isoformat(),
            'point_id': point.point_id,
            'type': 'sensor',
            'sensor_response': np.random.uniform(0.8, 1.2),
            'calibration_drift': np.random.uniform(0, 0.05),
            'signal_quality': np.random.uniform(0.9, 1.0)
        }
        
        point.current_data = inspection_data
        
        # Check for sensor anomalies
        if inspection_data['calibration_drift'] > 0.03:
            self.report_anomaly(point, "Sensor calibration drift detected")
    
    def execute_maintenance_tasks(self, tasks: List[MaintenanceTask]):
        """Execute maintenance tasks"""
        self.current_state = PatrolState.MAINTENANCE
        
        for task in tasks:
            self.get_logger().info(f"Executing maintenance task: {task.name}")
            
            # Simulate task execution
            execution_time = np.random.uniform(30, 180)  # 30 seconds to 3 minutes
            
            # Record task completion
            task.last_completed = datetime.now()
            task.status = "completed"
            
            # Generate maintenance report
            self.generate_maintenance_report(task)
        
        self.current_state = PatrolState.PATROLLING
    
    def report_anomaly(self, point: InspectionPoint, description: str):
        """Report detected anomaly"""
        anomaly = {
            'timestamp': datetime.now().isoformat(),
            'point_id': point.point_id,
            'point_name': point.name,
            'description': description,
            'severity': self.assess_anomaly_severity(point, description),
            'inspection_data': point.current_data
        }
        
        self.anomaly_reports.append(anomaly)
        
        # Publish anomaly alert
        alert_msg = String()
        alert_msg.data = json.dumps(anomaly, indent=2)
        self.anomaly_alert_pub.publish(alert_msg)
        
        self.get_logger().warn(f"🚨 Anomaly detected at {point.name}: {description}")
    
    def assess_anomaly_severity(self, point: InspectionPoint, description: str) -> str:
        """Assess severity of detected anomaly"""
        if "structural" in description.lower() or "deformation" in description.lower():
            return "high"
        elif "damage" in description.lower():
            return "medium"
        else:
            return "low"
    
    def generate_maintenance_report(self, task: MaintenanceTask):
        """Generate maintenance completion report"""
        report = {
            'timestamp': datetime.now().isoformat(),
            'task_id': task.task_id,
            'task_name': task.name,
            'location': task.location,
            'completion_time': task.last_completed.isoformat(),
            'status': task.status,
            'findings': f"Task {task.name} completed successfully",
            'next_due': (task.last_completed + timedelta(hours=task.frequency_hours)).isoformat()
        }
        
        self.maintenance_history.append(report)
        
        # Publish maintenance report
        report_msg = String()
        report_msg.data = json.dumps(report, indent=2)
        self.maintenance_report_pub.publish(report_msg)
    
    def handle_emergency_response(self, emergency_alerts: List[Dict]):
        """Handle emergency response"""
        self.current_state = PatrolState.EMERGENCY_RESPONSE
        
        self.get_logger().error(f"🚨 Emergency response activated for {len(emergency_alerts)} alerts")
        
        # Navigate to emergency location (simplified)
        # In practice, would prioritize based on alert type and location
        emergency_location = (0.0, 0.0)  # Default to habitat center
        self.navigate_to_location(emergency_location)
    
    def analyze_structural_data(self, scan_msg):
        """Analyze LiDAR data for structural inspection"""
        # Simplified structural analysis
        ranges = np.array(scan_msg.ranges)
        valid_ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]
        
        if len(valid_ranges) > 0:
            # Look for unusual patterns that might indicate structural issues
            range_variance = np.var(valid_ranges)
            if range_variance > 1.0:  # High variance might indicate structural irregularities
                self.get_logger().info("Structural irregularity detected in scan data")
    
    def analyze_visual_data(self, image_msg):
        """Analyze camera data for visual inspection"""
        # Placeholder for visual analysis
        # In practice, would use computer vision algorithms
        self.get_logger().debug("Processing visual inspection data")
    
    def publish_status(self):
        """Publish patrol status"""
        status_data = {
            'timestamp': datetime.now().isoformat(),
            'state': self.current_state.value,
            'current_task': self.current_task.name if self.current_task else None,
            'route_progress': f"{self.current_route_index}/{len(self.patrol_route)}",
            'active_anomalies': len([a for a in self.anomaly_reports if 
                                   datetime.fromisoformat(a['timestamp']) > 
                                   datetime.now() - timedelta(hours=24)]),
            'completed_tasks_today': len([t for t in self.maintenance_history if 
                                        datetime.fromisoformat(t['timestamp']) > 
                                        datetime.now() - timedelta(days=1)]),
            'due_tasks': len([t for t in self.maintenance_tasks.values() if t.is_due()])
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.patrol_status_pub.publish(status_msg)
        
        # Publish task markers
        self.publish_task_markers()
    
    def publish_task_markers(self):
        """Publish visualization markers for maintenance tasks"""
        marker_array = MarkerArray()
        
        for i, task in enumerate(self.maintenance_tasks.values()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "maintenance_tasks"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = task.location[0]
            marker.pose.position.y = task.location[1]
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            # Size based on priority
            size = 0.2 + (task.priority * 0.1)
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            
            # Color based on urgency
            urgency = task.get_urgency_score()
            if urgency > 5.0:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif urgency > 3.0:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.8
            marker.lifetime.sec = 10
            
            marker_array.markers.append(marker)
        
        self.task_markers_pub.publish(marker_array)
    
    def create_maintenance_task_from_diagnostic(self, diagnostic_status):
        """Create maintenance task from diagnostic error"""
        # Simplified task creation from diagnostics
        task_id = f"diag_{diagnostic_status.name.replace(' ', '_').lower()}"
        
        if task_id not in self.maintenance_tasks:
            new_task = MaintenanceTask(
                task_id, f"Address: {diagnostic_status.name}", 
                (0.0, 0.0), 4, 1  # High priority, due in 1 hour
            )
            self.maintenance_tasks[task_id] = new_task
            self.get_logger().info(f"Created maintenance task from diagnostic: {diagnostic_status.name}")

def main(args=None):
    rclpy.init(args=args)
    
    maintenance_patrol = MaintenancePatrol()
    
    try:
        rclpy.spin(maintenance_patrol)
    except KeyboardInterrupt:
        pass
    finally:
        maintenance_patrol.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
