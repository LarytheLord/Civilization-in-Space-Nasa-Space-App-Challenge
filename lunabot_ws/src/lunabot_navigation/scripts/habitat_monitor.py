#!/usr/bin/env python3
"""
LunaBot Habitat Monitoring System
Monitors environmental parameters and habitat integrity for lunar base operations
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import time
from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Tuple

# ROS2 message types
from std_msgs.msg import String, Float32, Bool, Header
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Temperature, FluidPressure
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import MarkerArray, Marker

class AlertLevel(Enum):
    NORMAL = 0
    WARNING = 1
    CRITICAL = 2
    EMERGENCY = 3

class EnvironmentalParameter:
    """Represents an environmental parameter with thresholds and history"""
    def __init__(self, name: str, unit: str, normal_range: Tuple[float, float], 
                 critical_range: Tuple[float, float]):
        self.name = name
        self.unit = unit
        self.normal_min, self.normal_max = normal_range
        self.critical_min, self.critical_max = critical_range
        self.current_value = 0.0
        self.history = []
        self.last_update = time.time()
        self.alert_level = AlertLevel.NORMAL
    
    def update_value(self, value: float):
        """Update parameter value and assess alert level"""
        self.current_value = value
        self.last_update = time.time()
        self.history.append((self.last_update, value))
        
        # Keep only last 100 readings
        if len(self.history) > 100:
            self.history.pop(0)
        
        # Assess alert level
        if value < self.critical_min or value > self.critical_max:
            self.alert_level = AlertLevel.EMERGENCY
        elif value < self.normal_min or value > self.normal_max:
            self.alert_level = AlertLevel.WARNING
        else:
            self.alert_level = AlertLevel.NORMAL
    
    def get_trend(self) -> str:
        """Calculate trend from recent history"""
        if len(self.history) < 5:
            return "insufficient_data"
        
        recent_values = [h[1] for h in self.history[-5:]]
        if recent_values[-1] > recent_values[0] * 1.05:
            return "increasing"
        elif recent_values[-1] < recent_values[0] * 0.95:
            return "decreasing"
        else:
            return "stable"

class HabitatZone:
    """Represents a monitored zone within the habitat"""
    def __init__(self, zone_id: str, name: str, position: Tuple[float, float, float]):
        self.zone_id = zone_id
        self.name = name
        self.position = position
        self.parameters = {}
        self.last_inspection = 0.0
        self.status = "operational"
        self.alerts = []

class HabitatMonitor(Node):
    """
    Comprehensive habitat monitoring system for lunar base
    Monitors air quality, pressure, temperature, power, and structural integrity
    """
    
    def __init__(self):
        super().__init__('habitat_monitor')
        
        # Initialize monitoring zones
        self.zones = self.initialize_habitat_zones()
        
        # Initialize environmental parameters
        self.initialize_environmental_parameters()
        
        # Alert system
        self.active_alerts = []
        self.alert_history = []
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers (simulated sensor inputs)
        self.create_subscription(Float32, '/sensors/temperature', self.temperature_callback, sensor_qos)
        self.create_subscription(Float32, '/sensors/pressure', self.pressure_callback, sensor_qos)
        self.create_subscription(Float32, '/sensors/oxygen_level', self.oxygen_callback, sensor_qos)
        self.create_subscription(Float32, '/sensors/co2_level', self.co2_callback, sensor_qos)
        self.create_subscription(Float32, '/sensors/humidity', self.humidity_callback, sensor_qos)
        self.create_subscription(Float32, '/sensors/power_level', self.power_callback, sensor_qos)
        self.create_subscription(Float32, '/sensors/radiation', self.radiation_callback, sensor_qos)
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.alerts_pub = self.create_publisher(String, '/habitat_alerts', 10)
        self.status_pub = self.create_publisher(String, '/habitat_status', 10)
        self.emergency_pub = self.create_publisher(Bool, '/habitat_emergency', 10)
        self.monitoring_markers_pub = self.create_publisher(MarkerArray, '/monitoring_zones', 10)
        
        # Timers
        self.monitoring_timer = self.create_timer(1.0, self.monitoring_loop)
        self.diagnostics_timer = self.create_timer(5.0, self.publish_diagnostics)
        self.sensor_simulation_timer = self.create_timer(2.0, self.simulate_sensors)
        
        # Data logging
        self.log_file = f"/tmp/habitat_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        self.get_logger().info("🏠 LunaBot Habitat Monitor initialized")
        self.get_logger().info(f"Monitoring {len(self.zones)} habitat zones")
    
    def initialize_habitat_zones(self) -> Dict[str, HabitatZone]:
        """Initialize habitat monitoring zones"""
        zones = {}
        
        # Main habitat modules
        zones['module_1'] = HabitatZone('module_1', 'Habitat Module 1', (5.0, 0.0, 1.0))
        zones['module_2'] = HabitatZone('module_2', 'Habitat Module 2', (-5.0, 0.0, 1.0))
        zones['corridor'] = HabitatZone('corridor', 'Connecting Corridor', (0.0, 0.0, 0.5))
        zones['airlock'] = HabitatZone('airlock', 'Main Airlock', (0.0, -5.0, 0.5))
        zones['power_station'] = HabitatZone('power_station', 'Power Station', (10.0, 0.0, 1.0))
        zones['life_support'] = HabitatZone('life_support', 'Life Support Bay', (0.0, 5.0, 1.0))
        
        return zones
    
    def initialize_environmental_parameters(self):
        """Initialize environmental parameter definitions"""
        self.env_parameters = {
            'temperature': EnvironmentalParameter(
                'Temperature', '°C', (18.0, 24.0), (10.0, 35.0)
            ),
            'pressure': EnvironmentalParameter(
                'Atmospheric Pressure', 'kPa', (95.0, 105.0), (80.0, 120.0)
            ),
            'oxygen': EnvironmentalParameter(
                'Oxygen Level', '%', (19.0, 23.0), (16.0, 25.0)
            ),
            'co2': EnvironmentalParameter(
                'CO2 Level', 'ppm', (300.0, 1000.0), (0.0, 5000.0)
            ),
            'humidity': EnvironmentalParameter(
                'Humidity', '%', (30.0, 60.0), (10.0, 80.0)
            ),
            'power': EnvironmentalParameter(
                'Power Level', '%', (80.0, 100.0), (20.0, 100.0)
            ),
            'radiation': EnvironmentalParameter(
                'Radiation Level', 'mSv/h', (0.0, 0.1), (0.0, 1.0)
            )
        }
    
    def temperature_callback(self, msg):
        """Handle temperature sensor data"""
        self.env_parameters['temperature'].update_value(msg.data)
    
    def pressure_callback(self, msg):
        """Handle pressure sensor data"""
        self.env_parameters['pressure'].update_value(msg.data)
    
    def oxygen_callback(self, msg):
        """Handle oxygen sensor data"""
        self.env_parameters['oxygen'].update_value(msg.data)
    
    def co2_callback(self, msg):
        """Handle CO2 sensor data"""
        self.env_parameters['co2'].update_value(msg.data)
    
    def humidity_callback(self, msg):
        """Handle humidity sensor data"""
        self.env_parameters['humidity'].update_value(msg.data)
    
    def power_callback(self, msg):
        """Handle power level data"""
        self.env_parameters['power'].update_value(msg.data)
    
    def radiation_callback(self, msg):
        """Handle radiation sensor data"""
        self.env_parameters['radiation'].update_value(msg.data)
    
    def simulate_sensors(self):
        """Simulate sensor readings for demonstration"""
        current_time = time.time()
        
        # Simulate realistic variations with occasional anomalies
        base_temp = 21.0 + 2.0 * np.sin(current_time / 100.0)  # Slow temperature cycle
        temp_noise = np.random.normal(0, 0.5)
        
        # Occasionally simulate anomalies
        if np.random.random() < 0.02:  # 2% chance of anomaly
            if np.random.random() < 0.5:
                temp_noise += np.random.uniform(-5, -3)  # Cold anomaly
            else:
                temp_noise += np.random.uniform(3, 8)   # Hot anomaly
        
        simulated_temp = base_temp + temp_noise
        
        # Simulate other parameters
        simulated_pressure = 101.3 + np.random.normal(0, 1.0)
        simulated_oxygen = 21.0 + np.random.normal(0, 0.5)
        simulated_co2 = 400.0 + np.random.normal(0, 50.0)
        simulated_humidity = 45.0 + np.random.normal(0, 5.0)
        simulated_power = 95.0 + np.random.normal(0, 2.0)
        simulated_radiation = 0.05 + np.random.normal(0, 0.01)
        
        # Publish simulated data
        self.publish_sensor_data('/sensors/temperature', simulated_temp)
        self.publish_sensor_data('/sensors/pressure', simulated_pressure)
        self.publish_sensor_data('/sensors/oxygen_level', simulated_oxygen)
        self.publish_sensor_data('/sensors/co2_level', simulated_co2)
        self.publish_sensor_data('/sensors/humidity', simulated_humidity)
        self.publish_sensor_data('/sensors/power_level', simulated_power)
        self.publish_sensor_data('/sensors/radiation', simulated_radiation)
    
    def publish_sensor_data(self, topic: str, value: float):
        """Publish simulated sensor data"""
        # This would normally come from actual sensors
        # For simulation, we create the publishers dynamically
        if not hasattr(self, '_sim_publishers'):
            self._sim_publishers = {}
        
        if topic not in self._sim_publishers:
            self._sim_publishers[topic] = self.create_publisher(Float32, topic, 10)
        
        msg = Float32()
        msg.data = value
        self._sim_publishers[topic].publish(msg)
    
    def monitoring_loop(self):
        """Main monitoring loop"""
        # Check for alerts
        self.check_environmental_alerts()
        
        # Update zone status
        self.update_zone_status()
        
        # Publish status
        self.publish_habitat_status()
        
        # Log data
        self.log_environmental_data()
    
    def check_environmental_alerts(self):
        """Check all environmental parameters for alert conditions"""
        new_alerts = []
        
        for param_name, param in self.env_parameters.items():
            if param.alert_level != AlertLevel.NORMAL:
                alert = {
                    'timestamp': datetime.now().isoformat(),
                    'parameter': param_name,
                    'value': param.current_value,
                    'unit': param.unit,
                    'level': param.alert_level.name,
                    'trend': param.get_trend(),
                    'message': self.generate_alert_message(param_name, param)
                }
                new_alerts.append(alert)
        
        # Update active alerts
        self.active_alerts = new_alerts
        
        # Check for emergency conditions
        emergency_alerts = [a for a in new_alerts if a['level'] == 'EMERGENCY']
        if emergency_alerts:
            self.handle_emergency_condition(emergency_alerts)
        
        # Publish alerts
        if new_alerts:
            self.publish_alerts(new_alerts)
    
    def generate_alert_message(self, param_name: str, param: EnvironmentalParameter) -> str:
        """Generate human-readable alert message"""
        trend_text = {
            'increasing': 'and rising',
            'decreasing': 'and falling', 
            'stable': 'stable',
            'insufficient_data': ''
        }
        
        if param.alert_level == AlertLevel.WARNING:
            return f"{param.name} is {param.current_value:.1f} {param.unit} (outside normal range) {trend_text.get(param.get_trend(), '')}"
        elif param.alert_level == AlertLevel.EMERGENCY:
            return f"CRITICAL: {param.name} is {param.current_value:.1f} {param.unit} (critical level) {trend_text.get(param.get_trend(), '')}"
        
        return f"{param.name}: {param.current_value:.1f} {param.unit}"
    
    def handle_emergency_condition(self, emergency_alerts: List[Dict]):
        """Handle emergency environmental conditions"""
        self.get_logger().error(f"🚨 HABITAT EMERGENCY: {len(emergency_alerts)} critical parameters")
        
        # Publish emergency signal
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        # Log emergency
        for alert in emergency_alerts:
            self.get_logger().error(f"EMERGENCY: {alert['message']}")
        
        # Could trigger automated responses:
        # - Activate backup life support
        # - Seal habitat sections
        # - Alert Earth mission control
        # - Initiate evacuation procedures
    
    def update_zone_status(self):
        """Update status of habitat zones"""
        for zone_id, zone in self.zones.items():
            # Assess zone health based on environmental parameters
            zone_alerts = [a for a in self.active_alerts if self.is_alert_relevant_to_zone(a, zone)]
            
            if any(a['level'] == 'EMERGENCY' for a in zone_alerts):
                zone.status = 'emergency'
            elif any(a['level'] == 'WARNING' for a in zone_alerts):
                zone.status = 'warning'
            else:
                zone.status = 'operational'
            
            zone.alerts = zone_alerts
    
    def is_alert_relevant_to_zone(self, alert: Dict, zone: HabitatZone) -> bool:
        """Determine if an alert is relevant to a specific zone"""
        # For simulation, assume all alerts affect all zones
        # In reality, this would depend on sensor locations and zone characteristics
        return True
    
    def publish_habitat_status(self):
        """Publish overall habitat status"""
        status_data = {
            'timestamp': datetime.now().isoformat(),
            'overall_status': self.calculate_overall_status(),
            'active_alerts': len(self.active_alerts),
            'zones': {
                zone_id: {
                    'status': zone.status,
                    'alerts': len(zone.alerts)
                }
                for zone_id, zone in self.zones.items()
            },
            'environmental_parameters': {
                name: {
                    'value': param.current_value,
                    'unit': param.unit,
                    'status': param.alert_level.name,
                    'trend': param.get_trend()
                }
                for name, param in self.env_parameters.items()
            }
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.status_pub.publish(status_msg)
    
    def calculate_overall_status(self) -> str:
        """Calculate overall habitat status"""
        if any(a['level'] == 'EMERGENCY' for a in self.active_alerts):
            return 'EMERGENCY'
        elif any(a['level'] == 'WARNING' for a in self.active_alerts):
            return 'WARNING'
        else:
            return 'OPERATIONAL'
    
    def publish_alerts(self, alerts: List[Dict]):
        """Publish current alerts"""
        alerts_msg = String()
        alerts_msg.data = json.dumps(alerts, indent=2)
        self.alerts_pub.publish(alerts_msg)
    
    def publish_diagnostics(self):
        """Publish ROS diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Overall habitat status
        overall_status = DiagnosticStatus()
        overall_status.name = "Habitat Overall Status"
        overall_status.message = self.calculate_overall_status()
        
        if self.calculate_overall_status() == 'OPERATIONAL':
            overall_status.level = DiagnosticStatus.OK
        elif self.calculate_overall_status() == 'WARNING':
            overall_status.level = DiagnosticStatus.WARN
        else:
            overall_status.level = DiagnosticStatus.ERROR
        
        diag_array.status.append(overall_status)
        
        # Individual parameter diagnostics
        for name, param in self.env_parameters.items():
            status = DiagnosticStatus()
            status.name = f"Environmental: {param.name}"
            status.message = f"{param.current_value:.2f} {param.unit}"
            
            if param.alert_level == AlertLevel.NORMAL:
                status.level = DiagnosticStatus.OK
            elif param.alert_level == AlertLevel.WARNING:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR
            
            # Add key-value pairs
            status.values.append(KeyValue(key="value", value=str(param.current_value)))
            status.values.append(KeyValue(key="unit", value=param.unit))
            status.values.append(KeyValue(key="trend", value=param.get_trend()))
            
            diag_array.status.append(status)
        
        self.diagnostics_pub.publish(diag_array)
        
        # Publish zone markers
        self.publish_zone_markers()
    
    def publish_zone_markers(self):
        """Publish visualization markers for habitat zones"""
        marker_array = MarkerArray()
        
        for i, (zone_id, zone) in enumerate(self.zones.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "habitat_zones"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = zone.position[0]
            marker.pose.position.y = zone.position[1]
            marker.pose.position.z = zone.position[2]
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 0.5
            
            # Color based on zone status
            if zone.status == 'operational':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif zone.status == 'warning':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:  # emergency
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
            marker.color.a = 0.3
            marker.lifetime.sec = 10
            
            marker_array.markers.append(marker)
            
            # Add text marker for zone name
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "zone_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = zone.position[0]
            text_marker.pose.position.y = zone.position[1]
            text_marker.pose.position.z = zone.position[2] + 1.0
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.5
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{zone.name}\n{zone.status.upper()}"
            text_marker.lifetime.sec = 10
            
            marker_array.markers.append(text_marker)
        
        self.monitoring_markers_pub.publish(marker_array)
    
    def log_environmental_data(self):
        """Log environmental data to file"""
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'parameters': {
                name: {
                    'value': param.current_value,
                    'alert_level': param.alert_level.name
                }
                for name, param in self.env_parameters.items()
            },
            'alerts': self.active_alerts
        }
        
        try:
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
        except Exception as e:
            self.get_logger().warn(f"Failed to log data: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    habitat_monitor = HabitatMonitor()
    
    try:
        rclpy.spin(habitat_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        habitat_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
