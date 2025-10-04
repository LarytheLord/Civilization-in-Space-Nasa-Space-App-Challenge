#!/usr/bin/env python3
"""
LunaBot Habitat Site Analyzer
Analyzes terrain data from existing sensors to score potential habitat sites
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import json
from collections import deque

class HabitatSiteAnalyzer(Node):
    def __init__(self):
        super().__init__('habitat_site_analyzer')
        
        # Subscribers - use EXISTING topics
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.habitat_sub = self.create_subscription(
            String, '/habitat_status', self.habitat_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers - NEW topics
        self.sites_pub = self.create_publisher(
            MarkerArray, '/habitat_sites', 10)
        self.analysis_pub = self.create_publisher(
            String, '/site_analysis', 10)
        
        # Data storage
        self.terrain_map = np.zeros((100, 100))  # 50m x 50m @ 0.5m resolution
        self.robot_position = (0, 0)
        self.habitat_data = {}
        self.scan_history = deque(maxlen=100)
        
        # Analysis parameters
        self.grid_size = 0.5  # meters
        self.analysis_radius = 25  # meters
        
        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(5.0, self.analyze_sites)
        
        self.get_logger().info("Habitat Site Analyzer initialized")

    def scan_callback(self, msg):
        """Process LiDAR scan to build terrain map"""
        self.scan_history.append(msg)
        
        # Update terrain map using scan data
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        for i, (r, angle) in enumerate(zip(ranges, angles)):
            if np.isnan(r) or np.isinf(r):
                continue
            
            # Convert to Cartesian relative to robot
            x = r * np.cos(angle) + self.robot_position[0]
            y = r * np.sin(angle) + self.robot_position[1]
            
            # Update terrain map (height approximation from range variance)
            grid_x = int((x + 25) / self.grid_size)
            grid_y = int((y + 25) / self.grid_size)
            
            if 0 <= grid_x < 100 and 0 <= grid_y < 100:
                self.terrain_map[grid_x, grid_y] = r

    def habitat_callback(self, msg):
        """Store habitat environmental data"""
        try:
            self.habitat_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def odom_callback(self, msg):
        """Update robot position"""
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def analyze_sites(self):
        """Main analysis loop - score potential sites"""
        sites = []
        
        # Analyze grid cells in a pattern around robot
        for dx in range(-20, 21, 5):  # Every 2.5m
            for dy in range(-20, 21, 5):
                x = self.robot_position[0] + dx
                y = self.robot_position[1] + dy
                
                site = self.analyze_single_site(x, y)
                if site['total_score'] > 50:  # Only consider decent sites
                    sites.append(site)

        # Sort by score
        sites.sort(key=lambda s: s['total_score'], reverse=True)
        
        # Publish top 5
        self.publish_site_markers(sites[:5])
        self.publish_site_analysis(sites[:5])

    def analyze_single_site(self, x, y):
        """Analyze a single location"""
        # Extract local terrain (5m x 5m around point)
        grid_x = int((x + 25) / self.grid_size)
        grid_y = int((y + 25) / self.grid_size)
        
        local_terrain = self.terrain_map[
            max(0, grid_x-10):min(100, grid_x+10),
            max(0, grid_y-10):min(100, grid_y+10)
        ]
        
        # Calculate scores
        safety = self.calculate_safety_score(local_terrain)
        buildability = self.calculate_buildability_score(local_terrain)
        resources = self.calculate_resource_score(x, y)
        expandability = self.calculate_expandability_score(local_terrain)
        
        total = (safety * 0.35 + buildability * 0.25 + 
                resources * 0.20 + expandability * 0.20)
        
        return {
            'position': {'x': x, 'y': y},
            'scores': {
                'safety': safety,
                'buildability': buildability,
                'resources': resources,
                'expandability': expandability
            },
            'total_score': total
        }

    def calculate_safety_score(self, terrain):
        """Safety: low slope, low roughness"""
        if terrain.size == 0:
            return 50.0
        
        # Terrain roughness (variance)
        roughness = np.std(terrain)
        roughness_score = max(0, 100 - roughness * 50)
        
        # Slope (gradient magnitude)
        if terrain.shape[0] > 1 and terrain.shape[1] > 1:
            gy, gx = np.gradient(terrain)
            slope = np.mean(np.sqrt(gx**2 + gy**2))
            slope_score = max(0, 100 - slope * 100)
        else:
            slope_score = 50

        # Radiation from habitat_data
        radiation = self.habitat_data.get('environmental_parameters', {}).get('radiation', 50)
        radiation_score = max(0, 100 - radiation)
        
        return (roughness_score + slope_score + radiation_score) / 3

    def calculate_buildability_score(self, terrain):
        """Buildability: flat, uniform terrain"""
        if terrain.size == 0:
            return 50.0
        
        # Flatness
        flatness = 100 - min(100, np.std(terrain) * 100)
        
        # Uniformity
        uniformity = 100 - min(100, np.ptp(terrain) * 50)
        
        return (flatness + uniformity) / 2

    def calculate_resource_score(self, x, y):
        """Resources: simulated based on location"""
        # Simulate solar exposure (better near poles)
        solar = 50 + np.random.uniform(0, 50)
        
        # Simulate water ice potential (random for now)
        water_ice = np.random.uniform(30, 100)
        
        return (solar + water_ice) / 2

    def calculate_expandability_score(self, terrain):
        """Expandability: available flat area"""
        if terrain.size == 0:
            return 50.0
        
        # Count flat cells (low variance)
        flat_cells = np.sum(np.abs(terrain - np.mean(terrain)) < 0.5)
        expandability = (flat_cells / terrain.size) * 100
        
        return expandability

    def publish_site_markers(self, sites):
        """Publish visualization markers"""
        marker_array = MarkerArray()

        for i, site in enumerate(sites):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "habitat_sites"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = site['position']['x']
            marker.pose.position.y = site['position']['y']
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            # Size based on score
            size = 1.0 + (site['total_score'] / 100) * 2.0
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 1.0
            
            # Color based on score
            if site['total_score'] >= 80:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif site['total_score'] >= 60:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker.lifetime.sec = 10
            marker_array.markers.append(marker)

        self.sites_pub.publish(marker_array)

    def publish_site_analysis(self, sites):
        """Publish JSON analysis"""
        analysis = {
            'timestamp': self.get_clock().now().to_msg(),
            'robot_position': {
                'x': self.robot_position[0],
                'y': self.robot_position[1]
            },
            'sites': sites
        }
        
        msg = String()
        msg.data = json.dumps(analysis, indent=2)
        self.analysis_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    analyzer = HabitatSiteAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()