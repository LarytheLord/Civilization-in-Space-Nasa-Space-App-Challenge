#!/usr/bin/env python3
"""
LunaBot WebSocket Bridge
Stream ROS2 data to frontend via WebSocket
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import asyncio
import websockets
import json
import threading

class WebSocketBridge(Node):
    def __init__(self):
        super().__init__('websocket_bridge')
        
        # Subscribers
        self.site_analysis_sub = self.create_subscription(
            String, '/site_analysis', self.site_analysis_callback, 10)
        self.habitat_status_sub = self.create_subscription(
            String, '/habitat_status', self.habitat_status_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Data buffers
        self.latest_data = {
            'sites': None,
            'habitat': None,
            'scan': None,
            'odom': None
        }
        
        # WebSocket clients
        self.clients = set()
        
        # Start WebSocket server in separate thread
        self.ws_thread = threading.Thread(target=self.start_websocket_server, daemon=True)
        self.ws_thread.start()
        
        self.get_logger().info("WebSocket Bridge initialized on ws://localhost:8765")

    def site_analysis_callback(self, msg):
        self.latest_data['sites'] = json.loads(msg.data)
        asyncio.run(self.broadcast('sites', self.latest_data['sites']))

    def habitat_status_callback(self, msg):
        self.latest_data['habitat'] = json.loads(msg.data)
        asyncio.run(self.broadcast('habitat', self.latest_data['habitat']))

    def scan_callback(self, msg):
        # Downsample scan data for WebSocket
        self.latest_data['scan'] = {
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': msg.ranges[::10]  # Every 10th point
        }

    def odom_callback(self, msg):
        self.latest_data['odom'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': 0  # Extract from quaternion if needed
        }

    async def broadcast(self, data_type, data):
        if self.clients:
            message = json.dumps({'type': data_type, 'data': data})
            await asyncio.gather(
                *[client.send(message) for client in self.clients],
                return_exceptions=True
            )

    async def handler(self, websocket, path):
        self.clients.add(websocket)
        try:
            # Send initial data
            await websocket.send(json.dumps({
                'type': 'init',
                'data': self.latest_data
            }))
            
            # Keep connection alive
            async for message in websocket:
                pass  # Handle client commands if needed
        finally:
            self.clients.remove(websocket)

    def start_websocket_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        start_server = websockets.serve(self.handler, "localhost", 8765)
        loop.run_until_complete(start_server)
        loop.run_forever()

def main(args=None):
    rclpy.init(args=args)
    bridge = WebSocketBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()