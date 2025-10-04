# LunaBot Habitat Site Selection Dashboard

This is the frontend dashboard for the LunaBot habitat site selection system. It provides real-time visualization of habitat analysis data from the robot.

## Features

- Real-time habitat site analysis
- Environmental sensor monitoring
- 3D terrain visualization
- Site comparison tools
- WebSocket-based data streaming

## Setup

1. Install dependencies:
```bash
npm install
```

2. Start the development server:
```bash
npm start
```

The dashboard will be available at `http://localhost:3000`.

## Integration with ROS2 Backend

The frontend connects to the ROS2 backend via WebSocket at `ws://localhost:8765`. Make sure the `websocket_bridge.py` node is running before starting the frontend.

## Components

- **Site Analysis Panel**: Shows scores for potential habitat sites
- **Sensor Dashboard**: Real-time environmental data
- **Site Comparison**: Radar chart comparing top sites
- **3D Terrain View**: Visual representation of sites and robot position

## Technologies Used

- React 18
- TypeScript
- Recharts for data visualization
- WebSocket for real-time data