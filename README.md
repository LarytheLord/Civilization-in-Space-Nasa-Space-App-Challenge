# LunaBot: Autonomous Navigation Robot for Lunar Habitats

## Project Overview

LunaBot is an advanced autonomous robotic system designed for lunar habitat operations, developed for the NASA Space Apps Challenge 2025. This ROS2-based system demonstrates comprehensive capabilities for navigation, monitoring, and maintenance in challenging lunar environments.

## 🚀 Features

- **Autonomous Navigation**: SLAM-based mapping with multi-sensor fusion
- **Habitat Monitoring**: Environmental parameter tracking with alert systems
- **Maintenance Operations**: Automated patrol and inspection missions
- **AI Decision Making**: Cognitive AI integration for intelligent behavior
- **Habitat Site Selection**: Advanced analysis of potential lunar habitat locations
- **Real-time Dashboard**: Web-based visualization of robot data

## 📁 Project Structure

```
robot/
├── lunabot_ws/                 # ROS2 workspace
│   ├── src/
│   │   └── lunabot_navigation/ # Main ROS2 package
│   │       ├── scripts/        # Python ROS2 nodes
│   │       ├── launch/         # Launch files
│   │       ├── config/         # Configuration files
│   │       └── ...
│   ├── install_dependencies.py # Dependency installer
│   └── README.md
├── frontend/                   # React dashboard frontend
│   ├── src/
│   ├── package.json
│   └── ...
├── docker-compose.yml          # Deployment configuration
└── ...
```

## 🛠️ Setup Instructions

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.8+
- Webots R2023b
- Node.js 16+ (for frontend)

### Installation

1. **Clone the repository:**
```bash
git clone <repository-url>
cd robot
```

2. **Install ROS2 dependencies:**
```bash
cd lunabot_ws
python3 install_dependencies.py
```

3. **Build the workspace:**
```bash
colcon build --symlink-install
source install/setup.bash
```

4. **Install frontend dependencies:**
```bash
cd ../frontend
npm install
```

### Running the System

1. **Launch the complete system:**
```bash
cd lunabot_ws
source install/setup.bash
ros2 launch lunabot_navigation lunabot_complete.launch.py
```

2. **Start the frontend dashboard:**
```bash
cd frontend
npm start
```

The dashboard will be available at `http://localhost:3000`.

## 🏗️ Architecture

The system consists of several key components:

- **Navigation Controller**: Advanced path planning with lunar-specific optimizations
- **Sensor Fusion**: Integration of LiDAR, camera, and IMU data
- **Habitat Monitor**: Environmental parameter tracking
- **Maintenance Patrol**: Automated inspection routines
- **AI Decision Engine**: Cognitive system for autonomous decision making
- **Habitat Site Analyzer**: Evaluation of potential habitat locations
- **WebSocket Bridge**: Real-time data streaming to frontend
- **Web Dashboard**: Real-time visualization and monitoring

## 🎯 Habitat Site Selection

The new habitat site selection system analyzes potential lunar habitat locations using:

- **Safety Score**: Evaluates terrain roughness, slope, and radiation levels
- **Buildability Score**: Assesses terrain flatness and uniformity
- **Resource Score**: Estimates solar exposure and water ice potential
- **Expandability Score**: Measures available flat area for expansion

## 🚀 Running the Demo

To run the complete system with habitat site selection:

```bash
# Terminal 1: Start the ROS2 system
cd lunabot_ws
source install/setup.bash
ros2 launch lunabot_navigation lunabot_complete.launch.py

# Terminal 2: Start the frontend dashboard
cd frontend
npm start
```

## 📊 Data Flow

1. Robot sensors collect environmental data
2. Sensor fusion node processes multi-modal data
3. Habitat site analyzer evaluates potential sites
4. WebSocket bridge streams data to frontend
5. Dashboard visualizes real-time information

## 🐛 Troubleshooting

### Common Issues

**WebSocket connection fails:**
- Ensure websocket_bridge.py is running
- Check that the frontend is connecting to ws://localhost:8765

**ROS2 nodes not starting:**
- Verify ROS2 installation and sourcing of setup.bash
- Check for missing dependencies

## 🤝 Contributing

This project was developed for NASA Space Apps Challenge 2025. For improvements:

1. Fork the repository
2. Create feature branch
3. Implement changes with tests
4. Submit pull request with documentation

## 📄 License

This project is developed for NASA Space Apps Challenge 2025, supporting NASA's Artemis program and future lunar exploration missions.

## 🏆 Achievements

- Complete ROS2 integration with modular architecture
- Multi-sensor fusion for robust perception
- AI-powered decision making with learning capabilities
- Comprehensive simulation environment
- Real-time monitoring and alerting systems
- Autonomous mission execution with adaptive planning
- Advanced habitat site selection capabilities
- Real-time web-based dashboard

---

**Developed for NASA Space Apps Challenge 2025**  
**Challenge: Lunar Habitat Site Selection & Autonomous Exploration**  
**Organization: NASA (National Aeronautics and Space Administration)**  
**Theme: Space Exploration & Robotics**

🌙 **LunaBot - Pioneering Autonomous Lunar Operations** 🚀