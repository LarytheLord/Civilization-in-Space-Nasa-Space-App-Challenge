# LunaBot: Autonomous Navigation Robot for Lunar Habitats

## 🌙 Project Overview

LunaBot is an advanced autonomous robotic system designed for lunar habitat operations, developed for the NASA Space Apps Challenge 2025. This ROS2-based system demonstrates comprehensive capabilities for navigation, monitoring, and maintenance in challenging lunar environments.

### 🎯 NASA Space Apps Challenge
Design and develop a ROS-based autonomous robot prototype to support NASA's Artemis program, capable of:
- Navigating lunar habitat environments for site selection and monitoring
- Mapping and localizing using sensor fusion (LiDAR, cameras, IMU)
- Detecting obstacles and hazards for safe path planning
- Monitoring environmental parameters for habitat suitability
- Autonomous site analysis and habitat location selection

## 🚀 Key Features

### 🤖 Autonomous Navigation
- **SLAM-based mapping** using LiDAR and visual sensors
- **Multi-sensor fusion** combining LiDAR, camera, and IMU data
- **Dynamic obstacle avoidance** with real-time path replanning
- **Lunar gravity simulation** (1.62 m/s²) for realistic movement

### 🏠 Habitat Monitoring
- **Environmental parameter tracking** (temperature, pressure, O2, CO2)
- **Real-time anomaly detection** with severity classification
- **Automated alert system** with emergency protocols
- **Multi-zone monitoring** for different habitat areas

### 🔧 Maintenance Operations
- **Autonomous patrol missions** with scheduled inspections
- **Visual and structural inspection** capabilities
- **Predictive maintenance** based on sensor data analysis
- **Maintenance task prioritization** and scheduling

### 🧠 AI-Powered Decision Making
- **Cognitive AI integration** using existing Prometheus system
- **Adaptive mission planning** based on environmental conditions
- **Learning from experience** with decision effectiveness analysis
- **Emergency response protocols** with autonomous decision making

## 📁 System Architecture

```
lunabot_ws/
├── src/lunabot_navigation/
│   ├── config/                 # Configuration files
│   │   ├── nav2_params.yaml   # Navigation parameters
│   │   ├── mapper_params_online_async.yaml
│   │   └── lunabot_view.rviz  # RViz visualization config
│   ├── launch/                # Launch files
│   │   ├── lunabot_complete.launch.py    # Complete system
│   │   ├── lunabot_simulation.launch.py  # Simulation only
│   │   └── navigation.launch.py          # Navigation stack
│   ├── scripts/               # Python nodes
│   │   ├── sensor_fusion_node.py        # Multi-sensor fusion
│   │   ├── navigation_controller.py     # Advanced navigation
│   │   ├── habitat_monitor.py           # Environmental monitoring
│   │   ├── maintenance_patrol.py        # Patrol and maintenance
│   │   ├── ai_decision_engine.py        # AI integration
│   │   └── demo_controller.py           # Automated demo
│   ├── urdf/                  # Robot models
│   │   └── lunabot.urdf.xacro # Robot description
│   ├── worlds/                # Simulation environments
│   │   └── lunar_habitat.world # Lunar habitat simulation
│   └── maps/                  # Pre-built maps
├── install_dependencies.py    # Dependency installer
└── README.md                 # This file
```

## 🛠️ Installation & Setup

### Prerequisites
- **Ubuntu 22.04 LTS**
- **ROS2 Humble** (or compatible distribution)
- **Python 3.8+**
- **Webots R2023b** (for simulation)
- **4GB+ RAM** recommended

### 🚀 **SYSTEM STATUS: READY FOR DEMO** 🚀
All major components completed and integrated! See `PROJECT_STATUS.md` for detailed status.

### Quick Start

1. **Clone the repository:**
```bash
cd ~/
git clone <repository-url>
cd lunabot_ws
```

2. **Install dependencies:**
```bash
python3 install_dependencies.py
```

3. **Build the workspace:**
```bash
colcon build --symlink-install
source install/setup.bash
```

4. **Launch complete system:**
```bash
ros2 launch lunabot_navigation lunabot_complete.launch.py
```

### Manual Installation

If automatic installation fails, install dependencies manually:

```bash
# ROS2 packages
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-xacro

# Python packages
pip3 install numpy scipy opencv-python matplotlib scikit-learn pandas pyyaml transforms3d
```

## 🎮 Usage

### Complete Demo
Launch the full automated demonstration:
```bash
ros2 launch lunabot_navigation lunabot_complete.launch.py
```

### Individual Components

**Simulation Only:**
```bash
ros2 launch lunabot_navigation lunabot_simulation.launch.py
```

**Navigation Stack:**
```bash
ros2 launch lunabot_navigation navigation.launch.py
```

**Individual Nodes:**
```bash
# Sensor fusion
ros2 run lunabot_navigation sensor_fusion_node.py

# Habitat monitoring
ros2 run lunabot_navigation habitat_monitor.py

# Maintenance patrol
ros2 run lunabot_navigation maintenance_patrol.py

# AI decision engine
ros2 run lunabot_navigation ai_decision_engine.py
```

### Manual Control

**Start patrol mission:**
```bash
ros2 topic pub /patrol_command std_msgs/String "data: 'start_patrol'"
```

**Navigate to specific location:**
```bash
ros2 topic pub /mission_command std_msgs/String "data: 'goto 5.0 3.0'"
```

**Emergency stop:**
```bash
ros2 topic pub /mission_command std_msgs/String "data: 'emergency_stop'"
```

## 📊 Monitoring & Visualization

### RViz Visualization
The system includes comprehensive RViz configuration showing:
- **Robot model** and sensor data
- **Real-time mapping** and localization
- **Obstacle detection** and path planning
- **Habitat zones** and monitoring points
- **Maintenance tasks** and patrol routes

### System Status Topics
Monitor system status through ROS topics:

```bash
# Navigation status
ros2 topic echo /navigation_status

# Habitat monitoring
ros2 topic echo /habitat_status

# Patrol status
ros2 topic echo /patrol_status

# AI decisions
ros2 topic echo /ai_decisions

# Demo progress
ros2 topic echo /demo_status
```

## 🧪 Demo Scenarios

The automated demo showcases:

1. **System Initialization** (30s)
   - Component startup and calibration
   - Sensor initialization and health checks

2. **Autonomous Navigation** (120s)
   - Waypoint navigation with obstacle avoidance
   - Dynamic path replanning

3. **SLAM Mapping** (90s)
   - Real-time environment mapping
   - Localization in unknown environment

4. **Habitat Monitoring** (60s)
   - Environmental parameter monitoring
   - Anomaly detection and alerting

5. **Maintenance Patrol** (150s)
   - Systematic habitat inspection
   - Equipment status verification

6. **AI Decision Making** (60s)
   - Autonomous mission planning
   - Adaptive behavior based on conditions

7. **Emergency Response** (30s)
   - Emergency detection and response
   - Safety protocol execution

## 🔧 Configuration

### Navigation Parameters
Edit `config/nav2_params.yaml` to adjust:
- **Velocity limits** for lunar gravity
- **Obstacle detection** sensitivity
- **Path planning** algorithms

### Habitat Monitoring
Modify `habitat_monitor.py` to configure:
- **Environmental thresholds** for alerts
- **Monitoring zones** and locations
- **Alert severity** levels

### AI Decision Engine
Customize `ai_decision_engine.py` for:
- **Decision confidence** thresholds
- **Learning parameters**
- **Emergency response** protocols

## 🚨 Troubleshooting

### Common Issues

**Gazebo fails to start:**
```bash
# Check Gazebo installation
gazebo --version

# Reset Gazebo configuration
rm -rf ~/.gazebo
```

**Navigation not working:**
```bash
# Verify transform tree
ros2 run tf2_tools view_frames

# Check sensor topics
ros2 topic list | grep scan
```

**AI system not responding:**
```bash
# Check AI dependencies
python3 -c "import fastapi, uvicorn"

# Verify AI topic connections
ros2 topic echo /ai_status
```

### Performance Optimization

**For better performance:**
- Reduce simulation quality in Gazebo
- Limit RViz visualization elements
- Adjust sensor update rates
- Use faster computer or reduce complexity

## 📈 Performance Metrics

Expected system performance:
- **Navigation accuracy:** ±0.1m in simulation
- **Obstacle detection range:** 0.1-30m
- **Environmental monitoring:** 1Hz update rate
- **AI decision latency:** <2 seconds
- **System uptime:** >99% in normal conditions

## 🤝 Contributing

This project was developed for NASA Space Apps Challenge 2025. For improvements:

1. Fork the repository
2. Create feature branch
3. Implement changes with tests
4. Submit pull request with documentation

## 📄 License

This project is developed for NASA Space Apps Challenge 2025, supporting NASA's Artemis program and future lunar exploration missions.

## 🏆 Achievements

### Technical Accomplishments
- ✅ **Complete ROS2 integration** with modular architecture
- ✅ **Multi-sensor fusion** for robust perception
- ✅ **AI-powered decision making** with learning capabilities
- ✅ **Comprehensive simulation** environment
- ✅ **Real-time monitoring** and alerting systems
- ✅ **Autonomous mission execution** with adaptive planning

### Innovation Highlights
- **Lunar-specific adaptations** (gravity, dust, radiation)
- **Integrated AI cognitive system** for intelligent behavior
- **Predictive maintenance** scheduling
- **Emergency response protocols** with autonomous decision making
- **Comprehensive demonstration** system for validation

## 📞 Support

For technical support or questions:
- Review troubleshooting section
- Check ROS2 documentation
- Verify system requirements
- Test individual components

## 🎯 Future Enhancements

Potential improvements for production deployment:
- **Hardware integration** with real sensors
- **Advanced AI models** for complex decision making
- **Multi-robot coordination** for team operations
- **Earth-Moon communication** simulation
- **Extended mission scenarios** and testing

---

**Developed for NASA Space Apps Challenge 2025**  
**Challenge: Lunar Habitat Site Selection & Autonomous Exploration**  
**Organization: NASA (National Aeronautics and Space Administration)**  
**Supporting: NASA Artemis Program**

🌙 **LunaBot - Pioneering Autonomous Lunar Operations** 🚀
