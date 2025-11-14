# Minesweeper Robot

> **Assiut Robotics Team Project**

An autonomous robot system designed for mine detection and navigation tasks. This project integrates computer vision, ROS (Robot Operating System), and custom robotics code to create an intelligent minesweeper robot.

## 📋 Project Overview

The Minesweeper Robot is an advanced robotics project that combines autonomous navigation, object detection, and strategic path planning to identify and navigate around simulated mines in a competition environment.

## ✨ Key Features

- **Autonomous Navigation**: Self-directed movement using ROS navigation stack
- **Computer Vision**: Real-time mine detection using camera systems
- **Path Planning**: Intelligent route calculation to avoid detected obstacles
- **ROS Integration**: Modular architecture using ROS for scalability
- **Multi-Sensor Fusion**: Combines vision and sensor data for accurate detection
- **Competition-Ready**: Designed for robotics competitions and challenges

## 🛠️ Technologies Used

- **ROS (Robot Operating System)**: For robot control and communication
- **Python**: Computer vision and high-level control logic
- **C/C++**: Low-level hardware interfacing and real-time processing
- **OpenCV**: Image processing and object detection
- **Sensor Suite**: 
  - Camera systems for visual detection
  - Ultrasonic/IR sensors for proximity detection
  - IMU for orientation tracking

## 📁 Project Structure

```
Minesweeper/
├── Project Folders/
│   ├── Documentation
│   ├── CAD files
│   └── Schematics
├── ROS Codes/
│   ├── Navigation nodes
│   ├── Control systems
│   └── Launch files
└── Vision Codes/
    ├── Image processing
    ├── Object detection
    └── Classification models
```

## 🚀 Getting Started

### Prerequisites

- **ROS** (Melodic/Noetic or later)
- **Python 3.x** with OpenCV
- **Ubuntu 18.04/20.04** (recommended for ROS)
- **Camera drivers** (specific to your camera hardware)
- **Required ROS packages**:
  ```bash
  sudo apt-get install ros-<distro>-navigation
  sudo apt-get install ros-<distro>-gmapping
  sudo apt-get install ros-<distro>-move-base
  ```

### Installation

1. **Clone the repository**:
```bash
git clone https://github.com/RamadanMohamed11/Minesweeper.git
cd Minesweeper
```

2. **Set up ROS workspace**:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s /path/to/Minesweeper/ROS\ Codes/* .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. **Install Python dependencies**:
```bash
cd Minesweeper/Vision\ Codes
pip install -r requirements.txt
```

4. **Configure camera settings**:
   - Update camera parameters in the configuration files
   - Calibrate camera using ROS camera calibration tools

### Running the System

1. **Launch ROS nodes**:
```bash
roslaunch minesweeper_navigation main.launch
```

2. **Start vision system**:
```bash
rosrun minesweeper_vision mine_detector.py
```

3. **Monitor in RViz**:
```bash
rosrun rviz rviz
```

## 🎯 System Architecture

### Vision System
- **Mine Detection**: Uses color/shape recognition to identify mines
- **Image Preprocessing**: Filters and enhances camera feed
- **Feature Extraction**: Identifies key visual markers
- **Classification**: Distinguishes mines from background objects

### Navigation System
- **SLAM**: Simultaneous Localization and Mapping
- **Path Planning**: A* or Dijkstra algorithms for optimal routing
- **Obstacle Avoidance**: Dynamic replanning around detected mines
- **Localization**: Sensor fusion for accurate position estimation

### Control System
- **Motor Control**: PWM-based speed regulation
- **State Machine**: Behavior coordination
- **Safety Systems**: Emergency stop and boundary detection

## ⚙️ Configuration

### Vision Parameters
Edit `Vision Codes/config/vision_params.yaml`:
```yaml
detection:
  color_threshold: [lower_hsv, upper_hsv]
  min_area: 500
  confidence: 0.85
```

### Navigation Parameters
Edit `ROS Codes/config/navigation_params.yaml`:
```yaml
move_base:
  max_vel_x: 0.5
  min_vel_x: 0.1
  obstacle_range: 2.5
```

## 🛡️ Safety Features

- Emergency stop functionality
- Collision prevention systems
- Boundary detection and containment
- Fail-safe modes for sensor failures
- Manual override capability

## 🔧 Troubleshooting

**Camera not detected:**
```bash
ls /dev/video*  # Check camera device
v4l2-ctl --list-devices  # List video devices
```

**ROS nodes not communicating:**
```bash
rosnode list  # Verify running nodes
rostopic echo /topic_name  # Check topic messages
```

**Navigation not working:**
- Verify odometry is publishing
- Check sensor data streams
- Ensure map is being generated
- Validate TF tree connections

## 📊 Performance Metrics

- **Detection Accuracy**: Target >90% mine recognition
- **Navigation Speed**: Configurable (default: 0.3 m/s)
- **Response Time**: <100ms for obstacle detection
- **Coverage Efficiency**: Optimized search patterns

## 🤝 Contributing

This is a team project from Assiut Robotics Team. Contributions and improvements are welcome:
- Enhanced detection algorithms
- Improved navigation strategies
- Performance optimizations
- Documentation improvements

## 📝 License

This project is developed for educational and competitive robotics purposes.

## 👥 Team

**Assiut Robotics Team**
- Project Lead: Ramadan Mohamed
- GitHub: [@RamadanMohamed11](https://github.com/RamadanMohamed11)

## 🙏 Acknowledgments

- Assiut University Robotics Lab
- ROS community for excellent documentation
- Competition organizers and sponsors
- Open-source computer vision community

## 📞 Contact

For questions about this project:
- Open an issue on GitHub
- Contact the Assiut Robotics Team

---

**Competition Results**: [Add achievements and competition placements here]

**Project Status**: Active Development ✅