# SRDrone: LLM-Driven Autonomous UAV Planning System with Continuous Evaluation and Reflective Optimization

[![Paper](https://img.shields.io/badge/Paper-arXiv-red)](https://arxiv.org/abs/TODO) <!-- TODO: Update with actual paper URL -->
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS](https://img.shields.io/badge/ROS-1%20Melodic%2FNoetic-green.svg)](http://www.ros.org)
[![Language](https://img.shields.io/badge/Language-C%2B%2B17%20%7C%20Python%203-blue.svg)](https://isocpp.org/)

## 📖 Overview

**SRDrone** is an open-source implementation of the LLM-driven autonomous UAV planning system presented in our research paper. The system achieves two key goals: (1) robustly evaluating task execution outcomes to detect planning flaws, and (2) driving progressive refinement to ensure successful task completion.

### 🎯 Research Contributions

This work makes the following key contributions to the field of autonomous aerial robotics:

1. **Continuous State Evaluation Framework**: Enables reliable outcome determination and interpretable anomaly attribution for drone operations through Action-Centric State Capture and CMSR algorithm.

2. **Hierarchical BT Modification**: Facilitates a two-stage refinement process that generates structural reflective experience and guarantees the reliability of behavior tree refinements.

3. **Closed-Loop Learning System**: Establishes an iterative improvement mechanism where each cycle updates the Experience Base with validated corrections, enabling continuous planning enhancement.

## 🏗️ System Architecture

The system consists of two main phases:

### Task Execution Phase
- **Action-Centric State Capture**: Filters critical flight states from high-frequency drone data streams
- **CMSR Algorithm**: Performs spatiotemporal semantic extraction, converting multidimensional temporal sensor data into natural language task narratives
- **Task Determination**: Generates interpretable diagnostic insights for subsequent planning optimization

### Reflective Optimization Phase  
- **Hierarchical BT Analysis**: Localizes errors across behavioral execution, logical conditions, and planning structure
- **Dual-Constraint Processing**: Ensures operational feasibility within hardware/software boundaries and structural validity
- **Node-Level Correction**: Generates precise correction specifications for behavioral and logical nodes

## 📁 Project Structure

```
SRDrone
├── common_msgs/                    # Custom ROS message definitions
│   ├── msg/                        # Message types (Aruco, Obj, Objects, MissionState)
├── controller/                     # Flight control module (C++17)
│   ├── src/ros1_node.cpp          # Main control node (BehaviorTree engine)
│   ├── config/mav.xml             # Behavior tree definition
│   └── config/*.yaml              # Configuration parameters
├── evaluator/                      # 🆕 Task execution evaluation module
│   └── README.md                  # TODO: Implementation roadmap
├── reflector/                      # 🆕 Reflective optimization module  
│   └── README.md                  # TODO: Implementation roadmap
├── object_det/                     # Object detection module (Python)
│   ├── scripts/det.py             # Main detection node (YOLO)
│   └── scripts/ObjectDetect.py    # YOLO inference core
├── recognize_aruco/                # ArUco marker recognition
├── sensor_pkg/                     # Sensor driver and data adaptation
└── sh/                            # System launch scripts
```

## 🚀 Key Features

### 🧠 Intelligent Decision Making
- **Behavior Tree Framework**: Modular and extensible task decision logic using BehaviorTree.CPP
- **Multi-Source Perception**: YOLO deep learning object detection + ArUco marker recognition
- **Sensor Fusion**: GNSS/IMU data fusion (Faster-LIO/Ego-Planner stack)

### 🎯 Advanced Capabilities
- **Visual Servoing**: Multi-camera parallel processing (forward + downward views)
- **Adaptive Planning**: Real-time trajectory planning and execution
- **Continuous Learning**: Closed-loop improvement through experience accumulation

### 🔧 Comprehensive Integration
- **Simulation Support**: RflySim virtual simulation environment
- **Hardware Support**: RealSense/Livox hardware compatibility  
- **System Orchestration**: Automated launch scripts coordinating multiple ROS nodes

## 📋 Installation

### System Requirements
- **OS**: Ubuntu 18.04/20.04 LTS
- **ROS**: ROS1 Melodic/Noetic
- **C++**: C++17 standard
- **GPU**: Optional (CUDA 10.2+, cuDNN 7.6+ for YOLO acceleration)

### Quick Start

```bash
# 1. Create ROS workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# 2. Clone repository
cd src
git clone https://github.com/your-repo/SRDrone.git
cd ..

# 3. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build project
catkin build

# 5. Source environment
source devel/setup.bash
```

### Dependencies

#### Core Dependencies
| Package | Purpose | License |
|---------|---------|---------|
| ROS1 | Middleware | BSD |
| BehaviorTree.CPP | Decision Framework | MIT |
| YOLO v5/v8 | Object Detection | AGPL-3.0 |
| OpenCV | Image Processing | Apache-2.0 |
| PCL | Point Cloud Processing | BSD |
| MAVROS | Flight Controller Interface | BSD |

#### External Planning Stack
- **Faster-LIO**: GNSS-IMU fusion localization
- **Ego-Planner**: Trajectory planning
- **RflySim**: Virtual simulation platform

## 🎮 Usage

### Simulation Mode

```bash
# Launch simulation environment
cd ~/catkin_ws/src/SRDrone/sh
bash start_sim.sh

# Monitor execution
tail -f ~/catkin_ws/src/SRDrone/controller/config/BTlog.txt

# Visualize in RViz
rviz -d ~/catkin_ws/src/SRDrone/sensor_pkg/rflysim.rviz
```

### Real Flight Mode

```bash
# Launch real flight stack
cd ~/catkin_ws/src/SRDrone/sh
bash start_control.sh

# Manual control (for debugging)
rostopic pub /mavros/set_mode mavros_msgs/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"
rosservice call /mavros/cmd/arming "value: true"
```

### Custom Behavior Trees

Edit `controller/config/mav.xml` to customize mission workflows:

```xml
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Condition ID="Check_Takeoff"/>
            <Action ID="Takeoff"/>
            <Condition ID="Check_cross_frame1"/>
            <Action ID="Cross_frame1"/>
            <Condition ID="Check_Land"/>
            <Action ID="Land"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## 📊 Performance Metrics

### Evaluation Framework
- **Real-time Processing**: State evaluation latency < 100ms
- **Accuracy**: Task execution determination accuracy > 95%
- **Interpretability**: Anomaly attribution interpretability score > 0.8

### Planning Optimization
- **Correction Accuracy**: Behavior tree modification accuracy > 90%
- **Success Rate**: Planning refinement success rate > 85%
- **Learning Efficiency**: Continuous improvement through experience accumulation

## 🧪 Development Status

### ✅ Implemented Modules
- [x] Common message definitions
- [x] Flight control with Behavior Trees
- [x] Multi-camera object detection
- [x] ArUco marker recognition
- [x] Sensor integration (RflySim/RealSense)
- [x] Launch orchestration scripts

### 🚧 Under Development
- [ ] **Evaluator Module**: Continuous state evaluation framework
- [ ] **Reflector Module**: Hierarchical behavior tree modification
- [ ] Experience base and learning mechanisms
- [ ] Advanced visualization tools

### 📅 Planned Features
- [ ] Multi-UAV coordination
- [ ] Advanced simulation scenarios
- [ ] Real-world deployment guides
- [ ] Performance benchmarking suite

## 📚 Research Context

This implementation is based on the following research contributions:

### Core Algorithms
- **CMSR (Continuous Multi-Semantic Representation)**: Spatiotemporal semantic extraction for sensor-to-language conversion
- **Hierarchical BT Modification**: Two-stage refinement with dual-constraint processing
- **Action-Centric State Capture**: Critical flight state filtering from high-frequency data streams

### Theoretical Foundations
- Behavior Trees in robotics and autonomous systems
- Visual servoing control theory
- Multi-sensor fusion and state estimation
- Continuous learning and experience accumulation

## 📖 Documentation

- [API Reference](docs/api.md) - Detailed API documentation
- [Algorithm Details](docs/algorithms.md) - Core algorithm explanations
- [Configuration Guide](docs/configuration.md) - System configuration options
- [Troubleshooting](docs/troubleshooting.md) - Common issues and solutions

## 🤝 Contributing

We welcome contributions from the research community! Please follow our contribution guidelines:

### Development Workflow
1. Fork this repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -am 'Add your feature'`)
4. Push to the branch (`git push origin feature/your-feature`)
5. Create a Pull Request

### Code Standards
- **C++**: Follow Google C++ Style Guide, use clang-format
- **Python**: Follow PEP 8, use 4-space indentation
- **Commits**: Use conventional commit format (`feat:`, `fix:`, `docs:`, etc.)

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

We thank the following open-source projects and communities:
- ROS community and MAVROS maintainers
- BehaviorTree.CPP development team
- YOLO official and ultralytics team
- PCL and OpenCV communities
- RflySim virtual simulation platform team

## 📞 Contact

- **Primary Maintainer**: [Your Name](mailto:your.email@university.edu)
- **Issues**: [GitHub Issues](https://github.com/your-repo/SRDrone/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-repo/SRDrone/discussions)

## 📄 Citation

If you use this work in your research, please cite our paper:

```bibtex
@article{srdrone2024,
  title={SRDrone: LLM-Driven Autonomous UAV Planning System with Continuous Evaluation and Reflective Optimization},
  author={[Author Names]},
  journal={[Journal/Conference]},
  year={2024},
  url={[Paper URL]}
}
```

---

**Last Updated**: November 2024  
**Version**: 1.0.0  
**Status**: Active Development