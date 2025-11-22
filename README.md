# SRDrone - LLM驱动的无人机自主任务规划系统

[English](#overview) | [中文](#概述)

## 概述

SRDrone是一个基于ROS1的大型语言模型(LLM)驱动的无人机自主任务规划系统。本项目集成了行为树(BehaviorTree)决策框架、视觉感知(YOLO深度学习目标检测、ArUco标记识别)、GNSS-IMU多传感器融合定位、轨迹规划和实时飞行控制等核心技术，支持RflySim仿真平台和FS-J310四轴飞行器实体控制。

该项目旨在为无人机提供灵活的任务规划、自适应决策和视觉伺服能力，满足复杂环境中的自主飞行需求。

### 主要特性

- **行为树框架**：采用BehaviorTree.CPP库实现模块化、可扩展的任务决策逻辑
- **多源视觉感知**：
  - YOLO v8/v5深度学习目标检测（PyTorch/TensorRT推理后端）
  - OpenCV ArUco标记识别与姿态估计
  - 多摄像头并行处理（前视+下视）
- **多传感器融合**：
  - 深度相机点云处理与聚类
  - GNSS/IMU数据融合（Faster-LIO/Ego-Planner栈）
  - 实时里程计反馈
- **灵活的飞行控制**：
  - MAVROS接口集成，兼容PX4/Ardupilot
  - 位置目标、速度目标、姿态指令多种控制方式
  - 自动起飞、走廊穿过、标记识别、目标追踪、自动着陆等复合动作
- **仿真与实验支持**：
  - RflySim虚拟仿真环境（支持多传感器模拟）
  - 实体飞行系统（RealSense/Livox硬件支持）
  - 快速切换仿真/实飞配置
- **完整的系统集成**：
  - 传感器驱动与数据适配
  - 自动化启动脚本，协调多个ROS节点
  - 可视化工具（RViz配置）

## 项目结构

```
SRDrone
├── common_msgs/                    # 自定义ROS消息定义
│   ├── msg/
│   │   ├── Aruco.msg             # ArUco标记信息
│   │   ├── Obj.msg               # 单个检测目标
│   │   ├── Objects.msg           # 目标集合
│   │   └── MissionState.msg      # 任务状态
│   ├── CMakeLists.txt
│   └── package.xml
│
├── controller/                     # 飞行控制模块（C++17）
│   ├── src/
│   │   └── ros1_node.cpp         # 主控制节点（行为树执行引擎）
│   ├── include/
│   │   └── controller.h          # 控制器类定义
│   ├── config/
│   │   ├── mav.xml               # 行为树定义（主任务流程）
│   │   ├── sim.yaml              # 仿真环境配置参数
│   │   ├── control.yaml          # 实体飞行配置参数
│   │   ├── cfg.yaml              # 基础系统配置
│   │   ├── realsensor.yaml       # RealSense相机参数
│   │   └── BTlog.txt             # 行为树日志
│   ├── launch/
│   │   ├── sim.launch            # 仿真启动脚本
│   │   └── control.launch        # 实体飞行启动脚本
│   ├── CMakeLists.txt
│   └── package.xml
│
├── object_det/                     # 目标检测模块（Python）
│   ├── scripts/
│   │   ├── det.py                # 主检测节点（集成前视+下视）
│   │   ├── ObjectDetect.py       # YOLO推理核心类（PyTorch）
│   │   ├── ObjectDetect_TensorRT.py # YOLO推理核心类（TensorRT）
│   │   ├── Forward_det.py        # 前视检测节点
│   │   ├── Down_det.py           # 下视检测节点
│   │   ├── frame.py              # 图像缓冲管理
│   │   ├── detectBoard.py        # 棋盘检测工具
│   │   ├── Model/                # 预训练权重目录
│   │   │   └── det.pt           # YOLO模型权重
│   │   ├── models/               # 模型架构定义
│   │   ├── utils/                # 工具函数
│   │   └── Readme.md             # 检测模块说明
│   ├── launch/
│   │   └── *.launch              # 检测节点启动配置
│   ├── CMakeLists.txt
│   └── package.xml
│
├── recognize_aruco/                # ArUco标记识别模块
│   ├── image.py                   # ArUco检测与姿态估计
│   └── Config.yaml
│
├── sensor_pkg/                     # 传感器驱动与数据适配
│   ├── main.py                    # RflySim传感器接口
│   ├── Config.json                # 传感器配置（分辨率、内参等）
│   ├── rflysim.rviz              # RViz可视化配置
│   ├── tf_cfg.yaml               # TF坐标系配置
│   ├── readbag.py                # 数据包读取工具
│   └── test.py                   # 功能测试脚本
│
└── sh/                             # 系统启动脚本
    ├── start_sim.sh              # 仿真模式启动（含Faster-LIO+Ego-Planner）
    ├── start_control.sh          # 实体飞行启动
    ├── mavros.sh                 # MAVROS启动
    ├── sensor.sh                 # 传感器启动
    ├── localization.sh           # 定位栈启动
    ├── detect.sh                 # 检测节点启动
    └── kill_ros_pid.sh           # ROS进程清理

```

## 核心模块说明

### 1. common_msgs - 自定义消息接口
定义系统各模块间通信的数据结构：
- **Aruco.msg**：标记ID、位置、姿态、置信度
- **Obj.msg**：单个目标的分类、位置、大小、置信度
- **Objects.msg**：多个目标的集合
- **MissionState.msg**：任务执行状态反馈

### 2. controller - 飞行控制核心
基于BehaviorTree.CPP的决策执行引擎，具体功能：
- **行为树解析**：从XML配置文件加载任务树结构
- **条件判断节点**：检查任务阶段、目标状态、飞行条件
- **动作执行节点**：
  - `Takeoff`：自动起飞到设定高度
  - `Cross_frame`：穿过标记框架（视觉伺服）
  - `Recognize_aruco`：检测与定位ArUco标记
  - `Recognize_H`：识别H形目标
  - `Land`：自动着陆
- **多源数据融合**：
  - 深度点云处理：体素栅格下采样、欧几里得聚类
  - 目标追踪：基于视觉和深度的自适应伺服
  - 轨迹反馈：从Ego-Planner订阅规划轨迹
- **MAVROS接口**：发送OFFBOARD模式指令、位置/速度目标、武装/解武装命令

### 3. object_det - 视觉目标检测
Python多线程处理多摄像头流：
- **YOLO推理**：支持PyTorch和TensorRT两种后端
- **前视检测**：识别走廊、框架、障碍物
- **下视检测**：识别地面目标、着陆点
- **ROS发布**：将检测结果作为Objects消息实时发布

### 4. recognize_aruco - ArUco标记识别
集成OpenCV ArUco库实现：
- 自动标记检测与ID识别
- 单标记姿态估计（PnP算法）
- 摄像头内参标定支持

### 5. sensor_pkg - 传感器适配层
连接仿真与实体传感器：
- **RflySim桥接**：通过VisionCapture API接收虚拟相机、IMU、GPS数据
- **实体硬件驱动**：RealSense深度相机、Livox激光雷达、USB摄像头
- **数据转换**：将传感器原始数据转为ROS标准消息格式

## 安装与环境配置

### 系统要求
- **操作系统**：Ubuntu 18.04/20.04 LTS
- **ROS版本**：ROS1 Melodic/Noetic
- **C++标准**：C++17
- **GPU可选**：用于加速YOLO推理（CUDA 10.2+, cuDNN 7.6+）

### 依赖项

#### 系统依赖
```bash
sudo apt-get update
sudo apt-get install -y \
    python3-dev python3-pip \
    libopencv-dev python3-opencv \
    libyaml-dev libyaml-cpp-dev \
    libpcl-dev \
    ros-melodic-mavros ros-melodic-mavros-msgs \
    ros-melodic-behaviortree-cpp \
    ros-melodic-cv-bridge \
    ros-melodic-sensor-msgs \
    ros-melodic-geometry-msgs \
    ros-melodic-nav-msgs
```

#### Python依赖
```bash
pip3 install \
    torch==1.9.0 torchvision==0.10.0 \
    opencv-python==4.5.3.56 \
    numpy \
    scipy \
    pyyaml \
    catkin-pkg
```

#### ROS工作空间依赖包
- **MAVROS**：`sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-msgs`
- **BehaviorTree.CPP**：参考[官方安装指南](https://github.com/BehaviorTree/BehaviorTree.CPP)
- **Faster-LIO**：用于GNSS-IMU融合定位
- **Ego-Planner**：用于轨迹规划
- **vision_opencv**：cv_bridge依赖
- **quadrotor_msgs**：轨迹指令消息（来自Ego-Planner）

#### 仿真环境（可选）
- **RflySim**：CopterSim虚拟仿真平台
- VisionCapture API（随RflySim提供）

#### 实体硬件驱动（可选）
- **realsense-ros**：RealSense深度相机驱动
- **livox_ros_driver2**：Livox激光雷达驱动
- **usb_cam**：通用USB摄像头驱动

### 构建步骤

1. **创建ROS工作空间**：
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```

2. **克隆本仓库**：
```bash
cd src
git clone <repository-url> SRDrone
cd ..
```

3. **安装依赖**：
```bash
# 方式一：使用rosdep自动解析
rosdep install --from-paths src --ignore-src -r -y

# 方式二：手动安装关键包
apt-get install ros-${ROS_DISTRO}-behaviortree-cpp \
                ros-${ROS_DISTRO}-mavros \
                ros-${ROS_DISTRO}-cv-bridge
```

4. **编译项目**：
```bash
catkin_make
# 或使用catkin build（推荐）
catkin build

# 仅编译controller包（加快编译）
catkin_make -DCATKIN_WHITELIST_PACKAGES="common_msgs;controller"
```

5. **配置环境变量**：
```bash
source ~/catkin_ws/devel/setup.bash

# 添加到~/.bashrc以便永久生效
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 配置调整

#### 仿真环境配置（sim.yaml）
```yaml
# 摄像头参数（与RflySim配置一致）
rflysim:
    f_rgb: 320              # RGB焦距（像素）
    f_depth: 320            # 深度焦距
    rgb_image_w: 640        # 分辨率宽
    rgb_image_h: 480        # 分辨率高
    min_score: 0.7          # YOLO置信度阈值
    
# 飞行参数
takeoff_h: 1.0             # 起飞高度（米）
vx_max: 0.2                # 最大前进速度
vy_max: 0.2                # 最大横向速度
kx: 0.003                  # 视觉伺服系数X
ky: 0.003                  # 视觉伺服系数Y
```

#### 实体飞行配置（control.yaml）
根据实际相机标定结果调整内参数据（f_rgb, rgb_ppx, rgb_ppy等）

## 使用示例

### 仿真模式运行

1. **启动仿真环境**（假设RflySim已启动）：
```bash
cd ~/catkin_ws/src/SRDrone/sh
bash start_sim.sh
```

此脚本自动启动：
- 传感器数据源（RflySim桥接）
- 目标检测节点
- ArUco识别节点
- Faster-LIO定位栈
- Ego-Planner规划栈
- 飞行控制节点

2. **监控执行过程**：
```bash
# 在另一个终端查看行为树日志
tail -f ~/catkin_ws/src/SRDrone/controller/config/BTlog.txt

# 查看ROS话题
rostopic list
rostopic echo /objects           # 检测结果
rostopic echo /aruco            # ArUco标记
rostopic echo /mavros/local_position/pose  # 无人机位置
```

3. **可视化**（RViz）：
```bash
rviz -d ~/catkin_ws/src/SRDrone/sensor_pkg/rflysim.rviz
```

### 实体飞行模式运行

1. **硬件连接检查**：
   - PX4/Ardupilot飞控连接到MAVROS
   - RealSense深度相机USB连接
   - 下视USB摄像头连接
   - Livox激光雷达（可选）连接

2. **启动实体飞行栈**：
```bash
cd ~/catkin_ws/src/SRDrone/sh
bash start_control.sh
```

此脚本启动：
- MAVROS（连接飞控）
- RealSense驱动
- USB摄像头驱动
- 目标检测节点
- ArUco识别节点
- Faster-LIO定位
- Ego-Planner规划
- 飞行控制节点

3. **手动控制飞行**（初次调试）：
```bash
# 在MAVROS交互界面中执行
# 1. 切换OFFBOARD模式
rostopic pub /mavros/set_mode mavros_msgs/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# 2. 武装无人机
rosservice call /mavros/cmd/arming "value: true"

# 3. 发送起飞指令
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped \
    "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'map'}, \
      pose: {position: {x: 0, y: 0, z: 1}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

### 任务定制（修改行为树）

编辑`controller/config/mav.xml`定制任务流程：

```xml
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Condition ID="Check_Takeoff"/>
            <Action ID="Takeoff"/>
            
            <Condition ID="Check_cross_frame1"/>
            <Action ID="Cross_frame1"/>
            
            <Condition ID="Check_recognize_aruco"/>
            <Action ID="Recognize_aruco"/>
            
            <Condition ID="Check_Land"/>
            <Action ID="Land"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## 项目架构与数据流

```
┌─────────────────────────────────────────────────────────────┐
│                        ROS Master (roscore)                 │
└─────────────────────────────────────────────────────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Sensor Pkg  │     │ Object Det   │     │ Recognize    │
│  (RflySim)   │     │  (YOLO)      │     │ ArUco        │
│              │     │              │     │              │
│ /camera/rgb  │     │ /objects     │     │ /aruco       │
│ /camera/depth│     │              │     │              │
│ /imu         │     │              │     │              │
│ /gps         │     │              │     │              │
└──────────────┘     └──────────────┘     └──────────────┘
        │                    │                    │
        └────────┬───────────┴────────┬───────────┘
                 │                    │
                 ▼                    ▼
        ┌──────────────────────────────────┐
        │    Faster-LIO Localization       │
        │    (GNSS/IMU/LiDAR Fusion)       │
        │         /tf (map→base_link)      │
        └──────────────────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────────┐
        │    Ego-Planner Path Planning     │
        │    /planning/trajectory          │
        └──────────────────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────────┐
        │   Controller (BehaviorTree)      │
        │   - Vision Servoing              │
        │   - Behavior Execution           │
        │   /mavros/setpoint_position      │
        │   /mavros/setpoint_velocity      │
        │   /mavros/cmd/arming             │
        │   /mavros/set_mode               │
        └──────────────────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────────┐
        │        MAVROS Interface          │
        │    (PX4/Ardupilot FCU)           │
        └──────────────────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────────┐
        │     Quadrotor Controller         │
        │    (Attitude/Thrust Control)     │
        └──────────────────────────────────┘
```

## 常见问题与解决

### 1. BehaviorTree.CPP编译失败
```bash
# 确保使用C++17
export CXXFLAGS="-std=c++17"
catkin_make

# 或手动安装
sudo apt-get install ros-${ROS_DISTRO}-behaviortree-cpp
```

### 2. YOLO模型推理缓慢
- 启用GPU加速（requires CUDA）：修改`det.py`中的设备选择
- 考虑使用TensorRT后端（ObjectDetect_TensorRT.py）
- 降低输入分辨率或置信度阈值

### 3. 目标检测不稳定
- 检查相机标定（内参矩阵）
- 调整YOLO置信度阈值（min_score参数）
- 验证光照条件是否满足训练数据分布

### 4. 飞控连接失败
```bash
# 检查MAVROS连接
rostopic echo /mavros/state

# 查看飞控日志
rosnode info /mavros
rosgraph  # 可视化节点连接

# 检查USB设备
ls -la /dev/ttyACM* /dev/ttyUSB*
```

### 5. 点云处理内存溢出
- 增加体素栅格下采样率（depth_down_sample参数）
- 限制点云处理ROI范围
- 检查深度滤波配置

## 依赖项与许可

### 核心依赖
| 项目 | 用途 | 许可 |
|-----|------|------|
| ROS1 | 中间件 | BSD |
| BehaviorTree.CPP | 决策框架 | MIT |
| YOLO v5/v8 | 目标检测 | AGPL-3.0 |
| OpenCV | 图像处理 | Apache-2.0 |
| PCL | 点云处理 | BSD |
| MAVROS | 飞控接口 | BSD |
| Faster-LIO | 定位 | Apache-2.0 |
| Ego-Planner | 路径规划 | Apache-2.0 |

### 项目许可
本项目采用 **Apache License 2.0** 许可证。详见LICENSE文件。

## 贡献指南

我们欢迎社区的贡献！请按以下步骤参与：

### 开发工作流
1. **Fork本仓库**
2. **创建特性分支**：`git checkout -b feature/your-feature`
3. **提交更改**：
   ```bash
   git add .
   git commit -m "feat: add your feature description"
   ```
4. **推送到分支**：`git push origin feature/your-feature`
5. **发起Pull Request**，描述您的改进

### 代码规范
- **C++代码**：遵循Google C++风格指南，使用clang-format格式化
- **Python代码**：遵循PEP 8，使用4个空格缩进
- **提交信息**：使用规范化消息格式
  - `feat:` 新功能
  - `fix:` 错误修复
  - `docs:` 文档更新
  - `refactor:` 代码重构
  - `test:` 测试用例

### 报告问题
请在GitHub Issues中提交问题，包含：
- 详细的问题描述
- 重现步骤
- 环境信息（Ubuntu版本、ROS版本、GPU型号等）
- 相关日志文件

### 讨论建议
对于特性建议和架构讨论，请在Discussion中发起话题。

## 参考资源

- [BehaviorTree.CPP文档](https://www.behaviortree.dev/)
- [MAVROS用户指南](http://docs.ros.org/en/melodic/api/mavros/html/)
- [ROS官方教程](http://wiki.ros.org/)
- [YOLO官方](https://github.com/ultralytics/yolov5)
- [OpenCV教程](https://docs.opencv.org/)
- [PCL教程](https://pcl.readthedocs.io/)

## 相关论文与资源

- **行为树在机器人中的应用**：Colledanchise, M., & Ögren, P. (2018). *Behavior trees in robotics and games.*
- **视觉伺服控制**：Hutchinson, S., Hager, G. D., & Corke, P. I. (1996). *A tutorial on visual servo control.*
- **多传感器融合定位**：研究论文可参考Faster-LIO和Ego-Planner原作论文

## 维护者

- **主要开发者**：zhuoyi@todo.todo
- **许可证**：Apache License 2.0

## 致谢

感谢以下开源项目和社区的支持：
- ROS社区与MAVROS维护者
- BehaviorTree.CPP开发团队
- YOLO官方与ultralytics团队
- PCL和OpenCV社区
- RflySim虚拟仿真平台团队

---

**最后更新**：2024年11月
**项目版本**：0.1.0

如有问题或建议，欢迎通过Issue或Email联系我们！
