# SRDrone


## Overview

This project is a large-language-model-driven UAV task planning system, based on the RflySim simulation platform and the FS-J310 quadcopter.

## Project Structure

SRDRONE  
├─ common_msgs                 # Custom ROS Messages  
│  └─ msg                      # Message definitions (.msg files)  
├─ controller                  # Flight Control Module  
│  ├─ config                   # Configuration files  
│  ├─ include                  # C++ header files  
│  ├─ launch                   # Launch files (.launch)  
│  └─ src                      # Source code for control algorithms  
├─ object_det                  # Object Detection Module  
│  ├─ launch                   # Detection node launch scripts  
│  └─ scripts                  # Python detection scripts  
│      ├─ Model                # Pre-trained model weights  
│      ├─ models               # Model architectures  
│      └─ utils                # Helper functions  
├─ recognize_aruco             # ArUco Marker Recognition  
├─ sensor_pkg                  # Sensor Drivers & Processing  
└─ sh                          # System Scripts  

## Usage

The path to the Behavior Tree (BT) file to be executed can be specified by setting the `bt_tree_path` parameter in the launch file within the `controller/launch` directory.

```xml
<param name="bt_tree_path" type="string" value="$(find controller)/config/mav.xml"/>
```

### Simulation

```shell
cd sh
sh ./start_sim.sh
```

### Real Flight​

```sh
cd sh
sh ./start_control.sh
```

