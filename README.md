# Nav Robot Package

A ROS2 package implementing a navigation robot with behavior tree-based task management and battery monitoring.

## Features

- Behavior Tree-based task management
- Battery state monitoring and charging behavior
- Waypoint navigation
- Pick and drop task simulation
- Integration with Nav2 navigation stack

## Requirements

### System Requirements
- Ubuntu 22.04
- ROS2 Humble
- CMake 3.8 or higher
- C++17 compatible compiler

### ROS2 Dependencies
```bash
sudo apt install ros-humble-behaviortree-cpp-v3
sudo apt install ros-humble-nav2-behavior-tree
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-common
sudo apt install ros-humble-nav2-util
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-nav-msgs
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-urdf
sudo apt install ros-humble-xacro
```

## Installation

1. Create a ROS2 workspace (if not already created):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the package:
```bash
git clone https://github.com/dcbothra/nav_robot.git nav_robot
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the package:
```bash
colcon build
```

5. Source the workspace:
```bash
source install/setup.bash
```


## Usage

Launching just the navigation system with Gazebo:
```bash
ros2 launch nav_robot gspawn.launch.py
```

Launching with BT Planner:
```bash
ros2 launch nav_robot bt_nav.launch.py
```


## Behavior Tree Structure

The behavior tree implements the following logic:
- Check battery level
- If battery OK:
  - Navigate to next waypoint
  - Perform pick/drop operation
- If battery LOW:
  - Navigate to charging station
  - Simulate charging
  - Resume previous task

## Configuration

### Waypoints
Edit `config/poses.yaml` to modify waypoints and charging station location.

### Battery Parameters
Battery simulation parameters can be modified in the battery_state.cpp file:
- Battery capacity
- Drain rate
- Charging rate
