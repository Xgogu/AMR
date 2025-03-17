# 小车Gazebo仿真Demo

## 1. 介绍

本项目为小车Gazebo仿真Demo，使用ROS2进行开发，主要功能为小车在Gazebo仿真环境中进行运动控制，包括键盘控制、等

## 2. 运行环境

- Ubuntu 22.04
- ROS2 humble
- Gazebo 11

## 3. 使用方法

### 3.1 编译

```bash
cd ~/catkin_ws
colcon build
```

### 3.2 启动仿真环境

```bash
source install/local.setup.bash
ros2 launch originbot_gazebo originbot_gazebo.launch.py
```

### 3.3 启动键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3.4 启动rviz2

```bash
ros2 run rviz2 rviz2 
```

## 4. 效果演示
![image](./pictures/output.gif)

