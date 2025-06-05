# Inspire 灵巧手（左手）Rviz 外部控制

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)  
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blueviolet)](https://docs.ros.org/en/humble/)

本仓库实现通过外部控制器在 Rviz 中控制 Inspire 灵巧手（左手）的仿真模型。

## 功能特性
- 通过外部设备实时控制灵巧手关节运动
- 在 Rviz 中渲染 Inspire 灵巧手 URDF 模型
- 支持自定义控制映射（如关节角度/速度控制）

## 依赖环境
- **ROS2 版本**: Humble 或 Foxy（推荐 Humble）
- **硬件依赖**（TODO）

## 参考项目
- URDF 模型来源: [unitreerobotics/avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate)
- Rviz 集成参考: [HaofeiMa/urdf_ros2_rviz2](https://github.com/HaofeiMa/urdf_ros2_rviz2)