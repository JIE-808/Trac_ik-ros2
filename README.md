# TRAC-IK 逆运动学求解器 (适配 ROS2 )

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://releases.ubuntu.com/22.04/)
[![C++ 17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-orange)](https://moveit.picknik.ai/humble/index.html)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-green.svg)](https://opensource.org/licenses/Apache-2.0)

---
## 为什么换掉默认的 KDL？
### 1. 在 MoveIt 中控制机械臂时，经常遇到:
*  **明明手能勾到的位置，却报 `No IK solution found`**
*  **机械臂在某个姿态突然“突变”或是规划路径异常曲折**
*  **稍微靠近关节限位，规划就直接罢工**
### 2. trac_ik相较于KDL规划器: 
```bash
*  成功率：TRAC-IK 高于 KDL
*  精度：两者可达到接近精度
*  时间：Speed 模式下常常更划算；Distance/Manip 模式可能更慢，但结果更稳
*  工程体验：TRAC-IK 更适合复杂位姿、接近极限、连续 Cartesian IK 和实时遥操作
```
---
## 本仓库贡献：ROS 2 适配

### 1. 当前版本TRAC-IK 适配说明
```bash
当前版本的TRAC-IK，基于社区作者: https://github.com/aprotyas  提供的 trac_ik-ros2 版本继续修改而来。

- 原始基础：ROS1 时代的 TRAC-IK
- 中间版本：社区作者aprotyas基于 ROS1 代码做的 ROS2 移植
- 当前版本：在该移植基础上，继续完成 ROS2 / MoveIt2 适配与工程集成
```
### 2. 上游版本现状
```bash
trac_ik-ros2 已经完成了部分 ROS2 移植，尤其是 trac_ik_lib 核心求解库可在 ROS2 环境下使用。
但在 trac_ik_kinematics_plugin 这一层，仍保留较多 ROS1 风格实现，例如：

- 仍使用 catkin
- 仍依赖 roscpp、tf_conversions
- 插件初始化接口仍偏向 ROS1 / MoveIt1 写法
- README 内容仍保留 ROS1 时代说明
```
因此，上游版本可以作为移植基础，但还不能直接满足本项目的 MoveIt2 插件接入需求。

### 3. 当前版本的主要修改

本项目在上游基础上，重点完成了以下修改：

#### 1. 完成 kinematics plugin 的 ROS2 化
```bash
- 将插件包构建系统由 catkin 改为 ament_cmake
- 将依赖从 roscpp、tf_conversions 迁移到 rclcpp、tf2_kdl、tf2_geometry_msgs
- 将消息与 TF 转换接口切换为 ROS2 写法
```
#### 2. 适配 MoveIt2 插件接口
```bash
- 重构插件 initialize 接口，使其适配 MoveIt2 的节点与 RobotModel 传参方式
- 调整参数读取逻辑，适配 robot_description_kinematics.<group_name> 命名空间
- 按 MoveIt2 方式导出 plugin description，使插件可被正常加载
```
#### 3. 接入机械臂工程
```bash
- 在机械臂 MoveIt 配置中，均切换为 TRAC-IK 作为运动学求解器
- 已实际接入 dual_arm_moveit_action_server 的 MoveIt2 规划链路
- 同时保留 trac_ik_lib 直连能力，供本地节点直接调用 IK
```
### 4. 相比上游版本的改进
```bash
当前版本相较上游版本，主要提升如下：

- 不只是“编译库”，而是“插件可以在 MoveIt2 中实际使用”
- 不只是“完成部分 ROS2 移植”，而是“完成工程接入”
- 同时支持两种使用方式：
- 作为 MoveIt2 kinematics plugin 用于规划
- 作为本地 IK 库用于实时控制与操作
```
---
## 快速上手

### 1. 首先安装trac运行必要依赖 NLopt

```bash
sudo apt-get update
sudo apt-get install -y libnlopt-cxx-dev
```
如果你还需要 Python 侧的相关能力，可以额外考虑：
```bash
sudo apt-get install -y libnlopt-dev python3-nlopt
```
### 2. 构建Nlopt
依赖装好后，在工作区里构建：
```bash
cd /home/cnsa/ROS2/Hivecore_os
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3. 安装trac_ik
将代码克隆到你的工作空间 **src** 目录中并编译：
```bash
#### 1. git & colcon编译

cd ~/ros2_ws/src #进入你的工作空间下的src文件内
git clone <本仓库地址> #将本仓库git到src目录下
cd .. #退出src，进入你的工作空间
colcon build #编译

#### 2. 修改配置文件

打开你的 **kinematics.yaml**，将 **kinematics_solver** 修改为：

<your_arm_group>:<-你自己机械臂的规划组名
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  solve_type: Distance
  kinematics_solver_timeout: 0.05

```
---

## 文件详解
### TRAC-IK 说明
#### 1. trac_ik: 这是元包，只负责聚合其他几个包，本身不提供 IK 求解功能，也不生成可执行程序。
#### 2. trac_ik_lib: 核心 IK 求解库，也是最重要的包。负责真正的逆运动学计算，支持多种求解策略：
- `Speed`：尽快返回第一个解
- `Distance`：返回最接近种子解的结果
- `Manip1` / `Manip2`：偏向选择可操作度更好的解
如果你要在自己的 C++ 节点里直接做 IK，应优先使用这个包。
#### 3. trac_ik_examples: 这是示例与测试包,它提供 ik_tests 程序.
用来比较：
- `KDL 默认 IK`
- `TRAC-IK`

主要看两个指标：
- `求解成功率`
- `平均求解时间`

适合做验证和对比测试，不直接用于生产控制。
#### 4. trac_ik_kinematics_plugin: 这是给 MoveIt2 使用的运动学插件包。
```bash
它的作用是替换 MoveIt2 默认的 KDL IK 插件，让 MoveIt2 在规划时调用 TRAC-IK。
如果你是通过 MoveIt2 做机械臂规划，那么主要使用的就是这个包。
在机械臂项目中，机械臂的 `kinematics.yaml` 应当配置为使用该插件。
```
#### 5. trac_ik_python: 这是 Python 包装层，理论上可以在 Python 中直接调用 TRAC-IK。
### 总结
- `trac_ik`：入口
- `trac_ik_lib`：核心求解库
- `trac_ik_examples`：测试与示例
- `trac_ik_kinematics_plugin`：MoveIt2 插件
- `trac_ik_python`：Python 包装层

---
## 贡献与反馈

如果你觉得这个 ROS 2 适配版帮到了你，请点一个 **⭐ Star**！
如果在其他机械臂上遇到问题，欢迎提交 **Issue**。












