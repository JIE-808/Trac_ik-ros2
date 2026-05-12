# trac_ik_kinematics_plugin

## 作用

`trac_ik_kinematics_plugin` 是面向 MoveIt2 的运动学插件，用来把默认的 KDL IK 插件替换成 TRAC-IK。

它的定位很明确：

- 你继续使用 MoveIt2 的规划、碰撞检测和执行流程
- 但在“给定末端目标位姿，先求一组关节解”这一步，改由 TRAC-IK 来完成

## 适用场景

- 已经在用 MoveIt2，但默认 KDL IK 成功率不够稳定
- 机械臂接近奇异位姿或关节限位较复杂
- 想通过 `kinematics.yaml` 低侵入替换 IK 后端

## 依赖

这个包除了 `trac_ik_lib`，还依赖 MoveIt2 相关组件：

- `moveit_core`
- `pluginlib`
- `tf2_kdl`
- `tf2_geometry_msgs`
- `kdl_parser`
- `urdf`
- `rclcpp`

建议先执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
colcon build --packages-up-to trac_ik_kinematics_plugin
source install/setup.bash
```

## 如何接入 MoveIt2

在你的 MoveIt 配置包里找到 `kinematics.yaml`，把目标规划组的 `kinematics_solver` 改成：

```yaml
arm:
    kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
    kinematics_solver_timeout: 0.01
    position_only_ik: false
    solve_type: Speed
```

如果你是双臂或多规划组模型，就分别为每个 group 配置。

## 参数说明

### kinematics_solver_timeout

单次 IK 求解超时，单位为秒。

### position_only_ik

是否只关注位置，不严格约束姿态。某些抓取或对位任务中，这个选项可以提高求解成功率。

### solve_type

TRAC-IK 求解策略，可选：

- `Speed`
- `Distance`
- `Manipulation1`
- `Manipulation2`

通常先从 `Speed` 开始；如果你更关心返回解和当前关节状态的接近程度，可以尝试 `Distance`。

## ROS2 下的参数读取方式

这个插件在 ROS2/MoveIt2 中通常从 `robot_description_kinematics.<group_name>` 这棵参数树读取组配置，而不是沿用 ROS1 时期的旧式读法。

因此你只要把配置写进对应 group 的 `kinematics.yaml`，由 MoveIt2 配置系统注入即可。

## 行为边界

这个包只替换 IK 阶段，不替换 MoveIt2 的整体规划器。

也就是说：

- 末端位姿到关节解这一步，由 TRAC-IK 完成
- 后续路径搜索、碰撞检测、轨迹平滑，仍然是 MoveIt2 自己的流程

如果你调用的是笛卡尔路径 API，MoveIt2 仍然是沿插值路径不断做 IK，而不是进行完整的笛卡尔空间搜索。

## 已知限制

- 目前仍不支持 mimic joints
- 该插件要求机器人模型、关节限位和 group 定义本身是正确的
- 如果 MoveIt2 配置里 group、base frame 或 tip frame 写错，插件也无法正常工作
