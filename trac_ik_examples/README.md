# trac_ik_examples

## 作用

`trac_ik_examples` 是示例与测试包，主要用来演示如何调用 `trac_ik_lib`，以及如何把 TRAC-IK 和 KDL IK 在同一组样本上做对比。

目前这个包的核心程序是：

- `ik_tests`：对比 TRAC-IK 与 KDL 在一条机械臂链上的求解成功率和平均耗时

## 适用场景

- 想快速验证 TRAC-IK 是否能在某个 URDF 上跑起来
- 想粗略比较 TRAC-IK 与 KDL 的求解表现
- 想参考一个最小可运行的 C++ 示例

## 依赖

- ROS2
- `trac_ik_lib`
- `orocos_kdl`
- `rclcpp`
- `libnlopt-dev`

建议先执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
colcon build --packages-up-to trac_ik_examples
source install/setup.bash
```

## 可执行文件

### ik_tests

这个程序会：

- 从 `robot_description` 读取 URDF
- 根据给定的 `chain_start` 和 `chain_end` 建立机械臂链
- 生成随机关节样本
- 使用正运动学得到目标位姿
- 分别调用 KDL 和 TRAC-IK 求逆解
- 统计成功次数和平均求解时间

因此，它更适合做“可达性和效率对比”，而不是生产环境控制程序。

## 运行方法

### 方法一：直接运行 launch 文件

包内自带 PR2 示例启动文件：

```bash
ros2 launch trac_ik_examples pr2_arm.launch.py
```

默认参数包括：

- `num_samples`：采样数量，默认 `1000`
- `chain_start`：链起点，默认 `torso_lift_link`
- `chain_end`：链终点，默认 `r_wrist_roll_link`
- `timeout`：单次 IK 超时，默认 `0.005`

例如：

```bash
ros2 launch trac_ik_examples pr2_arm.launch.py num_samples:=2000 timeout:=0.01
```

### 方法二：自己给节点提供参数

如果你已经在别的 launch 里准备好了 `robot_description`，也可以直接启动 `ik_tests` 并传入同名参数。

## 输出结果怎么看

你通常会看到这类统计信息：

- 样本总数
- KDL 成功次数
- TRAC-IK 成功次数
- 两者平均求解耗时

一般来说：

- TRAC-IK 往往成功率更高
- `Speed` 模式下的单次调用耗时通常仍然足够低
- 若链接近奇异位形或关节限位复杂，TRAC-IK 的优势更明显

## 说明

这个包的重点是“演示和对比”，不是给生产系统提供稳定 API。若你要在业务代码里直接集成，请参考 `trac_ik_lib` 的 README。
