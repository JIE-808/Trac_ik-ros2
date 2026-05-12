# trac_ik_python

## 作用

`trac_ik_python` 是 `trac_ik_lib` 的 Python 封装，基于 SWIG 生成绑定，适合以下场景：

- 写实验脚本快速验证 IK
- 在 Python 节点中做姿态到关节解的求解
- 做数据生成、批量采样、离线测试

它不是完整的 ROS 消息层封装，而是把核心 IK 能力直接暴露给 Python。

## ROS2 版本特点

这个包已经适配到 ROS2/ament：

- 使用 Python3
- 不再依赖 `rospy`
- 支持直接传入 URDF 字符串
- 也支持从 ROS2 节点参数中读取 URDF

## 依赖

- ROS2
- `trac_ik_lib`
- `swig`
- `python3-dev`
- `python3-numpy`
- `rclpy`
- `libnlopt-dev`

建议先执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
colcon build --packages-up-to trac_ik_python
source install/setup.bash
```

## 最常用的调用方式

### 方式一：直接传入 URDF 字符串

```python
from trac_ik_python.trac_ik import IK

with open("/path/to/robot.urdf", "r", encoding="utf-8") as stream:
  urdf_string = stream.read()

ik_solver = IK(
  "base_link",
  "tool0",
  urdf_string=urdf_string,
)

seed_state = [0.0] * ik_solver.number_of_joints

solution = ik_solver.get_ik(
  seed_state,
  0.45, 0.10, 0.30,
  0.0, 0.0, 0.0, 1.0,
)

print(solution)
```

### 方式二：从 ROS2 参数读取 URDF

```python
import rclpy
from rclpy.node import Node

from trac_ik_python.trac_ik import IK


class IkNode(Node):
  def __init__(self):
    super().__init__("trac_ik_example")
    self.declare_parameter("robot_description", "")

    self.ik_solver = IK(
      "base_link",
      "tool0",
      node=self,
      urdf_parameter="robot_description",
    )


rclpy.init()
node = IkNode()
```

## 常用接口

### 获取关节数

```python
ik_solver.number_of_joints
```

### 求逆解

```python
solution = ik_solver.get_ik(
  seed_state,
  x, y, z,
  qx, qy, qz, qw,
)
```

### 带容差求解

```python
solution = ik_solver.get_ik(
  seed_state,
  x, y, z,
  qx, qy, qz, qw,
  bx, by, bz,
  brx, bry, brz,
)
```

其中：

- `bx by bz` 是位置容差
- `brx bry brz` 是姿态容差

## 自检和调试

你可以读取求解器内部识别到的链信息：

```python
print(ik_solver.base_link)
print(ik_solver.tip_link)
print(ik_solver.joint_names)
print(ik_solver.link_names)
```

也可以检查并修改关节限位：

```python
lower_bound, upper_bound = ik_solver.get_joint_limits()
ik_solver.set_joint_limits(lower_bound, upper_bound)
```

## 自带测试脚本

这个包安装后会带两个测试脚本：

- `test_pkg.py`：通过高层 Python 封装测试 IK
- `test_wrapper.py`：直接测试底层 SWIG 包装对象

运行方式示例：

```bash
ros2 run trac_ik_python test_pkg.py --urdf /path/to/robot.urdf --base-link base_link --tip-link tool0
```

```bash
ros2 run trac_ik_python test_wrapper.py --urdf /path/to/robot.urdf --base-link base_link --tip-link tool0
```

## 注意事项

- 输入位姿必须在求解器的基坐标系下表达
- 这个包不直接处理 TF 变换，也不要求你传入 ROS Pose 消息
- 如果 URDF 根链接带惯量，底层 KDL 可能打印警告，这不是 Python 包本身的问题
- 如果你只是想在生产级 C++ 控制器里集成，优先考虑直接使用 `trac_ik_lib`
