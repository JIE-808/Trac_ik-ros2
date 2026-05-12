# trac_ik_lib

## 作用

`trac_ik_lib` 是 TRAC-IK 的核心 C++ 求解库，提供逆运动学求解器 `TRAC_IK::TRAC_IK`。如果你想在自己的 ROS2/C++ 节点里直接做 IK，而不是通过 MoveIt2 插件或 Python 封装，那么这个包就是最核心的部分。

相比传统 KDL 伪逆 Jacobian 求解方式，TRAC-IK 的目标是：

- 更高的求解成功率
- 更稳定的超时控制
- 更灵活的求解策略选择
- 支持分别设置笛卡尔六维容差

## 适用场景

- 在自定义控制节点中直接计算关节解
- 不依赖 MoveIt2，只需要一个独立 IK 求解器
- 想替换 KDL IK，但仍保留接近 KDL 的调用习惯
- 需要在超时范围内尽量找到可用解

## ROS2 版本的关键变化

在 ROS1 时代，构造函数常常依赖全局参数服务器中的 `/robot_description`。ROS2 没有全局参数服务器，因此这个版本不再默认从全局参数名取 URDF，而是要求调用者自己提供 URDF 字符串。

也就是说，这里的 `urdf_param` 实际上应传入完整 URDF XML 字符串，而不是参数名。

URDF 字符串通常有三种获取方式：

- 在程序中直接读取 `.urdf` 或 `.xacro` 展开后的 XML
- 从当前 ROS2 节点参数中读取 `robot_description`
- 从 `robot_description` 相关话题或其他上游模块接收后传入

## 依赖

至少需要以下基础依赖：

- ROS2
- `kdl_parser`
- `urdf`
- `rclcpp`
- `libnlopt-dev`
- `libnlopt-cxx-dev`

建议在工作区根目录执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
colcon build --packages-up-to trac_ik_lib
source install/setup.bash
```

## 头文件和链接

头文件：

```c++
#include <trac_ik/trac_ik.hpp>
```

在自己的 `CMakeLists.txt` 中：

```cmake
find_package(trac_ik_lib REQUIRED)

target_link_libraries(your_target trac_ik_lib::trac_ik_lib)
ament_target_dependencies(your_target rclcpp kdl_parser urdf)
```

## 基本用法

### 方式一：直接传入 URDF 字符串

```c++
#include <fstream>
#include <sstream>
#include <trac_ik/trac_ik.hpp>

std::ifstream stream("/path/to/robot.urdf");
std::stringstream buffer;
buffer << stream.rdbuf();

TRAC_IK::TRAC_IK ik_solver(
  "base_link",
  "tool0",
  buffer.str(),
  0.005,
  1e-5,
  TRAC_IK::SolveType::Speed);
```

### 方式二：基于 KDL 链和关节限位构造

```c++
TRAC_IK::TRAC_IK ik_solver(
  chain,
  lower_joint_limits,
  upper_joint_limits,
  0.005,
  1e-5,
  TRAC_IK::SolveType::Speed);
```

### 求解调用

```c++
KDL::JntArray result;
KDL::Twist tolerances;

int rc = ik_solver.CartToJnt(
  seed_joints,
  desired_pose,
  result,
  tolerances);

if (rc >= 0) {
  // 求解成功
}
```

## 求解策略

`SolveType` 支持以下几种模式：

- `Speed`：优先快速返回第一个可用解
- `Distance`：在超时范围内搜索，并返回最接近初始种子解的结果
- `Manip1`：偏向雅可比可操作性更高的解
- `Manip2`：偏向奇异性更低、数值条件更好的解

如果你的控制流程对“尽快有解”更敏感，优先用 `Speed`。如果你更希望解平滑、靠近当前关节状态，可尝试 `Distance`。

## 和 KDL IK 的区别

KDL 常用方式是通过迭代次数控制终止，而 TRAC-IK 直接使用超时秒数控制，更适合实时系统或需要统一计算预算的场景。

同时，TRAC-IK 允许为末端位姿的六个维度单独设置容差，因此对“位置要准、姿态可放宽”这类任务更友好。

## 注意事项

- 输入位姿必须和 `base_link` 所在坐标系一致
- 连续关节会被视为无界或大范围处理
- 若 URDF 根链接带惯量，KDL 可能输出警告，这属于 KDL 的已知限制
- 仅有 `libnlopt-dev` 或 `swig` 并不足以支撑整个 TRAC-IK 体系，`trac_ik_lib` 仍然依赖标准 ROS2 解析与模型组件
