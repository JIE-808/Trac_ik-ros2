# trac_ik

## 简介

`trac_ik` 是这个 TRAC-IK 功能集合的元包，用来把以下几个子包组织在一起：

- `trac_ik_lib`：C++ 求解核心库
- `trac_ik_examples`：示例与对比测试程序
- `trac_ik_kinematics_plugin`：MoveIt2 运动学插件
- `trac_ik_python`：Python 绑定

它本身不提供可执行程序，也不包含核心算法实现，主要作用是方便统一构建、安装和分发。

## 适用场景

- 想把整套 TRAC-IK 功能一次性加入 ROS2 工作区
- 想统一安装 C++ 库、MoveIt2 插件和 Python 接口
- 想对外发布一组完整的 TRAC-IK 包，而不是单独发某一个子包

## 包结构

### 1. trac_ik_lib

核心逆运动学库。直接提供 `TRAC_IK::TRAC_IK` 求解器，适合 C++ 程序直接链接调用。

### 2. trac_ik_examples

示例和测试包。当前主要用于比较 TRAC-IK 与 KDL IK 的求解成功率和耗时。

### 3. trac_ik_kinematics_plugin

MoveIt2 运动学插件。用于在 `kinematics.yaml` 中替换默认 KDL 插件，让 MoveIt2 在 IK 阶段调用 TRAC-IK。

### 4. trac_ik_python

Python 封装。通过 SWIG 暴露 Python 接口，方便脚本或实验代码快速调用 IK。

## 依赖说明

这套包不是“只装 SWIG 和 NLopt 就能编译”的最小裸包，它还依赖标准 ROS2 组件；如果需要 MoveIt2 插件，还需要额外安装 MoveIt2。

建议在工作区根目录执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建方法

如果你要构建整套包：

```bash
colcon build --packages-up-to trac_ik
source install/setup.bash
```

如果你只需要其中某一个功能，可以单独构建对应子包。

## 选型建议

- 只想在 C++ 代码里做 IK：使用 `trac_ik_lib`
- 想在 MoveIt2 里替换 KDL：使用 `trac_ik_kinematics_plugin`
- 想用 Python 快速试验：使用 `trac_ik_python`
- 想对比性能或验证效果：使用 `trac_ik_examples`

## 说明

这个元包主要承担“打包入口”的角色，因此真正的使用方式、参数和示例请分别查看各子包自己的 README。