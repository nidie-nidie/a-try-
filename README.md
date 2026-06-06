# rm_control 工作区总览


## third_party 第三方库管理

当前工程已经预留了顶层 `third_party/`，用来统一管理会影响编译的第三方库。最直接的用法是把 MuJoCo SDK 放成下面这种结构：

```text
third_party/mujoco/include/mujoco/mujoco.h
third_party/mujoco/lib/libmujoco.so

```

实际 SDK 里如果是 `libmujoco.so.3.3.0` 这种版本化文件也可以，CMake 会在 `third_party/mujoco/lib` 里自动查找。

CMake 会按这个顺序找 MuJoCo：

```text
1. cmake -DMUJOCO_ROOT=/path/to/mujoco 显式指定
2. third_party/mujoco
3. 环境变量 MUJOCO_ROOT
4. 仓库同级目录 ../mujoco-3.3.0 或 ../mujoco
```

工程源码里通过 `third_party/include/rm_third_party/` 下的 wrapper 头文件引用第三方库，例如：

```c
#include "rm_third_party/mujoco.h"
#include "rm_third_party/glfw.h"
```

所以以后新增第三方库时，推荐做两件事：库本体放到 `third_party/<lib_name>/`，再在 `third_party/include/rm_third_party/` 下面加一个小 wrapper 头文件。具体约定见 `third_party/README.md`。

这个仓库是轮腿机器人控制与 MuJoCo 仿真的综合工作区。它不是单一工程，而是同时保存了几套不同用途的内容：

- 从实车嵌入式工程抽取出来的 C 控制代码。
- 接入 MuJoCo 的主线仿真 bridge。
- 专门调腿部机构、闭链、碰撞和 IK/VMC 的调试 viewer。
- URDF/MJCF/mesh 模型资源。
- 原始 STM32 工程和早期 Python 仿真参考。
- 构建产物、日志、缓存和本机工具配置。

整理这个仓库时，最重要的是分清两件事：

```text
源文件 / 模型资源 / 文档
    应该进 GitHub。

编译产物 / 运行日志 / 缓存 / 本机环境
    不应该进 GitHub。
```

## 顶层目录

| 路径 | 内容 | 当前用途 |
| --- | --- | --- |
| `mujoco_control_extract/` | 主线 C 控制代码和 MuJoCo bridge。 | 当前最重要的仿真控制工程。 |
| `mujoco_control_leg_debug/` | 独立腿部调试 viewer，含调试控制器、模型、mesh。 | 用来单独调五连杆腿部几何、关节映射、碰撞和辅助线。 |
| `wheel_leg_urdf4/` | ROS/URDF/MJCF 导出的机器人模型资源。 | 模型源资源库，不直接承担控制。 |
| `rm_test_dev/` | 原始 STM32 工程、旧 Python MuJoCo 仿真、工具和文档。 | 参考工程和历史版本。 |
| `third_party/` | 第三方库和统一 wrapper 头文件。 | 优先放 MuJoCo SDK，也可扩展 GLFW 等依赖。 |
| `build/` | 顶层旧 CMake 构建输出。 | 编译产物，不应长期跟踪。 |
| `build_leg_debug/` | `mujoco_control_leg_debug/` 的构建输出。 | 编译产物，不应长期跟踪。 |
| `build_origin_stand/` | 旧 standing/origin 调试构建输出。 | 编译产物，不应长期跟踪。 |
| `build_pos_debug/` | 旧位置调试构建输出。 | 编译产物，不应长期跟踪。 |
| `logs/` | 运行日志。 | 调试临时文件，不应长期跟踪。 |
| `.venv/` | Python 虚拟环境。 | 本机环境，不应上传。 |
| `.vscode/` | VS Code 配置。 | 共享调试配置可保留，个人配置需谨慎。 |
| `.claude/`, `.agents/`, `.codex` | 本地工具或助手状态文件。 | 通常不应上传。 |
| `MUJOCO_LOG.TXT` | MuJoCo 运行日志。 | 运行产物，不应长期跟踪。 |
| `串腿控制.pdf` | 轮腿/串腿控制相关资料。 | 文档资料，可保留。 |


## `mujoco_control_extract/`

这是当前主线的 C 语言 MuJoCo 控制工程。它的目标是把原来实车上的底盘控制代码尽量复用到 MuJoCo 里：MuJoCo 负责物理世界，原控制器负责根据状态算力矩。

### 子目录和文件

| 路径 | 内容 |
| --- | --- |
| `Application/RobotParam/Inc/` | 机器人参数配置，例如关节零点、方向、腿长、PID/LQR 参数等。当前常看的 `robot_param.h` 就在这里。 |
| `Application/Task/Inc/` | 任务层头文件，例如 `Chassis_Task.h`, `ChassisL_Task.h`, `ChassisR_Task.h`, `INS_Task.h`。 |
| `Application/Task/Src/` | 任务层实现。左右腿反馈、左右腿控制、底盘状态机和控制逻辑主要在这里。 |
| `Application/TypeDef/Inc/` | 工程共用结构体和类型定义。 |
| `Components/Algorithm/` | 算法模块，例如 VMC、滤波、Ramp、Kalman、Mahony、QuaternionEKF。 |
| `Components/Controller/` | 控制器模块，例如 PID。 |
| `Components/Device/` | 设备抽象，例如电机和遥控器结构。 |
| `Components/Lib/` | 工具函数和基础库。 |
| `sim/` | MuJoCo bridge 工程。 |
| `sim/main_mujoco.c` | MuJoCo 主程序：加载模型、读取状态、处理键盘/模式、写入 actuator ctrl。 |
| `sim/sim_adapter.c/.h` | MuJoCo 与原 C 控制器之间的适配层。 |
| `sim/port/` | 为了让嵌入式代码在 PC/MuJoCo 下编译而写的兼容头文件和 stub。 |
| `sim/models/` | 当前 bridge 使用的 MuJoCo XML 模型。 |
| `sim/build_wheel_leg_model.py` | 构建或处理模型的辅助脚本。 |
| `README.md` | 当前主线仿真的运行命令、按键、模式说明和一些 C/MuJoCo 学习笔记。 |
| `sim/README_MUJOCO_WSL.md` | MuJoCo bridge 接入流程和控制链路细节。 |
| `STAND_TUNING_LOG.md` | 站立/调参记录。 |
| `MJMODEL.TXT` | MuJoCo 模型导出的文本 dump，文件较大，后续可判断是否需要长期保留。 |
| `build/` | 子工程 CMake 构建输出，不属于源码。 |
| `mjmodel.mjb` | MuJoCo 编译出的二进制模型，属于生成产物，不应进 GitHub。 |
| `MUJOCO_LOG.TXT` | MuJoCo 日志，不应长期跟踪。 |

### 主线控制链路

`mujoco_control_extract/` 的控制链路是：

```text
MuJoCo 模型状态
    ↓
sim/main_mujoco.c 读取 qpos / qvel / base 姿态 / 轮速
    ↓
SimControllerState
    ↓
sim/sim_adapter.c 写入原工程变量
    - INS
    - DM_8009_Motor
    - LK_9025_Motor
    - chassis_move
    ↓
ChassisL_feedback_update()
ChassisR_feedback_update()
    ↓
ChassisR_control_loop()
ChassisL_control_loop()
    ↓
ChassisConsole()
    ↓
SimControllerOutput
    ↓
sim/main_mujoco.c 写入 MuJoCo actuator ctrl
    ↓
mj_step()
```

这套链路的重点是：**尽量复用实车 C 控制逻辑**。MuJoCo 只是替代真实世界、IMU、电机反馈和 CAN 输出。

### 当前主线仿真入口

常用编译命令：

```bash
cmake -S mujoco_control_extract/sim -B mujoco_control_extract/build
cmake --build mujoco_control_extract/build -j
```

如果 MuJoCo 没有放在 `third_party/mujoco`，也可以显式指定：

```bash
cmake -DMUJOCO_ROOT=/path/to/mujoco -S mujoco_control_extract/sim -B mujoco_control_extract/build
```

常用运行命令：

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive stand
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive forward
./mujoco_control_extract/build/mujoco_bridge --headless --time 3 --mode safe --drive stand
```

## `mujoco_control_leg_debug/`

这是独立的腿部调试工程，不走完整底盘控制状态机，也不复用 `mujoco_control_extract/build/`。它主要服务于五连杆腿部几何、闭链约束、IK/VMC、关节符号、碰撞模型和辅助线检查。

### 子目录和文件

| 路径 | 内容 |
| --- | --- |
| `CMakeLists.txt` | 腿部调试 viewer 的 CMake 入口。 |
| `README.md` | 腿部调试 viewer 的构建、运行、键盘和 XML 模式说明。 |
| `include/leg_debug.h` | 调试状态结构、常量、模型映射等定义。 |
| `include/control_task_space.h` | task-space 控制接口。 |
| `include/control_joint_space.h` | joint-space 控制接口。 |
| `include/debug_draw.h` | MuJoCo 场景辅助线绘制接口。 |
| `include/model_helpers.h` | 模型查找、关节映射等辅助函数接口。 |
| `src/main.c` | viewer 主程序、命令行参数、键盘、仿真循环。 |
| `src/control_task_space.c` | 根据虚拟腿长 `L0` 和角度 `phi0` 解算关节目标。 |
| `src/control_joint_space.c` | 直接调四个主动关节目标角。 |
| `src/debug_draw.c` | 绘制髋轴、轮轴、虚拟腿长等辅助线。 |
| `src/model_helpers.c` | MuJoCo 模型中的 joint/actuator/site 查找与映射。 |
| `src/logging.c` | 周期性打印调试量。 |
| `models/wheel_leg_debug.xml` | 默认浮空悬挂调试模型。 |
| `models/wheel_leg_debug_collision.xml` | 自由底盘接触/碰撞增强模型。 |
| `models/wheel_leg_debug_collision_self_mesh_all.xml` | 整车 mesh 全自碰撞测试模型。 |
| `models/assets/` | 调试模型使用的 STL/OBJ/PNG 资源。 |

### 腿部调试控制链路

`mujoco_control_leg_debug/` 的控制链路是：

```text
MuJoCo 当前关节状态
    ↓
leg_debug_update_measurements()
    ↓
选择控制模式
    ├─ task-space:
    │      键盘给 L0 / phi0
    │      ↓
    │      IK / VMC 几何解算目标关节角
    │
    └─ joint-space:
           键盘直接给四个主动关节目标角
    ↓
软件 PD
    ↓
可选 mj_inverse() hold feedforward
    ↓
限幅
    ↓
写入四个腿部 actuator
    ↓
mj_step()
```

这套链路的重点是：**调腿部机构本身**。它不关心完整车体平衡，也基本不控制轮子。

### 与主线仿真的区别

| 对比项 | `mujoco_control_extract/` | `mujoco_control_leg_debug/` |
| --- | --- | --- |
| 目标 | 复现实车底盘控制闭环。 | 调腿部几何、IK、关节映射和碰撞。 |
| 控制输入 | 速度、位置、腿长、roll/yaw、状态机模式。 | 键盘直接调 `L0/phi0` 或关节角。 |
| 控制核心 | 原 C 工程的 `ChassisL/R_control_loop()`。 | 本目录自己的 task-space/joint-space 调试控制。 |
| 是否控制轮子 | 是。 | 基本不控制，轮子 actuator 通常置 0。 |
| 是否依赖原工程状态 | 是，依赖 INS、电机反馈、chassis_move。 | 否，使用 `LegDebugState`。 |
| 最适合解决的问题 | 站立、前进、力矩输出、实车逻辑移植。 | 五连杆几何、闭链姿态、接触、符号和可视化辅助线。 |

## `wheel_leg_urdf4/`

这是机器人模型资源目录，主要保存 URDF、MJCF、mesh 和转换脚本。它更像“模型源仓库”，不是控制器工程。

### 子目录和文件

| 路径 | 内容 |
| --- | --- |
| `wheel_leg_urdf4/urdf/` | 原始 URDF、CSV 和从 URDF 转 MuJoCo 的脚本。 |
| `wheel_leg_urdf4/meshes/` | URDF 引用的 STL mesh。 |
| `wheel_leg_urdf4/MJCF/` | 从模型导出的 MJCF 版本以及对应 STL/贴图。 |
| `wheel_leg_urdf4/mujoco_export_obj/` | 导出为 OBJ/mesh 版本的 MuJoCo 模型和资源。 |
| `wheel_leg_urdf4/mujoco_import/` | MuJoCo import 相关资源链接。 |
| `wheel_leg_urdf4/config/` | ROS/Gazebo 相关关节名配置。 |
| `wheel_leg_urdf4/launch/` | ROS/Gazebo 展示和仿真 launch 文件。 |
| `wheel_leg_urdf4/CMakeLists.txt`, `package.xml` | ROS 包描述。 |
| `wheel_leg_urdf4/export.log` | 导出日志，属于生成记录，后续可考虑不跟踪。 |

### 模型资源链路

`wheel_leg_urdf4/` 本身没有控制闭环。它的链路更像：

```text
CAD / URDF / CSV
    ↓
wheel_leg_urdf4/urdf/
    ↓
导出脚本
    ↓
MJCF / mesh / OBJ
    ↓
被 mujoco_control_extract 或 mujoco_control_leg_debug 引用
```

这套目录的重点是：**模型输入和模型转换**。如果控制器跑不起来，这里主要检查 joint 名字、mesh 路径、坐标系和模型导出是否正确。

## `rm_test_dev/`

这是原始参考工程和历史实验目录，里面内容较杂，但很有价值。整理时不要急着删，可以先当作“资料库”和“对照版本”。

### 子目录和文件

| 路径 | 内容 |
| --- | --- |
| `rm_test-dev/` | 原始 STM32/FreeRTOS 工程。 |
| `rm_test-dev/Core/` | STM32CubeMX 生成的 core 初始化代码。 |
| `rm_test-dev/Application/` | 实车应用层任务、机器人参数、音乐等。 |
| `rm_test-dev/Components/` | 算法、控制器、设备、基础库。 |
| `rm_test-dev/BSP/` | 板级外设驱动封装，例如 CAN、GPIO、PWM、UART、DWT。 |
| `rm_test-dev/Drivers/` | STM32 HAL/CMSIS 驱动。 |
| `rm_test-dev/Middlewares/` | FreeRTOS、USB 等中间件。 |
| `rm_test-dev/MDK-ARM/` | Keil/MDK 工程文件。 |
| `rm_test-dev/USB_DEVICE/` | USB CDC 相关代码。 |
| `rm_test-dev/SystemView/` | SEGGER SystemView 配置和源码。 |
| `rm_test-dev/doc/` | 用户指南、算法/建模 PDF。 |
| `rm_test-dev/tools/` | 电机手册、上位机工具、Matlab LQR 计算脚本。 |
| `rm_test-dev/mujoco_control_extract/` | 原始工程内部的一份 MuJoCo 接入副本/旧版本。 |
| `wheel_leg_mujoco/` | 早期 Python MuJoCo 仿真原型。 |
| `*.zip` | 原始工程或旧 Python 工程压缩包，属于归档/备份。 |

### 原始实车控制链路

`rm_test_dev/rm_test-dev/` 的控制链路接近真实嵌入式运行方式：

```text
电机 / IMU / 遥控器 / PS2 / CAN
    ↓
BSP 驱动和设备层
    ↓
INS_Task / Remote_Task / PS2_Task / Detect_Task 等任务
    ↓
Chassis_Task 调度底盘状态
    ↓
ChassisL_Task / ChassisR_Task
    ↓
VMC / LQR / PID / 滤波
    ↓
Motor.c 组织电机命令
    ↓
CAN 输出到真实电机
```

这套链路的重点是：**实车工程**。它包含硬件初始化、外设、中断、RTOS、遥控器、电机协议和真实 CAN 输出。

### Python 旧仿真链路

`rm_test_dev/wheel_leg_mujoco/` 是早期 Python 仿真原型，链路大致是：

```text
Simulation.py
    ↓
LegWheelRobot 加载 MJCF/env.xml
    ↓
sensor_read_data() 从 MuJoCo sensor 读姿态、关节、轮速
    ↓
VMC.py 计算五连杆几何和腿部力矩
    ↓
actuator_set_torque() 写入 MuJoCo ctrl
    ↓
mujoco.mj_step()
```

这套链路的重点是：**快速验证 VMC 几何和 Python 逻辑**。它不是当前主线，因为主线已经转向 C 语言 bridge，以便复用实车控制代码。

## 已有 README 汇总

| 文件 | 主要内容 | 什么时候看 |
| --- | --- | --- |
| `mujoco_control_extract/README.md` | 主线 MuJoCo 仿真的运行命令、`--mode safe`、`--drive stand/forward`、键盘控制，以及一些 C/MuJoCo/Matlab 学习笔记。 | 想直接运行当前主线仿真时先看。 |
| `mujoco_control_extract/sim/README_MUJOCO_WSL.md` | MuJoCo bridge 接入流程、模型命名要求、状态映射、编译、GUI/headless 运行、输出数据解释和调试顺序。 | 想理解 `main_mujoco.c` 和 `sim_adapter.c` 怎么把 MuJoCo 接到原控制器时看。 |
| `mujoco_control_leg_debug/README.md` | 腿部调试 viewer 的构建运行、键盘、task-space/joint-space、不同 XML 碰撞模式、日志字段和辅助线说明。 | 想单独调腿部闭链、碰撞、IK/VMC 和关节符号时看。 |
| `rm_test_dev/rm_test-dev/README.md` | 原始 STM32 工程背景、硬件资料、电机/遥控器参考、五连杆算法资料、PS2 控制说明、实车方向定义。 | 查硬件、实车工程、遥控器、电机资料时看。 |
| `rm_test_dev/rm_test-dev/mujoco_control_extract/sim/README_MUJOCO_WSL.md` | 原始工程内部旧版 MuJoCo 接入说明。 | 和当前 `mujoco_control_extract/sim/README_MUJOCO_WSL.md` 对照时看。 |

## 各目录控制链路对比

| 目录 | 控制目标 | 输入 | 控制核心 | 输出 | 适合调什么 |
| --- | --- | --- | --- | --- | --- |
| `rm_test_dev/rm_test-dev/` | 实车平衡步兵控制。 | 真实 IMU、电机反馈、遥控器、PS2、CAN。 | FreeRTOS 任务 + Chassis/VMC/LQR/PID。 | CAN 电机命令。 | 硬件、实车任务、遥控器、电机协议。 |
| `mujoco_control_extract/` | 在 MuJoCo 中复现实车底盘闭环。 | MuJoCo qpos/qvel/base 姿态/轮速。 | `sim_adapter` + 原 C 底盘控制代码。 | MuJoCo actuator ctrl。 | 站立、前进、力矩映射、实车逻辑移植。 |
| `mujoco_control_leg_debug/` | 单独调腿部机构。 | MuJoCo 关节状态 + 键盘目标。 | 本目录 task-space/joint-space 调试控制。 | 四个腿部 actuator torque。 | 五连杆几何、闭链、碰撞、关节符号。 |
| `rm_test_dev/wheel_leg_mujoco/` | 早期 Python 快速仿真。 | MuJoCo sensor + Python 变量。 | Python VMC。 | MuJoCo ctrl。 | 快速验证 VMC 公式和 Python 原型。 |
| `wheel_leg_urdf4/` | 提供模型资源。 | URDF/CAD/CSV/mesh。 | 导出脚本和模型文件。 | URDF/MJCF/XML/STL/OBJ。 | joint 名字、坐标系、mesh 路径、模型导出。 |

## 源文件和生成物判断

### 应该进 GitHub

- C 源码和头文件：`*.c`, `*.h`
- CMake 工程文件：`CMakeLists.txt`
- Python 脚本：`*.py`
- MuJoCo / URDF / ROS 描述文件：`*.xml`, `*.urdf`, `*.yaml`, `*.csv`
- 仿真必需模型资源：`*.STL`, `*.obj`, `*.png`
- 说明文档和调参记录：`README.md`, `*.md`, 必要 PDF
- 必要的项目配置：`.gitignore`、可共享的 `.vscode` 配置

### 不应该进 GitHub

- CMake 构建目录：`build/`, `build_*`, `build-*`, `cmake-build-*`
- CMake 临时文件：`CMakeCache.txt`, `CMakeFiles/`, `cmake_install.cmake`, `compile_commands.json`
- 编译产物：`*.o`, `*.a`, `*.so`, `*.dll`, `*.exe`, `a.out`
- MuJoCo 生成二进制模型：`*.mjb`
- 运行日志：`MUJOCO_LOG.TXT`, `logs/*.log`
- Python 缓存：`__pycache__/`, `*.pyc`
- Python 虚拟环境：`.venv/`
- Windows/WSL 拷贝残留：`*:Zone.Identifier`
- 已经解压且可重新获得的压缩包：`*.zip`

## 当前整理建议

1. `dev` 可以作为“整理前但已经能推到 GitHub 的主分支”。
2. 继续在 `cleanup/workspace-files` 或新的 cleanup 分支里删构建产物、缓存、日志。
3. 不要先大规模移动源码目录，先确认 CMake、XML include、mesh 路径不会断。
4. 大于 50 MB 的模型资源可以先保留，但后续建议判断是否改用 Git LFS。
5. `mujoco_control_extract/mjmodel.mjb` 属于生成物，不应该再进入 Git 历史。
