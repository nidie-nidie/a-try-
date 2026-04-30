# MuJoCo 仿真接入分析笔记

> 自动生成于 2026-04-28，基于对 `rm_test-dev` 嵌入式项目和 `mujoco_control_extract` 仿真目录的全面代码审查。

---

## 1. 项目背景

这是一个 RoboMaster 平衡步兵机器人的底盘控制系统，原运行于 STM32H723 + FreeRTOS。目标是将其核心控制算法接入 MuJoCo 物理仿真，用于安全快速地调参。

### 机器人物理参数 (robot_param.h)

| 参数 | 值 | 单位 |
|---|---|---|
| 机身质量 | 8.5 | kg |
| 轮子质量 | 0.65 | kg |
| 轮子半径 | 0.0625 | m |
| 轮距 | 0.51175 | m |
| 大腿长度 (L1, L4) | 0.215 | m |
| 小腿长度 (L2, L3) | 0.258 | m |
| 关节间距 (L5) | 0.0 | m |

### 控制架构

```
IMU (BMI088) → INS_Task (Mahony/EKF)
                  ↓ Pitch, Gyro
遥控器 (DT7/PS2) → Remote_Task
                  ↓ v_set, x_set, leg_set, roll_set
         Chassis_Task (状态机)
         ├── ChassisR_Task (右腿控制循环)
         │   ├── VMC_calc_1 (前向运动学: 关节角度 → theta, L0)
         │   ├── LQR_K_calc (查表: 腿长 → 12个增益)
         │   ├── CalcLQR (6状态反馈 → F0, Tp)
         │   ├── PID_Calculate (腿长补偿、防劈叉、roll/turn补偿)
         │   └── VMC_calc_2 (雅可比: F0,Tp → 关节力矩)
         ├── ChassisL_Task (左腿控制循环，镜像)
         └── Motor CAN Tx (力矩指令发送)
```

### 控制模式状态机

```
CHASSIS_OFF → CHASSIS_CALIBRATE → CHASSIS_SAFE → CHASSIS_STAND_UP
                 (校准零点)        (正常平衡)      (倒地起立)
```

---

## 2. 已有仿真基础设施 (`mujoco_control_extract/`)

### 2.1 目录结构

```
mujoco_control_extract/
├── Application/
│   ├── Task/Src/          # Chassis_Task.c, ChassisL_Task.c, ChassisR_Task.c, INS_Task.c 等
│   ├── Task/Inc/          # 对应头文件
│   ├── RobotParam/Inc/    # robot_param.h, Config.h
│   └── TypeDef/Inc/       # struct_typedef.h, robot_typedef.h
├── Components/
│   ├── Algorithm/Src/     # VMC_Calc.c (核心运动学/LQR), Kalman_Filter.c, Mahony_Filter.c 等
│   ├── Controller/Src/    # PID.c
│   ├── Device/Src/        # Motor.c (电机数据结构 + CAN 发送函数)
│   └── Lib/Src/           # User_Lib.c (theta_transform, arm_sin/cos 等工具)
└── sim/
    ├── main_mujoco.c      # [骨架，待完善] MuJoCo 主循环
    ├── sim_adapter.c/h    # [已完成] MuJoCo ↔ 控制代码的桥梁
    ├── CMakeLists.txt     # [已完成] 编译 librm_controller.a + mujoco_bridge
    ├── README_MUJOCO_WSL.md
    └── port/              # 硬件抽象桩
        ├── main.h         # 替代 STM32 main.h: 提供 M_PI, GRAVITY 等
        ├── cmsis_os.h     # 替代 FreeRTOS: osDelay(), pvPortMalloc→malloc
        ├── arm_math.h     # 替代 CMSIS-DSP: arm_sin_f32→sinf, arm_cos_f32→cosf
        ├── bsp_can.h      # 替代 BSP CAN: FDCAN_TxFrame_TypeDef 空结构体
        ├── stm32h723xx.h  # 空文件
        ├── bsp_dwt.h      # 空文件
        ├── bsp_pwm.h      # 空文件
        ├── tim.h          # 空文件
        ├── BMI088driver.h # IMU 数据结构桩
        └── sim_platform.c # 全局变量 + HAL_GetTick/osDelay 桩实现
```

### 2.2 仿真接口 (`sim_adapter.h`)

```c
// === 输入：MuJoCo → 控制器 ===
typedef struct {
    float joint_pos[4];        // 4个关节电机位置 (rad)
    float joint_vel[4];        // 4个关节电机速度 (rad/s)
    float joint_torque_fdb[4]; // 4个关节电机力矩反馈 (Nm)
    float wheel_vel[2];        // 2个轮毂电机速度 (rad/s)
    float roll, pitch, yaw;    // 欧拉角 (rad)
    float gyro[3];             // 角速度 x/y/z (rad/s)
    float accel_body[3];       // 机体加速度 (m/s^2)
    float accel_world[3];      // 世界加速度 (m/s^2)
    float body_x;              // 车体位移 (m)
    float body_v;              // 车体速度 (m/s)
} SimControllerState;

// === 输出：控制器 → MuJoCo ===
typedef struct {
    float joint_torque[4];  // 关节力矩指令 (Nm)
    float wheel_torque[2];  // 轮毂力矩指令 (Nm)
} SimControllerOutput;

// === API ===
void SimController_Init(void);
void SimController_SetState(const SimControllerState *state);
void SimController_SetCommand(float v, float x, float leg, float roll, float yaw);
void SimController_Step(float dt);
void SimController_GetOutput(SimControllerOutput *output);
```

### 2.3 sim_adapter.c 关键逻辑

```c
SimController_Init():
  - INS.ins_flag = 1 (跳过 IMU 收敛等待)
  - chassis_move.start_flag = 1
  - chassis_move.mode = CHASSIS_SAFE (直接进入平衡模式)
  - 调用 ChassisL_init(), ChassisR_init(), Pensation_init()

SimController_SetState():
  - 将 joint_pos/vel/torque 写入 DM_8009_Motor[].Data
  - 将 wheel_vel 写入 LK_9025_Motor[].Data
  - 将 roll/pitch/yaw/gyro/accel 写入 INS 全局结构体
  - 将 body_x/body_v 写入 chassis_move.x_filter/v_filter

SimController_Step():
  - ChassisL_feedback_update() → 关节角度 → 电机坐标系
  - ChassisR_feedback_update() → 关节角度 + IMU → 电机坐标系
  - ChassisR_control_loop() → VMC+LQR+PID → 右腿力矩
  - ChassisL_control_loop() → VMC+LQR+PID → 左腿力矩

SimController_GetOutput():
  - joint_torque[0] = -left.torque_set[1]   // 匹配 CAN 发送时的符号
  - joint_torque[1] = -left.torque_set[0]
  - joint_torque[2] = -right.torque_set[0]
  - joint_torque[3] = -right.torque_set[1]
  - wheel_torque[0] = left.wheel_T
  - wheel_torque[1] = right.wheel_T
```

### 2.4 硬件桩 (sim_platform.c)

| 原始依赖 | 桩行为 |
|---|---|
| `HAL_GetTick()` | 返回累计的 `sim_tick_ms` |
| `osDelay(ms)` | `sim_tick_ms += ms` |
| `USER_FDCAN_AddMessageToTxFifoQ()` | 空函数（力矩不通过 CAN 发出） |
| `GetRcOffline()` | 永远返回 false |
| `ps2_lost` | 永远为 false |
| `DM_8009_Motor[4]` | 完整定义（含 CAN ID），但 Data 由 SetState 填充 |
| `LK_9025_Motor[2]` | 完整定义（含 CAN ID），但 Data 由 SetState 填充 |
| `INS` | 全局实例，由 SetState 填充 |

### 2.5 CMakeLists.txt 编译目标

```
rm_controller (静态库):
  - Chassis_Task.c, ChassisL_Task.c, ChassisR_Task.c
  - VMC_Calc.c, PID.c, Motor.c, User_Lib.c
  - sim_adapter.c, sim_platform.c
  - 编译定义: SIM_MUJOCO
  - 链接: libm

mujoco_bridge (可执行文件):
  - main_mujoco.c
  - 链接: rm_controller, mujoco, libm
```

---

## 3. 当前状态：已完成 vs 待完成

### 已完成 (80%)

- [x] 控制算法代码完整提取
- [x] 硬件抽象层桩代码完整
- [x] sim_adapter 桥接层完整
- [x] CMake 构建系统
- [x] WSL 编译说明
- [x] main_mujoco.c 骨架

### 待完成 (20%)

- [ ] **MuJoCo XML 模型** — 机器人物理模型（MJCF/URDF）
- [ ] **完善 main_mujoco.c** — 填入 MuJoCo 主循环
- [ ] **坐标系对齐** — MuJoCo 欧拉角/角速度轴可能与嵌入式 IMU 有符号或轴差异
- [ ] **编译验证** — 在 WSL 中实际编译通过
- [ ] **仿真调参** — 跑起来后调 PID/LQR 参数

---

## 4. main_mujoco.c 需要填入的 TODO

当前 `main_mujoco.c` 只有骨架：

```c
SimController_Init();
// TODO: 1. Load MJCF
// TODO: 2. Read qpos/qvel into SimControllerState
// TODO: 3. Call SetState/SetCommand/Step
// TODO: 4. Write SimControllerOutput to MuJoCo actuators
printf("skeleton is ready\n");
```

需要补充为完整的 MuJoCo 主循环（详见计划）。

---

## 5. 潜在问题和注意事项

### 5.1 坐标系转换

嵌入式代码中的 IMU 轴映射（Config.h）：
- IMU Pitch 在 index 2，Roll 在 index 1，Yaw 在 index 0
- Gyro Pitch 在 index 0，Roll 在 index 1，Yaw 在 index 2
- 这些是 BMI088 的物理安装方向决定的

MuJoCo 中的欧拉角是 XYZ 顺序（roll→pitch→yaw），和 IMU 的原始数据可能不同。`sim_adapter.c` 直接将 MuJoCo 的 roll/pitch/yaw 赋值给 INS 结构体，**需要验证轴对应关系**。

### 5.2 左腿 Pitch 符号

```c
// ChassisL_feedback_update:
chassis_move.myPithL = 0.0f - INS.Pitch;   // 左腿取反
// ChassisR_feedback_update:
chassis_move.myPithR = INS.Pitch;           // 右腿直接用
```

这暗示左右腿看到的 Pitch 符号相反（因为腿安装在机身两侧，朝向相反）。

### 5.3 关节编号映射

MuJoCo joint 编号需要与代码一致：
- joint_motor[0] = DM_8009_Motor[0] → 左腿 J0 (phi1 关节)
- joint_motor[1] = DM_8009_Motor[1] → 左腿 J1 (phi4 关节)
- joint_motor[2] = DM_8009_Motor[2] → 右腿 J2 (phi1 关节)
- joint_motor[3] = DM_8009_Motor[3] → 右腿 J3 (phi4 关节)
- wheel_motor[0] = LK_9025_Motor[0] → 左轮
- wheel_motor[1] = LK_9025_Motor[1] → 右轮

### 5.4 力矩符号

CAN 发送时的符号约定（注意 `-` 号）：
```c
// 正常平衡模式:
DM_Motor_CAN_TxMessage(..., joint_motor[0], ..., -left.torque_set[1]);  // 注意: [1]
DM_Motor_CAN_TxMessage(..., joint_motor[1], ..., -left.torque_set[0]);  // 注意: [0]
DM_Motor_CAN_TxMessage(..., joint_motor[2], ..., -right.torque_set[0]);
DM_Motor_CAN_TxMessage(..., joint_motor[3], ..., -right.torque_set[1]);
```

`sim_adapter.c` 的 `GetOutput` 已正确匹配这些符号。

### 5.5 初始调试建议

- 关掉校准: `ENABLE_CHASSIS_CALIBRATE = false`
- 关掉跳跃: 不触发 jump_flag
- 关掉倒地自恢复: `recover_flag = 0`
- 先测试 `CHASSIS_SAFE` 模式（正常平衡）
- 初始腿长设 `INIT_LEG_LENGTH = 0.2m`
- dt 用 MuJoCo 的 `opt.timestep`（典型值 0.002-0.005s），控制代码内部 `CHASS_FSM_TIME = 3ms`

---

## 6. 文件清单 (待修改/创建)

| 文件 | 操作 | 说明 |
|---|---|---|
| `sim/robot.xml` | **新建** | MuJoCo MJCF 模型 |
| `sim/main_mujoco.c` | **修改** | 填入 MuJoCo 主循环 |
| `sim/CMakeLists.txt` | 可能需改 | 如果要加 Python 绑定或多线程 |
| `Application/RobotParam/Inc/robot_param.h` | 不改 | 通过编译时修改参数来调参 |

---

## 7. 控制算法核心公式速查

### VMC 前向运动学 (VMC_calc_1)
```
theta = π/2 - Pitch - phi0     // 机身绝对倾角
d_theta = -PitchGyro - d_phi0  // 倾角速率
L0 = sqrt((XC - L5/2)^2 + YC^2)  // 腿长
```

### LQR 状态反馈 (CalcLQR)
```
x = [theta, theta_dot, x, x_dot, phi, phi_dot]
F0 = K[0..5] · x     // 沿腿推力
Tp = K[6..11] · x    // 绕髋扭矩
```

### VMC 雅可比 (VMC_calc_2)
```
torque_set[0] = j11*F0 + j12*Tp  // 前关节
torque_set[1] = j21*F0 + j22*Tp  // 后关节
```

### 离地检测 (ground_detection)
```
FN = F0*cos(θ) + Tp*sin(θ)/L0 + 惯性补偿项
离地条件: mean_filter(FN) < 3.0N
```
