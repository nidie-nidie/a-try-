# MuJoCo 仿真接入计划

> 将平衡底盘 C 控制代码接入 MuJoCo 物理仿真以调参。已有 80% 基础设施在 `mujoco_control_extract/` 中。详细分析见 `../mujoco_control_extract/sim/analysis_notes.md`。

---

## 已完成

| 组件 | 状态 |
|---|---|
| `sim_adapter.c/h` — MuJoCo ↔ 控制器的桥接层 | 完成 |
| `sim/port/` — 全部硬件桩 (HAL, FreeRTOS, CAN, CMSIS-DSP 等) | 完成 |
| `CMakeLists.txt` — 编译 `librm_controller.a` + `mujoco_bridge` | 完成 |
| 控制代码已提取 (`Application/`, `Components/`) | 完成 |

---

## 待完成

### Step 1: 编译验证

在 WSL Ubuntu 编译，确保 C 代码零错误：

```bash
cp -r /mnt/c/Users/shun/Desktop/rm_test_dev/mujoco_control_extract ~/MuJoCoBin/rm_control/
cd ~/MuJoCoBin/rm_control/mujoco_control_extract
export MUJOCO_ROOT=/home/shun/MuJoCoBin
cmake -S sim -B build
cmake --build build -j
```

### Step 2: 建立 MuJoCo XML 模型

根据 `robot_param.h` 的物理参数手写 MJCF，配合用户提供的 URDF/STL 文件。机器人结构：
- 机身 8.5kg + 双腿 (5连杆机构，L1=L4=0.215m, L2=L3=0.258m) + 双轮 (0.65kg, r=0.0625m, 轮距 0.51175m)
- 4 关节电机 torque actuator + 2 轮毂 torque actuator
- IMU sensor (framequat, gyro, accelerometer) + jointpos/jointvel sensors

### Step 3: 完善 main_mujoco.c

把当前骨架中的 TODO 填完：加载 MJCF → `SimControllerState` → `SetState/SetCommand/Step` → `SimControllerOutput` → 写入 `d->ctrl` → `mj_step`。

### Step 4: 坐标系对齐 & 初始调试

MuJoCo 欧拉角 (XYZ 顺序) 可能与嵌入式 IMU 轴/符号不同，需要逐个验证。先用 `CHASSIS_SAFE` 模式 + 固定腿长 `INIT_LEG_LENGTH = 0.2m` 调通平衡。

### Step 5: 调参迭代

修改 `robot_param.h` 中的 PID/LQR 参数，在仿真中迭代。关键参数组：站立 PID、腿长 PID、Roll PID、防劈叉 PID、偏航 PID、LQR 增益表。

---

## 验证方法

1. `cmake --build build -j` 编译通过
2. 仿真跑起来不崩溃
3. 机器人能站稳 5s 以上
4. 微扰后能恢复平衡
