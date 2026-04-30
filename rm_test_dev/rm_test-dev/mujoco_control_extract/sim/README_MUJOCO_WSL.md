# MuJoCo / WSL integration notes

你的 Windows 路径：

```text
\\wsl.localhost\Ubuntu\home\shun\MuJoCoBin
```

在 Ubuntu / WSL 里面对应：

```bash
/home/shun/MuJoCoBin
```

建议把这个 `mujoco_control_extract` 目录复制到 WSL 的 Linux 文件系统里再编译，不要直接在 `/mnt/c/...` 下编译，速度会好很多。

## 1. Copy into WSL

在 WSL Ubuntu 终端里执行：

```bash
mkdir -p /home/shun/MuJoCoBin/rm_control
cp -r /mnt/c/Users/shun/Desktop/rm_test_dev/rm_test-dev/mujoco_control_extract /home/shun/MuJoCoBin/rm_control/
cd /home/shun/MuJoCoBin/rm_control/mujoco_control_extract
```

## 2. Controller boundary

接 MuJoCo 的边界是 `sim/sim_adapter.h`：

```text
MuJoCo qpos/qvel/body imu
        -> SimControllerState
        -> SimController_SetState()
        -> SimController_Step()
        -> SimController_GetOutput()
        -> MuJoCo actuator ctrl / qfrc_applied
```

初期先让 MuJoCo 直接给控制器这些量：

- 4 个关节电机角度/角速度：`joint_pos[0..3]`, `joint_vel[0..3]`
- 2 个轮毂电机角速度：`wheel_vel[0..1]`
- 姿态：`roll`, `pitch`, `yaw`
- 角速度：`gyro[0..2]`
- 车体速度/位移估计：`body_v`, `body_x`

控制器输出：

- 4 个关节扭矩：`joint_torque[0..3]`
- 2 个轮子扭矩：`wheel_torque[0..1]`

## 3. Build skeleton

如果你的 MuJoCo 安装目录是 `/home/shun/MuJoCoBin`，一般可以这样：

```bash
sudo apt update
sudo apt install -y build-essential cmake

export MUJOCO_ROOT=/home/shun/MuJoCoBin
cmake -S sim -B build
cmake --build build -j
```

这个模板先编译“控制器核心库”，后面你把自己的 MJCF 模型和 MuJoCo 主循环接到 `sim/main_mujoco.c` 里。

## 4. MuJoCo main loop shape

主循环大概长这样：

```c
while (!done) {
    SimControllerState s = {0};

    // TODO: read qpos/qvel/body orientation from mjData into s
    SimController_SetState(&s);
    SimController_SetCommand(v_set, x_set, leg_set, roll_set, yaw_set);
    SimController_Step((float)m->opt.timestep);

    SimControllerOutput u = {0};
    SimController_GetOutput(&u);

    // TODO: write u.joint_torque/u.wheel_torque to d->ctrl or d->qfrc_applied
    mj_step(m, d);
}
```

最开始建议先关掉跳跃、校准和自恢复，只调 `CHASSIS_SAFE` 下的平衡控制。等站稳后，再加腿长变化、roll/yaw 补偿和跳跃。

