# MuJoCo Position Control Debug

这个目录是独立的位控调试工程，不接入原来的底盘平衡控制器。

默认目标：

- 加载 `MJCF/env.xml`
- 把车身 base 固定吊在空中
- 运行时临时放宽四个主动关节的 range，避免 XML 里过窄的限位先卡住位控验证
- 左右腿分别维护自己的腿长目标
- 左右腿分别维护自己的 `l0 pitch` 目标
- 左右腿分别用腿长和 `l0 pitch` 逆解算出主动关节位置目标
- 对四个腿部主动关节做纯软件 PD 位控
- 用键盘分别控制左腿、右腿伸缩


## 命令行
### Build

```bash
cd /home/shun/MuJoCoBin/rm_control
env MUJOCO_ROOT=/home/shun/MuJoCoBin/mujoco-3.3.0 cmake -S mujoco_pos_debug -B build_pos_debug
cmake --build build_pos_debug -j
```

### Run

```bash
/home/shun/MuJoCoBin/rm_control/build_pos_debug/pos_debug /home/shun/MuJoCoBin/rm_control/MJCF/env.xml
```


悬空初始姿态，正常仿真：
./mujoco_control_extract/build/mujoco_bridge --hang-init MJCF/env.xml

落地初始姿态，正常仿真：
./mujoco_control_extract/build/mujoco_bridge --ground-init MJCF/env.xml

只冻结初始姿态，不跑仿真：
./mujoco_control_extract/build/mujoco_bridge --freeze-init --hang-init MJCF/env.xml

或者落地冻结：
./mujoco_control_extract/build/mujoco_bridge --freeze-init --ground-init MJCF/env.xml






push 到 git hub 上面的 指令
cd /home/shun/MuJoCoBin/rm_control
git fetch origin
git switch -c dev
git add .gitignore
git commit -m "Ignore Zone.Identifier files"
git push -u origin dev
















如果不传 XML，默认也会加载：

```text
/home/shun/MuJoCoBin/rm_control/MJCF/env.xml
```

运行时终端会周期性打印控制链路：

```text
input -> leg_set L/R -> solver L0 L/R -> phi0 pitch L/R -> measured leg L/R -> target LR/LF/RF/RR -> q LR/LF/RF/RR -> tau LR/LF/RF/RR
```

其中：

- `input`：当前按下的键位指令，例如 `UP both extend`、`J left retract`
- `leg_set L/R`：和 `mujoco_control_extract` 里的 `chassis_move.leg_set` 同义，键盘控制后的腿长命令
- `solver L0 L/R`：和 `ConsoleStandUp()` 一样，用 `MIN_LEG_LENGTH + MAX_LEG_LENGTH - leg_set` 送进逆解
- `phi0 pitch L/R`：和 `INIT_L0_PITCH` 同义，实际使用 `phi0 = pi/2 + phi0_pitch`
- `measured leg L/R`：MuJoCo 中从髋部中点到轮子 body 的实测距离，用来和视觉动作对照
- `target`：腿长逆解得到的四个主动关节目标角
- `q`：MuJoCo 当前四个主动关节角
- `tau`：最终写入 actuator 的 PD 力矩

日志默认每 `0.25s` 打印一次，可以调整：

```bash
/home/shun/MuJoCoBin/rm_control/build_pos_debug/pos_debug /home/shun/MuJoCoBin/rm_control/MJCF/env.xml --print-period 0.1
```

关闭日志：

```bash
/home/shun/MuJoCoBin/rm_control/build_pos_debug/pos_debug /home/shun/MuJoCoBin/rm_control/MJCF/env.xml --no-print
```

## Keyboard

```text
↑ / ↓  两条腿伸腿 / 缩腿
U / J  左腿伸腿 / 缩腿
O / L  右腿伸腿 / 缩腿
A / D  两条腿 l0 pitch 正向 / 反向
Q / E  左腿 l0 pitch 正向 / 反向
Z / C  右腿 l0 pitch 正向 / 反向
R      左右腿目标回到初始腿长和 pitch
Esc    退出
```

注意：这里的变量语义刻意对齐 `mujoco_control_extract`，也就是 `pos_debug` 里显示和按键调的 `leg_set` 可以直接迁移到 `chassis_move.leg_set`。

```text
leg_set 变大 -> solver L0 变小
leg_set 变小 -> solver L0 变大
```

这个模式下 `ctrl` 仍写入 MuJoCo actuator，但数值完全由位置误差算出：

```text
tau = kp * (q_target - q_now) - kd * qvel
```

## Control Logic

左腿单独控制：

```text
left leg_set
    ↓
solver_L0 = MIN_LEG_LENGTH + MAX_LEG_LENGTH - leg_set
    ↓
phi0 = pi/2 + left_phi0_pitch
    ↓
CalcPhi1AndPhi4(phi0, solver_L0)
    ↓
left rear target = -theta_transform(phi4, -J0_OFFSET)
left front target = -theta_transform(phi1, -J1_OFFSET)
    ↓
left tau = kp * (left target - left q) - kd * left qvel
```

右腿单独控制：

```text
right leg_set
    ↓
solver_L0 = MIN_LEG_LENGTH + MAX_LEG_LENGTH - leg_set
    ↓
phi0 = pi/2 + right_phi0_pitch
    ↓
CalcPhi1AndPhi4(phi0, solver_L0)
    ↓
right front target = -theta_transform(phi1, -J2_OFFSET)
right rear target = -theta_transform(phi4, -J3_OFFSET)
    ↓
right tau = kp * (right target - right q) - kd * right qvel
```

所以它是独立的纯位控验证模式，不会混入原工程的 `torque_set` 平衡控制。

## Tuning Points

如果初始姿态一边长一边短，优先看这里：

```text
pos_debug.c
    solve_left_leg_targets()
    solve_right_leg_targets()
    set_initial_target_pose()
```

- `visual_left_leg_length` / `visual_right_leg_length`：历史字段名，现在语义等同于 `control_extract` 的 `leg_set`
- `left_leg_length` / `right_leg_length`：内部送进 VMC 逆解的 solver L0，计算方式和 `ConsoleStandUp()` 一致
- `left_l0_pitch` / `right_l0_pitch`：内部送进 VMC 逆解的 `phi0` 偏置量，默认值和 `INIT_L0_PITCH` 一致
- `solve_left_leg_targets()`：左腿腿长到左后/左前主动关节角的映射
- `solve_right_leg_targets()`：右腿腿长到右前/右后主动关节角的映射
- `set_initial_target_pose()`：启动第一帧前直接把四个主动关节放到目标角

如果两个腿长目标一样但视觉上一边短一边长，通常就是左右腿某个关节的符号、前后顺序、或者 XML 里的 joint/actuator 名字映射需要调。

## Mapping Check

判断 XML 名字映射和代码映射是否一致，看三层：

```text
逻辑槽位       XML joint          XML actuator
left rear  -> jIO            -> Left_rear_joint_act
left front -> jIJ            -> Left_front_joint_act
right front-> jAB            -> Right_front_joint_act
right rear -> jAG            -> Right_rear_joint_act
left wheel -> jwheel_left    -> Left_Wheel_act
right wheel-> jwheel_right   -> Right_Wheel_act
```

程序启动时会打印 `Model mapping check`，重点看每一行最后的：

```text
actuator=... -> joint=...
```

例如：

```text
left rear  joint=jIO ... actuator=Left_rear_joint_act -> joint=jIO
```

这说明代码里的 `left rear` 槽位、XML 里的 `jIO` 关节、XML 里的 `Left_rear_joint_act` 执行器三者是连在一起的。

如果出现：

```text
left rear ... actuator=Left_rear_joint_act -> joint=jIJ
```

那就是 actuator 和 joint 对不上。
