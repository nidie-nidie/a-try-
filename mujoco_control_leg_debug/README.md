# MuJoCo Leg Debug

这个目录是一套独立的腿部调试 viewer，不复用 `mujoco_pos_debug` 的构建目录。它默认加载 `models/wheel_leg_debug.xml`，并把本次需要的 mesh / texture 收在 `models/assets/`。

## Build

```bash
cd /home/shun/MuJoCoBin/rm_control
env MUJOCO_ROOT=/home/shun/MuJoCoBin/mujoco-3.3.0 cmake -S mujoco_control_leg_debug -B build_leg_debug
cmake --build build_leg_debug -j
```

## Run

```bash
/home/shun/MuJoCoBin/rm_control/build_leg_debug/leg_debug
```

碰撞调试模型：

```bash
/home/shun/MuJoCoBin/rm_control/build_leg_debug/leg_debug --collision
```

mesh 自碰撞测试模型：

```bash
/home/shun/MuJoCoBin/rm_control/build_leg_debug/leg_debug /home/shun/MuJoCoBin/rm_control/mujoco_control_leg_debug/models/wheel_leg_debug_collision_self_mesh.xml
```

整车 mesh 全自碰撞测试模型：

```bash
/home/shun/MuJoCoBin/rm_control/build_leg_debug/leg_debug /home/shun/MuJoCoBin/rm_control/mujoco_control_leg_debug/models/wheel_leg_debug_collision_self_mesh_all.xml
```

无窗口快速检查：

```bash
/home/shun/MuJoCoBin/rm_control/build_leg_debug/leg_debug --headless --time 2 --scripted-demo
```

默认关节伺服使用调试参数 `kp=80 kd=4 max_tau=80`。普通浮空调试模型默认以 `0.01s` 周期刷新 inverse-dynamics hold 前馈，目的是让 MuJoCo 里的主动关节尽量贴住 IK 目标，避免闭链模型自身的动力学下垂干扰几何判断。`--collision` 自由底盘接触模型默认关闭这个前馈，因为 `mj_inverse()` 会把地面接触和闭链 equality 的约束力也算进 hold torque，姿态跑偏后容易出现很大的 `ff` 并导致仿真不稳定；需要强制对照时可手动加 `--gravity-ff`。需要复现实机固件的弱位置环时用：

```bash
/home/shun/MuJoCoBin/rm_control/build_leg_debug/leg_debug --firmware-gains
```

## Keyboard

通用：

```text
M       task-space / joint-space 切换
P       暂停 / 继续
N       暂停时单步
Space   控制输出开 / 关
B       辅助线显示开 / 关
G       MuJoCo inverse-dynamics hold 前馈开 / 关
R       回到 XML 初态位姿，并重建当前模式目标
Esc     退出
```

`task-space`：

```text
Up / Down   左右腿 L0 同增 / 同减
U / J       左腿 L0 增 / 减
O / L       右腿 L0 增 / 减
A / D       左右腿 phi0 同增 / 同减
Q / E       左腿 phi0 增 / 减
Z / C       右腿 phi0 增 / 减
```

`joint-space`：

```text
1 / 2 / 3 / 4   选择 LF / LR / RR / RF 主动关节
Left / Right    切换选中关节
Up / Down       当前选中关节目标角加 / 减
[ / ]           当前选中关节目标角减 / 加
, / .           四个主动关节目标角同减 / 同加
```

## XML Modes

`models/wheel_leg_debug.xml` 默认是浮空悬挂解算调试模式：

- `floor` 可见但关闭 contact，用来做环境/高度参考，不会产生地面接触。
- 视觉 mesh 默认不参与 contact，轮子 proxy 保留但当前不会接触地面，避免 mesh 自碰撞干扰闭链腿部测量。


- `debug_suspend_anchor` 和 `debug_suspend_base` weld 默认启用，锚点放在空中；这样整车浮空且 base 不漂，适合专心调串腿闭链和 task-space / joint-space 跟踪。


- 每个闭链接口现在使用两组 `connect` site：`*_1` 负责把铰点拉到一起，`*_2` 额外约束同一条轴线附近的第二个点，减少单点 connect 绕轴扭动导致的视觉错位。


- 想看自由底盘地面动力学时，直接用 `--collision`；想把普通模型也放回接触地面，则把 `floor` 的 `contype/conaffinity` 改成 `1/2`，把 `base` 和 `debug_suspend_anchor` 的 Z 改回 `0.120724319674778`，并按需注释 `debug_suspend_base` weld。

`models/wheel_leg_debug_collision.xml` 不做模式切换，只做碰撞增强：

- 所有 mesh geom 默认参与地面 contact，不再只是显示用。
- 关键腿部连杆和轮子仍保留橙色 proxy geom，作为额外接触辅助和可视化参考。
- 当前 mesh/proxy 主要和地面接触，不开启全局腿部自碰撞；该模型保留自由 base，用来检查真实接触趋势。
- 闭链接口同样按 `CF-GH / KN-OP / EC-AG / MK-IO` 顺序使用 `_1` 和 `_2` 两组 `connect`。
- 默认关闭 inverse-dynamics hold 前馈，只保留 PD 输出；如果手动打开 `--gravity-ff`，日志里的 `ff` 可能会因为接触/闭链约束在奇异姿态附近快速放大。

`models/wheel_leg_debug_collision_self_mesh.xml` 是第一档 mesh 自碰撞测试：

- 所有 `type="mesh"` 的车体 geom 互相允许碰撞，也继续允许和地面碰撞。
- 父子 body 仍保留 MuJoCo 默认的碰撞过滤，所以主要用来检查非相邻 link 的 mesh 干涉。
- 所有橙色 proxy geom 都被改成纯显示，不再参与接触，避免干扰 mesh 自碰撞判断。

`models/wheel_leg_debug_collision_self_mesh_all.xml` 是整车 mesh 全自碰撞测试：

- 在上一份基础上额外关闭 `filterparent`，让父子 body 的相邻 link 也参与碰撞。
- 对当前这套车模来说，这是最接近“整车 mesh 全自碰撞”的模式；因为每个 body 只挂了一个 mesh geom，所以 MuJoCo 的“同 body 不自碰”限制基本不会影响 mesh-mesh 检查。
- 这份模型也会自动按 collision 模式关闭 hold 前馈默认值，避免接触和闭链约束一起把 `ff` 放大。

## Logs And Lines

viewer 每 `0.25s` 默认打印：

```text
ncon / ik_L0_cmd / target_L0_vmc / L0_vmc / L0_world / phi0_vmc / q / target / q_err / pd / ff / tau
```

`ik_L0_cmd` 是送进 `CalcPhi1AndPhi4()` 的期望虚拟腿长，只在 `task-space` 模式里用于生成四个关节目标角。

`target_L0_vmc` 是把 IK 输出的 `target_q` 再送回 `VMC_calc_1()` 得到的目标几何腿长。它用来检查 IK、关节映射、符号和支路是否自洽。

`L0_vmc` 是根据当前实际关节角重新跑 `VMC_calc_1()` 得到的测量腿长。如果 `ik_L0_cmd` 和 `target_L0_vmc` 接近，但 `L0_vmc` 不接近，优先看 `q_err` 和 `tau`，这通常表示关节位置环没有真的把实际关节推到 IK 目标。

默认控制是 MuJoCo `motor` + 软件 PD + zero-acceleration inverse dynamics 前馈。`pd` 是位置环输出，`ff` 是 MuJoCo 当前姿态下保持零加速度所需的 hold torque，包含重力、速度项和闭链 equality 约束带来的静态力矩，`tau` 是两者相加后再经过 `--max-torque` 限幅的实际电机命令。悬挂模式下如果关节保持不住 XML 初态，先看 `q_err` 是否长期很大，以及 `tau` 是否贴住 `max_tau`。如果 `--firmware-gains` 下 `q_err` 变大而默认调试参数下正常，说明差异来自 MuJoCo 动力学执行层，不是 IK 的 L0 定义。

辅助线直接加入 MuJoCo scene：

- 左右髋轴中点到左右轮轴。
- 左右 VMC 虚拟点 `C` 的 `L0` 线。
- 左右髋轴连线、左右轮轴连线。

这样可以直接对照 `VMC_calc_1()` 算出的腿长和模型世界坐标里的真实髋轮距离。
