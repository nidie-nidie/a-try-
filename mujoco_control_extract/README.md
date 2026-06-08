# rm_control

## MuJoCo control_extract 运行命令

以下命令默认都在仓库根目录 `/home/shun/MuJoCoBin/rm_control` 运行。
当前 `mujoco_bridge` 默认加载的模型已经是：




```text
mujoco_control_extract/sim/models/wheel_leg_urdf4_self_mesh_all.xml
```

## 调参版本 v1.0

记录日期：2026-06-06

这个版本是当前已经调出来的 MuJoCo 仿真基线版本。默认调参对象是：

```text
mujoco_control_extract/sim/models/wheel_leg_urdf4_self_mesh_all.xml
```

调参条件：

```text
启动目录：/home/shun/MuJoCoBin/rm_control
推荐启动：--mode safe --drive stand 或 --mode safe --drive forward
初始姿态：默认 --ground-init，对应 init-key pos_debug_ground
腿长目标：INIT_LEG_LENGTH = 0.20 m
腿部 phi0 参考：INIT_L0_PITCH = pi/2
底盘控制周期：CHASS_FSM_TIME = 3 ms
默认前进速度：0.20 m/s
yaw：默认锁航向，不加 --free-yaw
wheel override：默认开启，不加 --no-wheel-override
调参时不使用：--zero-control、--zero-wheels、--invert-right-joints、--freeze-init
```

当前 v1.0 的站立/前进效果最早是按 MuJoCo 侧 wheel override 开启时调出来的。现在同一套轮子平衡律已经接入 `sim_adapter` 的原生状态机输出路径，所以 `--no-wheel-override` 会关闭 `main_mujoco.c` 的覆盖层，但仍会由状态机输出等效的轮毂平衡扭矩。

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive stand --no-wheel-override
```

如果要看完全未补平衡项的原始 LQR 轮毂输出，需要临时在 `sim_adapter.c` 中绕开 `sim_apply_native_wheel_balance()`。

v1.0 wheel override 参数：

```text
pitch_target = -0.075
pitch_kp     = 75.0
pitch_kd     = 18.0
pos_kp       = 65.0
vel_kd       = 32.0
pos_ramp     = 0.5
drive_kff    = 100.0
yaw_kp       = 1.2
yaw_kd       = 0.25
wheel_limit  = 35.0
```

v1.0 no-wheel-override 状态机调参记录：

```text
调参目标：关闭 main_mujoco.c 的 wheel override 后，状态机自身输出接近 override 开启时的效果。
实现方式：把同一套 pitch / position / velocity / drive / yaw 平衡律接入 sim_adapter 原生输出。
参数位置：Application/RobotParam/Inc/robot_param.h 中 MUJOCO_WHEEL_BALANCE_*。
作用状态：CHASSIS_SAFE 和 CHASSIS_STAND_UP。
验证结果：--mode safe --drive stand --no-wheel-override 6 s 稳定；--mode safe --drive forward --no-wheel-override 6 s 稳定；--mode stand --drive stand --no-wheel-override 可在 0.2 s 后切到 SAFE 并稳定。
```

v1.0 停车调参记录：

```text
调参目标：前进松手，从 DRIVE_FORWARD 切回 DRIVE_STAND 时减少轮毂扭矩突变。
实现方式：drive_kff 前馈不再只在 DRIVE_FORWARD 生效，而是跟随 current_speed 平滑衰减到 0。
验证命令：--mode safe --drive forward --auto-stop-time 2 --no-wheel-override
验证结果：2 s 自动停车后可稳定回到 stand；override 开启和 --no-wheel-override 两条路径输出一致。
```

跳跃状态机试调记录：

```text
调参目标：在快速蹬伸前增加基于实际 VMC_L0 的深压缩阶段，并在落地前后增加预落地/缓冲/恢复阶段；目标是延长腾空时间，同时让较高跳跃落地后还能恢复稳定站立。
状态 1 压缩：腿长目标按 rate 平滑下降，重力支撑前馈乘 support_scale；左右实际 VMC_L0 都进入 target+tolerance 并保持 hold 时间后才允许蹬伸。
状态 2 蹬伸：目标切到 MAX_LEG_LENGTH，并给腿部 F0 增加 MUJOCO_JUMP_THRUST_FF 前馈；轮子仍接触地面时用小幅差动轮矩阻尼 base yaw 角速度，检测到实际离地后立即进入收腿。
状态 3 收腿/腾空：四个主动关节直接用 PD 回到 XML 初态外形；在空中保持该外形，直到开始下降并接近地面。
状态 4 预落地：当已经离地、base z 速度向下且左右轮共同离地间隙低于 preland_clearance 时，退出纯收腿 PD，腿长目标按 rate 伸到 preland_l0，为触地留出缓冲行程。
状态 5 触地缓冲：首次触地后腿长目标按 rate 收向 buffer_l0，重力前馈和腿长 PID 都降低比例，先让腿吸收冲击；左右轮接触力都稳定超过阈值并保持 buffer_hold 后进入恢复。
状态 6 恢复站立：腿长目标慢慢回 INIT_LEG_LENGTH，轮子普通平衡重新接管；恢复时间、pitch 角速度和双轮稳定触地都满足后，DRIVE_JUMP 才退出到 stand。
异常退出：压缩阶段超过 timeout 仍未达到实际 VMC_L0 判据时取消跳跃并回到 INIT_LEG_LENGTH。
XML 初态收腿：启动时保存解算、闭链稳定后的四个主动关节目标；进入收腿阶段和实际腾空后，直接用关节 PD 回到该目标。世界位置和轮子自转不参与“外形一致”比较。
姿态处理：蹬伸阶段使用腿部虚拟髋力矩 Tp 对 pitch 做 PD 闭环，目标为 XML 初态 pitch-0.05 rad；收腿、预落地和缓冲阶段使用双轮反作用力矩继续压制 pitch，并切换到独立的 XML 初态 pitch+0.005 rad 落地目标。yaw 分两段处理：phase 2 借助轮地摩擦阻尼离地前的 yaw 角速度，phase 3-5 使用较小的 P+D 差动轮矩；phase 5 必须等双轮稳定触地后才启用 yaw 修正。
默认深压缩参数：target=0.150 m，rate=0.300 m/s，support_scale=0.600，tolerance=0.010 m，hold=0.010 s，timeout=0.800 s。左右实际 VMC_L0 都进入 0.160 m 以内后才允许蹬伸。
落地默认参数：preland=[clearance=0.30,l0=0.34,rate=1.20,pid=0.80]，touchdown_force=5 N，buffer=[l0=0.22,rate=0.30,support=0.85,pid=0.65,hold=0.20]，recover=[rate=0.18,time=0.35,pitch_rate=1.5]。
落地对称补偿：F0_diff=[roll_kp=80,roll_kd=6,contact_kp=0,limit=10]，L0_diff=[roll_kp=0.40,roll_kd=0.03,limit=0.035]。负 roll 时会让左腿目标略长、右腿目标略短，同时给左右腿加反向 F0 差分，目标是减小单边先触地冲击。
其他默认参数：thrust=235，takeoff_pitch_offset=-0.05，landing_pitch_offset=0.005，pitch_tp=[-10,-2,5]，pitch_wheel=[600,60,50]，takeoff_yaw=[kp=0,kd=1.2,limit=2.5]，landing_yaw=[start_phase=3,kp=1,kd=1,limit=1]，leg_swing=[-0.12,30,2,5]，tuck=[80,4,25]，extend_end_margin=0.06。注意：起跳 yaw 差动限幅为 2.5 Nm，但空中 pitch 控制仍使用 50 Nm 仿真上限，现实轮毂峰值约 2.5 Nm 时需要重新降级验证。
推荐验证：--mode safe --drive stand --jump-at 1 --no-wheel-override
当前结果：默认深压缩参数下 wheel clearance max 约 0.354 m，连续明显离地约 0.536 s。独立落地 pitch 目标和更早的预落地伸腿将落地瞬态最大 pitch 从约 0.415 rad 降到约 0.251 rad；触地窗口最大 roll/yaw 约 0.038/0.103 rad，左右轮触地时刻差约 6 ms。MuJoCo 的 base_proxy 地面接触峰值为 0 N。
旧的直接蹬伸对照需使用 --jump-skip-compression --jump-thrust 380，结果约为 0.068 m / 0.149 s；深压缩后不能继续沿用 380 的蹬伸前馈，否则落地冲击会显著增大并可能翻转。
调参备注：增大 thrust 会延长腾空但提高落地冲击；减小 extend_end_margin 会蹬得更充分；leg_swing_offset 主要调起跳角动量，pitch_offset 主要调 pitch 参考，两者都应小步调整。
```

底盘状态机调参状态：

| 状态 | v1.0 调参状态 | 备注 |
| --- | --- | --- |
| `CHASSIS_SAFE` | 已调，当前推荐主状态 | 这里在仿真里不是无力安全状态，而是正常站立/行走控制输出状态；腿部 VMC/MIT 输出继续使用 extract，轮子平衡由 wheel override 接管。 |
| `CHASSIS_STAND_UP` | 部分可用，不作为 v1.0 主调状态 | `--mode stand` 会进入该状态，并默认 `standup_time = 0.2 s` 后切到 `CHASSIS_SAFE`；当前模型 ground-init 已经接近站立，所以更稳的调参入口是直接 `--mode safe`。 |
| `CHASSIS_CALIBRATE` | 未作为 v1.0 调参目标 | 代码路径存在，关节走速度校准，轮子输出为 0；MuJoCo v1.0 推荐流程没有用它闭环调参。 |
| `CHASSIS_OFF` | 未调参，仅保留失能/关闭入口 | `--mode off` 可进入，主要用于停机/对照，不属于站立和前进效果基线。 |
| `CHASSIS_FOLLOW_GIMBAL_YAW` | 未接入 MuJoCo v1.0 | 状态枚举存在，但当前仿真入口和 `sim_adapter` 没有把它作为可选运行模式调通。 |
| `CHASSIS_OFF_HOOK` | 未调通 | 倒地自恢复入口存在，`ConsoleOffHook()` 仍是注释状态，不属于 v1.0 调参结果。 |

上层 MuJoCo drive 状态机调参状态：

| 状态 | v1.0 调参状态 | 备注 |
| --- | --- | --- |
| `DRIVE_STAND` / `--drive stand` | 已调，推荐验证项 | 目标速度为 0，开启位置保持；适合验证原地站立是否漂移和是否能稳定锁航向。 |
| `DRIVE_FORWARD` / `--drive forward` | 已调，推荐验证项 | 默认前进速度 `0.20 m/s`；前进时 position hold 不参与，主要由 pitch、速度项和 `drive_kff` 维持。 |
| `DRIVE_JUMP` / `--drive jump` | 试调通过，当前是明显离地基线 | GUI 可按 `J` 触发；headless 可用 `--jump-at <time>` 触发。`--drive jump` 会先站稳 1 秒再自动跳，避免初始瞬间起跳。 |

先编译：

```bash
cmake -S mujoco_control_extract/sim -B mujoco_control_extract/build
cmake --build mujoco_control_extract/build --target mujoco_bridge
```

默认会优先查找 `third_party/mujoco`。如果 MuJoCo SDK 放在别的位置，可以这样指定：

```bash
cmake -DMUJOCO_ROOT=/path/to/mujoco -S mujoco_control_extract/sim -B mujoco_control_extract/build
```

推荐 GUI 启动命令：

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive stand
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive forward
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive stand --no-wheel-override
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive jump --no-wheel-override
```

推荐 headless 验证命令：

```bash
./mujoco_control_extract/build/mujoco_bridge --headless --time 3 --mode safe --drive stand
./mujoco_control_extract/build/mujoco_bridge --headless --time 3 --mode safe --drive forward
./mujoco_control_extract/build/mujoco_bridge --headless --time 6 --mode safe --drive forward --auto-stop-time 2 --no-wheel-override
./mujoco_control_extract/build/mujoco_bridge --headless --time 3 --mode safe --drive stand --jump-at 1 --no-wheel-override
./mujoco_control_extract/build/mujoco_bridge --headless --time 6 --mode safe --drive stand --jump-at 1 --no-wheel-override --jump-telemetry mujoco_control_extract/output/jump_default
```

说明：

```text
推荐优先使用 --mode safe。
--drive stand / --drive forward / --drive jump 是上层运行状态机。
--mode stand 会先走 STAND_UP 起立流程；目前 self_mesh_all.xml 下更稳的是直接从 SAFE 起步。
--jump-at <time> 可以在 headless 中定时触发跳跃。
--jump-telemetry <prefix> 输出 <prefix>.csv 原始数据和 <prefix>.svg 曲线图，并自动创建中间目录。
--jump-thrust <value> 可以临时覆盖跳跃腿部 F0 前馈，深压缩版本当前默认 235。
--jump-skip-compression 跳过压缩阶段，用于与旧的直接蹬伸流程做 A/B 对照。
--jump-compress-target / --jump-compress-rate 设置压缩腿长目标和目标变化速率。
--jump-compress-support-scale 设置压缩阶段重力支撑前馈比例。
--jump-compress-tolerance / --jump-compress-hold 设置实际 VMC_L0 的到位窗口与保持时间。
--jump-compress-timeout 设置压缩阶段超时取消时间。
--jump-pitch-target 或 --jump-pitch-offset 设置 pitch 参考；未指定 target 时，参考值等于 XML 启动 pitch 加 offset。
--jump-landing-pitch-target 或 --jump-landing-pitch-offset 单独设置 phase 3-5 的 pitch 参考，不再改变蹬伸阶段 Tp 的 pitch 目标。
--jump-pitch-tp-kp / --jump-pitch-tp-kd / --jump-pitch-tp-limit 用于蹬伸阶段的腿部 pitch 闭环。
--jump-pitch-wheel-kp / --jump-pitch-wheel-kd / --jump-pitch-wheel-limit 用于收腿、预落地和缓冲阶段的轮子反作用力矩。
--jump-leg-swing-offset / --jump-leg-swing-kp / --jump-leg-swing-kd / --jump-leg-swing-limit 用于蹬伸阶段腿摆角补偿。
--jump-tuck-kp / --jump-tuck-kd / --jump-tuck-limit 控制空中回到 XML 初态外形的关节 PD。
--jump-extend-end-margin 控制提前结束伸腿的余量，当前默认 0.06 m。
--jump-preland-clearance / --jump-preland-l0 / --jump-preland-rate / --jump-preland-pid-scale 用于临时覆盖预落地触发间隙、腿长、变化速率和 PID 比例。
--jump-buffer-l0 / --jump-buffer-rate / --jump-buffer-support-scale / --jump-buffer-pid-scale 用于临时覆盖触地缓冲腿长、变化速率、重力支撑和腿长 PID 比例。
--jump-landing-roll-kp / --jump-landing-roll-kd / --jump-landing-contact-kp / --jump-landing-balance-limit 用于临时覆盖落地 F0 差分补偿。
--jump-landing-l0-kp / --jump-landing-l0-kd / --jump-landing-l0-limit 用于临时覆盖落地左右腿长目标差分。
--jump-landing-clearance-kp / --jump-landing-clearance-rate / --jump-landing-clearance-limit 用于临时覆盖 phase 4 的轮地间隙差腿长控制。
--jump-takeoff-yaw-kp / --jump-takeoff-yaw-kd / --jump-takeoff-yaw-limit 用于临时覆盖 phase 2 尚未离地时的 yaw 差动阻尼。
--jump-landing-yaw-start-phase / --jump-landing-yaw-kp / --jump-landing-yaw-kd / --jump-landing-yaw-limit 用于临时覆盖 phase 3-5 的 yaw 差动轮矩。
```

phase 4 预落地时直接使用左右轮几何间隙差：

```text
clearance_error = left_wheel_clearance - right_wheel_clearance
balance_l0 = clamp(2.0 * clearance_error, -0.045, 0.045)

left_l0_set  = landing_l0 + balance_l0
right_l0_set = landing_l0 - balance_l0
```

`balance_l0` 每秒最多变化 `3.0 m`。左轮更高时会增大左腿目标、减小右腿目标；进入 phase 5/6 后切回 roll/roll-rate 腿长差分，避免接触间隙噪声继续驱动腿长。

跳跃 yaw 控制分为起跳和空中/落地两段：

```text
phase 2 且轮子仍接触地面：
yaw_term = clamp(1.2 * yaw_rate, -2.5, 2.5)

phase 3-5：
yaw_term = clamp(1.0 * yaw_error + 1.0 * yaw_rate, -1.0, 1.0)
left_wheel_torque  = pitch_wheel_torque - yaw_term
right_wheel_torque = pitch_wheel_torque + yaw_term
```

phase 2 利用轮地摩擦在离地前减少 yaw 角动量；纯空中阶段的差动轮矩主要调整轮速，不能像有地面接触时一样直接产生稳定的 base yaw 力矩。phase 5 在双轮接触力都达到 5 N 后才恢复 yaw 差动，避免单轮先触地期间放大不对称冲击。最终左右轮输出仍受 `±50 Nm` 仿真限幅。

跳跃遥测输出：

```text
SVG 第一部分：base 世界 z、base 相对实际离地时刻的上升高度、左右轮共同离地间隙。
SVG 第二部分：左右腿 VMC 滤波支持力、左右轮 MuJoCo 接触法向力，以及 3 N 离地阈值。
SVG 第三部分：base roll/pitch/yaw。
SVG 第四部分：4 个腿部执行器和 2 个轮毂执行器的 MuJoCo 实际施加扭矩。
CSV：每个仿真步的时间、跳跃阶段、三种离地观测、共同及左右轮间隙、高度、base RPY、控制指令扭矩和实际执行器扭矩。
实际扭矩读取自 mj_step 之后的 d->actuator_force；指令扭矩读取自 d->ctrl。
峰值统计窗口从跳跃触发开始，持续到跳跃结束后 0.5 s，因此包含主要落地冲击。
base_rise_from_takeoff_m 以两轮共同离地超过 5 mm 的首个仿真步为零点。
wheel_clearance_m 取左右轮离地间隙的较小值，表示两只轮子都真正离开地面的高度。
vmc_*_fn_raw_n 使用原 ground_detection 公式和 MuJoCo base 世界 z 加速度计算。
vmc_*_fn_filtered_n 是独立四点均值；vmc_*_airborne 表示该值低于 3 N。
contact_*_normal_n 是 mj_contactForce 给出的轮地接触法向力；contact_airborne 表示左右均不超过 0.1 N。
VMC 支持力字段只用于观测，不回写 INS、left_flag/right_flag，也不参与跳跃状态切换。
轮地接触力和左右 wheel clearance 现在会作为跳跃落地阶段的观测输入，用于触发预落地、生成 phase 4 腿长目标差、触地缓冲和恢复站立。
```

当前默认深压缩样例输出：

```text
mujoco_control_extract/output/jump_default.csv
mujoco_control_extract/output/jump_default.svg

base 实际离地后的最大上升：约 0.276 m
左右轮共同最大离地间隙：约 0.354 m
首次触地到跳跃状态退出：约 0.617 s
落地瞬态最大 roll/pitch/yaw 误差：约 0.038/0.251/0.103 rad
触地后 1 s roll/pitch/yaw 误差：约 -0.002/-0.099/-0.011 rad
左右轮触地时间差：约 6 ms
左右轮落地接触峰值：约 1911/1825 N
base_proxy 地面接触峰值：0 N
腿部电机最大实际扭矩：约 45.197 Nm
轮毂电机最大实际扭矩：约 50.000 Nm

默认测试的蹬伸阶段首次观测：
MuJoCo 接触力归零约 1.297 s；5 mm 几何离地约 1.304 s。
VMC 支持力在蹬伸切换瞬间约 1.249 s 出现一次短暂误报。
```

仿真窗口按键：

```text
按住 W：前进
按住 S：后退
松开 W / S：自动回到原地站立 stand
J：触发一次跳跃
SPACE：暂停 / 继续
ESC：退出
```

默认仿真会锁住车体航向，避免串腿在原地绕圈；如果要调试 yaw 自由度，可以加 `--free-yaw`。

默认前进速度是 `0.20 m/s`，可以用命令行改：

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive forward --forward-speed 0.2
```

如果想启动后直接进入前进状态：

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive forward
```

只冻结初始姿态，不跑仿真：

```bash
./mujoco_control_extract/build/mujoco_bridge --freeze-init --mode safe --drive stand
```

悬空冻结：

```bash
./mujoco_control_extract/build/mujoco_bridge --freeze-init --hang-init --mode safe --drive stand
```

如果你已经 `cd mujoco_control_extract` 进入子目录，也可以这样运行：

```bash
./build/mujoco_bridge --mode safe --drive stand
./build/mujoco_bridge --mode safe --drive forward
```

push 到 git hub dev 上面的 指令
cd /home/shun/MuJoCoBin/rm_control
git fetch origin
git switch -c dev
git add .gitignore
git commit -m "Ignore Zone.Identifier files"
git push -u origin dev







#### mujoco 仿真


urdf 文件：
urdf 文件放在my robot目录底下，传文件的话就直接拷贝进去就行了

目录顺序是 MuJoCoBin 里面的 my robot。

命令行：
simulate ~/MuJoCoBin/urdf13.SLDASM/urdf/urdf13.SLDASM.urdf

cd 可以进入任何文件夹， 直接输入 cd 就可以返回主界面。
