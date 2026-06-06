# 串并联腿强化学习论文速读

本目录由 `uv` 创建本地 Python 环境，并用 `pypdf` 读取《暑期科研任务书：双腿轮足机器人的强化学习.pdf》后整理。

## 文件清单

1. `01_mechanical_intelligence_aware_curriculum_rl_parallel_actuation.pdf`
   - Mechanical Intelligence-Aware Curriculum Reinforcement Learning for Humanoids with Parallel Actuation
   - 来源: https://arxiv.org/abs/2507.00273
2. `02_learning_to_walk_hybrid_serial_parallel_kangaroo.pdf`
   - Learning to Walk with Hybrid Serial-Parallel Linkages: a Case Study on the Kangaroo Robot
   - 来源: https://dkanou.github.io/publ/C77.pdf
3. `03_kamino_gpu_massively_parallel_simulation_challenging_topologies.pdf`
   - Kamino: GPU-based Massively Parallel Simulation of Multi-Body Systems with Challenging Topologies
   - 来源: https://arxiv.org/abs/2603.16536
4. `04_lips_large_scale_humanoid_rl_parallel_series.pdf`
   - LiPS: Large-Scale Humanoid Robot Reinforcement Learning with Parallel-Series Structures
   - 来源: https://arxiv.org/abs/2503.08349
5. `05_pace_systematic_sim_to_real_diverse_legged_robots.pdf`
   - Towards bridging the gap: Systematic sim-to-real transfer for diverse legged robots
   - 来源: https://arxiv.org/abs/2509.06342
6. `06_zest_zero_shot_embodied_skill_transfer.pdf`
   - ZEST: Zero-shot Embodied Skill Transfer for Athletic Robot Control
   - 来源: https://arxiv.org/abs/2602.00401

第 7 篇 `Design and Analysis of a Novel Reconfigurable Closed-Chain Robot Leg` 只找到题录和 DOI `10.1109/WRCSARA53879.2021.9612696`，未找到可合法直接下载的公开 PDF。ResearchGate 页面显示无全文，需要向作者请求。

## 每篇讲什么

### 1. Mechanical Intelligence-Aware Curriculum RL

这篇最贴近你的双腿轮足问题。它的核心观点是：并联机构不应该总是粗暴等效成串联机构，因为并联结构本身带来的非线性、耦合、机械优势会影响策略学习。作者在 BRUCE 小型人形机器人上处理了差分滑轮、五连杆、四连杆三类并联机构，用 MJX 直接模拟闭链约束，并通过课程学习训练端到端步态策略。

对你的启发：如果后面做 RL，优先考虑在 MJX 里保留五连杆闭链或至少保留它的关键动力学特性，而不是一开始就完全串联化。

### 2. Kangaroo Hybrid Serial-Parallel Linkages

这篇是 Isaac Sim/Isaac Lab 路线的案例。Kangaroo 是一个包含很多串并联混合链的双足机器人，作者没有简化运动学结构，而是在 Isaac Sim 里用 PhysX 约束训练 PPO 行走策略，再把策略拿到 MuJoCo 里做 Sim-to-Sim 验证。

重点结果是：策略大多数情况下能跨仿真器迁移，但对 action rate penalty 这类奖励权重非常敏感。也就是说，闭链结构下策略能训出来，但奖励项会强烈影响策略是否平滑、是否能迁移。

对你的启发：可以走 `Isaac Lab 训练 + MuJoCo 验证`，但奖励函数里动作变化率、关节速度、力矩平滑项要认真调。

### 3. Kamino

Kamino 不是一篇具体机器人控制论文，而是仿真器/物理求解器论文。它用 NVIDIA Warp 写 GPU 多体动力学求解器，目标是原生支持复杂拓扑，比如多闭链、多约束、强耦合机构，不再依赖把闭链拆成树结构后用额外约束补回来。

它展示了 DR Legs 这种有六个嵌套运动学环的双足机器人，可以在单 GPU 上并行 4096 个环境训练行走策略。

对你的启发：这是长期关注方向。当前项目短期更现实的是 MuJoCo/MJX 或 Isaac Lab；Kamino 代表未来闭链 RL 仿真工具可能会往哪里走。

### 4. LiPS

LiPS 关注人形机器人里的并联-串联混合结构。它指出常见做法是在训练时用开链或简化串联模型，部署时再映射到真实并联机构，这会制造 Sim2Real gap。LiPS 的思路是在训练环境里把并联结构的多刚体动力学也算进去，让训练和部署结构更一致。

这篇更像方法框架：把并联结构的广义坐标、动力学矩阵、外力雅可比等纳入 RL 环境，使策略训练时就“看到”真实结构。

对你的启发：如果你以后做“并联五连杆等效串联”，需要特别注意等效模型与真实闭链动力学之间的误差；LiPS 支持另一条路线：直接把闭链动力学纳入训练。

### 5. PACE / Systematic Sim-to-Real

这篇不专门讲闭链，但很适合做实车落地参考。它提出 PACE，把 Sim2Real RL 与永磁同步电机的能量损耗模型结合起来，并用较少参数做系统辨识。它强调 Sim2Real 不是只调摩擦和质量，电机损耗、执行器动力学、状态估计、接触模型都要系统处理。

一个重要点是奖励函数不追求堆很多手工项，而是使用更紧凑的奖励，并加入基于物理的一阶能耗项，最后在多种腿足机器人上部署。

对你的启发：你的轮足平台如果要上实车，电机模型、延迟、限幅、能耗、状态估计误差要和仿真一起标定，不能只盯 XML 质量惯量。

### 6. ZEST

ZEST 是面向高动态技能迁移的运动模仿框架。它从动作捕捉、单目视频、动画等不同数据源训练 RL 策略，并能零样本部署到 Atlas、Unitree G1、Spot 等不同机器人上。

和你的项目最相关的是它对闭链执行器的处理：作者给出一种根据近似 armature 值自动选择关节级 PD 增益的方法，让仿真中的关节控制器和真实硬件的等效惯量更匹配。它还使用自适应采样和辅助力课程学习来训练长时域、高动态动作。

对你的启发：如果你把五连杆等效成某种串联/任务空间控制接口，PD 增益不要纯手调，可以根据等效 armature 或关节惯量来定，这有助于缩小 Sim2Real gap。

### 7. Reconfigurable Closed-Chain Robot Leg

这篇不是 RL 论文，更偏机械设计和运动学分析。它讨论可重构闭链机器人腿，用运动学分析得到足端工作空间/轨迹范围，目标是解决传统闭链腿足端轨迹单一、适应性有限的问题。

对你的启发：可借鉴它的工作空间分析方法，用来确定五连杆轮轴的可达区域、奇异边界和等效串联模型的关节限位。

## 快速结论

短期最值得精读的是 1、2、4：它们直接回答“闭链/并联腿怎么做 RL”。第 5、6 篇用于实车部署和控制器参数设计。第 3 篇看仿真器趋势。第 7 篇用于五连杆可行域和等效模型标定。

如果要结合当前项目，我建议优先试两条路线：

1. MJX 直接闭链训练：保留 `<equality connect>` 或等价闭链约束，先训练站立和平衡。
2. 串联等效训练：先用五连杆解析运动学标定可行域，再用 armature 感知 PD 增益减少等效误差。
