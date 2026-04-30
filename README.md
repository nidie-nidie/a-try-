# rm_control

## MuJoCo control_extract 运行命令

悬空初始姿态，正常仿真：

```bash
./mujoco_control_extract/build/mujoco_bridge --hang-init MJCF/env.xml
```

落地初始姿态，正常仿真：

```bash
./mujoco_control_extract/build/mujoco_bridge --ground-init MJCF/env.xml
```

只冻结初始姿态，不跑仿真：

```bash
./mujoco_control_extract/build/mujoco_bridge --freeze-init --hang-init MJCF/env.xml
```

落地冻结：

```bash
./mujoco_control_extract/build/mujoco_bridge --freeze-init --ground-init MJCF/env.xml
```
