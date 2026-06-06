# MuJoCo SDK Placeholder

把 MuJoCo SDK 解压或复制到这个目录后，结构应该类似：

```text
third_party/mujoco/include/mujoco/mujoco.h
third_party/mujoco/lib/libmujoco.so
```

版本化的库文件名也可以，例如 `libmujoco.so.3.3.0`。

如果不想把 SDK 放进仓库，也可以在配置 CMake 时使用：

```bash
cmake -DMUJOCO_ROOT=/path/to/mujoco ...
```
