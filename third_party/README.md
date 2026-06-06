# third_party

这个目录用来集中管理会影响编译的第三方库。这样换电脑时，工程优先从仓库内的 `third_party/` 找依赖，不必在每台电脑上改源码里的绝对路径。

推荐结构：

```text
third_party/
  include/rm_third_party/
    mujoco.h
    glfw.h
    third_party.h
  mujoco/
    include/mujoco/mujoco.h
    lib/libmujoco.so
  glfw/
    CMakeLists.txt
    include/GLFW/glfw3.h
```

## MuJoCo

如果希望别人拿到工程后直接编译，把 MuJoCo SDK 解压或复制到：

```text
third_party/mujoco/
```

至少需要有：

```text
third_party/mujoco/include/mujoco/mujoco.h
third_party/mujoco/lib/libmujoco.so
```

如果 SDK 里是 `libmujoco.so.3.3.0` 这种版本化文件，也可以直接保留原名。

也可以不把 MuJoCo 放进仓库，而是在配置 CMake 时指定：

```bash
cmake -DMUJOCO_ROOT=/path/to/mujoco -S mujoco_control_extract/sim -B mujoco_control_extract/build
```

查找顺序是：

```text
1. CMake 参数 -DMUJOCO_ROOT=...
2. third_party/mujoco
3. 环境变量 MUJOCO_ROOT
4. 仓库同级目录 ../mujoco-3.3.0 或 ../mujoco
```

## GLFW

当前工程也依赖 GLFW。默认会继续使用系统安装的 `glfw3`；如果以后想把 GLFW 也放到仓库里，可以把 GLFW 源码放到：

```text
third_party/glfw/
```

只要这个目录里有 GLFW 自己的 `CMakeLists.txt`，工程会优先用 vendored GLFW。

## Include 约定

工程源码里不要直接写本机路径，也尽量不要散落第三方库头文件路径。统一从 wrapper 入口引用：

```c
#include "rm_third_party/mujoco.h"
#include "rm_third_party/glfw.h"
```

新增第三方库时，也建议在 `third_party/include/rm_third_party/` 下面加一个小 wrapper 头文件，然后在 `cmake/ThirdParty.cmake` 里集中处理 include 和 link。
