# rm_control workspace

这个仓库是一个轮腿机器人控制与 MuJoCo 仿真的工作区。当前目录里混合了几类内容：主线控制代码、调试工程、机器人模型资源、原始参考工程、构建输出和运行日志。

整理这个仓库时，先把“能重新生成的东西”和“真正的源文件/模型资源”分开看。源文件和必要模型资源应该上传到 GitHub；构建产物、缓存、日志和本机环境不应该继续进入 Git 历史。

## 目录结构

| 路径 | 作用 | GitHub 建议 |
| --- | --- | --- |
| `mujoco_control_extract/` | 当前主线的控制代码和 MuJoCo 仿真适配工程。包含从嵌入式工程抽取出来的控制、算法、设备、任务代码，以及 `sim/` 下的仿真入口和模型。 | 应该保留并上传。 |
| `mujoco_control_extract/Application/` | 机器人参数、任务层代码、类型定义等应用层代码。 | 源文件，应该上传。 |
| `mujoco_control_extract/Components/` | 控制器、算法、设备抽象、基础库等模块。 | 源文件，应该上传。 |
| `mujoco_control_extract/sim/` | MuJoCo 仿真工程，包含 `CMakeLists.txt`、`main_mujoco.c`、`sim_adapter.*`、平台兼容头文件和模型 XML。 | 源文件，应该上传。 |
| `mujoco_control_extract/sim/models/` | MuJoCo XML 模型和模型资源引用。 | 运行仿真需要的 XML/资源引用应该上传。 |
| `mujoco_control_leg_debug/` | 独立的腿部调试 MuJoCo 工程，包含 `src/`、`include/`、`models/`。 | 如果仍用于调试，应该上传。 |
| `mujoco_control_leg_debug/models/assets/` | 调试模型需要的 mesh、贴图、OBJ/STL 资源。 | 仿真加载必需的资源应该上传；特别大的资源建议后续考虑 Git LFS。 |
| `wheel_leg_urdf4/` | 轮腿机器人 URDF/MJCF 导出目录，包含 URDF、MJCF、mesh、launch/config 和转换脚本。 | 机器人描述与模型源资源，应该上传。 |
| `rm_test_dev/` | 原始/参考 STM32 工程、旧 Python MuJoCo 工程、工具和文档。 | 如果作为参考基线，暂时保留；整理分支里可以考虑归档或精简。 |
| `logs/` | 运行过程中产生的日志。 | 不是源文件，后续不应继续上传。 |
| `build/`, `build_*`, `build-*` | CMake/编译输出目录。 | 编译产物，不应上传。 |
| `.venv/` | Python 虚拟环境。 | 本机环境，不应上传。 |
| `.vscode/` | VS Code 配置。 | 项目共享配置可保留；纯个人配置不应上传。 |
| `.claude/`, `.agents/`, `.codex` | 本地工具/助手状态文件。 | 通常不应上传，除非明确要共享。 |

## 应该上传到 GitHub 的内容

这些内容属于工程源文件或复现实验所需的输入：

- C/C++ 源码与头文件：`*.c`, `*.h`, `CMakeLists.txt`
- Python 脚本：例如模型转换、构建辅助脚本
- MuJoCo/URDF/机器人描述文件：`*.xml`, `*.urdf`, `*.yaml`, `*.csv`
- 仿真必需资源：`*.STL`, `*.obj`, `*.png`
- README、调参记录、设计文档：`README.md`, `*.md`, 必要 PDF
- 项目级配置：例如必要的 `.vscode` 配置、`.gitignore`

注意：mesh 和 OBJ/STL 资源虽然是二进制或大文件，但如果 XML/URDF 直接依赖它们，它们就更像“模型源资源”，不是普通编译产物。GitHub 普通 Git 单文件硬限制是 100 MB；接近或超过 50 MB 的资源后续建议放 Git LFS 或单独归档。

## 不应该上传到 GitHub 的内容

这些内容可以重新生成，或者只对本机当前运行有效：

- CMake 构建目录：`build/`, `build_*`, `build-*`, `cmake-build-*`
- CMake 临时文件：`CMakeCache.txt`, `CMakeFiles/`, `cmake_install.cmake`, `compile_commands.json`
- 编译产物：`*.o`, `*.a`, `*.so`, `*.dll`, `*.exe`, `a.out`
- MuJoCo 生成的二进制模型：`*.mjb`
- 运行日志：`MUJOCO_LOG.TXT`, `logs/*.log`
- Python 缓存：`__pycache__/`, `*.pyc`
- Python 虚拟环境：`.venv/`
- Windows/WSL 拷贝残留：`*:Zone.Identifier`
- 临时压缩包或重复备份：例如已经解压并纳入整理的 `*.zip`

当前 `.gitignore` 已经覆盖了大部分 build/CMake 产物、`*.mjb` 和 `*:Zone.Identifier`。后续整理时，还可以继续把日志、Python 缓存和虚拟环境规则补齐，并把已经被 Git 跟踪过的缓存/日志从索引中移除。

## 当前需要注意的文件

- `mujoco_control_extract/mjmodel.mjb` 是 MuJoCo 生成的二进制模型，已经从 Git 跟踪中移除，并由 `.gitignore` 的 `*.mjb` 忽略。本地可以保留，GitHub 不应该上传它。
- `mujoco_control_leg_debug/models/assets/base_link_original.obj` 大约 92 MB，GitHub 允许上传但会警告。它如果是仿真必需资源，可以暂时保留；如果能从 URDF/CAD 导出再生成，后续建议改为 Git LFS 或归档。
- `mujoco_control_extract/MJMODEL.TXT` 大约 40 MB，看起来像模型导出的文本 dump。它不超过 GitHub 限制，但后续可以判断是否真的需要长期跟踪。
- 当前仓库里还有一些历史上已经跟踪的日志和 `__pycache__` 文件。它们不是源文件，建议在整理分支里集中移除。

## 建议的整理顺序

1. 保持 `dev` 作为整理前备份版本。
2. 新开 `cleanup/workspace-files` 分支做删除和移动。
3. 先删除可以重新生成的内容：build 目录、缓存、日志、Zone.Identifier。
4. 再处理大文件：判断大 OBJ/STL/PDF/zip 是必需资源、文档、归档，还是重复副本。
5. 最后再调整目录结构，避免先移动源码导致 CMake 或 XML 的相对路径断掉。

