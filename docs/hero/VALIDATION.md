# DGE 英雄迁移验证

日期：2026-10-05。本记录仅针对本次迁移代码，不沿用源工程旧验证结果。下面保留首次迁移、共享修复收敛和录像回放的历史验证；当前将弹道归为通用工具后的结果见末节。当前共 28 个改动文件（16 新增、12 修改），原 Tracker 两个文件已恢复。

## 首次迁移候选代码的实际检查

| 检查 | 实际结果 |
|---|---|
| 完整英雄构建 | `hero`、`hero_replay`、`hero_core_test`、`hero_integration_test`、`hero_cboard_test` 均成功 |
| 完整 CTest | 三组测试 3/3 通过，约 0.32 s |
| 无视觉依赖构建 | `SP_HERO_CORE_ONLY=ON`，构建成功，CTest 1/1 通过 |
| 核心检查 | 77 项通过，覆盖无阻力基线、阻力、不可达/无效输入、延迟、冷却、目标会话、正反旋转车辆、不同装甲数及对准约束 |
| 原生集成 | 使用 DGE Solver / Tracker / Target / Adapter，验证四装甲、两装甲、基地大装甲、失锁和重建、前哨站拒绝、非零 IMU 姿态坐标链及不修改输入 EKF |
| CBoard 虚拟串口 | Linux PTY：AA 分包、噪声、状态时效、非法模式置 idle、精确 CC 输出字节；DGE 29 字节雷达包内的 AA/BB 内容不产生遥测 |
| CLI | `hero --help`、`hero_replay --help` 成功；默认 `calibration_confirmed=false` 时在创建 Camera/CBoard 前退出 |
| 原入口兼容 | `standard` 构建成功；没有运行真实设备 |

PTY 测试中的 33 字节测试数据是 **29 字节 DGE 雷达包 + 4 字节噪声**，不是源工程的 33 字节雷达协议。模式 4 的报文解析测试用于保留原协议；英雄不支持前哨站控制。

## 离线检测、跟踪和决策链

使用源工程已有的配对测试录像 `/home/zh/桌面/sp25_learn /build/hero-validation/demo_paired.avi/.txt`，共 620 帧；素材没有复制进 DGE。原始录像的帧与姿态数量不一致，既有测试素材只保留有对应姿态的 620 帧，并转为 MJPEG，未改动原素材。

本次运行使用 **DGE 自带 YOLOv5 模型与 DGE 英雄模板内外参**，模拟弹速 15 m/s、图像至决策耗时 0.005 s、自动发弹关闭。

| 配置 | 帧数 | 控制帧 | 发弹帧 | 说明 |
|---|---:|---:|---:|---|
| 默认 DGE 英雄模板，敌方为红色 | 620 | 0 | 0 | 测试录像目标为蓝方，全部为 `not_tracking`，未修改正式敌方颜色配置 |
| 临时副本只改 `enemy_color: blue` | 620 | 240 | 0 | 实际通过 YOLO、Solver、Tracker、Adapter 和 Controller；CSV 数值全部有限 |

蓝方临时配置的拒绝原因分布：`not_tracking=332`、`outside_window=145`、`unaligned=36`、`no_solution=48`、`settling=59`。控制可用和开火窗口是不同判定，因此不能通过原因数量推算控制帧数。

这里保留 DGE 内外参是为了检查迁入后的调用链；录像相机标定与本模板不匹配。这些结果不用于评价弹道、目标位置或命中精度，也不能把 240 帧控制等同于命中。

## 构建环境与复现

Ubuntu 22.04，GCC 11.4，CMake 3.22。使用已存在的隔离依赖前缀
`/home/zh/桌面/sp25_learn /build/hero-validation/deps/root/usr`，包括 OpenVINO 2024.6.0，未安装或替换系统依赖，未复制依赖到 DGE。

配置时指定该前缀的 `CMAKE_PREFIX_PATH` 和 `OpenVINO_DIR=前缀/lib/cmake/openvino2024.6.0`；编译/链接使用其 include/lib，pkg-config 指向同一前缀，运行时设置相应 `LD_LIBRARY_PATH`。构建和测试没有打开真实串口、相机或发射机构。

正常安装工程依赖后的项目根命令见 [MIGRATION.md](MIGRATION.md)。本次临时构建辅助脚本是 `/tmp/dge-hero-build.py`，CLI/回放辅助脚本是 `/tmp/dge-hero-smoke.py`。临时日志、CSV 和汇总位于 `/tmp/dge-hero-*.log`、`/tmp/dge-hero-replay*.csv`、`/tmp/dge-hero-replay*-summary.json`，不计入迁移文件清单。

## 已有编译问题和验证边界

1. DGE 原 `shooter.cpp` 的 `std::min(double, int)` 阻塞整个 `auto_aim` 库。为构建英雄，只将 `2` 改为 `2.0`，没有迁移源工程的 Shooter 策略。
2. 尝试编译原 `auto_aim_debug_mpc` 时出现 Gimbal 未定义引用。原 `io/CMakeLists.txt` 已注释 `gimbal/gimbal.cpp`，该入口源码仍使用 Gimbal。本次未引入英雄不需要的 Gimbal 驱动改造；因此不声称全仓库默认构建成功，请按文档指定英雄目标。
3. 本次未执行 sanitizer、硬件联调、英雄本车标定、发弹反馈验证或命中率测试。默认 `hero.auto_fire=false` 与 `hero.calibration_confirmed=false` 保持不变。

## 首次迁移的目标目录复核

文件已应用到 `/home/zh/桌面/DGE_vision_26_zzx`，共 29 个文件（16 新增、13 修改）。落盘时全部文件 SHA256 与通过候选检查的版本一致，Git 状态也只有这 29 个文件；文档清单逐项对应，`git diff --check` 通过。

随后使用目标源码重新配置并构建到 `/tmp/dge-hero-final-build-20261005`：`hero`、`hero_replay`、三组测试及 `standard` 均构建成功。目标目录版本 CTest **3/3 通过**，共约 2.21 s。实际目标二进制的两个 `--help` 通过，默认未确认标定的配置仍在打开设备前退出。

原候选和 MPC 失败记录：`/tmp/dge-hero-candidate-build.log`；首次目标 CLI 日志：`/tmp/dge-hero-final-cli.log`。测试工作目录和生成物位于 `/tmp`，DGE 内没有新增录像、模型或依赖目录。未提交 Git commit。

## 保留必要共享修复后的版本

已从 Git `HEAD` 精确恢复 `tasks/auto_aim/tracker.hpp/.cpp`，二者字节与原版本一致。会话管理移至 `hero::Adapter`；英雄入口、回放和集成测试不再调用 `Tracker::tracking_id()`。共享层只保留目标候选越界、字段默认初始化、编译类型、遥测快照/时效和串口并发修复，以及注册英雄目标所需的构建改动。

`BUILD_TESTING` 默认关闭，检查新的默认配置得到 `BUILD_TESTING:BOOL=OFF`；验证构建显式使用 `ON`。撤回了全局 `CMAKE_CXX_STANDARD_REQUIRED` 的新增设置，英雄 core 通过自己的 C++17 编译特性保持要求。

| 本次实际检查 | 结果 |
|---|---|
| 实际 DGE 源码完整构建 | `hero`、`hero_replay`、三组测试和原 `standard` 均成功 |
| 完整 CTest | 3/3 通过，约 0.24 s；核心仍为 77 项 |
| core-only 独立构建 | `/tmp/dge-hero-minimal-core-20261005`，CTest 1/1 通过，约 0.14 s |
| 新增会话回归检查 | 编号跨检测/跟踪稳定；短暂失锁恢复保持编号；失锁重建、检测期间长间隔及模式重置建立新会话；重建后重新累计稳定帧 |
| CLI 与配置保护 | 两个 `--help` 通过；默认未确认标定时在打开设备前退出 |
| 蓝方临时配置录像回放 | 620 帧、240 控制帧、0 发弹；所有 CSV 数值有限，与首次蓝方回放结果一致 |
| 改动边界 | Git 状态为 16 新增、11 修改；MIGRATION 清单逐项对应；原 Tracker 不在修改列表中 |

本次重新执行的回放仍使用 DGE 模型和内外参，仅临时将敌方颜色改为蓝色；正式配置、录像输入和验证边界与前文相同。没有执行真实设备联调、标定或命中率测试。

构建及 CTest 日志：`/tmp/dge-hero-minimal-build.log`；CLI/回放日志：`/tmp/dge-hero-minimal-smoke.log`；CSV：`/tmp/dge-hero-minimal-replay.csv`；汇总：`/tmp/dge-hero-minimal-replay-summary.json`。本次检查仍未新增或提交 Git commit。

## 用户指定 demo 的离线回放

输入是 `/home/zh/桌面/sp25_learn ——test/assets/demo/demo.avi` 和同目录 `demo.txt`。使用当前 DGE 构建的 `hero_replay`，没有修改业务代码、正式配置或原始录像；输入 SHA256 在测试前后保持一致。

视频实际可解码 687 帧，分辨率 1440×1080，标称 30 FPS，但容器报告帧数为 0。姿态文本有 620 行，均为五个有限数值；时间戳严格递增，四元数符合单位长度要求。时间间隔中位数约 0.0398 s，最大间隔约 172.582 s。因此不能把标称 FPS 当作跟踪时间戳，也不能声称整段视频与姿态已同步配对。

使用 DGE 模型及 `configs/hero.yaml` 的内外参，弹速模拟为 15 m/s，图像到决策延迟模拟为 0.005 s，自动发弹关闭。临时配置将模型路径改为 DGE 的绝对路径，保证在 `/tmp` 工作目录加载宿主模型；蓝色测试副本仅另将敌方颜色设为蓝色。

| 输入与配置 | 返回值 | 已处理帧 | 控制帧 | 发弹帧 | 结果 |
|---|---:|---:|---:|---:|---|
| 原始视频和姿态，蓝色配置 | 1 | 620 | 238 | 0 | 下一帧因 `Missing/invalid pose row` 退出；CSV 是部分结果，整段回放未通过 |
| 临时截取并转码前 620 帧，红色配置 | 0 | 620 | 0 | 0 | 全部为 `not_tracking`，与画面蓝色目标一致 |
| 临时截取并转码前 620 帧，蓝色配置 | 0 | 620 | 240 | 0 | 完整运行检测、Solver、原生 Tracker 和英雄决策，所有 CSV 数值有限 |

临时副本按顺序将前 620 个可解码图像与 620 行姿态配对，并转为 MJPEG；这个顺序对应关系尚未通过录制信息验证。转码副本和原始输入的控制帧数略有差异，因此上述数值仅用于检查调用链与输出完整性。蓝色副本拒绝原因分布为 `not_tracking=332`、`outside_window=145`、`unaligned=36`、`no_solution=48`、`settling=59`。沿用 DGE 模板标定不能证明这份录像的目标位置、弹道或命中精度正确。

该次测试产物曾位于 `/tmp/dge-hero-user-demo-20261005/`：`summary.json` 保存输入哈希、实际参数和三次运行结果；`original-blue.log/.csv` 保留原始输入报错及部分输出；`first620-blue.csv` 是完整临时副本结果。历史辅助脚本 `run_replay.py` 的运行命令如下；本轮通用工具改动验证时该临时脚本已被清理，当前可用产物见末节：

```bash
python3 /tmp/dge-hero-user-demo-20261005/run_replay.py
```

这些临时文件和模型、依赖未复制到 DGE 项目，不增加迁移文件数量。测试没有连接真实相机或串口，没有执行实际发弹。

## 弹道归为通用 tools 工具后的验证

将原英雄弹道两个文件移到 `tools/ballistics.hpp/.cpp`，头文件保护宏和命名空间改为 `TOOLS__BALLISTICS_HPP` / `tools`。英雄 Controller、核心测试改用通用接口；`tools/CMakeLists.txt` 注册独立静态库 `tools_ballistics`，普通 `tools` 目标公开链接该库，core-only 模式仅注册通用弹道库及英雄核心。

按头文件保护宏和命名空间归一化后，弹道源码与移动前逐字相同。原 `tools::Trajectory`、普通 `Aimer` / `Shooter` 的文件内容与本轮修改前一致；原 Tracker 两个文件仍与 Git `HEAD` 一致。配置参数、求解公式、英雄决策和设备协议没有变化。

| 本轮实际检查 | 结果 |
|---|---|
| 无视觉依赖的核心构建 | `/tmp/dge-tools-ballistics-20261005/core` 配置及构建成功，CTest 1/1 通过 |
| 核心算法检查 | 77 项通过，改用 `tools::` 弹道接口 |
| 独立工具调用方 | 仅包含 `tools/ballistics.hpp`、链接 `libtools_ballistics.a`，编译并运行成功，不需要英雄、视觉或设备库 |
| 完整目标构建 | `hero`、`hero_replay`、三组测试和原 `standard` 均成功 |
| 完整 CTest | 3/3 通过，约 0.34 s |
| 620 帧回放 | 240 控制帧、0 发弹，所有数值有限，拒绝原因分布与移动前一致 |
| 文件与文档清单 | 28 个文件（16 新增、12 修改），与 Git 状态逐项一致；新增的共享层修改是 `tools/CMakeLists.txt` |

回放沿用源工作区已有的 `demo_paired.avi/.txt`，使用 DGE 模型和模板内外参，临时敌方颜色为蓝色，模拟弹速 15 m/s、延迟 0.005 s，自动发弹关闭。拒绝原因仍为 `not_tracking=332`、`outside_window=145`、`unaligned=36`、`no_solution=48`、`settling=59`。录像配对和标定边界与前文相同，本轮不重新声称原始 687 帧素材已完整配对。

本轮产物位于 `/tmp/dge-tools-ballistics-20261005/`：`full-build.log` 记录完整构建和 CTest；`replay.csv`、`replay.log`、`replay-summary.json` 记录回放；`build_full.py` 和 `replay.py` 可在当前依赖前缀下复现本轮验证。上文部分历史 `/tmp` 辅助脚本已被清理，不应当作当前可用命令。正常环境的持久构建命令仍见 [MIGRATION.md](MIGRATION.md)。未增加项目内生成物，未创建 Git commit，未连接真实设备。
