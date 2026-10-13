# Hero 自瞄学习规划

> 配套文档：[AUTO\_AIM\_DATAFLOW.md](./AUTO_AIM_DATAFLOW.md)（数据流详解版，下称"主文档"）。
> 规划原则：**沿数据流方向推进**，每阶段"读代码 → 动手实验 → 自测验收"三步闭环，不跳层。
> 总周期约 3 周（可按实际节奏压缩/拉伸）；每阶段末有验收标准，不达标不进入下一阶段。

***

## 阶段 0：环境就绪与全局认知（第 1 天）

**目标**：能编译、能跑测试、脑中有总览图。

| 任务                           | 说明                                         |
| ---------------------------- | ------------------------------------------ |
| 编译工程                         | 走通 `tasks/hero/CMakeLists.txt` 的构建，确认无警告通过 |
| 跑 `tests/hero_core_test.cpp` | 全绿；这是无设备学习 Controller 的唯一入口                |
| 读 `src/hero.cpp` 的 `main`    | 只看结构不看细节，数出一帧循环的 12 个站点（主文档 §0 表）          |
| 读主文档 §0 + §1                 | 对照 ASCII 图，能在白纸上默画数据流                      |

**验收**：

- [ ] `hero_core_test` 全部通过
- [ ] 默画数据流图（相机/CBoard → 感知 → 跟踪 → 决策 → 下发），标出每站的数据结构名
- [ ] 说出一帧内"三次串口快照"发生的位置（行号）

***

## 阶段 1：IO 层——时间与遥测（第 2-3 天）

**目标**：吃透全链路的时基与电控交互，这是后面一切补偿的锚点。

**精读顺序**：

1. `io/camera.hpp` + `src/hero.cpp` L
2. 64-104 —— 硬件时间戳、空帧分支`io/cboard.hpp` 全文 —— `CBoardState` 逐字段、`ControlDataFrame` 字节布局、四元数队列（容量 5000）
3. `src/hero.cpp` L55-62 的 `fresh()` + L90-92 的 `imu_at` —— 数据龄检查、图像时刻插值
4. 主文档 §1-§2 对照

**动手实验**：

- 在 `imu_at` 调用处打日志，对比 `timestamp`、`now`、`state.imu_time` 三个时间，实测推理耗时
- 人为把 `telemetry_timeout` 调到 0.01，观察 `enabled` 抖动（模拟串口断流）

**验收**：

- [ ] 解释 IMU 为什么取"图像时刻"而非"当前时刻"，画图说明云台快速转动时的误差来源
- [ ] 写出 `ControlDataFrame` 的字节布局与 yaw 的编码方式（×1000, rad）
- [ ] 说出 `replace_pending=true` 防什么竞态

***

## 阶段 2：感知与跟踪层（第 4-7 天）

**目标**：理解装甲板如何变成 11 维 EKF 状态，掌握两道发散防线。

**精读顺序**：

1. `tasks/auto_aim/yolo.hpp` + `hero.cpp` L93-95 —— Armor 结构、前哨站过滤
2. `tasks/auto_aim/solver.hpp` —— PnP、`R_gimbal2world`、重投影优化（调用点在 tracker.cpp L236/L287）
3. `tasks/auto_aim/tracker.cpp` —— 状态机、`set_target`/`update_target`、装甲排序（L56-62）、dt>0.1 分支
4. `tasks/auto_aim/target.cpp` —— EKF 封装、`diverged()`（L243）、`convergened()`（L254）
5. `tools/extended_kalman_filter.cpp` —— NIS/NEES 卡方检验（L58-88）
6. 主文档 §3-§4 对照

**动手实验**：

- 用 `--display` 跑录制的视频/实时画面，观察状态机字符串流转（detecting→tracking→temp\_lost）
- 触发 `Target diverged!`：观察伴随日志 `[Target] r=, l=` 的数值，验证越界逻辑
- 统计 100 帧窗口内 `data["nis"]` 分布，对照 0.711 阈值思考主文档 §4.3 的阈值方向疑点

**验收**：

- [ ] 默写 11 维状态表（下标/含义/单位），指出 `P(6,6)`、`P(7,7)`、`P(6,7)` 后续去向
- [ ] 区分两道防线：diverged（结果检查）vs NIS（过程检查），各自触发条件与容忍度
- [ ] 解释 `rotation_observed`（target.jumped）为什么决定 Controller 解算几块板

***

## 阶段 3：决策层深挖（第 8-12 天）★核心阶段

**目标**：逐行吃透 Controller，能预测任意参数下的决策行为。

**精读顺序**：

1. `tasks/hero/controller.hpp` —— Config/TargetState/Input/Decision 四个结构体逐字段
2. `tasks/hero/controller.cpp` 分四块精读：
   - 校验链 L104-121（12 个 reject，主文档 §6.1 表）
   - `solve_candidate` L24-59（迭代伪代码见主文档 §6.2，亲手推一遍数字示例）
   - 窗口与五道门 L160-173
   - `reset_tracking` L89-94（保留/清除对照表，主文档 §6.6）
3. `tasks/hero/adapter.cpp` —— 状态搬运（L74-87）+ 会话管理（L89-106）
4. `tests/hero_core_test.cpp` —— 每个用例对应决策链的哪一关

**动手实验（改测试验证理解）**：

- 构造固定 TargetState（静止目标 5m），手算 `delay/flight/horizon`，与测试输出比对
- 把 `facing_tolerance` 从 0.20 逐步降到 0.01，观察 `outside_window` 触发边界
- 验证冷却跨会话：连续两帧开火后 reset，再进目标，确认 `cooldown` 而非 `fire_pulse`
- 重放实验：同一 `image_time` 喂两帧，确认第二帧 `duplicate_frame`，且 reset 后仍拒绝

**验收**：

- [ ] 推导 `solve_candidate` 不动点迭代为何收敛（flight ↔ ballistic.time）
- [ ] 解释小陀螺"固定正面交点"、`miss` 与 `phase_std` 各度量什么
- [ ] 默写五道门顺序与各自默认值；解释容差随距离收紧公式
- [ ] 说出四板车 id 奇偶的半径/高度分支，及其对命中精度的意义
- [ ] 改 `tests/hero_core_test.cpp` 新增一个用例：陀螺目标 `phase_std` 超限 → `outside_window`

***

## 阶段 4：实战调参与故障排查（第 13-17 天）

**目标**：建立"reason 仪表盘"思维，能独立完成标定与调参闭环。

**任务清单**：

1. **静态标定 drill**（主文档 §9 第一层）：
   - 设计 `launch_delay` 实测方案（指令时间戳 ↔ 出膛时刻）
   - 模拟静态打靶数据，反推 `yaw_offset/pitch_offset`
   - 用多距离落点数据拟合 `ballistic.drag`（可以先造数据练手）
2. **reason 统计 drill**：
   - 跑一段数据，按秒统计 reason 占比，定位瓶颈关
   - 针对性地只动一个参数复测，记录前后占比变化
3. **故障场景演练**（对照主文档 §8 日志速查表）：

| 注入故障    | 预期日志                                       | 你的排查步骤 |
| ------- | ------------------------------------------ | ------ |
| 拔串口     | `stale_before_send` / disabled             | <br /> |
| 遮挡目标 2s | `not_tracking` → detecting                 | <br /> |
| 换弹速异常弹  | `invalid_speed`                            | <br /> |
| 云台限速调慢  | `unaligned` 占比升高                           | <br /> |
| 目标高速机动  | `Target diverged!` / `Bad Converge Found!` | <br /> |

**验收**：

- [ ] 完成一份"调参报告"：初始 reason 分布 → 逐参数调整记录 → 最终分布 + 命中率变化
- [ ] 上面 5 个故障场景全部演练并写出排查步骤
- [ ] 给"`unaligned` 占比 60%"写出至少 3 步排查清单（云台带宽 → alignment\_tolerance 距离关系 → 解算抖动来源）

***

## 阶段 5：扩展方向（第 18 天起，可选）

按兴趣选做：

- **前哨站模型**：hero.cpp L94-95 目前直接滤除前哨站；参考步兵系 `outpost_max_temp_lost_count: 75` 的特殊处理，理解为何三元环需要独立建模
- **打符（大能量机关）**：`io::Mode` 已有 small\_buff/big\_buff 枚举，走通模式切换→Tracker 重建链路（hero.cpp L84-87）
- **测试补全**：为 adapter 会话管理、NIS 防线补单元测试
- **理论补课**：EKF 一致性（NIS/NEES 检验的统计学原理）、RM 弹道模型（重力+空气阻力拟合方法）

***

## 每日节奏建议

```text
30 min  精读当日代码段（对照主文档对应章节）
30 min  动手实验（改参数/加日志/跑测试）
15 min  写学习笔记：数据结构变化 + 自己的疑问
每周    做一次主文档 §10 自测清单，未通过的条目回炉
```

## 里程碑总览

| 里程碑       | 时间      | 标志                   |
| --------- | ------- | -------------------- |
| M1 全局图建立  | 第 1 天末  | 默画数据流 + 测试全绿         |
| M2 时基通透   | 第 3 天末  | 讲清图像时刻姿态对齐           |
| M3 EKF 通透 | 第 7 天末  | 默写 11 维状态 + 两道防线     |
| M4 决策通透   | 第 12 天末 | 能预测参数改动后的行为 + 新增测试用例 |
| M5 可实战    | 第 17 天末 | 独立完成调参闭环报告           |

