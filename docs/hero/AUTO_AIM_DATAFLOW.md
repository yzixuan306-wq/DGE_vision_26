# Hero 自瞄学习文档 —— 以数据流为主线（详解版）

> 阅读路线：沿一帧图像的旅程，从相机出发到射击指令下发，逐层拆解每一站的**数据结构、核心算法、关键代码与安全设计**。
> 入口：`src/hero.cpp`；核心配置：`configs/hero.yaml`（`hero:` 块）；单元测试：`tests/hero_core_test.cpp`（无设备跑 Controller 的最佳学习材料）。

## 文件地图

| 文件 | 角色 |
|---|---|
| `src/hero.cpp` | 主循环：装配所有组件，一帧一决策 |
| `tasks/hero/controller.{hpp,cpp}` | 决策核心：纯函数 `update(Input)→Decision`，无设备/ML 依赖 |
| `tasks/hero/adapter.{hpp,cpp}` | 适配层：步兵 Tracker 输出 → hero TargetState；会话管理 |
| `tasks/auto_aim/yolo.{hpp,cpp}` | 神经网络推理 → `list<Armor>` |
| `tasks/auto_aim/solver.{hpp,cpp}` | PnP 解算 + 云台-世界姿态变换 |
| `tasks/auto_aim/tracker.{hpp,cpp}` | 状态机 + 目标管理 |
| `tasks/auto_aim/target.{hpp,cpp}` | 单目标 EKF 封装 |
| `tools/extended_kalman_filter.{hpp,cpp}` | 通用 EKF + NIS/NEES 卡方检验 |
| `tools/ballistics.hpp` | 弹道求解（RK4） |
| `io/cboard.{hpp,cpp}` | 电控串口：遥测上行 + 指令下行 |
| `io/camera.hpp` | 相机取图 + 硬件时间戳 |
| `io/command.hpp` | `io::Command` 指令结构 |

## 0. 数据流总览

```text
        ┌─────────────┐      ┌──────────────────┐
        │ 相机 Camera  │      │ 电控 CBoard (串口) │
        │ image+时间戳 │      │ IMU四元数·弹速·模式 │
        └──────┬──────┘      └───────┬──────────┘
               │ image+ts            │ IMU姿态(图像时刻)
               ▼                     ▼
        ┌──[感知层]──────────────────────────────┐
        │ YOLO 推理 → list<Armor> (滤除前哨站)      │
        │ Solver: 姿态同步 + PnP 解算               │
        └──────────────┬────────────────────────┘
                       │ list<Armor>
                       ▼
        ┌──[跟踪层]──────────────────────────────┐
        │ Tracker: 状态机 · EKF(11维) · 发散/NIS检测│
        └──────────────┬────────────────────────┘
                       │ list<Target> + tracker_state
                       ▼
        ┌──[决策层]──────────────────────────────┐
        │ Adapter: EKF→TargetState · 会话管理      │
        │ Controller: 校验链→弹道解算→窗口→五道门   │
        └──────────────┬────────────────────────┘
                       │ Decision (io::Command)
                       ▼
        ┌───────────────────────────────────────┐
        │ 发送前复检(第3次串口快照) → board.send    │
        └───────────────────────────────────────┘
```

一帧循环逐站点（`src/hero.cpp` L64-129）：

| 站点 | 行号 | 产出 |
|---|---|---|
| 取图 | L67 | `image` + 硬件时间戳 |
| 第1次读串口 | L75-77 | `CBoardState`；`fresh()` 数据龄检查 + 模式判定 |
| 模式切换检测 | L84-87 | 变化则重建 Tracker + reset |
| IMU 姿态同步 | L90-92 | `imu_at(图像时刻+offset)` → `R_gimbal2world` |
| YOLO 推理 | L93-95 | `list<Armor>`，滤除前哨站 |
| Tracker | L96 | `list<Target>` + `tracker_state` |
| 第2次读串口 | L98-100 | 推理后复检，失败则 `enabled=false` 本帧作废 |
| 云台角解算 | L101-102 | 最新 `q` → 欧拉角 yaw/pitch |
| 决策 | L103-104 | `Decision` |
| 第3次读串口 | L105-112 | 发送前复检，失败强制停发 `stale_before_send` |
| 下发 | L113 | `ControlDataFrame` 二进制帧 |

---

## 1. 起点：图像与统一时基（Camera）

```cpp
cv::Mat image;
std::chrono::steady_clock::time_point timestamp;
camera.read(image, timestamp);          // hero.cpp L67
```

- **全链路统一 `steady_clock`**：单调时钟，不受 NTP/系统时间跳变影响。Adapter 用 `hero::seconds()` 把时间点转成 `double` 秒传给 Controller。
- 空帧（相机离线/丢帧）→ 停发 `{false,false,0,0}` + `adapter.reset_tracking()` + 睡 10ms（L68-74）。

**为什么时间戳是全系统的锚点**：后续每一处补偿——图像龄检查（`max_frame_age`）、发射延迟（`launch_delay`）、弹丸飞行时间预测（`solve_candidate` 的迭代）、相位不确定度传播（`horizon`）——全部以图像时间戳为基准。时间错了，弹道解算再准也打不中。

## 2. 电控遥测（CBoard，双向串口）

### 2.1 上行：`board->state()` → `CBoardState`

```cpp
struct CBoardState {           // io/cboard.hpp
  double bullet_speed;          // 弹速 m/s
  Mode mode;                    // idle/auto_aim/small_buff/big_buff/outpost
  double ft_angle;
  Eigen::Quaterniond q;         // 云台姿态（IMU）
  steady_clock::time_point imu_time, status_time;  // 空=无数据
};
```

线程安全快照，串口回调线程持续更新；**IMU 帧和状态帧分别做数据龄检查**：

```cpp
auto fresh = [&](state, now) {
  imu_age    = now - state.imu_time;      // 均 ≤ telemetry_timeout (0.2s) 且 ≥ 0
  status_age = now - state.status_time;
  ...;
};
bool enabled = fresh(state, now) && state.mode == io::Mode::auto_aim;   // L77
```

### 2.2 `imu_at()`：取"图像时刻"的姿态（关键！）

```cpp
solver.set_R_gimbal2world(board->imu_at(timestamp + offset));   // L92
```

- CBoard 内部维护 5000 帧四元数时间队列，`imu_at` **插值**出参数时刻的姿态。
- 取的是**图像曝光时刻**而非当前时刻——推理耗时几十 ms，若用"现在"的姿态去解算"曝光时"的装甲板像素，云台转动越快偏差越大。
- `imu_time_offset`（hero.yaml，当前 -0.001s）补偿相机与 IMU 的固定同步误差；启动时校验 `|imu_offset| ≤ 0.1`。

### 2.3 下行：`board->send(command, true)`

```cpp
#pragma pack(push, 1)
struct ControlDataFrame {      // io/cboard.hpp
  uint8_t header;
  uint8_t control;             // 云台控制使能
  uint8_t shoot;               // 射击脉冲
  int16_t yaw;                 // ×1000, rad
  int16_t pitch;               // ×1000, rad（协议向上为负）
  uint8_t tail;
};
```

`replace_pending=true`：覆盖串口发送缓冲里还没发出去的旧指令——决策永远只代表最新一帧。

### 2.4 启动闸门（hero.cpp L40-44）

- `telemetry_timeout ∈ (0,1]`、`|imu_time_offset| ≤ 0.1`，否则抛异常。
- `hero.calibration_confirmed` 必须为 `true`——未标定的相机内外参配置直接拒绝启动，防止拿步兵标定凑合用。

## 3. 感知层（YOLO + Solver）

### 3.1 YOLO 推理（hero.cpp L93-95）

```cpp
auto armors = detector.detect(image);                          // list<Armor>
armors.remove_if([](a) { return a.name == ArmorName::outpost; }); // 滤前哨站
```

- Armor 含灯条四点、装甲类别（英雄/步兵/哨兵…）、`priority`。
- 模型配置：`min_confidence: 0.8`（hero.yaml，比步兵严格——hero 弹贵，宁缺勿滥）。
- 滤除前哨站的原因：本特性组没有前哨站模型（三元环、特殊转速），硬按车辆解算会污染 EKF。

### 3.2 Solver

- `set_R_gimbal2world(q)`（L92）：接收图像时刻姿态，构建云台系→世界系旋转。
- `solve(armor)`：PnP + 重投影优化（`optimize_yaw`/`armor_reprojection_error`），输出装甲板世界系位姿。**调用点在 Tracker 内部**（`tracker.cpp` L236/L287），逐装甲解算。
- 灯条遮挡/畸变导致的 yaw 二义性靠重投影误差择优。

## 4. 跟踪层（Tracker + EKF）

### 4.1 状态机与目标选择

状态：`lost / detecting / temp_lost / switching / tracking`。

- lost → `set_target(armors)`：从排序后的装甲里选新目标；否则 `update_target(armors)`。
- 装甲排序（tracker.cpp L56-62）：先按 `priority`（数字小者优先），同优先级按距图像中心近者优先。
- 帧间 dt > 0.1s → 直接 lost（L112-115，疑似相机离线）。
- 关键阈值（hero.yaml `tracker参数` 块）：`min_detect_count: 5`（确认帧数）、`max_temp_lost_count: 15`（temp_lost 容忍帧数）。

### 4.2 EKF：11 维车辆模型

```cpp
// x = [xc vx yc vy zc vz angle omega radius radius_delta height_delta]
//      0  1  2  3  4  5   6     7      8        9            10
```

| 下标 | 含义 | 单位 |
|---|---|---|
| 0-1 | 车心 x 位置/速度 | m, m/s |
| 2-3 | 车心 y 位置/速度 | m, m/s |
| 4-5 | 车心 z（高）/速度 | m, m/s |
| 6 | 朝向角（第 0 块装甲法线角） | rad |
| 7 | 角速度（陀螺正负判定用它的符号） | rad/s |
| 8 | 短轴半径（r） | m |
| 9 | 长短轴差（对面板半径 = r + x[9]） | m |
| 10 | 对面板高度差 | m |

协方差 `P` 中 `P(6,6)`（相位方差）、`P(7,7)`（角速度方差）、`P(6,7)`（互协方差）会一路传到决策层，做**命中时刻相位不确定度传播**（§6.4）。

### 4.3 两道发散防线

**防线一：状态物理越界**（`target.cpp` `diverged()`）

```cpp
r_ok = x[8]         > 0.05 && x[8]         < 0.5;   // 短轴
l_ok = x[8] + x[9]  > 0.05 && x[8] + x[9]  < 0.5;   // 长轴
if (!(r_ok && l_ok)) → "Target diverged!" → state_ = "lost"
```

真实车半径 0.15~0.3m；估计跑出 (0.05, 0.5) 说明滤波已崩，输出只会是垃圾解算。触发后果：丢弃估计、下一帧重新 `set_target`；hero 侧收到空 targets → `not_tracking` → `reset_tracking()` 清 settle 计数。

**防线二：NIS 统计一致性**（`tracker.cpp` L82-90 + `tools/extended_kalman_filter.cpp`）

```cpp
S   = H * P * Hᵀ + R;                    // 新息协方差
nis = rᵀ * S⁻¹ * r;                       // r = 观测残差
recent_nis_failures: 100 帧滑动窗口, nis > 0.711 记 1
Σfailures ≥ 0.4 * 100 → "Bad Converge Found!" → lost
```

- NIS 衡量"残差 vs 滤波器自报不确定度"是否匹配；模型正确时应服从 χ²(4)，仅小比例超阈。40% 超阈 = 预测与观测长期系统性不符（运动模型失配 / Q,R 调错 / 装甲关联错误）。
- **已知疑点**：注释写"自由度=4，置信水平95%"，但 0.711 是 χ²(4) 的 **5% 下分位**；95% 上分位应为 **9.488**。方向疑似写反，频繁报此日志时优先核对（`extended_kalman_filter.cpp` L66）。
- 与防线一的分工：diverged 是**结果检查**（已经崩了，立即触发）；NIS 是**过程检查**（正在崩，窗口累计 40% 才触发，容忍瞬时异常）。

### 4.4 收敛判定

```cpp
// target.cpp convergened()
name != outpost && update_count_ > 3 && !diverged() → is_converged_ = true
```

决策层的 `settle_frames`（默认 5）与它配合：Tracker 侧 3-4 帧收敛，hero 侧要求连续 5 帧 tracking，瞄准建立在收敛估计之上。

## 5. 适配层（Adapter，`tasks/hero/adapter.cpp`）

职责：把步兵系 `Target` 翻译成 hero `TargetState`，**不改动共享 Tracker**。

### 5.1 状态搬运（L74-87）

```cpp
const auto x = target.ekf_x();        // 11 维状态
const auto & p = target.ekf().P;      // 11×11 协方差
TargetState state;
state.x[i]            = x[i];                              // 11 维原样
state.armor_count     = target.armor_xyza_list().size();   // 2/3/4
state.rotation_observed = target.jumped;     // 是否已观测到完整旋转
state.phase_variance      = p(6, 6);         // 相位方差
state.phase_speed_covariance = (p(6,7)+p(7,6))/2;
state.speed_variance      = p(7, 7);         // 角速度方差
```

- `rotation_observed`（即 `target.jumped`）决定 Controller 解算几块板：没跳变前只解算已观测那块（`controller.cpp` L129），否则会出现"用未验证的几何猜对面板"。
- 只有 `x.size()==11 && P 是 11×11 && 全有限` 才搬运，畸形数据按无目标处理。

### 5.2 会话管理（L89-106）

```cpp
eligible = enabled && input.target &&
           tracker_state ∈ {detecting, tracking, temp_lost};
!eligible → session_active_ = false;
新帧且 (eligible && (!session_active_ ||
       (detecting && (!was_detecting_ || 帧间隔>100ms))))
  → session_id_++;                        // 新跟踪会话
input.target->identity = session_id_;
```

- `identity` 是**跟踪会话号**（不是装甲类别！）：detecting 重启或断流 >100ms 就换新号。
- Controller 侧看到 identity 变化 → `reset_tracking()` 清 settle。效果：目标切换后必须重新稳定 5 帧才可能开火。
- `input.tracking = (tracker_state == "tracking")`：detecting/temp_lost 都不算 tracking，不可能开火。

## 6. 决策层（Controller，`tasks/hero/controller.cpp`）

纯函数：`Decision update(const Input &)`。`Input` = TargetState + image_time + now + bullet_speed + gimbal_yaw/pitch + enabled + tracking。`Decision` = command + aim_point + flight_time + impact_phase + phase_std + spin + window_open + armor_id + reason。

### 6.1 校验链：12 个 reject 原因（L104-153）

| # | reason | 条件（默认值） |
|---|---|---|
| 1 | `invalid_time` | now / image_time 非有限 |
| 2 | `nonmonotonic_time` | `now ≤ last_now_`（时间倒流/重放） |
| 3 | `disabled` | 上游 enabled=false |
| 4 | `not_tracking` | tracker 非 tracking 态或无目标 |
| 5 | `stale_frame` | `now - image_time > 0.1s` 或时序颠倒 |
| 6 | `duplicate_frame` | `image_time ≤ last_image_time_`（同帧重放） |
| 7 | `invalid_speed` | 弹速 ∉ [10, 20] m/s 或非有限 |
| 8 | `invalid_gimbal` | 云台 yaw/pitch 非有限 |
| 9 | `invalid_target` | armor_count ∉ {2,3,4}；半径越界；协方差矩阵非半正定 |
| 10 | `no_solution` | 所有装甲板候选解算失败（超 max_range=10m / 弹道无解 / 迭代不收敛） |
| 11 | `invalid_covariance` | 相位方差传播出负值（数值异常） |
| 12 | `invalid_solution` | 解算 pitch 超弹道搜索边界 [-0.35, 0.70] rad |

注意 #2/#6：**时间水位跨 reset 保留**（L89-94 注释）——被拒绝过的帧重放进来不会因 reset 而变合法。

### 6.2 候选解算 `solve_candidate`（L24-59）：核心算法

对每块装甲板 id，求"延迟补偿后的命中解"：

**迭代结构**（16 次定点迭代）：

```text
delay = now - image_time + launch_delay     // 图像延迟(≈0.03) + 发射延迟(0.15)
flight = 水平距离 / 弹速                     // 初值
loop 16 次:
    horizon = delay + flight                 // 从"现在"到弹丸命中的总时间
    (cx, cy) = (x0,x2) + (x1,x3)·horizon     // 预测车心
    az  = x4 + x5·horizon + height_delta     // 预测高度
    angle = x6 + x7·horizon + 2π·id/N        // 预测该板朝向
    (ax, ay) = 车心 - radius·(cosθ, sinθ)     // 该板实际位置
    spin? (px, py) = 车心 - r·(cosφ, sinφ)    // 固定正面交点，φ=车心方位角
    else? (px, py) = (ax, ay)                // 直接瞄该板
    ballistic = solve_ballistic(弹速, 距离(px,py), az)
    若 |ballistic.time - flight| < 1e-4 → 收敛，返回候选
    否则 flight = ballistic.time              // 用弹道时间刷新，再迭代
```

**小陀螺策略**：|x[7]| ≥ `spin_threshold`(1.5 rad/s) 时**不追单块板**，瞄准旋转环的固定正面交点（车心朝向相机方向、半径 r 处的定点）——该点几何上恒定，命中条件变为"某块板转到交点时弹丸恰好到达"。`miss = |定点 - 该时刻最近板|` 是这个策略的误差度量。

**四板车细节**：`id` 为奇数（对面板）时半径用 `x[8]+x[9]`、高度加 `x[10]`——英雄必须区分长短轴，否则 4mm 级误差直接脱靶。

**候选择优**：`score = |phase|`（命中时刻该板相对车心的相位，越小越正对），取最小者（L130-139）。

**数字示例**：目标 5m 处匀速横移，推理耗时 30ms，`launch_delay=0.15` → `delay=0.18s`；弹速 16 m/s → `flight≈0.31s` → `horizon≈0.49s`。即解算的是"半秒后"的世界。

### 6.3 云台指令（L149-154）

```cpp
yaw   = wrap_angle(atan2(aim_y, aim_x) + yaw_offset);
pitch = -(ballistic.pitch + pitch_offset);   // SP25: 协议向上为负，抬头补偿为正
command = {control=true, shoot=false, yaw, pitch};
```

`yaw_offset/pitch_offset` 是 hero 独立零偏（hero.yaml，单位 rad，勿填 degree）。

### 6.4 相位不确定度传播（L144-148）

```cpp
variance = Pφφ + 2·horizon·Pφω + horizon²·Pωω;
phase_std = sqrt(variance);
```

把 EKF 的相位/角速度协方差沿时间外推到**命中时刻**：预测越远（horizon 越大）、陀螺越不确定，`phase_std` 越大。这是"敢不敢打"的量化置信度。

### 6.5 开火窗口 + 五道门（L160-173）

```cpp
window_open = |phase| ≤ facing_tolerance (0.20 rad) &&
              (!spin || (rotation_observed &&
                         miss ≤ position_tolerance (0.03 m) &&
                         phase_std ≤ max_phase_std (0.15 rad)));
```

| 门 | reason | 条件（默认值） |
|---|---|---|
| 1 | `outside_window` | 上式不满足 |
| 2 | `settling` | 同一 identity 连续帧数 < `settle_frames` (5) |
| 3 | `unaligned` | 云台偏差超容差（见下） |
| 4 | `auto_fire_off` | 配置 `auto_fire: false` |
| 5 | `cooldown` | `now - last_shot < min_interval` (0.7s) |
| ✓ | `fire_pulse` | 全过 → `shoot=true` 单脉冲 + 记录 `last_shot_` |

**云台容差随距离收紧**（L156-159）：

```cpp
yaw_tol   = min(yaw_tolerance,   atan2(alignment_tolerance, 水平距离));
pitch_tol = min(pitch_tolerance, atan2(alignment_tolerance, 三维距离));
```

近处 (≈2m) 由角度容差 0.01 rad 主导（≈2cm）；远处被线性容差 0.03m 收紧（8m 处 ≈0.004 rad）——本质是"弹着点横向偏差不超过 3cm"。

### 6.6 `reset_tracking()` 的精确语义（L89-94）

| 状态 | reset 后 | 原因 |
|---|---|---|
| `identity_` | 清为 -1 | 下次目标必然重新 settle |
| `settled_` | 清为 0 | 防止未稳定就开火 |
| `last_shot_` | **保留** | 冷却跨目标丢失/模式切换继续计时，找回目标不许立即连发 |
| `last_now_` / `last_image_time_` | **保留** | 时间水位防重放：被拒帧不能因 reset 复活 |

### 6.7 配置校验（构造函数 L77-87）

Config 每个字段都有类型/范围/单位检查（如 `launch_delay ∈ [0,2]`、`facing_tolerance ∈ (0, π/2)`、`max_speed ≤ 100`），非法直接抛异常——启动即失败，不带病运行。

## 7. 下发与安全设计：一帧三次串口快照

推理+决策有耗时，期间模式可能被切换、串口可能断流：

| 快照 | 行号 | 检查 | 失败动作 |
|---|---|---|---|
| ① 推理前 | L75-77 | fresh + mode==auto_aim | 停发 + reset |
| ② 推理后 | L98-100 | fresh + mode 未变 | `enabled=false` → 本帧决策必然 reject |
| ③ 发送前 | L105-112 | fresh + mode 未变 + 总帧龄 ≤ max_frame_age | 强制 `command={false,...}`、`reason="stale_before_send"`、reset |

原则：**绝不下发基于过期遥测的射击指令**；`shoot` 是单帧脉冲（L170 注释："One update pulse; never schedule detached future writes"），从不预约未来写操作。

## 8. 常见日志速查

| 日志 / reason | 来源 | 含义 | 排查方向 |
|---|---|---|---|
| `Target diverged!` | tracker.cpp L77 | EKF 半径跑出 (0.05,0.5)m | 初始化、装甲关联、运动模型；伴随 `[Target] r=, l=` 打印当时值 |
| `Bad Converge Found!` | tracker.cpp L87 | NIS 窗口 40% 超阈 | Q/R 调参；核对 NIS 阈值方向（§4.3 疑点） |
| `Large dt: {:.3f}s` | tracker.cpp L113 | 帧间隔 >0.1s | 相机掉帧/离线 |
| `stale_before_send` | hero.cpp L110 | 发送前复检失败 | 串口连通、推理耗时 vs max_frame_age |
| `not_tracking` | controller | tracker 未到 tracking | 检测稳定性、min_detect_count |
| `outside_window` | controller | 不正对/陀螺置信不足 | facing_tolerance、position_tolerance、max_phase_std |
| `settling` | controller | 稳定帧数不足 | settle_frames |
| `unaligned` | controller | 云台没跟上解算点 | 云台控制带宽；远距离先查 alignment_tolerance |
| `cooldown` | controller | 射击间隔不足 | min_interval |
| `fire_pulse` | controller | **已开火**（成功终点） | — |

调试工具：`--display` 命令行参数可视化装甲框 + reason；EKF 的 `data["residual_*"]/["nis"]` 可经 `tools/plotter`（UDP）实时曲线。

## 9. 调参指引（hero.yaml `hero:` 块，附当前值）

### 第一层：物理标定（一次性，决定"打得准"）

| 参数 | 当前值 | 标定方法 |
|---|---|---|
| `launch_delay` | 0.15（示例） | 录指令时间戳↔出膛时刻（灯板/声音/串口回传） |
| `yaw_offset` / `pitch_offset` | 0.0 | 静态打靶，弹着点系统偏差换算 rad 补入；pitch 注意符号（抬头补偿为正、协议向上为负） |
| `ballistic.drag` | 0.0（无阻力基线） | 3/5/7/9m 多距离打靶拟合；42mm 大弹低速阻力显著，不标定 10m 级偏差可达几十 cm |
| `min/max_speed` | 10/20 | 弹速散布实测 ± 裕度 |
| `imu_time_offset` | -0.001 | 打运动目标左右偏弹方向一致 → 时间同步有偏 |

### 第二层：行为参数（对着 reason 统计调）

| 参数 | 当前值 | 说明 |
|---|---|---|
| `settle_frames` | 5 | ≈100ms@50fps；与 EKF 收敛（3-4 帧）匹配 |
| `spin_threshold` | 1.5 rad/s | 看日志 omega 分布；典型小陀螺 4-8 rad/s |
| `min_interval` | 0.7s | 由机械供弹周期决定，不小于实际射速 |

### 第三层：精度窗口（保守起步，逐步放宽提射速）

联动关系：`有效命中窗口 ≈ facing_tolerance − k·phase_std (k≈2~3)`

| 参数 | 当前值 | 调整逻辑 |
|---|---|---|
| `facing_tolerance` | 0.20 rad | 非陀螺可收紧提精度；陀螺放宽换射速 |
| `max_phase_std` | 0.15 rad | 与上式联动（0.20-0.15 裕度仅 0.05） |
| `position_tolerance` | 0.03 m | 陀螺定点打不准时优先查 |
| `alignment_tolerance` | 0.03 m | 远距离 unaligned 频繁先查线性收紧 |
| `yaw/pitch_tolerance` | 0.01 rad | 近距离主导项 |

**闭环流程**：`auto_fire: false` 手动跑全场 → 统计 reason 分布 + 记录开火时刻 `impact_phase/phase_std/aim_point` → 命中复盘（开了没中→收紧或查第一层；能中但 cooldown/outside_window 占比高→逐项放宽）→ **每次只动一个参数**复测。

## 10. 学习检查点（自测）

- [ ] 能默画完整数据流，说出每站的输入/输出数据结构
- [ ] 解释 IMU 为什么取"图像时刻"而非"当前时刻"，`imu_time_offset` 补偿什么
- [ ] 三次串口快照各自防住什么故障模式
- [ ] 推导 `solve_candidate` 的迭代为什么能收敛（flight 与 ballistic.time 互为不动点）
- [ ] 解释小陀螺"固定正面交点"策略、`miss` 与 `phase_std` 各自度量什么
- [ ] 四板车 id 奇偶的半径/高度差异为什么对 hero 生死攸关
- [ ] `reset_tracking` 保留什么、清除什么，各自防什么
- [ ] 给一条"`unaligned` 占比 60%"的排查清单（至少 3 步）
- [ ] 说明 NIS 阈值 0.711 的疑点及验证方法

## 附录：一帧时序（数字示例）

```text
t=0.000  相机曝光（timestamp）
t=0.005  IMU 队列已有 [t=0.000+offset] 的四元数（插值可得）
t=0.030  YOLO 推理完成（耗时 30ms）
t=0.032  Tracker 完成（dt≈0.02s，EKF 一步预测+更新）
t=0.033  Controller 决策完成
  delay = 0.033 - 0.000 + 0.15 = 0.183s
  flight ≈ 距离/弹速（5m@16m/s ≈ 0.31s）
  horizon = 0.49s → 解算的是 0.49s 后的世界
t=0.034  board.send 下发（总帧龄 34ms ≤ max_frame_age 100ms ✓）
t≈0.184  弹丸出膛（launch_delay 后）
t≈0.49   弹丸到达预测点（horizon 时刻）
```
