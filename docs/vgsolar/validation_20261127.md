# 2026-11-27 阶段记录：V9 时序与冻结回收修复及吸附转弯实测

日期：2026-11-27。分支：`v1.17.0_vgsolar`。

## 1. V9 软件修复与离线验证

- **冻结与安全回收路径**（`src/modules/vgsolar_mode/VgsolarRobotLogic.hpp`）：
  - `freeze()` 仅在 `Lowered` 阶段置 `_frozen_ready = true` 并保持密封与 `lowered_pwm`；在 `Lowering` / `WaitDown` / `Vacuum` 及 `Venting` / `Raising` / `WaitUp` 阶段冻结时保留当前 PWM 与阀态。
  - `emergency_release(now)` 对 `Venting` / `Raising` / `WaitUp` 保持幂等，冻结恢复时刷新 `_action_start`（及未过光电门前的 `WaitUp` `_phase_start`）防止误超时；`unfreeze()` 未就绪时改为走 `emergency_release(_last_update)`，消除 MAIN3 舵机瞬跳端点风险。
- **样本时序判定与限频诊断**（`src/modules/vgsolar_mode/VgsolarMode.cpp`）：
  - `update_robot()`、`update_turn()`、`turn_heading_valid()`、`velocity_feedback()`、`fresh_inputs()`、`range_safe()`、`service_release()`、`update_navigation()`、`observe_navigation_frame()`、`process_command_impl()`、`update_vehicle_mode()` 均改为先 `.copy()` / `.update()` 再读取 `hrt_absolute_time()`，消除循环起始 `now` 早于异步主题到达时间导致的伪未来时间戳误判；保留 `ts == 0`、真实未来时间戳、过期、缺失、NaN 及 ±30° 保护。
  - 为姿态保护（`fault_code=512`）与转弯拒绝（`reason=9`）增加 1 Hz 限频 `PX4_WARN` 诊断日志。
  - 姿态保护解除改为调用 `_suction.emergency_release(att_now)` 走统一泄压斜坡回收。
- **离线回归与构建**：
  - `python3 Tools/vgsolar/run_offline_tests.py --all --sanitize` 全部通过（12/12 C++ 测试及全部 11 项宿主检查）。
  - `make vg_vgsolar_default` 构建通过，运行标记为 `turn_heading=rover_mag_v9 flags_timeout=2000ms att_ts_fix=1 freeze_release=v9`，固件 SHA256：`0740030a905134e3647b516921ef0c620aeba60d6d4ad1a848a2d4a1ae1056e1`。

## 2. 实车联调与 ULog 证据

通过 Windows `usbipd`（CH340 BUSID `4-2`，`/dev/ttyUSB0 @ 115200`）连接 WSL ROS 2 `rover_fcu_bridge` 并结合 QGC MAVLink 控制台与板载 ULog 完成实测：

1. **首次小角度尝试（`log_186_2026-11-27-15-33-10.ulg`，`10° / 5°/s` 左转）**：
   - 吸盘成功下降、红外光电 `1 -> 0` 触发、真空达到 `-70.88 ~ -74.14 kPa` 并首次进入 `Turning`，全程无 `fault_code=512` 或 `reason=9`。
   - 因原参数 `RO_YAW_RATE_P=0.1`、`RO_YAW_RATE_I=0.0` 下输出仅 `±0.0563`（PWM `1523 us`），低于履带起转死区（`|mot| ≈ 0.18`），15 秒后以 `reason=8` 超时并正常泄压抬升。
2. **调参后小角度闭环验收（`log_185_2026-11-27-15-46-58.ulg`，`10° / 10°/s` 左转）**：
   - 参数调整：`RO_YAW_RATE_P: 0.1 -> 0.3`、`RO_YAW_RATE_I: 0.0 -> 0.5`、`RO_YAW_RATE_LIM: 10.0 -> 20.0`。
   - 完整状态机闭环（总耗时 13.10 s）：`LowerSuction`（光电 `1 -> 0`，真空 `-73.37 kPa`）→ `Turning`（1.400 s，航向 `-178.88° -> 171.02°`，实测左转 `-10.10°`，双侧编码器 `-0.057 / +0.048 m/s`）→ `RaiseSuction`（0.2 s 泄压至 `-4.41 kPa`，0.4 s 回常压 `-0.61 kPa`，光电 `0 -> 1`，MAIN3 平滑斜坡）→ `vgsolar_event` 发布 `kind=5, acc=1, reason=0` 并返回 `Standby`。
3. **大角度闭环验收（`90° / 15°/s` 右转）**：
   - 通过 `/fcu/turn` 下发 `turn_mode=1, direction=2, angle_deg=90.0, angular_velocity_degps=15.0`，总耗时 18.41 s，航向 `191.36° -> 282.01°`（实测右转 `+90.65°`），双侧履带编码器 `+0.06 / -0.06 m/s`，泄压抬升后干净回到 `Standby`。

## 3. 待续验证项

- 倾斜 >30° 保护触发与 V9 冻结斜坡回收实机验证；
- 地面点到点导航精度、断链停车与整车负载实测；
- 电压与电流标定复核。
