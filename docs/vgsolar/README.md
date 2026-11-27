# VGSolar PX4 功能迁移：当前状态与阅读目录

更新日期：2026-11-27。适用分支 `v1.17.0_vgsolar`。最新实测与下一步见 [11月27日阶段记录](validation_20261127.md)；[11月16日记录](validation_20261116.md) 与 [11月13日记录](validation_20261113.md) 保留历史状态。早期迁移分支和提交仅作为历史来源。

APM 参考为 Ubuntu-22.04 `/home/yuyuyyu/rover` 的 `5505db87886beb85383020d9d1376982babc988c`。早期任务书位于 Windows `E:/shuwei/px4/outputs/VGSolar_APM到PX4功能迁移计划.md`，该本地路径不随仓库发布。

## 当前实现

- 输入：两路 DYP-A02、负压/红外、左右编码器；编码器有效 CPR 配置后可发布标准轮角/轮速。
- 底盘：五路原生 PWM、差速 Rover、可选轮速反馈、安全及业务模式输出门控。
- 通信：NCU 状态回传及速度/转弯/导航/系统/参数命令处理；USART3 SWAP、寄存器只读诊断与工作队列内打开/关闭串口。2BLD6010 为 RS485 只读遥测。
- 业务：VGSolar 动态模式、清洁机构状态机、急停/待释放处理、掉压告警、转弯及点到点导航。
- 诊断：压力遥测、项目电量换算、ULog 链路/事件日志、双灯启动检查、编码器速度/米数，以及启动任务生命周期、解锁 PWM 瞬态和无 5V 监测能力的适配。

软件行为、参数和实现边界见 [software_parity.md](software_parity.md) 及 [模式说明](../../src/modules/vgsolar_mode/README.md)。软件路径已实现不等于完整 APM 动态行为或整车验收。

## 启动与默认状态

板级脚本自动启动 `vgsolar_link` 和 `vgsolar_mode`。模式号动态分配，不固定为 APM 的 17；任务运行不等于已经进入该模式。

`VGS_ENABLE`、`VGS_CLN_EN`、`VGS_RNG_EN`、`VGS_IN_EN`、`VGS_SAFE_EN`、`VGS_ESC_EN` 默认 0；任意一侧保存CPR大于0时编码器自动启动；CPR默认0，轮速闭环另行使能。五路 PWM 功能默认 0，DShot/PX4IO 未启用。已保存参数不受 `set-default` 覆盖，当前实物状态以回读为准。

## 推荐阅读顺序

1. [板级入口](../../boards/vg/vgsolar/README.md)：引脚、启动、默认值、构建与烧写适用范围。
2. [现状与实机验收清单](../../boards/vg/vgsolar/PORTING.md)：已实现内容、已有证据、待验项目。
3. [软件行为与参数](software_parity.md)：APM 对应关系、机构释放、电量和导航行为。
4. [工具与测试](../../Tools/vgsolar/README.md)、[NCU 客户端](../../Tools/vgsolar/README_ncu_client.md)：离线入口和当前双向串口联调方法。
5. [11月27日实测与下一步](validation_20261127.md)、[11月16日历史记录](validation_20261116.md)、[11月13日历史记录](validation_20261113.md)、[参数存储](parameter_storage.md)、[编码器速度/米数](ENCODER_QGC_SPEED.md)、[双超声波诊断](DYP_UART_DIAGNOSTICS.md)。
6. [核心板启动及近期修复](hardware_bringup.md)、[启动蓝灯](startup_led.md)、[阀泵测试](VALVE_PUMP_BENCH.md)：按具体问题查阅。

## 早期实现过程

以下文件保留各步骤当时的实现与验证记录。“未接入输出”“只读”“手动启动”等阶段性结论不能直接作为当前全分支状态：

- [STEP01_DYP_A02.md](STEP01_DYP_A02.md)、[实现过程](STEP01_DYP_A02_IMPLEMENTATION.md)：双超声波驱动。
- [STEP02_PWM_BENCH.md](STEP02_PWM_BENCH.md)：五路 PWM 配置与原始测试流程。
- [STEP03_PRESSURE_IR.md](STEP03_PRESSURE_IR.md)、[QGC 显示](STEP03_QGC_PRESSURE_TELEMETRY.md)：负压/红外和压力遥测。
- [STEP04_EDGE_SAFETY.md](STEP04_EDGE_SAFETY.md)：最初仅发布边缘安全请求，后续已接入差速输出门控。
- [STEP05_WHEEL_ENCODERS.md](STEP05_WHEEL_ENCODERS.md)：最初原始计数，后续增加可配置的标准轮角/轮速换算和可选闭环。
- [STEP06_ENCODERS_NCU_READONLY.md](STEP06_ENCODERS_NCU_READONLY.md)：最初只读 NCU 阶段，后续已接入命令与反馈并改为板级自动启动。
- [ci_build_compatibility.md](ci_build_compatibility.md)：可选安全参数对标准 Rover/SITL 编译的兼容修复。

## 已有证据与下一步

11月16日已验证两侧执行器方向、编码器反馈、架空0.2 m/s连续运行和WSL原生ROS2速度命令链路；吸盘下降、抽气和回收动作已观察。地面磁航向入口修复与回归见 [转弯航向修复](turn_ground_heading_20261116.md)。

11月27日完成 V9 样本时序与冻结安全回收修复并实机刷写验证：消除运行中 `fault_code=512` 与 `reason=9` 误报，调节 `RO_YAW_RATE_P=0.3` / `RO_YAW_RATE_I=0.5` / `RO_YAW_RATE_LIM=20.0` 后，实车完成 `10° / 10°/s` 左转（实测 `-10.10°`、`kind=5, acc=1, reason=0`）与 `90° / 15°/s` 右转（实测 `+90.65°`）的完整吸附、抽真空、差速转弯与泄压抬升闭环。详见 [11月27日阶段记录](validation_20261127.md)。倾斜 >30° 实机保护回收、地面轨迹、实际距离精度、断链停车和整车负载仍待验收；电压保留既有标定，电流尚未标定。
