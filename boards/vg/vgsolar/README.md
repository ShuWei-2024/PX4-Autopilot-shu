# VGSolar PX4 1.17 功能迁移与板级入口

更新日期：2026-11-13。适用分支：`v1.17.0_vgsolar`；本轮起点为 `b884b72c72`。目标为 `vg_vgsolar_default`，默认机型为通用差速 Rover（`SYS_AUTOSTART=50000`）。

本分支已在早期 B0/B1 板级移植基础上加入输入采集、五路原生 PWM、NCU 通信、VGSolar 业务模式及输出门控。软件实现和已有台架记录见下文；整车、负载及导航仍需实机验收。本入口不适用于另一条 `feat/vgsolar-v1.17_shu-hardware-baseline` 分支，不能将该分支的 Rover/Copter 配置或验证结论直接套用到这里。

## 从哪里开始读

- [11月13日实测和下一次测试起点](../../../docs/vgsolar/validation_20261113.md)：实际版本、逐机参数及未完成项。
- [功能总览与阅读目录](../../../docs/vgsolar/README.md)：当前模块、默认启用状态和文档分工。
- [迁移现状与验收清单](PORTING.md)：已经实现什么、下一步验证什么。
- [核心板启动与近期修复](../../../docs/vgsolar/hardware_bringup.md)：启动生命周期、解锁 PWM 瞬态、无 USB 航电电压检查。
- [软件行为与参数](../../../docs/vgsolar/software_parity.md)：清洁状态机、急停、导航、电量和 APM 对应关系。
- [工具与离线测试](../../../Tools/vgsolar/README.md)、[RK3588/PC 客户端](../../../Tools/vgsolar/README_ncu_client.md)：构建、测试、串口诊断与命令格式。
- [早期实板记录](BRINGUP.md)：2026-10-29～30 的 B0/B1 证据，保留当时版本及测量结果。

## 当前板级配置

- STM32H743VI，16 MHz 外部晶振。配套 Bootloader 位于 `0x08000000`，应用位于 `0x08020000`，应用链接上限为 `0x081e0000`；本地板号为 0。
- ICM42688P 使用 SPI1、PE12 片选、PB2 中断、8 MHz 和驱动旋转 `-R 2`；I2C4 探测 BMP388（0x76/0x77）及 QMC5883L/P（0x0d/0x2c）；I2C1 为外部总线。
- SPI2/PD4 的 FRAM 提供 32 KiB 参数分区，当前关闭SPI2 DMA并保留NuttX多块写源指针修复，见[存储说明](../../../docs/vgsolar/parameter_storage.md)；TF 卡使用 SDMMC2。电池电压/电流为 PC4/PA4，负压为 PC0，红外为 PB5。
- 串口顺序固定为 USART1/2/3、UART4/5、USART6、UART7/8 对应 `ttyS0～ttyS7`。GPS 为 `ttyS5`，数传为 `ttyS4`，遥控为 `ttyS6`。NCU 使用 USART3/`ttyS2`、115200 8N1；控制台配置在 USART1，USB MAVLink Shell 可用。NCU 的 `VGS_CC_SWAP`、线束方向与诊断见客户端说明。
- 原生 `pwm_out` 已编入。MAIN1～5 依次为 PE13、PE14、PD14、PD15、PC8，对应 TIM1_CH3/4、TIM4_CH3/4、TIM3_CH3；三组定时器默认 50 Hz，TIM8 留给 HRT。PA8/PE11、PA0/PA1 留作右/左编码器。
- PE5 气阀低有效、关闭为高；PE6 气泵高有效、关闭为低。DShot、PX4IO、加热器和蜂鸣器硬件驱动未启用。
- 本板声明 `BOARD_HAS_NO_5V_SENSE`，因为没有独立的 5V 航电 ADC；该声明只省略未监测轨电压的上下限判断，不代表实际供电质量已测量。

## 模块与默认状态

`dyp_a02`、`vgsolar_inputs`、`vgsolar_encoders`、`vgsolar_safety`、`vgsolar_esc`、`vgsolar_link`、`vgsolar_mode` 已编入。NCU 链路和业务模式任务在板级传感器脚本中启动；任务运行不等于进入 VGSolar 模式或允许执行机构动作，模式号由 Commander 动态分配。

`VGS_ENABLE`、`VGS_CLN_EN`、`VGS_RNG_EN`、`VGS_IN_EN`、`VGS_SAFE_EN`、`VGS_ESC_EN` 默认均为 0。任意一侧已保存CPR大于0时编码器自动启动，左右 `VGS_ENC_*_CPR` 默认0；设置有效 CPR 后才转换轮角/轮速，轮速闭环还需配置 `VGS_WHL_CTRL` 和轮径。

五路 `PWM_MAIN_FUNC1～5` 默认 0，DIS/FAIL 默认 0；1000/1500/2000 us 只是测试标尺，不是已验收的机构标定。`param set-default` 不覆盖已保存参数，升级后必须回读现有映射、端点和使能值。物理输出顺序与 Servo 逻辑编号的对应需要通过功能参数明确配置。

## 构建与检查

在本仓库根目录、已配置 PX4 编译环境的 WSL/Linux 中执行：

```sh
make -j4 vg_vgsolar_default
python3 -B Tools/vgsolar/run_offline_tests.py --sanitize --all
```

固件包为 `build/vg_vgsolar_default/vg_vgsolar_default.px4`。测试包括协议、输入处理、安全门控、业务模式及 PWM/电源/串口等宿主检查，具体入口见工具文档。上传前已运行完整离线回归；V6配置已有实板记录，软件测试不能代替相应实物验收。NuttX子模块采用仓库锁定的个人fork提交，克隆后先同步并初始化子模块。

## 烧写与参数

已有兼容 Bootloader 的板通过 QGC 自定义固件入口升级本板 `.px4`。STM32CubeProgrammer 使用带正确地址的 HEX 或 BIN；不能把 JSON 格式的 `.px4` 包直接写入 Flash。首次烧写的新板需要配套 Bootloader 和应用，两者板号及地址必须匹配。

仓库中的 `tools/make_first_flash.py` 仍是 B0 阶段脚本，包含“不得含 pwm_out”的旧断言，与当前固件配置冲突；不要将它作为本分支当前的一键交付命令。文件名含 B0 的历史 HEX 也不能据此视为最新固件。当前首次烧写包需另行核对生成脚本、构建版本、两段地址、板号及哈希。

升级前导出已有参数；从 AP 切换时尤其需要保留逐机校准，因为 PX4 使用同一颗 FRAM。早期实板 STM32 面朝上、USB 朝前时保存过 `SENS_BOARD_ROT=6`，当前固件没有把它设为默认值；新板安装方向及校准需单独确认。`BAT1_V_DIV=20.65`、`BAT1_A_PER_V=85`、6S 为参考配置，实物采样比例仍需标定。

## 验证记录的范围

最新台架结果见 [阶段验收](../../../docs/vgsolar/validation_20261113.md)：参数保存/回读和重上电标定保持、编码器与双超声波采集、上锁模式切换，以及右行走中位驱动回读。电压用户已标定；逐机BAT1_V_DIV=13.62不能作为其他板的默认值，电流待测。

当前尚未验收右侧带电中位及点动、五路电气波形、完整闭环与失联/急停/边缘停车、动态航向/定位及整车循环。预检通过不表示控制器和执行器调试已经完成。

`hardware-map.json` 保留 AP 引脚来源及早期 B0/B1 状态，其中执行器驱动、首次烧写和验证字段未同步到当前阶段；当前启用状态以 `default.px4board`、`init/`、`src/timer_config.cpp` 和本入口为准。
