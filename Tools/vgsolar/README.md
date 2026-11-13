# VGSolar 开发、测试与通信工具

更新：2026-11-13，适用 `v1.17.0_vgsolar`。

## 构建和离线回归

先按主仓库gitlink初始化NuttX等子模块，再在已配置工具链的Linux/WSL根目录执行：

```sh
git submodule sync -- platforms/nuttx/NuttX/nuttx
git submodule update --init --recursive
make -j4 vg_vgsolar_default
python3 -B Tools/vgsolar/run_offline_tests.py --sanitize --all
```

`--all`包含12组C++（含生产模式主机测试）、9项NCU客户端测试以及PWM/双灯/解锁中位/电源/串口/FRAM/遥测检查。仅跑11组纯逻辑协议测试可省略`--all`；生产模式测试需要已生成的uORB头。临时二进制由系统临时目录自动清理，不提交到Git。

## 保留的检查入口

- `run_offline_tests.py`：统一入口，新增检查均已接入`--all`。
- `ncu_client.py`、`ncu_client_test.py`：PC/RK3588协议客户端及布局/解析/退出停车/虚拟串口回归；真实串口依赖见requirements-rk3588.txt。
- `check_pwm_bench.py`：五路资源、编码器排除、HRT、默认禁用及台架脚本。
- `boards/vg/vgsolar/tests/test_boot_led.py`：当前双灯/失败锁存/启动钩子和生产C++前台等待。移除过时重复的`check_boot_led.py`。
- `check_arming_neutral.py`：双向中位、预解锁到解锁连续性及普通电机渐变。
- `check_power_sense.py`：有/无5V采样能力时的生产供电预检。
- `check_uart_swap.py`、`check_uart_owner.py`：NCU SWAP和工作队列UART生命周期。
- `check_dyp_uart_owner.py`：生产DYP spawn/Run的任务组归属、有效帧、停启及单侧打开失败恢复。
- `check_fram_bwrite.py`：生产FRAM普通/chunked多页源缓冲前移、内容、边界及出错解锁。
- `src/modules/vgsolar_encoders/test/test_speed_telemetry.py`：生产NAMED_VALUE_FLOAT的速度/米数、左右均值、NaN/过期及压力/调试共存。

新增后三项覆盖本轮实际故障或遥测接口，属于有用回归。临时修改脚本、打包脚本、试验快照/日志不加入源码仓库；测试fixture保留在有明确入口的测试目录。构建产物和逐机校准不提交。

## 结果范围

上传前完整离线回归通过；固件编译另行记录。UART任务组、GPIO/HRT、SPI页写及虚拟串口均采用宿主替身，不能代替现场电平、PWM波形、传感器精度、真实RK3588长时链路和整车验收。

[实机结果与接续](../../docs/vgsolar/validation_20261113.md) · [参数存储/子模块](../../docs/vgsolar/parameter_storage.md) · [NCU客户端](README_ncu_client.md)。
