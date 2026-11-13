# 双路 DYP-A02 串口诊断与实测

更新：2026-11-13。

左侧UART8 `/dev/ttyS7`，右侧USART2 `/dev/ttyS1`，9600 8N1，自动输出四字节协议。供电、TX/RX和电平按实际板卡/传感器资料核对；软件总线名不能代替物理针序。

## 生命周期修复

旧驱动在启动命令任务组open/configure，在另一个NuttX工作队列read，文件描述符可能无效或指向别的文件。现在启动命令只保存路径和消息元数据，工作队列内完成打开、配置、读取、关闭；打开失败每秒重试，单侧失败不阻塞另一侧。

主机回归提取实际spawn/Run流程，检查两任务组归属、有效帧发布、stop-before-open、关闭及单侧打开失败/重试恢复；旧实现失败、修复后通过。

## 查看与自启动

```sh
dyp_a02 status
```

未运行时可手动启动：

```sh
dyp_a02 start -l /dev/ttyS7 -r /dev/ttyS1
```

确认测距正常后，`param set VGS_RNG_EN 1`、`param save`使下次上电自动启动，仅启用采集，不等于边缘停机保护已验收。

诊断含义：

- `uart_open=1`：本工作队列已打开并配置串口。
- `rx_bytes`：累计收到字节；一直0时核对供电、实际插口、共地、TX到RX和传感器输出模式。
- `frames`：累计校验正确帧，不等于物理精度达标。
- `checksum` / `partial_timeout` / `io_errors`：校验、半帧超时、系统I/O错误。
- `last_errno`：最近实际打开/配置/读取错误；收到字节时清零，正常EAGAIN不计故障。
- `latest_mm`：有效距离mm；4294967295表示无效或过期。

`listener distance_sensor -i 0 -n 1`、`-i 1`可读uORB距离，单位m；QGC的MAVLink `DISTANCE_SENSOR.current_distance`单位cm。实例顺序以device_id总线号为准：SERIAL:7为左、SERIAL:1为右。quality=0和距离0表示无效，不能当作贴近目标。

## 已测范围

用户实测修复后两侧frames及字节持续增长，约0.52/0.55m随目标变近到0.063/0.066m，校验/超时/I/O错误均为0。接好底板重上电，仅status查询得到左62mm/379帧、右66mm/381帧，确认该轮自启动及收数。

尺量精度、逐侧对应、断线/超量程/恢复以及边缘保护的实际停车仍待分别验收。此前未接底板时not running的原因未单独由日志证明，不能将“不接底板”与“软件驱动停止”画等号。完整阶段结果见 [验收记录](validation_20261113.md)。
