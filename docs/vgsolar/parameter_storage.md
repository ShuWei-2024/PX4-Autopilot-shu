# VGSolar 参数存储与恢复

更新：2026-11-13，适用 `v1.17.0_vgsolar`。

## 当前配置和修复

参数默认保存在 SPI2 FRAM 的 `/fs/mtd_params`，分区大小32768字节；TF卡备份路径由 `param status` 查看。SPI2使用非DMA传输，SPI1 IMU及UART的DMA配置保留。

NuttX `drivers/mtd/ramtron.c` 的普通和chunked多块写入循环，原来只推进目标页号，没有在一页写完后推进源buffer。修复后每页增加 `1 << priv->pageshift`。生产函数体主机测试在旧代码上失败，修复后覆盖0/1/2/3/17/51页、非零起点、边界及出错解锁。

该修复单独上板仍保存失败。保留修复、仅关闭SPI2 DMA后，用户实测 `param save` 无错误，板载BSON完整解码1631字节（48项INT32、34项FLOAT）；重新上电回读编码器标定和超声波使能保持。现场异常与SPI2 DMA路径有关，DMA/缓存/硬件时序的具体机理及长期可靠性尚未分别验证。

NuttX修复位于独立子模块，`.gitmodules` 指向 `nihaoyugaxiang/NuttX` 的 `vgsolar/fram-multiblock-20261113` 分支；主仓库gitlink固定实际提交。按主仓库提交初始化，不能将子模块擅自换回上游HEAD：

```sh
git submodule sync -- platforms/nuttx/NuttX/nuttx
git submodule update --init --recursive
```

## 参数保存检查

保持上锁、执行器动力隔离，先另存独立备份并回读：

```sh
param save /fs/microsd/vgsolar_params_manual_backup.bson
bsondump /fs/microsd/vgsolar_params_manual_backup.bson
param save
bsondump /fs/mtd_params
```

要求保存不报错，BSON声明长度与实际解码长度一致且包含实际参数。文档长度随参数变化，不必固定1631。`mtd readtest`只证明能读完整分区，不能证明保存内容正确。

只有保存及回读均成功，才进行完全断电冷启动和标定值回读。累计路程按设计在采集重启时归零，不属于参数丢失。

## 已损坏参数区的恢复

先验证独立备份可完整解析；使用与本机硬件一致的备份，不能复制其他车辆的传感器校准和输出映射。保持动力隔离，导入后先核对参数，再保存：

```sh
param import /fs/microsd/vgsolar_params_manual_backup.bson
param save
bsondump /fs/mtd_params
```

若失败，不要反复擦除、覆盖独立备份或关闭校验来掩盖故障。记录完整输出与固件身份继续诊断。

11月13日的早期独立备份包含传感器校准、CPR及半径，但早于轮距0.36m等后续修改；正常启动不应重复导入旧备份。最新实机状态及接续步骤见 [阶段验收](validation_20261113.md)。
