# 2026-11-16 VGSolar 阶段测试与续接

## 当前结论

两侧履带、编码器及原生ROS2串口速度链路已得到阶段实机证据；架空0.2 m/s目标约10秒连续运行稳定。吸附下降、抽气及回收已观察，完整按角度转弯尚未通过。本文记录逐机设置，不改变固件默认值。

## 版本与配置

PX4分支v1.17.0_vgsolar，以370c81b13b为基线加入[地面航向修复](turn_ground_heading_20261116.md)。NuttX3562263b2645542e0a503c5b0c2cefa4e960100d，保留FRAM修复与SPI2非DMA。V8固件SHA256：d45cbc59fe945b8ec38cb43a3c00e351b052af9606b7acff17baa5c60df97057；用户刷入后确认 `turn_heading=rover_mag_v8 flags_timeout=2000ms`。增量构建的Build datetime不能单独区分版本。

最新已分析log161中：

```text
VGS_ENABLE=1 VGS_IN_EN=1 VGS_RNG_EN=1 VGS_CLN_EN=1
VGS_WHL_CTRL=1 VGS_CC_SWAP=1 VGS_SAFE_EN=0 VGS_ESC_EN=0
VGS_ENC_R_CPR=22197 VGS_ENC_R_REV=1
VGS_ENC_L_CPR=23203 VGS_ENC_L_REV=0
VGS_WHL_RAD=0.0404 RD_WHEEL_TRACK=0.36
RO_MAX_THR_SPEED=0.33 RO_SPEED_P=0.2 RO_SPEED_I=0
RO_SPEED_LIM=0.2 RO_ACCEL_LIM=0.15 RO_DECEL_LIM=0.2
RO_YAW_P=0.5 RO_YAW_RATE_P=0.1 RO_YAW_RATE_I=0 RO_YAW_RATE_LIM=10
PWM_MAIN_FUNC1=102 PWM_MAIN_FUNC2=101 PWM_MAIN_FUNC3=201
PWM_MAIN_FUNC4=302 PWM_MAIN_FUNC5=303 PWM_MAIN_REV=1
VGS_SC_PWM_R=1881 VGS_SC_PWM_L=1100 VGS_SC_RATE=400
VGS_SC_IR=1 VGS_SC_PRESS=1 VGS_SC_VAC_KPA=-70
VGS_SC_VAC_DEB=300 VGS_SC_VAC_DLY=10000 VGS_SC_VENT_DLY=2000
```

Motor1=右MAIN2；Motor2=左MAIN1，左输出反向保留。MAIN1/2范围1100～1900，中位1500；MAIN3改Servo1，范围1000～2000、中位1500。用户确认1881为物理抬起端，下降1100已进入实机流程。MAIN4/5仍为通用功能302/303，未完成前后滚刷Servo2/3映射验收。SD参数备份lift_1116.bson已保存并完整解码1882字节，备份的是调整后映射；此前长文件名备份失败。

## 运动与ROS2

右侧固定归一化输出0.16/0.20，编码器速度约0.0533/0.0661 m/s；两侧前进方向已确认。log155中三段0.2目标、约10秒架空运行，稳态约0.192～0.193 m/s，无早期周期归零；加速度0.10改0.15后，到0.18 m/s约由1.9～2.0秒缩短到1.34～1.36秒。正常补零后PWM中位、轮速归零。

0.33为低速数据推导的等效前馈，不是实测满油门速度。架空编码器距离不是地面行程，左右CPR与物理里程仍需校验。

上位机独立仓库vgsolar-pvcleaner-upper，WSL工作区/home/yu/vgsolar_ws，ROS2 Jazzy；CH340串口/dev/ttyUSB0、115200。原生ROS2 /cmd_vel 0.06 m/s、5秒已测试，回传两侧主要0.05～0.06，正常结束补零后为0。参数查询反馈修复在上位机独立分支，未纳入本PX4提交。上位机协议ACK只表示接收，不能作为动作完成证据。

## 吸附转弯与问题

log169（架空）：10°、5°/s请求接受，下降后抽气约5.08秒，以kind5/accepted=False/reason9失败，再泄压抬起。最低压力约−3.50 kPa，未进入Turning；精确航向/反馈拒绝分支待定位。

log161（落地）：936.061 s接受，938.581 s抽气，940.661 s出现fault512和suction_frozen，940.681 s结果reason6并中止。压力最低约−55.25 kPa，未到−70；抽气只有约2.08秒，不能判断给足时间能否达到目标。两侧轮速0，未进入Turning。

fault512是VGSolar吸附姿态保护，条件包含真实超30°、姿态缺失/过期/未来时间或非有限数。记录横滚−17.2°、俯仰2.9°未超阈值。先取now后读取异步姿态存在时序误判风险，日志尚不能证明具体实时分支。抽气阶段冻结恢复调用release可使舵机直接跳端点，须检查回收路径。

log161中970.641 s才切到Manual，距中止约30秒，不能用后来的active=False解释最初失败。最后放平复测姿态约0.2°/2.7°仍未确认成功，对应ULog未提供；不可套用上一日志原因。

## 下次从这里继续

先补齐放平复测ULog；在上锁、动力隔离下定位采样时序和具体拒绝分支，保留真实倾角与陈旧数据保护。修复冻结恢复后的泄压、限速抬起和到位确认，增加异步更新及各阶段异常恢复回归，再编译新版本。上述修复尚未实施，没有V9。不要通过降真空阈值或禁用保护强推转弯。

随后在可密封平面做小角度真实车体转动，完整记录真空建立、转动、结束回收；再验证取消、断链停车、地面直线、负载及边缘保护。VGS_SAFE_EN=0，测距启用不代表防跌落保护已验收。

电压保留BAT1_V_DIV=13.62，电流未标定。换电后仍有过压消息，需独立核实端电压、节数和阈值；不能以告警时序邻近推定它是本次中止原因。

当天结束要求上锁、logger off、断开执行器动力，最终物理断电未获回读确认。原ULog与详细分析归档在Windows项目flight_reports，最新桌面报告为VGSolar_PX4阶段测试报告与下次续接_20261116.md；这些本地文件不随Git发布。
