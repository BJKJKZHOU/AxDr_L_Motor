# USB Plot 分阶段测试记录

## 测试信息

- 测试日期：2026-08-18
- 测试脚本：`tools/plot_test.py`
- USB CDC VID:PID：`0483:5710`
- 测试条件：动力母线断开，控制板由 USB 供电

`plot_test.py` 自动执行以下协议流程：

```text
PLOT_CONFIG -> RESPONSE -> PLOT_START -> DATA -> PLOT_STOP -> RESPONSE
```

## 第一阶段：FAST USB 通信

### 结果

```text
CONFIG OK: group=FAST config=1 vars=['ia', 'ib', 'ic']
PLOT_START OK
PLOT_STOP OK

Summary
FAST: frames=10002 samples=100020 seq_lost=0
```

## 第二阶段：NORMAL 与编码器

静止约 2 秒后，依次手动正转、反转、停止，采集 MT6816 角度、速度和电流。

### 动态测试结果

```text
CONFIG OK: group=NORMAL config=2 vars=['wm', 'theta_m', 'vbus', 'iq']
PLOT_START OK
PLOT_STOP OK

Summary
NORMAL: frames=10012 seq_lost=0
```

完整帧统计：

| 信号 | 测量结果 |
|---|---:|
| `Wm` | -17.801718 ～ +12.179010 rad/s |
| `Wm` 正值帧数 | 1904 |
| `Wm` 负值帧数 | 2498 |
| `Theta_m` | 0.001917 ～ 6.282419 rad |
| `Theta_m` 最大相邻数值差 | 6.275516 rad，发生于正常 0/2π 回绕 |
| `Vbus` | 3.552978 ～ 3.654492 V |
| `Vbus` 平均值 | 3.605379 V |
| `Iq` | -0.242154 ～ +0.242925 A |
| `Iq` 平均绝对值 | 0.041218 A |

`Motor_Run.Theta_m` 范围为 `[0, 2π)`；位置圈数和速度计算使用经 ±π 回绕修正后的增量。

### 静止复测结果

静止保持 3 秒，共收到 3015 帧，`seq_lost=0`。

| 信号 | 测量结果 |
|---|---:|
| `Wm` | -0.255518 ～ +0.296235 rad/s |
| `Wm` 平均绝对值 | 0.013128 rad/s |
| `Theta_m` | 4.980452 ～ 4.980836 rad |
| `Theta_m` 静止波动 | 0.000384 rad，约 0.022° |
| `Vbus` | 3.569897 ～ 3.654492 V |
| `Vbus` 平均值 | 3.606075 V |
| `Iq` | -0.198282 ～ +0.088857 A |
| `Iq` 平均绝对值 | 0.043817 A |

USB 供电、动力母线断开时，Vbus 基线约 3.61 V。

## 第三阶段：FAST 与 NORMAL 并发压力测试

20 kHz 快环下，复位清零计数后自由运行，完成 60 秒 FAST/NORMAL 并发采集。采集期间不停核；结束后首次暂停，通过 ELF 符号定位并读取 Drop 与 Fast_Time 累计值。`seq_lost` 由采集脚本统计。

### 结果

```text
CONFIG OK: group=FAST config=1 vars=['ia', 'ib', 'ic', 'id', 'iq']
CONFIG OK: group=NORMAL config=2 vars=['wm', 'theta_m', 'vbus', 'iq']
PLOT_START OK
PLOT_STOP OK

Summary
FAST: frames=240000 samples=1200000 seq_lost=0
NORMAL: frames=60001 seq_lost=0
```

内部计数和时序结果：

| 指标 | 结果 |
|---|---:|
| `Plot_Fast_Drop` | 0 |
| `Plot_Normal_Drop` | 0 |
| `Fast_Time.Deadline_Miss` | 0 |
| `Fast_Time.Fast_Max` | 6858 cycles，42.8625 µs @ 160 MHz；PWM 周期 50 µs |
| `Fast_Time.ADC_ISR_Max` | 2400 cycles，15.0000 µs @ 160 MHz |

## 关联时序记录

ADC ISR 中 USB 唤醒导致的周期性长尾与 Deadline Miss 见[快环时序记录](20kHz快环时序调试记录.md#fast-plot-的-usb-唤醒导致-adc-isr-长尾)。

## 2026-11-02：参数入口与 Flash 保存期间采集

重复判断精简后，通过 USB 验证 150 项非法参数写入，包括可写浮点参数的 NaN、正负无穷，以及加减速度、运行限值、注入参数和枚举边界；全部在入口拒绝，原参数值保持。采集中提交未知通道配置返回 `ERR_VAR_ID`，原 FAST/NORMAL 配置继续采集。

完整编码器校准期间 FAST、NORMAL 各丢 1 块。随后在电机失能、母线约 24.9 V 时隔离测试：FAST 20 kHz 采集三相电流，NORMAL 1 kHz 采集 Wm、Wm_Ref、Id、Iq、Ud、Uq、Vbus 和机械 ESO 速度。临时将寻相电流乘以 1.001 后保存，再恢复原值并保存，分别统计动作请求耗时和固件 Drop 增量。

| 保存操作 | 请求耗时 / ms | FAST 丢块增量 | NORMAL 丢块增量 |
|---|---:|---:|---:|
| 保存临时值 | 12.564 | 10 | 11 |
| 保存原值 | 12.520 | 10 | 11 |

两次保存都复现丢块，主机接收序号连续；这两份波形标为 invalid。保存工作线程优先级为 9，USB 发送线程为 11，同步保存延后发送服务是当前推测，尚未直接测量线程调度时序。

本地数据：`test-data/20261102_check_cleanup/Plot-flash-save.json` 及对应 FAST/NORMAL 波形；完整校准结果见 `Encoder-complete.json`。
