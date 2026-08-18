# USB Plot 分阶段测试记录

## 1. 测试信息

- 测试日期：2026-08-18
- 测试固件：`c0f439e fix(usb): stabilize USBX CDC device`
- 测试脚本：`tools/plot_test.py`
- USB CDC：STM32 USB Device，VID:PID `0483:5710`
- 测试条件：动力母线断开，控制板由 USB 供电
- 系统串口映射：
  - `/dev/ttyACM0`：MICROLINK CMSIS-DAP 调试器
  - `/dev/ttyACM1`：STM32 USB CDC

串口编号由系统枚举顺序决定，不能假定 STM32 始终为
`/dev/ttyACM1`。测试前应检查：

```bash
ls -l /dev/ttyACM*
```

必要时使用 `udevadm info` 核对 VID、PID 和产品名称。测试主机没有
`python` 命令，因此实测使用 `python3`。依赖版本为 `pyserial 3.5`。

```bash
python3 -m pip install pyserial
```

`plot_test.py` 自动执行以下协议流程：

```text
PLOT_CONFIG -> RESPONSE -> PLOT_START -> DATA -> PLOT_STOP -> RESPONSE
```

## 2. 第一阶段：FAST USB 通信

### 2.1 目的

只验证 USB CDC、Plot FAST 配置、连续数据传输及停止响应。动力母线保持断开。

### 2.2 方法

```bash
python3 tools/plot_test.py \
    --port /dev/ttyACM1 \
    --fast Ia Ib Ic \
    --seconds 5
```

重点检查：

- FAST CONFIG Response 正确；
- PLOT_START Response 正确；
- 持续收到 FAST_DATA；
- FAST `seq_lost=0`；
- PLOT_STOP Response 正确。

### 2.3 结果

```text
CONFIG OK: group=FAST config=1 vars=['ia', 'ib', 'ic']
PLOT_START OK
PLOT_STOP OK

Summary
FAST: frames=10002 samples=100020 seq_lost=0
```

判定：**PASS**。

该阶段确认 USB CDC 枚举、命令收发、FAST 数据打包及序号连续性正常。

## 3. 第二阶段：NORMAL 与编码器

### 3.1 目的

在不接动力母线的条件下，通过手动转动电机验证：

```text
MT6816 -> SPI DMA -> 跨零处理 -> Wm 计算
       -> Motor_Run -> NORMAL Plot -> USB CDC
```

### 3.2 方法

```bash
python3 tools/plot_test.py \
    --port /dev/ttyACM1 \
    --normal Wm Theta_m Vbus Iq \
    --seconds 10
```

采集期间依次进行：

1. 保持电机静止约 2 秒；
2. 缓慢正向转动；
3. 缓慢反向转动；
4. 停止转动。

观察要求：

- `Theta_m` 静止稳定，手转时随位置变化；
- `Theta_m` 跨越 0/2π 时只发生正常包角回绕；
- `Wm` 正反转符号相反，转速增大时绝对值增大；
- 停止后 `Wm` 回到接近 0；
- 未使能时 `Iq` 接近 0；
- `Vbus` 符合当前 USB 供电、动力母线断开时的硬件基线；
- NORMAL `seq_lost=0`。

### 3.3 动态测试结果

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

`Motor_Run.Theta_m` 的设计范围是 `[0, 2π)`，所以跨零时原始数值会出现
接近 2π 的变化。这是正常包角，不是编码器异常跳变。位置圈数和速度计算使用
经过 ±π 判断修正后的增量。

### 3.4 静止复测结果

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

当前板卡在 USB 供电且动力母线断开时，`Vbus` 实测基线约为 3.61 V。
本阶段按实际硬件状态确认该结果合格。

### 3.5 判定

| 项目 | 判定 |
|---|---|
| NORMAL USB | PASS |
| `Theta_m` | PASS |
| `Wm` | PASS |
| `Iq` | PASS |
| `Vbus` | PASS |
| NORMAL `seq_lost` | 0，PASS |

## 4. 第三阶段：FAST 与 NORMAL 并发压力测试

### 4.1 目的

同时运行 FAST 和 NORMAL 数据流，检查 USB 吞吐、发送缓冲、序号连续性以及
20 kHz 快环实时性。

### 4.2 方法

```bash
python3 tools/plot_test.py \
    --port /dev/ttyACM1 \
    --fast Ia Ib Ic Id Iq \
    --normal Wm Theta_m Vbus Iq \
    --seconds 60
```

验收目标：

```text
FAST seq_lost          = 0
NORMAL seq_lost        = 0
Plot_Fast_Drop         = 0
Plot_Normal_Drop       = 0
Fast_Time.Deadline_Miss = 0
```

`plot_test.py` 直接统计两组 `seq_lost`。内部 Drop 和时序计数通过 CMSIS-DAP
在测试后读取。

为避免调试器停核污染时序结果，采用以下顺序：

1. 复位 MCU，清零 BSS 中的累计计数；
2. 让 MCU 自由运行，不连接调试会话；
3. 完成 60 秒 FAST 与 NORMAL 并发采集及 PLOT_STOP；
4. 测试结束后只停核一次；
5. 在首次恢复运行前读取 `Plot_Fast_Drop`、`Plot_Normal_Drop` 和实时
   `Fast_Time`；
6. 读取后恢复 MCU。若需要再次测量，应重新复位后重测。

调试器停止并恢复 CPU 后，实时 `Fast_Time` 可能记录由停核造成的异常大值，
不能继续用于本轮结论。读取变量时必须使用与已烧录固件完全一致的 ELF 和符号，
不能复用其他构建的绝对地址。

### 4.3 结果

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

| 指标 | 结果 | 判定 |
|---|---:|---|
| `Plot_Fast_Drop` | 0 | PASS |
| `Plot_Normal_Drop` | 0 | PASS |
| `Fast_Time.Deadline_Miss` | 0 | PASS |
| `Fast_Time.Fast_Max` | 6858 cycles，42.8625 µs @ 160 MHz | PASS，低于 50 µs PWM 周期 |
| `Fast_Time.ADC_ISR_Max` | 2400 cycles，15.0000 µs @ 160 MHz | PASS |

判定：**PASS**。

60 秒内 FAST 和 NORMAL 均无序号丢失，无内部缓冲丢帧，快环无 Deadline
Miss。

## 5. 阶段汇总

| 阶段 | 测试内容 | 结果 |
|---|---|---|
| 第一阶段 | FAST USB CDC，`Ia/Ib/Ic`，5 秒 | PASS |
| 第二阶段 | NORMAL、MT6816、角度与速度，10 秒动态及 3 秒静止 | PASS |
| 第三阶段 | FAST + NORMAL 并发，60 秒压力测试 | PASS |

在保持动力母线断开的测试范围内，USB CDC、Plot 协议、编码器反馈链路和
FAST/NORMAL 并发传输均满足当前验收要求。
