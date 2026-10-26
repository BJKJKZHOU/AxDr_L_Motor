# 20 kHz 快环时序调试与修改记录

更新时间：2026-10-12。

早期基线的功率状态：TIM1 CH1/2/3 及其互补输出尚未启用，SVPWM 只执行计算，不写入三相 PWM CCR。

2026-10-12 的数学函数审查和 Angle_Wrap 优化见[有感电流快环性能优化与验证（2026-10-12）](#fast-loop-20261012)；Vector2_Limit 缩放分支独立测试见[Vector2_Limit 缩放分支独立耗时（2026-10-12）](#vector-limit-20261012)；未完成的双限幅实验与板卡恢复见[双限幅联合实验中止及固件恢复（2026-10-12）](#dual-limit-cancelled-20261012)。历史 160 MHz 数据不与本次 170 MHz 数据混用。

## 早期硬件时序基准（160 MHz）

### TIM1 与 ADC

- CPU、TIM1 时钟：160 MHz；
- DWT 和 TIM1 每周期：6.25 ns；
- TIM1：中心对齐，ARR=4000；
- PWM 周期：`2 × 4000 / 160 MHz = 50 us`，即 20 kHz；
- TIM1 CH4：`CCR4=2240`，上升计数约14 us时产生比较中断并启动MT6816；
- TIM1 CH5：`CCR5=3665`，OC5REF通过TRGO2触发ADC injected；
- ADC触发位置：下溢后约 `3665 / 160 MHz = 22.906 us`；
- 三路电流采样孔径约为22.906～27.094 us，对称分布在25 us计数峰值两侧；
- ADC1：3 个 injected 通道；
- ADC2：1 个 injected 通道；
- ADC1、ADC2 时钟：160 MHz / 4 = 40 MHz；
- injected 采样时间：47.5 ADC 周期。

编码器和ADC时刻由 [AxDr_L_Motor.ioc](../../../AxDr_L_Motor.ioc) 中的`TIM1.PulseNoDither_4=2240`、`TIM1.PulseNoDither_5=3665`唯一管理，运行时不再保存或覆盖第二份比较值。

### MT6816 SPI

- SPI1：Mode 3、16 bit、10 MHz；
- 每个 PWM 周期读取寄存器 `0x03`、`0x04` 两帧；
- DMA1 Channel5：SPI1 RX，优先级 1；
- ADC1_2 IRQ：优先级 2；
- TIM1 update和CC IRQ：优先级 0；
- CSN：PD2，初始 High、内部 Pull-up。

## DWT 计时字段

DWT 时钟为160 MHz，换算公式：

```text
时间(us) = Cyc / 160
```

`Fast_Time`/`Fast_Snap` 主要字段：

| 字段 | 含义 |
|---|---|
| `T0` | TIM1 下溢中断入口的 DWT 基准 |
| `TIM_ISR_Cyc` | TIM1 下溢 ISR 执行时间 |
| `SPI_1_Cyc` | 第一帧 RX DMA HT 进入时刻，相对 `T0` |
| `SPI_1_ISR_Cyc` | 第一帧 HT 直接处理时间 |
| `SPI_2_Cyc` | 第二帧 RX DMA TC 进入时刻，相对 `T0` |
| `SPI_2_ISR_Cyc` | 第二帧 TC 直接处理时间 |
| `Enc_Cyc` | `Motor_Run.Theta_e` 发布时刻，相对 `T0` |
| `ADC_Cyc` | ADC1_2 IRQ 进入时刻，相对 `T0` |
| `ADC_Run_Cyc` | `ADC_Run()`中采样、电流环和SVPWM的执行时间 |
| `ADC_ISR_Cyc` | 整个 ADC1_2 ISR 墙钟时间 |
| `Fast_Cyc` | TIM1 下溢至 ADC ISR 结束的总时间 |
| `Enc_Late` | ADC IRQ 入口时 `Enc_Cyc` 尚未有效的次数 |
| `Enc_Miss` | 编码器状态、DMA 或 SPI 异常次数 |
| `Deadline_Miss` | ADC ISR结束前TIM1已经进入下一PWM周期的次数 |

冻结快照流程：

```text
0～2000周期       启动和预热
第2000周期        清零 Max 和错误计数
第2000～4000周期  稳态统计
第4000周期        复制到 Fast_Snap，Fast_Snap_Ready=1
```

调试器断点会让实时 `Fast_Time` 出现巨大的 `Fast_Max`、`Deadline_Miss` 或 `UINT32_MAX` 哨兵值，因此性能结论只使用不中断运行后得到的 `Fast_Snap`。

## 早期 SPI DMA 与 ADC 中断路径对照

三相 PWM 关闭，160 MHz DWT，预热 2000 周期后统计 2000 周期。以下为编码器从 TIM1 下溢开始采集的旧时序；当时 `Deadline_Miss` 按 25 μs 半周期边界统计。

| 路径 | 编码器角度发布 μs | ADC IRQ 入口 μs | ADC 主体最大 μs | ADC ISR 最大 μs | Fast_Max μs | 越过 25 μs 周期数 |
|---|---:|---:|---:|---:|---:|---:|
| HAL SPI Tx/Rx DMA，两次独立传输 | 27.550 | 19.056 | 7.769 | 19.213 | 38.381 | 2000 |
| RX Circular、直接 HT/TC，ADC JEOC | 7.463 | 2.294 | 10.344 | 15.263 | 17.675 | 0 |
| RX Circular、DMA 旁路 HAL，ADC JEOS | 7.413 | 5.281 | 7.463 | 12.556 | 17.913 | 0 |

旧 HAL 路径每周期有 4 次 RX、2 次 TX 相关事件；优化为长度 2 的 RX Circular，HT 启动第二帧、TC 发布角度，取消 TX DMA。SPI 由 5 MHz 改为 10 MHz，TIM1/DMA/ADC 优先级为 0/1/2，CSN 高电平间隔 200 ns。

<a id="jeos与dma中断旁路hal后的实测"></a>

### JEOS 修正与角度更新顺序

JEOC 在 ADC1 首通道结束时触发，2.294 μs 的入口时间对应首通道完成；回调读取 JDR 时，后两通道可能仍是上一周期结果。改用 `ADC_EOC_SEQ_CONV` 后，三通道完整序列结束才产生 JEOS，入口变为 5.281 μs。ADC2 单通道不产生中断。

JEOS 对照中，ADC IRQ 在 T0+845 cycles 进入，DMA TC 于 915 cycles 抢占，1186 cycles 发布角度、1203 cycles 返回 ADC。因 DMA 先于 Park 更新角度，`Enc_Late=2000` 表示 IRQ 入口早于角度发布，Park 实际使用本周期角度。

## 固定低边采样时序修改

JEOS 基线见前述路径对照。

当时配置变更：

- CH4比较中断在约14 us启动MT6816，替代下溢立即启动；
- CH5的OC5REF通过TRGO2在22.906 us触发ADC1、ADC2；
- ADC1_2正常JEOS路径直接清标志并调用 `ADC_Run()`，不再经过两次通用HAL ADC处理；
- MT6816两帧均增加至少100 ns的CSN低电平建立时间；
- `Deadline_Miss`改为检测是否越过下一次TIM1下溢，即50 us PWM更新边界；
- Release 反汇编确认 JEOS 和 CC4 正常路径为直接调用。

## FAST Plot 的 USB 唤醒导致 ADC ISR 长尾

### 分段计时方法

采用 2048 周期窗口计时。Sensorless I/F 达到 Ready 后自动请求一次Profile，分别累计以下分段的 Min/Sum/Max：

```text
ADC_Sample
Motor_Fast
  Flux_Observer
  PLL
  IF_Start
Current_Loop
SVPWM
PWM_Update
Plot_Fast
ADC_Run
```

窗口统计用于定位稳定路径中的具体长尾；独立清零后的 `Fast_Time` 最大值用于检查整轮运行中的最坏墙钟时间和 Deadline。

### 优化前的问题证据

优化前使用 7 路 FAST 加 1 路 NORMAL 运行 Sensorless Shadow。2048 周期窗口结果为：

| 分段 | Min | Mean | Max |
|---|---:|---:|---:|
| `Plot_Fast` | `2.9438 us` | `3.3608 us` | `11.5375 us` |
| `ADC_Run` | `18.4500 us` | `18.8721 us` | `27.1875 us` |

独立清零后整轮最大值为：

| 字段 | 优化前 |
|---|---:|
| `ADC_Run_Max` | `27.8438 us` |
| `ADC_ISR_Max` | `28.4125 us` |
| `Fast_Max` | `56.2688 us` |
| `Deadline_Miss` | `8015` |

NLOB 和 PLL 的固定耗时只有：

```text
Flux_Observer = 0.8313 us
PLL           = 1.6875 us
sum           = 2.5188 us
```

真正的异常是 `Plot_Fast` 每完成一个 FAST 数据块时出现约 `8 us` 的周期性长尾。当时 `Plot_Fast_Sample()` 在 ADC ISR 中调用 `USB_Tx_Wake()`，后者直接执行`tx_event_flags_set()`。ThreadX event 唤醒进入了 20 kHz ADC ISR 的硬实时路径。

### 优化方法

发送请求按调用上下文分流：

- 线程上下文中的 `USB_Tx_Wake()` 仍直接设置 USB TX event；
- 中断上下文只在短临界区内将发送位 OR 到 `USB_Tx_Pending`；
- `2 kHz` Motor Thread 在 `Motor_Control()` 后调用 `USB_Tx_Poll()`；
- Poll 原子取走 pending 位，再在线程上下文调用 `tx_event_flags_set()`；

### 实机复测条件

```text
HEAD          = ae4706b
CPU / PWM     = 160 MHz / 20 kHz
Vbus          ~= 14.54 V
Rs            = 0.08471736 ohm
Ld = Lq       = 17.836 uH
Flux          = 0.0031835556 Wb
Voff          = 0.0141716 V
```

测试参数：

参数由自动脚本重新辨识、Apply；每轮时长测试前在 DISABLE 状态下通过调试器独立清零计数，测试结束由脚本执行 `DISABLE`。

### 7 FAST + 1 NORMAL 分段结果

独立复测共收到 `250784` 个 FAST 样本、`62696` 帧，主机序号丢失为 0。

| 分段 | Min | Mean | Max |
|---|---:|---:|---:|
| `ADC_Sample` | `1.0125 us` | `1.0184 us` | `1.1375 us` |
| `Motor_Fast` | `6.1313 us` | `6.1313 us` | `6.1750 us` |
| `Flux_Observer` | `0.8313 us` | `0.8313 us` | `0.8313 us` |
| `PLL` | `1.6875 us` | `1.6875 us` | `1.7313 us` |
| `IF_Start` | `1.1625 us` | `1.1625 us` | `1.2000 us` |
| `Current_Loop` | `3.9625 us` | `3.9625 us` | `3.9625 us` |
| `SVPWM` | `2.2563 us` | `2.2563 us` | `2.2563 us` |
| `PWM_Update` | `0.4875 us` | `0.4875 us` | `0.4875 us` |
| `Plot_Fast` | `2.9438 us` | `2.9636 us` | `3.3375 us` |
| `ADC_Run` | `18.4500 us` | `18.4758 us` | `18.8438 us` |

整轮最大值：

| 字段 | cycles | 时间 | 结果 |
|---|---:|---:|---|
| `ADC_Run_Max` | 3149 | `19.6813 us` | PASS |
| `ADC_ISR_Max` | 3240 | `20.2500 us` | PASS |
| `Fast_Max` | 7742 | `48.3875 us` | PASS，距离 50 us 为 `1.6125 us` |
| `Enc_Late` | 0 | - | PASS |
| `Enc_Miss` | 0 | - | PASS |
| `Deadline_Miss` | 0 | - | PASS |

板内生产端 FAST Drop=0、NORMAL Drop=9，NORMAL 丢弃发生在序号生成前。

### NORMAL-only 对照

NORMAL-only 运行共收到 `12584` 帧，主机序号丢失、FAST Drop 和 NORMAL Drop 均为 0。

| 分段 | Min | Mean | Max |
|---|---:|---:|---:|
| `Plot_Fast` | `0.3000 us` | `0.3000 us` | `0.3000 us` |
| `ADC_Run` | `15.8063 us` | `15.8564 us` | `15.9313 us` |

| 字段 | 时间 | 结果 |
|---|---:|---|
| `ADC_Run_Max` | `17.1625 us` | 与优化前基线相同 |
| `ADC_ISR_Max` | `17.7313 us` | 与优化前基线相同 |
| `Fast_Max` | `45.7750 us` | 距离 50 us 为 `4.2250 us` |
| `Deadline_Miss` | `0` | PASS |

该对照说明优化没有把开销转移到普通快环路径。

### 优化前后结论

| 指标 | 优化前 7 FAST | 优化后 7 FAST | 变化 |
|---|---:|---:|---:|
| `Plot_Fast Mean` | `3.3608 us` | `2.9636 us` | `-0.3972 us` |
| `Plot_Fast Max` | `11.5375 us` | `3.3375 us` | `-8.2000 us` |
| `ADC_Run window Max` | `27.1875 us` | `18.8438 us` | `-8.3437 us` |
| `ADC_ISR_Max` | `28.4125 us` | `20.2500 us` | `-8.1625 us` |
| `Fast_Max` | `56.2688 us` | `48.3875 us` | `-7.8813 us` |
| `Deadline_Miss` | `8015` | `0` | 问题消失 |

实测差值与原 ThreadX event 唤醒长尾一致，支持根因和修改方向。

Fast_Max 从 TIM1 更新中断基准计至 ADC ISR 结束，包含编码器获取、ADC 触发、硬件转换等待和中断执行。本轮端到端 Deadline 余量为 1.6125 μs。

NLOB 与 PLL 的固定耗时合计 2.5188 us。

<a id="fast-loop-20261012"></a>

<a id="9-有感电流快环性能优化与验证2026-10-12"></a>

## 有感电流快环性能优化与验证（2026-10-12）

### 优化内容及版本边界

数学函数计时使用带 DWT 探针的实验固件；探针及实验宏未纳入正式固件。

本轮正式优化包括：有感电角度与 SinCos 结果复用；电流 PID 的 Kd=0 快速路径；SVPWM 复用母线电压倒数；dq 前馈解耦默认开启；`Angle_Wrap()`的多圈归一化使用 IEEE 754 binary32 整数余数运算，避免常规角度调用 Flash 中的 `fmodf`。未改变机械 ESO 方程、PWM 开关频率、采样触发点或两相采样方式。旧 NVS 中已经保存的前馈开关值不受默认值变化覆盖。

### 数学函数测量与数值校验

CPU/TIM1 170 MHz、20 kHz PWM、ST Arm Clang Release `-Oz`。电机在有感 POSITION + RUN 下保持当前位置，母线约 24.7～24.8 V、LCD 正常刷新。数学函数单独计时使用临时 DWT 探针，因此包含调用和时间戳开销。

| 调用点 | 优化前平均 µs | 优化后平均 µs | 本轮确认 |
|---|---:|---:|---|
| 电角度 `Angle_Wrap()` | 1.829087 | **0.617647** | 旧实现进入 `fmodf`，新实现正常输入不再调用 |
| ESO 内部 `Angle_Wrap()` | 0.300113 | 0.276529 | 单圈输入，旧实现已走快速分支 |
| `SinCos()` | 0.246303 | 0.245381 | 1024 点 LUT，代码位于 CCMRAM、表位于 Flash |
| 电流参考 `Vector2_Limit()` | 0.188235 | 0.188235 | 未触发缩放 |
| 电压 `Vector2_Limit()` | 0.176471 | 0.176471 | 未触发缩放 |
| d 轴 `PID_Run()` | 0.478454 | 0.478137 | Kd=0 |
| q 轴 `PID_Run()` | 0.449010 | 0.448611 | Kd=0 |
| `SVPWM_Calc()` | 1.170753 | 1.170763 | 正常调制路径 |

单独计时各约 20.9 万次。电角度归一化旧实现的最大值约 2.206 µs，新实现该工况约 0.618 µs。主机对照 **3,312,909 组 float32 输入**（正负零、边界、非正规数、大数、NaN/Inf 及随机位模式），与旧实现**逐位一致**；反汇编确认新实现没有 `fmodf` 或`memcpy` 函数调用。

新整数余数算法的迭代次数仍与输入指数有关；本次 105 cycles 仅适用于所测电角度，不是对任意有限 float 输入的最坏执行时间承诺。

### ARR→CCR 完整路径对照

关闭逐函数数学探针，但实验版本保留 ARR→CCR 时间直方图与超限计数；两组均为 10 s 左右、约 20.1 万次 PWM 有效更新。该测量版本不是已合入 `main` 的无诊断代码版本。

| 指标 | 旧 Angle_Wrap | 新 Angle_Wrap |
|---|---:|---:|
| ARR→CCR 最小 µs | 11.288235 | **10.388235** |
| ARR→CCR 平均 µs | 11.751649 | **10.537932** |
| ARR→CCR 最大 µs | 13.223529 | **11.605882** |
| 超过 12.5 µs 次数 | 152 | **0** |
| 出现超限的 50 ms 窗口 | 104 / 201 | **0 / 201** |
| P99.9 分位桶上边界 µs | 12.647059 | **10.876471** |
| Fast 完整路径最大 µs | 39.323529 | **37.688235** |
| ADC ISR 最大 µs | 11.376471 | **9.758824** |
| ARR→CCR 跨半周期 / Deadline / 编码器异常 | 0 | 0 |

本次位置保持工况下，优化后 ARR→CCR 最大值距 12.5 µs 仍有**0.894118 µs** 余量。直方图是每桶 43 个 TIM1 ticks 的区间统计，分位数据是桶上边界，不是精确分位点；严苛截止时间仍以周期最大值和超限次数判断。

### 无本轮专用诊断代码的正式固件复测

PR #41 的干净合并候选 `perf/foc-hotpath-cleanup`（`60a3359`）经本地 Release 烧录测试：170 MHz、20 kHz、母线约 24.8 V、有感 POSITION 保持、原有限流 10 A、dq 前馈开启、LCD 正常刷新，采集 10 s。

| 指标 | 结果 |
|---|---:|
| Fast 完整路径最大（含采样等待） | **39.106 µs** |
| ADC ISR 最大 | **8.718 µs** |
| `Fast_Loop()` 最大 | **7.894 µs** |
| Deadline Miss / 编码器迟到 / 编码器丢帧 / 故障 | **均为 0** |

<a id="vector-limit-20261012"></a>

## Vector2_Limit 缩放分支独立耗时（2026-10-12）

使用当前 Release 编译对象、170 MHz DWT；在 PWM 关闭时调用，测量阶段屏蔽中断。每组 100,000 次，正式结果保留调用和时间戳等少量开销：

| 路径 | cycles | µs |
|---|---:|---:|
| 未超限或恰好到限 | 29 | 0.170588 |
| **触发缩放** | **69** | **0.405882** |
| 限值为零、输出清零 | 20 | 0.117647 |

五组缩放测试（轻微超限、明显超限、负分量与零分量）合计**500,000 次**，最小/平均/最大均为 69 cycles，结果核验通过。缩放比未缩放增加 **40 cycles（0.235294 µs）**；`sqrtf` 在本工具链中使用硬件 `VSQRT.F32`，缩放另有硬件浮点除法。

<a id="dual-limit-cancelled-20261012"></a>

## 双限幅联合实验中止及固件恢复（2026-10-12）

双限幅联合实验两次均在基线阶段因反馈电流保护中止。

第二次中止后，调试连接中断导致 Flash 起始区被擦空；恢复备份固件后启动恢复。

## 2026-10-06：快环代码移入 CCMRAM

### 放置与启动验证

快环使用 FAST_CODE 和 .preinit_array 启动复制方式放入 CCMRAM。

- `User/Fast_Memory.h` 将指定函数放入 `.ccmram_text`；`User/Fast_Memory.c` 在 `main()` 前由 Flash 复制到 CCM。
- 普通 RAM 改为 96 KB，栈顶为 `0x20018000`。末尾 32 KB 的 SRAM 别名不再与 CCM 代码重复分配。
- CCM 内包括 ADC/PWM/编码器中断入口、快环分派、电流环、Flux Observer、PLL、Mechanical ESO、
  I/F 与接管热点、数学运算、编码器读数处理、快速保护、PWM 更新及 FAST 采样。
- 2 kHz 控制、辨识专用流程、USB/RTOS 和通用 HAL 仍在 Flash；DMA 缓冲区留在普通 RAM，查表常量留在 Flash。

Release 构建占用：CCM 8,948 B / 32 KB，普通 RAM 26,744 B / 96 KB，Flash 101,380 B。链接与反汇编确认关键函数位于 `0x10000000..0x100022F3`，启动复制函数本身仍位于 Flash。下载复位和额外一次复位后，SWD 读取的完整 CCM 代码均与 ELF 提取内容逐字节一致。

### 实机时序与速度

测试使用 Pp=7、Rs=0.271844 Ω、Ld=Lq=69.8889 μH、Flux=0.00228238 Wb、J=1.67043e-5 kg·m²、B=8.00185e-6 N·m·s/rad；电流环 500 Hz、速度环 20 Hz、ESO 100 Hz、电流限值 2 A。6 通道 FAST 20 kHz 与 15 通道 NORMAL 1 kHz 并发，SWD 低频读取时序。八组各从静止启动、闭环保持约 5 s；表中速度均值及 ADC 主体中位数取末段约 3 s，DWT 按 160 MHz 换算。

| 模式 | 目标 (rad/s) | ESO 均值 | 编码器差分均值 | PLL 机械速度均值 | ADC 主体中位数 (μs) | FAST / NORMAL 丢块增量 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 无感，旁路关闭 | +80 | — | 79.7991 | 79.8010 | 12.81 | 0 / 0 |
| 无感 + ESO 旁路 | +80 | 80.1154 | 80.1165 | 80.1170 | 16.66 | 0 / 0 |
| 无感 + ESO 旁路 | +120 | 119.7166 | 119.7154 | 119.7157 | 15.91 | 0 / 0 |
| 无感 + ESO 旁路 | +150 | 149.8888 | 149.8917 | 149.8913 | 16.24 | 3 / 2 |
| 无感 + ESO 旁路 | -80 | -80.1879 | -80.1884 | -80.1895 | 16.74 | 0 / 0 |
| 有感，以 ESO 闭环 | +20 | 19.7743 | 19.7736 | — | 11.31 | 16 / 15 |
| 有感，以 ESO 闭环 | +80 | 79.7894 | 79.7878 | — | 10.70 | 0 / 0 |
| 有感，以 ESO 闭环 | -80 | -79.7969 | -79.7966 | — | 11.32 | 0 / 0 |

对照 CCM 接入前同样负载的 +80 rad/s 数据：无感主体由 21.35 降至 12.81 μs，无感 + ESO 旁路由约 28.72 降至 16.66 μs，有感由约 17.16 降至 10.70 μs。

8 组全程 `Deadline_Miss`、`Enc_Late`、编码器 `Miss_Cnt` 与 CRC 错误增量均为零。从 TIM 更新中断计时起点至 ADC 快环完成，累计最大值为 48.6625 μs；这次测试没有越过 50 μs，本轮最小余量约 1.34 μs。编码器读数处理 IRQ 的稳态中位数约 1.71～1.84 μs，包含 SPI 传输与触发等待的`Enc_Cyc` 约 21.7～21.9 μs；两者不是同一种耗时。

有 ESO 的 7 组稳态角度误差峰值均小于 0.0019 rad；无感 + ESO 的平均速度与编码器差分相差小于 0.003 rad/s。正反转采集中均观察到多次单圈角度回绕，没有整圈误差尖峰。这些结果支持单圈 ESO 的速度来源修复；原有周期速度纹波仍存在。

### 遥测丢块

两组出现 Plot_Fast_Drop / Plot_Normal_Drop，计数对应发送前缓冲区丢块，具体数量见上表。

## 2026-08-21：NLOB 与 PLL Shadow 快环耗时

### 快环时序

为排除上电、Rs/Ls 和 Flux 阶段的历史最大值，先在 DISABLE 状态下通过调试器清零时序计数，再单独执行一次正转 Shadow。停机后读回：

| 指标 | cycles | 160 MHz 时间 | 结果 |
|---|---:|---:|---|
| `ADC_Run_Max` | 3202 | 20.0125 us | 低于 25 us 半周期 |
| `ADC_ISR_Max` | 3284 | 20.5250 us | 低于 25 us 半周期 |
| `Fast_Max` | 7741 | 48.3813 us | 低于 50 us 周期，余量 1.6187 us |
| `Enc_Late` | 1 | - | 存在一次迟到记录 |
| `Enc_Miss` | 0 | - | PASS |
| `Deadline_Miss` | 0 | - | PASS |
