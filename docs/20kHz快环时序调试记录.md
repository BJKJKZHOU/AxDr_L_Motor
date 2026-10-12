# 20 kHz 快环时序调试与修改记录

更新时间：2026-10-12<br>
工程：AxDr_L_Motor<br>
历史分支：`feat/motor-para`<br>
最新复测分支：`perf/fast-loop-timing`（基于 `6cc5380` 的工作区修改）

## 1. 记录范围

本文记录 20 kHz 控制路径的时序、实时性问题和修改验证。第 2～7 节是
不开启功率 MOS 阶段建立的硬件时序基线；第 8 节开始记录带载运行中的
ISR 分段耗时和 Deadline 问题。

- TIM1、MT6816 SPI DMA、ADC injected 和电流快环的实际时序；
- DWT 计时及冻结快照的含义；
- SPI DMA 路径的修改过程；
- ADC 从 JEOC 修正为 JEOS 的原因与状态；
- 已经实测、已修改未复测和后续计划之间的边界。

第 2～7 节对应的功率状态：TIM1 CH1/2/3 及其互补输出尚未启用，
SVPWM 只执行计算，不写入三相 PWM CCR。该限制不适用于后续带载章节。

2026-10-12 的数学函数审查、Angle_Wrap 优化及位置保持复测见
[第 10 节](#math-timing-20261012)，Vector2_Limit 缩放分支独立实测见
[第 11 节](#vector-limit-20261012)。双限幅整环实验已取消，边界见
[第 12 节](#dual-limit-cancelled-20261012)。以下历史 160 MHz 数据不与本次 170 MHz 数据混用。

## 2. 早期硬件时序基准（160 MHz）

### 2.1 TIM1 与 ADC

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

编码器和ADC时刻由 [AxDr_L_Motor.ioc](../AxDr_L_Motor.ioc) 中的
`TIM1.PulseNoDither_4=2240`、`TIM1.PulseNoDither_5=3665`唯一管理，
运行时不再保存或覆盖第二份比较值。

### 2.2 MT6816 SPI

- SPI1：Mode 3、16 bit、10 MHz；
- 每个 PWM 周期读取寄存器 `0x03`、`0x04` 两帧；
- DMA1 Channel5：SPI1 RX，优先级 1；
- ADC1_2 IRQ：优先级 2；
- TIM1 update和CC IRQ：优先级 0；
- CSN：PD2，初始 High、内部 Pull-up。

## 3. DWT 计时字段

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
| `Enc_Late` | ADC IRQ 入口时 `Enc_Cyc` 尚未有效的次数，不等同于编码器传输错误 |
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

## 4. 实测数据

### 4.1 旧路径：HAL SPI Tx/Rx DMA，两次独立传输

该阶段每帧分别启动 HAL Tx/Rx DMA，包含 TX DMA、RX DMA和两次 HAL完成回调。冻结快照：

| 字段 | 周期 | 时间 |
|---|---:|---:|
| `Snap_Cnt` | 4000 | 200 ms采样点 |
| `TIM_ISR_Cyc` | 1024 | 6.400 us |
| `SPI_TX_1_Cyc` | 1070 | 6.688 us |
| `SPI_1_Cyc` | 1248 | 7.800 us |
| `Enc_CB_1_Cyc` | 2072 | 12.950 us |
| `SPI_TX_2_Cyc` | 2869 | 17.931 us |
| `Enc_CB_2_Cyc` | 3877 | 24.231 us |
| `Enc_Cyc` | 4408 | 27.550 us |
| `ADC_Cyc` | 3049 | 19.056 us |
| `ADC_Run_Cyc` | 1224 | 7.650 us |
| `ADC_Run_Max` | 1243 | 7.769 us |
| `ADC_ISR_Cyc` | 3053 | 19.081 us |
| `ADC_ISR_Max` | 3074 | 19.213 us |
| `Fast_Cyc` | 6120 | 38.250 us |
| `Fast_Max` | 6141 | 38.381 us |

状态计数：

```text
SPI_RX_Cnt    = 4
SPI_TX_Cnt    = 2
Enc_CB_Cnt    = 2
Enc_Late      = 2000
Enc_Miss      = 0
Deadline_Miss = 2000
```

结论：

- 编码器角度到27.55 us才发布；
- 快环最大38.38 us，全部稳态周期越过25 us半周期边界；
- 无传输错误，主要问题是HAL DMA状态机、两组TX/RX DMA及回调造成的长路径。

### 4.2 新路径：RX循环DMA和直接HT/TC处理

修改后只保留 SPI1 RX DMA：

```text
TIM1下溢
→ CPU直接写第一帧命令
→ RX DMA HT
→ CSN翻转并直接写第二帧命令
→ RX DMA TC
→ 解析并发布 Theta_e
```

冻结快照：

| 字段 | 周期 | 时间 |
|---|---:|---:|
| `Snap_Cnt` | 4000 | 200 ms采样点 |
| `TIM_ISR_Cyc` | 239 | 1.494 us |
| `SPI_1_Cyc` | 451 | 2.819 us |
| `SPI_1_ISR_Cyc` | 164 | 1.025 us |
| `SPI_2_Cyc` | 923 | 5.769 us |
| `SPI_2_ISR_Cyc` | 288 | 1.800 us |
| `Enc_Cyc` | 1194 | 7.463 us |
| `ADC_Cyc` | 367 | 2.294 us |
| `ADC_Run_Cyc` | 1655 | 10.344 us |
| `ADC_Run_Max` | 1655 | 10.344 us |
| `ADC_ISR_Cyc` | 2417 | 15.106 us |
| `ADC_ISR_Max` | 2442 | 15.263 us |
| `Fast_Cyc` | 2796 | 17.475 us |
| `Fast_Max` | 2828 | 17.675 us |

DMA状态：

```text
SPI_RX_Cnt     = 2
SPI_1_Flag     = 0x00050000  (GIF5 + HTIF5)
SPI_1_CNDTR    = 1
SPI_2_Flag     = 0x00030000  (GIF5 + TCIF5)
SPI_2_CNDTR    = 2           (Circular自动重装)
Enc_Late       = 2000
Enc_Miss       = 0
Deadline_Miss  = 0
```

对比旧路径：

- `Theta_e`：27.55 us降至7.46 us；
- `Fast_Max`：38.38 us降至17.68 us；
- `Deadline_Miss`：2000降至0；
- DMA事件：每周期由4次RX、2次TX相关事件收敛为1次HT和1次TC。

### 4.3 对 `ADC_Cyc=2.29 us` 的重新判定

上述新路径快照采集时，ADC1仍配置为：

```c
hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
```

`HAL_ADCEx_InjectedStart_IT()` 因此使能的是 JEOC，而不是 JEOS。ADC1有3个 injected 通道，2.29 us与第一通道完成时间吻合：

```text
单通道时间约为 (47.5 + 12.5) / 40 MHz = 1.5 us
ADC触发时间约0.6 us
第一通道完成约2.1 us
实测ADC IRQ入口2.29 us
```

因此该快照的SPI优化结论有效，但它不是最终正确的ADC序列时序；回调开始读取JDR时，第2、第3通道可能仍是上一周期结果。该数据不能作为最终12 us验收基线。

### 4.4 JEOS与DMA中断旁路HAL后的实测

ADC1改为 `ADC_EOC_SEQ_CONV`，且DMA1 Channel5直接处理后返回。冻结快照：

| 字段 | 周期 | 时间 |
|---|---:|---:|
| `Snap_Cnt` | 4000 | 200 ms采样点 |
| `TIM_ISR_Cyc` | 239 | 1.494 us |
| `TIM_ISR_Max` | 239 | 1.494 us |
| `SPI_1_Cyc` | 452 | 2.825 us |
| `SPI_1_ISR_Cyc` | 164 | 1.025 us |
| `SPI_2_Cyc` | 915 | 5.719 us |
| `SPI_2_ISR_Cyc` | 288 | 1.800 us |
| `Enc_Cyc` | 1186 | 7.413 us |
| `ADC_Cyc` | 845 | 5.281 us |
| `ADC_Run_Cyc` | 1194 | 7.463 us |
| `ADC_Run_Max` | 1194 | 7.463 us |
| `ADC_ISR_Cyc` | 2000 | 12.500 us |
| `ADC_ISR_Max` | 2009 | 12.556 us |
| `Fast_Cyc` | 2857 | 17.856 us |
| `Fast_Max` | 2866 | 17.913 us |

DMA状态：

```text
SPI_RX_Cnt     = 2
SPI_1_Flag     = 0x00050000  (GIF5 + HTIF5)
SPI_1_CNDTR    = 1
SPI_2_Flag     = 0x00030000  (GIF5 + TCIF5)
SPI_2_CNDTR    = 2
Enc_Late       = 2000
Enc_Miss       = 0
Deadline_Miss  = 0
```

该快照确认：

- `ADC_Cyc` 由2.29 us变为5.28 us，与ADC1三个通道完整序列时间一致，JEOS修改生效；
- RX循环DMA的HT、TC、CNDTR和自动重装状态保持正确；
- DMA中断旁路HAL后未出现编码器传输错误；
- 快环仍满足25 us半周期边界，但 `Fast_Max=17.913 us`，距离12 us目标还差5.913 us。

### 4.5 ADC入口与Park角度的顺序

JEOS快照中的关键时序：

```text
T0 + 845周期    ADC JEOS IRQ进入，Enc_Cyc尚未有效，Enc_Late累加
T0 + 915周期    SPI RX DMA TC以优先级1抢占ADC优先级2
T0 + 1186周期   DMA ISR内发布Motor_Run.Theta_e
T0 + 1203周期   DMA ISR结束，ADC HAL恢复运行
之后            ADC回调 → ADC_Sample → Current_Ref_Get
                 → Current_Loop读取Theta_e → SinCos
```

ADC入口到DMA TC只有70周期，即0.438 us。Release反汇编确认，此时ADC路径尚未经过 `HAL_ADC_IRQHandler()` 到达回调；DMA先抢占并更新角度。`Current_Loop()` 开头的 `SinCos(Motor_Run.Theta_e, ...)` 因此使用本周期的新角度。

`Enc_Late=2000` 只说明ADC IRQ入口时角度尚未准备好，不表示Park使用旧角度。当前正确顺序依赖DMA高优先级抢占和HAL分发耗时；后续旁路ADC HAL前仍需先建立明确的编码器提前时序。

## 5. 修改记录

### 5.1 已提交的基础修改

| 提交 | 内容 |
|---|---|
| `648b4e0 chore(cubemx): align adc encoder timing` | SPI1由5 MHz改为10 MHz；增加SPI1 RX/TX DMA；设置中断优先级 TIM1=0、DMA=1、ADC=2；TIM1 CH4设为96 |
| `16c4c44 feat(encoder): add pwm-synchronous dma sampling` | TIM1下溢启动MT6816采样，建立PWM同步编码器读取 |
| `9b0c2fd fix(encoder): publish motor angle feedback` | 发布 `Theta_m`、`Theta_e`反馈并修正编码器相关状态 |
| `df57838 feat(ioc): 修正配SPI` | `.ioc` 中PD2 CSN改为初始High和Pull-up |
| `7acac44 feat(motor): connect current reference fast loop` | ADC injected回调接入 `ADC_Sample → Current_Ref_Get → Current_Loop → SVPWM_Calc`；不启用三相PWM输出 |

### 5.2 当前工作区修改

#### DWT计时与冻结快照

- 初始化并启动 `DWT->CYCCNT`；
- 增加 `Fast_Time`、`Fast_Snap`、`Fast_Snap_Ready`；
- 分别记录TIM ISR、SPI HT/TC、角度发布、ADC回调和快环总时间；
- 增加2000周期预热、2000周期稳态统计和一次性冻结；
- 增加 `Enc_Miss`、`Deadline_Miss`及DMA标志/CNDTR诊断。

#### SPI DMA路径优化

- SPI1 RX DMA由 Normal 改为 Circular，缓冲区长度固定为2；
- 删除SPI1 TX DMA和DMA1 Channel6 IRQ；
- TIM1 CH4上升计数比较中断直接写第一帧SPI命令；
- RX DMA HT中直接启动第二帧；
- RX DMA TC中解析两字节角度并发布 `Motor_Run.Theta_e`；
- 正常运行不再每周期重配DMA，只在启动和异常恢复时配置/重装；
- CSN通过GPIO BSRR直接控制；
- 增加200 ns CSN高电平间隔和SPI结束超时保护。

#### DMA中断去除重复HAL处理

DMA1 Channel5用户代码区执行：

```c
Encoder_DMA_IRQHandler();
return;
```

CubeMX生成的 `HAL_DMA_IRQHandler(&hdma_spi1_rx)` 仍保留在源码中，但实际执行路径不可达。Release反汇编确认中断入口直接尾跳转到 `Encoder_DMA_IRQHandler()`，不再进入HAL DMA状态机。

状态：Debug、Release编译通过；已通过4.4节冻结快照验证。

#### ADC1由JEOC修正为JEOS

[AxDr_L_Motor.ioc](../AxDr_L_Motor.ioc) 和 [adc.c](../Core/Src/adc.c) 已同步修改：

```c
hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
```

启动方式保持：

```c
HAL_ADCEx_InjectedStart(&hadc2);
HAL_ADCEx_InjectedStart_IT(&hadc1);
```

ADC1现在只在3通道完整序列结束后产生JEOS中断。ADC2只有1个 injected 通道，不产生中断；ADC1 JEOS到来时ADC2已完成。

状态：Debug、Release编译通过；已上板确认 `ADC_Cyc=5.281 us`。

## 6. 最近一次上板实测结论

1. RX循环DMA和直接HT/TC处理已经解决原HAL SPI DMA长路径问题，编码器角度发布由27.55 us降至7.41 us。
2. ADC1 JEOS已经实测生效，ADC IRQ入口由首通道JEOC的2.29 us变为完整序列结束的5.28 us。
3. `Enc_Late=2000` 表示ADC IRQ入口早于角度发布：`5.28 us < 7.41 us`，不表示SPI错误或Park使用旧角度。
4. 该次实测仍使用HAL ADC路径，DMA TC先抢占ADC并发布角度，随后回调进入 `Current_Loop()`，因此 `SinCos()` 使用本周期新角度。
5. 当前最长已测端到端时间为 `Fast_Max=17.913 us`，满足25 us边界，但距离12 us目标还差5.913 us。
6. 以上数据是固定低边采样时序修改前的基线，不能直接代表当前工作区的新时序。

## 7. 固定低边采样时序修改

当前JEOS基线已完成以下验证：

1. `ADC_Cyc=5.281 us`，符合3通道完整序列；
2. `SPI_RX_Cnt=2`；
3. HT时 `CNDTR=1`、TC时 `CNDTR=2`；
4. `Enc_Miss=0`；
5. `Deadline_Miss=0`；
6. `ADC_Run_Max=7.463 us`；
7. `ADC_ISR_Max=12.556 us`；
8. `Fast_Max=17.913 us`；
9. ADC IRQ入口角度未完成，但当前Park前已经完成。

已完成但尚未上板复测：

- CH4比较中断在约14 us启动MT6816，替代下溢立即启动；
- CH5的OC5REF通过TRGO2在22.906 us触发ADC1、ADC2；
- ADC1_2正常JEOS路径直接清标志并调用 `ADC_Run()`，不再经过两次通用HAL ADC处理；
- MT6816两帧均增加至少100 ns的CSN低电平建立时间；
- `Deadline_Miss`改为检测是否越过下一次TIM1下溢，即50 us PWM更新边界；
- Debug和Release编译通过；Release反汇编确认JEOS和CC4正常路径均为直接调用。

新时序的预期值：

```text
0 us          TIM1下溢，建立DWT基准
14.0 us       CH4比较中断启动MT6816
约21.6 us     Theta_e发布
22.906 us     CH5/TRGO2触发ADC
27.406 us     ADC1三通道JEOS
约35 us       Current_Loop、SVPWM完成
50 us         CCR preload在新PWM周期生效
```

新的性能验收口径：

```text
ADC硬件触发 → ADC完整序列 → Current_Loop → SVPWM完成 ≤ 12 us
```

下一次上板需要确认：

- `Enc_Late=0`、`Enc_Miss=0`、`Deadline_Miss=0`；
- `ADC_Cyc`约为27.4 us、`Enc_Cyc`早于`ADC_Cyc`；
- `ADC_ISR_Max`以及ADC触发至快环结束的总时间；
- 示波器确认低边开通后的放大器稳定时间 `T_blank`；
- 根据实测 `T_blank`确定最终调制度上限，当前不修改`VOLT_MOD_MAX=0.95`。

## 8. FAST Plot 的 USB 唤醒导致 ADC ISR 长尾

### 8.1 问题分类和版本

本问题属于“实时性 / ISR 职责边界”，不是 Observer 数学模型问题，也不是
USB 协议是否能够传输数据的问题。相关提交为：

```text
605b692 test(timing): add windowed fast-loop profiling
468513d fix(usb): defer ISR tx wake to motor thread
ae4706b fix(usb): include cmsis intrinsics
```

`605b692` 增加 2048 周期窗口计时。Sensorless I/F 达到 Ready 后自动请求一次
Profile，分别累计以下分段的 Min/Sum/Max：

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

### 8.2 优化前的问题证据

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

真正的异常是 `Plot_Fast` 每完成一个 FAST 数据块时出现约 `8 us` 的周期性长尾。
当时 `Plot_Fast_Sample()` 在 ADC ISR 中调用 `USB_Tx_Wake()`，后者直接执行
`tx_event_flags_set()`。ThreadX event 唤醒进入了 20 kHz ADC ISR 的硬实时路径。

### 8.3 优化方法

`468513d` 将发送请求按调用上下文分流：

- 线程上下文中的 `USB_Tx_Wake()` 仍直接设置 USB TX event；
- 中断上下文只在短临界区内将发送位 OR 到 `USB_Tx_Pending`；
- `2 kHz` Motor Thread 在 `Motor_Control()` 后调用 `USB_Tx_Poll()`；
- Poll 原子取走 pending 位，再在线程上下文调用 `tx_event_flags_set()`；
- Plot 双缓冲、USB TX 线程、协议和丢帧计数保持不变。

`ae4706b` 只补充 CMSIS intrinsic 声明，不改变上述执行逻辑。

该修改的控制含义是：ADC ISR 只发布“有数据待发送”的事实，RTOS 对象操作由线程完成。

### 8.4 `ae4706b` 实机复测条件

```text
HEAD          = ae4706bd51fd7130ca06129e303015ef4f7b90f5
CPU / PWM     = 160 MHz / 20 kHz
Vbus          ~= 14.54 V
Rs            = 0.08471736 ohm
Ld = Lq       = 17.836 uH
Flux          = 0.0031835556 Wb
Voff          = 0.0141716 V
```

Release 全量构建并烧录校验：

```text
RAM:   24648 B / 128 KB, 18.80%
FLASH: 66676 B / 512 KB, 12.72%
```

参数由自动脚本重新辨识、Apply；每轮时长测试前在 DISABLE 状态下通过调试器独立清零计数，测试结束由脚本执行 `DISABLE`。

### 8.5 7 FAST + 1 NORMAL 分段结果

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

板内生产端计数为 FAST Drop 0、NORMAL Drop 9，结束时 `USB_Tx_Pending=0`。
NORMAL 的 9 个丢弃发生在序号生成前，所以主机 `seq_lost=0` 不能检测这类丢弃。
当前没有优化前的独立 Drop 基线，因此不能将这 9 个样本判为本次修改引入的回归。

### 8.6 NORMAL-only 对照

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

### 8.7 优化前后结论

| 指标 | 优化前 7 FAST | 优化后 7 FAST | 变化 |
|---|---:|---:|---:|
| `Plot_Fast Mean` | `3.3608 us` | `2.9636 us` | `-0.3972 us` |
| `Plot_Fast Max` | `11.5375 us` | `3.3375 us` | `-8.2000 us` |
| `ADC_Run window Max` | `27.1875 us` | `18.8438 us` | `-8.3437 us` |
| `ADC_ISR_Max` | `28.4125 us` | `20.2500 us` | `-8.1625 us` |
| `Fast_Max` | `56.2688 us` | `48.3875 us` | `-7.8813 us` |
| `Deadline_Miss` | `8015` | `0` | 问题消失 |

实测差值与原 ThreadX event 唤醒长尾一致，支持根因和修改方向。

`Fast_Max` 从 TIM1 更新中断基准计到 ADC ISR 结束，包含编码器获取、ADC 触发位置、
硬件转换等待和中断执行，不等于纯 CPU 利用率。当前应该表述为“端到端 Deadline
余量 `1.6125 us`”，不能直接表述为“CPU 占用 96.8%”。

当前状态：

```text
USB ISR 唤醒长尾：       FIXED / VERIFIED
20 kHz Deadline：         PASS
NLOB + PLL 固定耗时：     2.5188 us
NORMAL-only 基线：        无回归
7 FAST 端到端余量：       偏低
FAST 生产端丢弃：         0
并发 NORMAL 生产端丢弃：  9，待后续按需求评估
```

该优化已经解决当前 Deadline Miss，但在加入 Observer Takeover、无感速度环或
Fine Flux 逻辑前，仍应保留耗时预算审查。诊断阶段如果不需要 7 路 FAST，应减少
FAST 通道以恢复更大的调试余量。


## 9. ARR→CCR 12.5 μs 专项计时口径与工具

`perf/fast-loop-timing` 在 170 MHz / 20 kHz 下沿用 `Fast_Time`、`Fast_Snap`，
在当前 ADC JEOS → `Fast_Loop()` 中增加以下统计，未调整电流环、机械 ESO、
采样触发或 PWM 更新时序。

| 字段 | 计时范围 | 单位 |
|---|---|---|
| `ESO_Cyc / ESO_Max` | `Mechanical_ESO_Run()` 调用前后 | DWT CPU cycles |
| `PI_Cyc / PI_Max` | `Current_Loop()` 两轴 PID 调用前后 | DWT CPU cycles |
| `SVPWM_Cyc / SVPWM_Max` | `SVPWM_Calc()` 调用前后 | DWT CPU cycles |
| `ADC_CCR_Cyc / ADC_CCR_Max` | ADC IRQ 入口到 CCR3 写入后紧接的 DWT 读取 | DWT CPU cycles |
| `ARR_CCR_Tick / ARR_CCR_Tick_Max` | ARR 顶点到 CCR3 写入后紧接的 TIM1 CNT 读取 | TIM1 timer ticks |
| `ARR_CCR_Count` | MOE 有效、到达 `PWM_Update()` 的计时次数 | 次 |
| `ARR_CCR_Late` | 更新时不在 ARR 后下降计数半周期，或已跨越新的 PWM 下溢基准 | 次 |

`ARR_CCR_Tick` 在 CCR3 写入后立刻读取 TIM1 CNT/DIR，以 `ARR-CNT`
估算 ARR 顶点至完成写入的时间；包含 CCR3 写入后读取计数器的少量开销，
采样点位于 `PWM_Update()` 内，不等待函数返回。
这不是从 TIM1 下溢 ISR 的 DWT 时间戳外推。若此时计数器不在对应的下降半周期，
将 `ARR_CCR_Tick` 设为 `UINT32_MAX` 并累加 `ARR_CCR_Late`，不把环绕后的
短时间误记为通过。没有 MOE 的周期不会计入 ARR→CCR 样本。

当前 TIM1 时钟与 CPU 都为 170 MHz，PSC=0，因此 TIM1 的一个计数也是 1/170 μs，
12.5 μs 对应 2125 计数。以后改变时钟树或预分频时，必须先核对 TIM1 实际时钟，
不能把 TIM1 ticks 永远当成 CPU cycles。

统计继续使用前 2000 次 ADC 中断预热、清空最大值，至 4000 次冻结一次快照的机制。
`Fast_Time` 的最大值在预热后持续累计；`Fast_Snap` 仅反映一次早期窗口。
每次通过 SWD 开始新一轮测试时，应重启快照窗口并保留固件真实运行状态。

### 本地测试

先编译、烧录与板上程序一致的 Release ELF，然后单独启动 OpenOCD：

```bash
cmake --preset Release
cmake --build --preset Release
openocd -f openocd.cfg
```

在另一个终端运行已有的无停机 SWD 检查脚本，先在安全电压下
通过现有控制接口进入有感 ENABLED/RUN 状态：

```bash
python3 tools/lcd_foc_check.py --elf build/Release/AxDr_L_Motor.elf --mode normal --seconds 10 --arr-ccr-limit-us 12.5
```

该参数仅在确实有 PWM-active CCR 样本、没有 `ARR_CCR_Late` 且最大值不超过
12.5 μs 时通过；不传 `--arr-ccr-limit-us` 则只报告指标，不以 12.5 μs 作为
退出条件。JSON 中的 `arr_ccr_max_us`、`adc_ccr_max_us`、`eso_max_us`、
`pi_max_us`、`svpwm_max_us` 对应关键诊断数据。

本节最初仅完成代码计时；2026-10-12 的实测结果见第 10 节。
测量本身增加少量访问寄存器、
DWT 与更新 Max 的开销，属于偏保守的带仪表测量；不同分段的最大值可能来自不同
采样周期，不可以直接相加充当同一周期的总耗时。之后以真实板上 Release 数据
判断是 ESO、PID、SVPWM 还是采样与 ISR 延迟为主要耗时来源。


### 9.1 多周期统计与偶发尖峰定位

在 `6eff5a5` 提交后，实际时刻采样移动至 `PWM_Update()` 写入 CCR3
之后的第一处代码，不再等待返回到 `Fast_Loop()`；因此新旧测量基线有
少量仪表开销差异，不能将差异全部当作电流算法变快。

新增统计均只在实际写入 CCR 的 PWM-active 周期累积：

| 字段 | 含义 |
|---|---|
| `ARR_CCR_Min` | ARR→CCR 有效周期最小计数 |
| `ARR_CCR_Over` | ARR→CCR 超过 2125 ticks（12.5 μs）的次数 |
| `ARR_CCR_Sum_Lo / Sum_Hi` | TIM1 tick 累计和，用于平均时间 |
| `ARR_CCR_Worst_Seq` | 最大值出现时的有效更新序号 |
| `ARR_CCR_Worst_ADC / ESO / PI / SVPWM` | **与最大值同一拍**的分段 DWT 周期数 |
| `ARR_CCR_Win_Total` | 每 1000 个 PWM-active 周期完成一个统计窗口 |
| `ARR_CCR_Win_Bad` | 上述窗口中出现过超限的窗口个数 |
| `ARR_CCR_Win_Last` | 最近已完成窗口的最大时间（TIM1 ticks） |
| `Fast_CCR_Hist[64]` | 所有有效样本的时延直方图：43 ticks/档（约 0.253 μs），最后一档包含所有更长时间 |

`tools/lcd_foc_check.py` 已支持输出 `arr_ccr_min_us`、
`arr_ccr_mean_us`、`arr_ccr_over_12_5us` 和百分比、
`arr_ccr_p50_upper_us` / `p95` / `p99` / `p999`、
`arr_ccr_windows_50ms_completed`、`arr_ccr_windows_50ms_over_limit`，
以及 `arr_ccr_worst_sample_stages_us`。分位数为直方图桶的上边界估计，
不是微秒级精确分位值；最高桶没有有限上界，返回 null。

运行命令保持不变：

```bash
python3 tools/lcd_foc_check.py --mode normal --seconds 10 \
    --arr-ccr-limit-us 12.5 --output /tmp/fast-timing.json
```

比较 `over_12_5us` 与 `windows_50ms_over_limit`：
若仅少数样本、少数窗口越界，更像偶发中断抢占、窗口切换或罕见计算路径；
若普遍越界则应优化常规热路径。即便分位数达标，严格的 12.5 μs
最大值要求仍保持不变。第 2000 次 ADC 中断会清空这些统计与直方图，
所以启动前的初始化不进入正式分布；`Fast_Time` 在预热后持续累积，
`Fast_Snap` 仍只冻结早期窗口的结构体字段，直方图未冻结。

直方图和窗口统计在写入 CCR3 **之后**执行，不计入 ARR→CCR 时间，
但仍会占用 ADC ISR 的剩余执行时间，应同时观察 `ADC_ISR_Max`、
`Fast_Max` 和 `Deadline_Miss`。频繁中断抢占仍可能发生在 CCR3
写入与紧接着读取 CNT 之间，若出现无法解释的个别尖峰，需要进一步
结合 GPIO/逻辑分析仪或 DWT 捕获确认，并核对中断优先级。

<a id="math-timing-20261012"></a>

## 10. 数学函数审查与 Angle_Wrap 优化实测（2026-10-12）

### 10.1 条件与结论范围

本轮使用 STM32G474、CPU/TIM1 170 MHz、TIM1 PSC=0、20 kHz 快环，
ST Arm Clang 21.1.1+st.7，Release `-Oz`。分支为 `perf/fast-loop-timing`，
基于 `6cc5380`，结果包含本地未提交修改，不能仅用该提交号重建测试固件。

电机进入有感 POSITION + RUN，目标设为当前机械位置，母线约 24.7～24.8 V，
临时参考电流上限 2 A、位置运动速度参数 5 rad/s；实际工况是位置保持，
不是以 5 rad/s 持续转动。LCD 为 `normal`，每轮主采集约 10 s。
这些是此前已完成的优化对照，区别于第 12 节后来提出的 0.3 A 实验。

**已完成的 Release 对照中，ARR→CCR 最大值由 13.223529 μs 降至
11.605882 μs，超过 12.5 μs 的次数由 152 降至 0。**
结论仅适用于该次位置保持工况；没有覆盖两次矢量缩放同时发生的整环最坏路径，
不能据此宣称全部运行工况均满足 12.5 μs。

本轮区分两类固件：

- `MATH_TIME=ON`：逐项 DWT 探针，用于比较调用耗时及分支覆盖；会增加 ISR 开销。
- `MATH_TIME=OFF`：关闭逐函数探针，保留已有 `Fast_Time` 和 ARR→CCR 统计，
  用于下面的 12.5 μs 端到端对照。它仍包含基础计时开销，并非完全无仪表固件。

### 10.2 函数审查与实际覆盖

| 函数/调用点 | 审查结论 | 本轮处理及覆盖 |
|---|---|---|
| `Angle_Wrap()` 电角度入口 | 旧实现的通用路径调用 `__builtin_fmodf()`，本工况每周期均进入该输入范围 | P0：替换为 binary32 整数余数算法，完成对照 |
| `Vector2_Limit()` | 超限时才执行 `sqrtf` 和除法 | P1：算法未改；位置保持探针两处均未缩放，另做第 11 节独立缩放测试 |
| `SinCos()` | 1024 点 Flash LUT，无 `sinf/cosf` 调用 | P2：保留实现，仅记录耗时 |
| `SVPWM_Calc()` | 正常路径包含 `1/Vbus` 浮点除法 | P2：保留实现，仅记录耗时 |
| `PID_Run()` | 仅 `Kd != 0` 时执行微分除法 | 已有条件跳过；两轴电流 PI 的 Kd=0，本轮未改算法 |
| `Mechanical_ESO_Run()` | 乘加、有限值检查、内部 `Angle_Wrap()` | P1：内部角度调用已处于快速输入范围；ESO 算法未改，受益于共用角度函数替换 |

`MATH_TIME` 使用带编译器屏障的 DWT 读取，避免普通计算跨越时间戳；
每周期统计在 CCR 写入后累加，但时间戳、现场读写及嵌套探针仍会影响前段时序。
下表为原始含调用/探针开销的时间，没有减去空探针，也不能将各项最大值相加。
ESO 总时间已经包含其内部 Angle_Wrap 探针。

| 调用点 | 旧实现均值 μs | 旧实现最大 μs | 新实现均值 μs | 新实现最大 μs |
|---|---:|---:|---:|---:|
| 电角度 `Angle_Wrap` | 1.829087 | 2.205882 | 0.617647 | 0.617647 |
| ESO 内部 `Angle_Wrap` | 0.300113 | 0.311765 | 0.276529 | 0.282353 |
| `SinCos` | 0.246303 | 0.276471 | 0.245381 | 0.276471 |
| 电流参考 `Vector2_Limit`，未缩放 | 0.188235 | 0.188235 | 0.188235 | 0.194118 |
| 电压 `Vector2_Limit`，未缩放 | 0.176471 | 0.176471 | 0.176471 | 0.176471 |
| d 轴 `PID_Run` | 0.478454 | 0.482353 | 0.478137 | 0.482353 |
| q 轴 `PID_Run` | 0.449010 | 0.452941 | 0.448611 | 0.452941 |
| `Mechanical_ESO_Run`，含内部角度探针 | 1.214547 | 1.229412 | 1.190381 | 1.205882 |
| `SVPWM_Calc` | 1.170753 | 1.176471 | 1.170763 | 1.176471 |
| 空 DWT 探针 | 0.005882 | 0.005882 | 0.005882 | 0.005882 |

旧/新探针各采集 **208835 / 208855** 个周期，时间约 **10.441933 / 10.442901 s**，
包含主脚本约 10 s 窗口之外的预热和读出时间。新电角度调用最小/平均/最大均为
**105 cycles**；旧调用最小 238、最大 375 cycles。平均调用耗时减少约 **66.23%**。

电角度调用的 `Alt_Count` 旧/新分别为 208835 / 208855，它表示输入满足旧实现的
通用取模条件（`Theta >= 4π` 或 `Theta < -2π`），**不表示新实现仍调用 fmodf**。
ESO 内部角度的该计数为 0；两处矢量限幅的缩放计数均为 0；两轴 PID 的
`Kd != 0` 计数均为 0。未执行的分支不从这组数据推算耗时。

开逐函数探针时，新实现 ARR→CCR 最大值为 13.476471 μs、超限 570 次；
这与下节关闭逐函数探针的结果属于不同计时条件，不能混用。

### 10.3 Angle_Wrap 实现与数值验证

[Algo/Math.c](../Algo/Math.c) 中的 Angle_Wrap 已改为整数余数实现：有限值先走常见角度的
直接返回/加周期路径，其余路径提取 float 位模式，以整数移位和条件减法求余数。
周期仍是原来舍入后的 float32 `TWO_PI_F`，没有更换控制角度定义。

文件内已增加中文说明及编译期检查：

- 依赖 IEEE 754 binary32：1 位符号、8 位指数、指数偏置 127；23 位显式尾数、
  含隐含首位共 24 位有效精度。
- 固定周期位模式为 `0x40C90FDB`，数值为 `0x1.921fb6p+2f`；指数域 129，
  含隐含首位的整数尾数 `0xC90FDB`，余数还原因子 `0x1p-21f`。
- `_Static_assert` 检查 float 的总位数、进制、精度、指数范围和固定周期数值；
  实际位布局仍以前述目标 ABI 的 binary32 假设为基础。
- 保留非有限值处理和旧实现的正/负零行为，包括 `-2π` 与更小负整周期的区别。

主机逐位对照旧实现，共 **3312909** 个输入、**0** 个输出位模式不一致：
包含正负零、非正规数和最大有限值、NaN/Inf、`-1024..1024` 倍周期周边位模式、
所有指数域的尾数采样，以及 200 万个伪随机位模式。
这是数值回归证据，不是对全部 2³² 输入的穷举，也不是 MCU 时序测试。

整数移位循环次数依赖输入指数；本工况的 105 cycles 不代表所有有限 float 输入
都具有固定耗时。除了 Angle_Wrap，本轮没有据此替换 sqrt、LUT、除法或 PI 算法。

### 10.4 关闭逐函数探针后的整环对照

两轮均为 `MATH_TIME=OFF`，未启用双限幅实验代码；测量期间均保持 POSITION + RUN，
LCD normal，ADC/编码器/PWM 的原有时序设置不变。

| 指标 | 旧 Angle_Wrap | 新 Angle_Wrap |
|---|---:|---:|
| 主窗口时长 s | 10.000739 | 10.000708 |
| 读取 `ARR_CCR_Count` 时的样本数 | 201046 | 201046 |
| ARR→CCR 最小 μs | 11.288235 | 10.388235 |
| ARR→CCR 平均 μs | 11.751649 | 10.537932 |
| ARR→CCR 最大 μs | **13.223529** | **11.605882** |
| 超过 12.5 μs 次数 | **152** | **0** |
| 超限比例 | 0.075605% | 0% |
| 完成的 50 ms 窗口数 | 201 | 201 |
| 出现超限的窗口数 | 104 | 0 |
| `Fast_Max` μs | 39.323529 | 37.688235 |
| `ADC_ISR_Max` μs | 11.376471 | 9.758824 |
| `ARR_CCR_Late` | 0 | 0 |
| `Deadline_Miss / Enc_Miss / Enc_Late` | 0 / 0 / 0 | 0 / 0 / 0 |

新实现该次最大值距 12.5 μs 尚有 **0.894118 μs（152 个 TIM1 ticks）**。
`Fast_Max` 从 PWM 下溢基准统计端到端链路，包含采样等待；不能把它当成纯 CPU
计算时间，也不能与 ARR→CCR 的 12.5 μs 门槛直接比较。

与 ARR→CCR 最大值**同一周期**的分段值如下；ADC IRQ→CCR 已包含这些算法，
不能重复求和：

| 同周期观测值 μs | 旧实现 | 新实现 |
|---|---:|---:|
| ADC IRQ→CCR | 10.200000 | 8.582353 |
| Mechanical ESO | 0.900000 | 0.870588 |
| 两轴 Current_Loop | 1.323529 | 1.323529 |
| SVPWM | 1.170588 | 1.176471 |

复用脚本时必须指定与烧录固件匹配的 ELF。以下命令仅说明已有测量口径，
脚本本身不使能电机；本文归档阶段没有再次执行上板测试：

```bash
cmake --preset Release -DMATH_TIME=OFF -DDUAL_LIMIT_TEST=OFF
cmake --build --preset Release
python3 tools/lcd_foc_check.py --elf build/Release/AxDr_L_Motor.elf \
    --mode normal --seconds 10 --arr-ccr-limit-us 12.5 --output /tmp/fast-timing.json
```

### 10.5 完整 ARR→CCR 直方图

保留原始 64 桶计数；下表将连续全零桶合并展示，未省略非零桶。
第 k 桶的 TIM1 tick 范围为 `[43k, 43(k+1))`，除以 170 得到 μs；
第 63 桶为 `[2709, +∞)`，不具有有限上界。

| 桶 k | 旧实现计数 | 新实现计数 |
|---|---:|---:|
| 0～40（各桶） | 0 | 0 |
| 41 | 0 | 199763 |
| 42 | 0 | 1199 |
| 43 | 0 | 94 |
| 44 | 88780 | 24 |
| 45 | 1849 | 12 |
| 46 | 21775 | 0 |
| 47 | 21184 | 0 |
| 48 | 67207 | 0 |
| 49 | 221 | 0 |
| 50 | 40 | 0 |
| 51 | 34 | 0 |
| 52 | 4 | 0 |
| 53～63（各桶） | 0 | 0 |
| 直方图计数合计 | **201094** | **201092** |

读取 `Fast_Time` 后才通过 SWD 读取仍在累计的直方图，因此直方图分别比之前读取的
`ARR_CCR_Count=201046` 多 **48 / 46** 个样本。这些读出不是原子冻结快照，
本记录保留原值，不裁剪计数来制造一致性。超限次数使用 `ARR_CCR_Over`，
不能仅按直方图估算，因为 2125 ticks 门槛落在第 49 桶内部。

| 直方图分位数上边界 μs | 旧实现 | 新实现 |
|---|---:|---:|
| P50 | 11.888235 | 10.623529 |
| P95 | 12.394118 | 10.623529 |
| P99 | 12.394118 | 10.623529 |
| P99.9 | 12.647059 | 10.876471 |

这些是桶上边界估计，不是精确分位点。最大值和逐周期超限计数仍是截止时间判定依据。

<a id="vector-limit-20261012"></a>

## 11. Vector2_Limit 实际缩放分支耗时（2026-10-12）

### 11.1 独立基准的测量方式

使用当前 Release 的 Math 编译对象，临时启动入口在 ADC/控制周期启动前执行基准。
函数及测试入口位于 CCMRAM，输入为 SRAM 中的局部变量，DWT 170 MHz；
先预热 16 次，正式计时期间屏蔽中断，测试前后 `TIM1.BDTR.MOE=0`。
这轮没有驱动电机，也没有修改 Vector2_Limit 的算法。

每组 100000 次；每次重新赋输入，在函数调用前后以编译器屏障约束 DWT 读取，
并比较输出与输入确认是否实际改变。输出值另与预期限幅结果核对。
空探针为 1 cycle；下表报告未扣除探针、调用及少量参数准备指令的原始时间。

### 11.2 实测结果

| 输入 (X, Y) | Lim | 预期输出（近似） | 实际改变次数 | 最小=平均=最大 cycles | 时间 μs |
|---|---:|---|---:|---:|---:|
| (0.3, 0.4) | 1 | (0.3, 0.4) | 0 | 29 | 0.170588 |
| (3, 4) | 5 | (3, 4) | 0 | 29 | 0.170588 |
| (1.0001, 0.0001) | 1 | (1, 0.00009999) | 100000 | 69 | 0.405882 |
| (3, 4) | 2 | (1.2, 1.6) | 100000 | 69 | 0.405882 |
| (6, 8) | 1 | (0.6, 0.8) | 100000 | 69 | 0.405882 |
| (-30, 40) | 2 | (-1.2, 1.6) | 100000 | 69 | 0.405882 |
| (0, 4) | 2 | (0, 2) | 100000 | 69 | 0.405882 |
| (3, 4) | 0 | (0, 0) | 100000 | 20 | 0.117647 |

五组正限值缩放共 **500000 次**，每次输出均实际改变；`Lim=0` 的清零另计，
它没有执行 sqrt/除法缩放，不能混入这 50 万次。该基准下实际缩放比未缩放
多 **40 cycles = 0.235294 μs/次**。

若两处都由不缩放变成缩放，单纯相加是 80 cycles、约 0.470588 μs；
这只表示上述独立基准的算术差值，**不是整环实测增量**。在线调用的寄存器、
Flash/数据访问、抢占和其他控制分支可能不同，不能把此差值加到第 10 节最大值后
宣称双缩放整环已经达标。

该独立实验结束后恢复原应用镜像并校验；NVS 前后相同，最后状态 DISABLED、
MOE=0、Error/Trip=0、CPU 时钟 170 MHz。该恢复结论仅对应本节实验。

<a id="dual-limit-cancelled-20261012"></a>

## 12. 双限幅整环实验：基线中止，未完成

计划在空载位置环中验证同一周期的电流参考缩放、电压缩放、dq 前馈和 PI 抗饱和，
并保留完整直方图。实验参考限幅设为
`sqrt(Id_ref² + Iq_ref²) <= 0.3 A`，**不是母线直流电流限流**。
采用约 24.8 V 母线及软件保护，没有确认独立硬件限流；实际反馈矢量关断阈值为 0.6 A。

临时 `DUAL_LIMIT_TEST` 方案为 0.5 s 基线、0.1 s 短激励（通过后才计划 10 s）、
0.5 s 恢复。激励设计为 d 轴 ±0.36 A、1 kHz 方波，仍经过实际 0.3 A 矢量限幅；
激励期电压上限拟降至 0.15 V。**两次尝试都在基线中止，未进入该激励阶段，
没有执行后续 10 s 双限幅整环测试。**

| 观测 | 首次短测 | 补充保护现场后的短测 |
|---|---:|---:|
| 完成的基线周期 | 409（20.45 ms） | 58（2.90 ms） |
| 激励/恢复阶段周期 | 0 / 0 | 0 / 0 |
| 同周期双缩放次数 | 0 | 0 |
| 中止前完整周期的反馈矢量峰值 A | 0.565880 | 0.528309 |
| ARR→CCR 最大 μs | 15.858824 | 16.323529 |
| 超过 12.5 μs 次数 | 409 | 58 |

第二次 `Limit_Trip_Code=64`，确认触发的是实际反馈矢量检查：
`Ialpha=-0.241699 A`、`Ibeta=-0.604695 A`，矢量幅值 **0.651210 A > 0.6 A**；
同时 `Wm=0.108064 rad/s`、`Vbus=24.701658 V`、参考限值约 0.3 A。
触发周期在写 CCR 前已关断，不进入上表的完整周期峰值统计，因此不与 0.528309 A 矛盾。
首次没有保存触发原因，不能凭相似现象直接认定原因相同。

本表时间包含额外检查和现场采集开销，且只测到了基线；不能替代第 10 节的 Release
对照，也不能证明或否定双缩放时原算法的 12.5 μs 上限。没有完成 dq 前馈与抗饱和的
联合工况验证。代码审查还发现：当前 PI 回退针对各轴输出限值，未接收前馈之后的
径向电压限幅反馈；仅看到两处缩放，也不能单独证明下游饱和时积分始终受控。

本项实验在基线保护中止后终止，未继续激励或联合工况验证。
`DUAL_LIMIT_TEST` 代码仍留在工作区且默认关闭，与 `MATH_TIME` 互斥，不能视作已验收功能。

恢复状态必须分别记录：

- 首次短测结束后，原固件恢复校验成功，NVS 不变，DISABLED、MOE=0、Error/Trip=0。
- 第二次短测已关断 PWM，主机随后确认 DISABLED；恢复操作期间连接异常，
  原镜像恢复校验未成功，最终 USB 串口消失，**当时无法确认 Flash 内容或重启状态**。
  该次 `nvs_after.bin` 实际为 0 字节，故恢复日志中的 `nvs_unchanged=false` 表示比较无效，
  不能解释为已经证实 NVS 被改写。后续故障排查及恢复结果见第 12.1 节。

### 12.1 启动区擦除后无响应的排查与恢复

故障现象为按下复位后程序未启动，原调试会话无法访问 MCU。排查范围为调试连接、
Flash 启动向量及固件恢复，未执行电机运行测试。

1. USB 已能枚举 MICROLINK 下载器，但原 OpenOCD 会话仍使用断开前的 USB 连接，
   持续输出 `No such device`、`CMD_CONNECT failed`，因此该会话的 `reset` 状态不可信。
2. 退出旧会话、以 100 kHz SWD 重新连接后，读到 `DPIDR=0x2BA01477`、
   STM32 芯片 ID `0x20036469`，CPU 调试接口可以正常访问。
3. `0x08000000` 起始向量全为 `0xFFFFFFFF`；随后备份并确认前 4 KiB 全部被擦空。
   复位暂停时 `PC=0xFFFFFFFE`、`MSP=0xFFFFFFFC`，对应无有效启动向量。
   证据支持应用恢复过程中擦除已发生、写回未完成，不能将此前无响应等同于 MCU 损坏。
4. 临时恢复脚本仅打印部分 Flash 命令返回值，没有在缺失写入/校验完成信息时立即失败退出；
   OpenOCD Tcl 错误也没有完整传递到主机判断。这是此次恢复未完成时的处理缺陷。
5. 三份测试前应用备份逐字节一致，均为 507904 字节，SHA-256 为
   `039cf6951e2031f6439b956df800d8205dc963af4715c384f2e800841cab475d`。
   恢复前读取的 16 KiB NVS 与测试前备份相同。

随后仅恢复原应用区 `0x08000000..0x0807BFFF`，未写 NVS 或修改保护选项。
完整写入及校验 **507904 字节**均成功，启动向量恢复为
`MSP=0x20018000`、`Reset_Handler=0x0800C03D`。
复位启动后 USB CDC 重新枚举并可读取参数，确认如下：

| 恢复后检查 | 结果 |
|---|---|
| 电机状态 | DISABLED（0） |
| PWM | `TIM1.BDTR=0x02002000`，MOE=0 |
| Error / Trip | 0 / 0 |
| CPU 时钟 | 170000000 Hz |
| NVS | 恢复前、恢复后、测试前备份逐字节一致 |
| 电流参考上限参数 | 原配置 10 A；0.3 A 临时测试设置未持久化 |
| 调试会话 | 检查结束后退出，释放下载器 |

恢复的是测试前原应用镜像，不是本地新增双限幅测试固件。
没有重新使能或运行电机。恢复日志、启动区现场和核验结果保留于
`build/mcu_recovery_20261012/`。

### 本地证据位置

按项目归档约定，关键条件、数值和完整直方图归入本专题，不重复提交临时 JSON、
ELF 或脚本。以下为本轮实验保留的本地原始证据，临时目录可能随环境清理而消失：

- `/tmp/angle_wrap_trial_20261012/`：`old/new_profile.json`、`old/new_release.json`、
  各轮状态、数值等价测试源码/结果及测试 ELF。
- `/tmp/vector_limit_bench_20261012/`：`result.json`、临时 `main.c`、反汇编及恢复记录。
- `build/dual_limit_results_20261012/`：首次中止记录、完整直方图、应用/NVS 备份；
  `guard_capture/` 为第二次中止现场和未完成恢复的记录。
