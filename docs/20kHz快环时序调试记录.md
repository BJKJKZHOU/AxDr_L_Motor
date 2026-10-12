# 20 kHz 快环时序调试与修改记录

更新时间：2026-10-12<br>
工程：AxDr_L_Motor<br>
历史分支：`feat/motor-para`<br>
2026-10-12 快环优化：已合入 `main`（PR #41），实验测量条件见第 9 节

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

2026-10-12 的数学函数审查和 Angle_Wrap 优化见
[第 9 节](#fast-loop-20261012)；Vector2_Limit 缩放分支独立测试见
[第 10 节](#vector-limit-20261012)；未完成的双限幅实验与板卡恢复见
[第 11 节](#dual-limit-cancelled-20261012)。
历史 160 MHz 数据不与本次 170 MHz 数据混用。

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

<a id="fast-loop-20261012"></a>

## 9. 有感电流快环性能优化与验证（2026-10-12）

### 9.1 优化内容及版本边界

`perf/fast-loop-timing` 是本轮性能实验分支；正式控制优化经
[PR #41](https://github.com/BJKJKZHOU/AxDr_L_Motor/pull/41)
合入 `main`，最终固件不包含此次试验新增的 ARR→CCR 直方图、
逐函数 DWT 探针及数学/限幅实验宏。

本轮正式优化包括：有感电角度与 SinCos 结果复用；电流 PID 的 Kd=0 快速路径；
SVPWM 复用母线电压倒数；dq 前馈解耦默认开启；`Angle_Wrap()`
的多圈归一化使用 IEEE 754 binary32 整数余数运算，避免常规角度
调用 Flash 中的 `fmodf`。未改变机械 ESO 方程、PWM 开关频率、采样触发点或
两相采样方式。旧 NVS 中已经保存的前馈开关值不受默认值变化覆盖。

### 9.2 数学函数测量与数值校验

平台为 STM32G474、CPU/TIM1 170 MHz、20 kHz PWM、ST Arm Clang Release `-Oz`。
电机在有感 POSITION + RUN 下保持当前位置，母线约 24.7～24.8 V、LCD 正常刷新。
数学函数单独计时使用临时 DWT 探针，因此包含调用和时间戳开销，
不能把单项最大值相加代表同一拍耗时。

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

单独计时各约 20.9 万次。电角度归一化旧实现的最大值约 2.206 µs，
新实现该工况约 0.618 µs。主机对照 **3,312,909 组 float32 输入**
（正负零、边界、非正规数、大数、NaN/Inf 及随机位模式），与旧实现
**逐位一致**，非全输入穷举；反汇编确认新实现没有 `fmodf` 或
`memcpy` 函数调用。

新整数余数算法的迭代次数仍与输入指数有关；本次 105 cycles 仅适用于所测电角度，
不是对任意有限 float 输入的最坏执行时间承诺。

### 9.3 ARR→CCR 完整路径对照

关闭逐函数数学探针，但实验版本保留 ARR→CCR 时间直方图与超限计数；
两组均为 10 s 左右、约 20.1 万次 PWM 有效更新。该测量版本不是
已合入 `main` 的无诊断代码版本。

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

本次位置保持工况下，优化后 ARR→CCR 最大值距 12.5 µs 仍有
**0.894118 µs** 余量。直方图是每桶 43 个 TIM1 ticks 的区间统计，
分位数据是桶上边界，不是精确分位点；严苛截止时间仍以周期最大值和
超限次数判断。此结论不外推至高速、满载及两次矢量缩放同时触发的工况。

### 9.4 无本轮专用诊断代码的正式固件复测

PR #41 的干净合并候选 `perf/foc-hotpath-cleanup`（`60a3359`）
经本地 Release 烧录测试：170 MHz、20 kHz、母线约 24.8 V、
有感 POSITION 保持、原有限流 10 A、dq 前馈开启、LCD 正常刷新，
采集 10 s。

| 指标 | 结果 |
|---|---:|
| Fast 完整路径最大（含采样等待） | **39.106 µs** |
| ADC ISR 最大 | **8.718 µs** |
| `Fast_Loop()` 最大 | **7.894 µs** |
| Deadline Miss / 编码器迟到 / 编码器丢帧 / 故障 | **均为 0** |

这是**20 kHz 整周期**（50 µs）时序通过的证据；因最终代码没有本次专用
ARR→CCR 探针，不能直接将这组数据解读为正式版本再次实测通过 12.5 µs。

<a id="vector-limit-20261012"></a>

## 10. Vector2_Limit 缩放分支独立耗时（2026-10-12）

使用当前 Release 编译对象、170 MHz DWT；在 PWM 关闭时调用，测量阶段屏蔽中断。
每组 100,000 次，正式结果保留调用和时间戳等少量开销：

| 路径 | cycles | µs |
|---|---:|---:|
| 未超限或恰好到限 | 29 | 0.170588 |
| **触发缩放** | **69** | **0.405882** |
| 限值为零、输出清零 | 20 | 0.117647 |

五组缩放测试（轻微超限、明显超限、负分量与零分量）合计
**500,000 次**，最小/平均/最大均为 69 cycles，结果核验通过。
缩放比未缩放增加 **40 cycles（0.235294 µs）**；
`sqrtf` 在本工具链中使用硬件 `VSQRT.F32`，缩放另有硬件浮点除法。
两个限幅点同时额外增加约 0.47 µs **仅是独立测试增量的算术估计**，
并非电流环同时缩放的实际最坏耗时。

<a id="dual-limit-cancelled-20261012"></a>

## 11. 双限幅联合实验中止及固件恢复（2026-10-12）

曾尝试通过临时实验代码验证同周期电流参考、电压矢量双缩放及积分抗饱和。
两次实验均在基线阶段因反馈电流保护中止，没有进入激励阶段，
**未得到双限幅整环 12.5 µs 的验收结论**；实验计时包含额外诊断负担，
不与第 9 节的正式优化对照混用。实验代码没有合入 `main`。

第二次中止后的应用镜像恢复过程中，调试连接中断且写回没有完成。
重新建立 SWD 连接后确认 Flash 起始向量及起始 4 KiB 被擦空，
故程序无法启动；并非已证实 MCU 硬件损坏。随后对测试前备份的
507,904 字节应用镜像完成写入与逐字节验证；未写入 NVS。
复位后 USB CDC 恢复枚举，可读取参数，DISABLED、MOE=0、Error/Trip=0，
NVS 与测试前备份一致。恢复后未重新运行电机。

这次事件反映出临时恢复流程缺少完整写入与校验失败检查；
后续恢复固件应将写入完成和校验成功作为退出条件，不得只凭
部分 OpenOCD 命令返回就判定恢复成功。
