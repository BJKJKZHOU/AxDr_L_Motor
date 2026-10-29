# AxDrive-L V1.3 引脚说明

## 1. 适用范围

本文档适用于 **AxDrive-L V1.3** 开发板，用于统一说明原理图信号、STM32G474RET6 引脚以及当前固件配置之间的对应关系。

引脚依据：

- V1.3 原理图工程：`docs/文档/开发板工程/ProDoc_axdr_v1.3_2026-08-15.epro2`
- 原理图标题：`axdr_v1.3`
- 固件配置：`AxDr_L_Motor.ioc`
- GPIO 宏定义：`Core/Inc/main.h`

`docs/文档/2. AxDrive-L 硬件设计报告.md` 中的原理图和 PCB 图片来自旧版设计，不能作为 V1.3 引脚定义依据。

当前软件状态一栏记录本工程现有配置：

- **已配置**：已经在 `.ioc` 中分配为对应外设或 GPIO。
- **未配置**：原理图有连接，但当前固件未使用，通常保持芯片复位后的模拟输入状态。

## 2. 电机功率控制

| 原理图信号 | MCU 引脚 | 当前配置 | 硬件用途 | 说明 |
| --- | --- | --- | --- | --- |
| `HIN1` | PA8 | TIM1_CH1 | A 相高侧 PWM | 由 TIM1 CCER/MOE 控制输出 |
| `HIN2` | PA9 | TIM1_CH2 | B 相高侧 PWM | 由 TIM1 CCER/MOE 控制输出 |
| `HIN3` | PA10 | TIM1_CH3 | C 相高侧 PWM | 由 TIM1 CCER/MOE 控制输出 |
| `LIN1` | PB13 | TIM1_CH1N | A 相低侧 PWM | CH1 互补输出 |
| `LIN2` | PB14 | TIM1_CH2N | B 相低侧 PWM | CH2 互补输出 |
| `LIN3` | PB15 | TIM1_CH3N | C 相低侧 PWM | CH3 互补输出 |

三相栅极驱动器为 FD6288。当前电机使能只控制 TIM1 的六路 PWM 输出。

## 3. 模拟采样

| 原理图信号 | MCU 引脚 | 当前配置 | 硬件用途 |
| --- | --- | --- | --- |
| `IC` | PA0 | ADC2_IN1，动态注入 Rank1 | C 相下管电流采样 |
| `IB` | PA1 | ADC1_IN2 / ADC2_IN2，动态注入 Rank1 | B 相下管电流采样 |
| `IA` | PA2 | ADC1_IN3，动态注入 Rank1 | A 相下管电流采样 |
| `IBUS` | PA3 | 未配置、未采样 | 母线电流采样 |
| `DAC` | PA4 | 未配置 | 板上模拟调试信号 |
| `VA` | PC0 | ADC2_IN6 | A 相电压采样 |
| `VB` | PC1 | ADC2_IN7 | B 相电压采样 |
| `VC` | PC2 | ADC2_IN8 | C 相电压采样 |
| `APH4` | PC3 | 未配置 | 原理图保留模拟信号 |
| `ADSPE` | PC4 | ADC2_IN5 | 原理图模拟采样信号 |
| `VBUS` | PC5 | ADC2_IN11 | 母线电压采样 |
| `NTC1` | PB1 | ADC1_IN12 | 温度采样 1 |
| `NTC3` | PB12 | ADC1_IN11 | 温度采样 3 |

当前动态采样实验分支将 ADC1/ADC2 配置为**独立 ADC、相同 TIM1 TRGO2 下降沿注入触发**，
不再使用硬件 `ADC_DUALMODE_INJECSIMULT`。每周期按 SVPWM 占空比
舍弃最大占空比对应的相：AB（ADC1 IA、ADC2 IB）、
AC（ADC1 IA、ADC2 IC）、BC（ADC1 IB、ADC2 IC）。
未采样的一相使用三相电流之和为零重建；ADC2 的注入 Rank2
始终保留 Vbus 采样，电压保护链路保持原有数据来源。
两个 ADC 的注入队列均保持 JQM=0（重复最后一个上下文），
仅切换 Rank1，触发源、极性及转换序列长度不变；
ADC1/ADC2 对 B 相各自校准独立零偏。

### 生成配置与运行时通道切换

`.ioc` 与 `Core/Src/adc.c` 已同步为独立 ADC、TIM1 TRGO2 下降沿触发。
生成配置中的初始组合为 ADC1 Rank1 = IA、ADC2 Rank1 = IB、Rank2 = Vbus；
PA0 的模拟引脚初始化及反初始化归属 ADC2。

`ADC_Calib()` 检查独立模式、注入触发配置和序列长度，预计算四个通道的
注入上下文并启用队列。运行时按下一周期的采样组合写入对应上下文；
生成配置定义初始序列，动态选相由 `Motor/Motor_ADC.c` 实现。

参考 ST MCSDK 的预计算注入上下文写入方式，但保留本工程的
20 kHz 快环、PWM 触发时刻与 Vbus 采样；动态采样窗口自适应调整尚未实现。

## 4. 编码器与 Hall

### 4.1 SPI1 编码器

| 原理图信号 | MCU 引脚 | 当前配置 | 说明 |
| --- | --- | --- | --- |
| `SPI1_SCK` | PB3 | SPI1_SCK | 编码器时钟 |
| `SPI1_MISO` | PB4 | SPI1_MISO | 编码器数据输入 |
| `SPI1_MOSI` | PB5 | SPI1_MOSI | 编码器数据输出 |
| `SPI1_CSN` | PD2 | GPIO 上拉输入 | 选择 SPI 后配置为片选输出，空闲置高；ABZ 模式用于 Z 输入 |

SPI1 初始化延后到读取编码器协议参数之后，仅选择 SPI 时才配置 PB3/PB4/PB5 复用。共享接口的 ABZ 接线和运行时切换见 [ABZ 编码器接入](ABZ编码器接入.md)。

### 4.2 Hall 接口

| 原理图信号 | MCU 引脚 | 当前配置 | 说明 |
| --- | --- | --- | --- |
| `HALL_A` | PA6 | ADC2_IN3 | 当前作为模拟输入采样 |
| `HALL_B` | PA7 | ADC2_IN4 | 当前作为模拟输入采样 |
| `HALL_C` | PB0 | 未配置 | 尚未接入 Hall 控制逻辑 |

## 5. CAN

| 原理图信号 | MCU 引脚 | 当前配置 | 说明 |
| --- | --- | --- | --- |
| `CAN_RX` | PB8 | FDCAN1_RX | CAN 接收 |
| `CAN_TX` | PB9 | FDCAN1_TX | CAN 发送 |
| `R_EN` | PC13 | GPIO 输出 | CAN 120 Ω 终端电阻开关控制 |

`R_EN` 的实际连接为：

```text
PC13 / R_EN
      │
      ├─ U17 GS4157B-CR 控制端
      │      └─ R96 120 Ω 接于 CAN_H 与 CAN_L 之间
      │
      └─ R28 1 kΩ + L6 指示灯
```

U17 的 `NC` 端连接 `CAN_H`，`COM` 端通过 R96 连接 `CAN_L`，`NO` 端悬空。当前固件将 `R_EN` 初始化为低电平，使终端电阻保持接入。该信号只影响 CAN 总线终端匹配，不影响 FD6288、MOS 或三相 PWM。

CAN 总线通常在物理两端接入终端电阻，`R_EN` 控制本板终端电阻的接入。

## 6. 按键与 RGB

### 6.1 用户按键

| 原理图信号 | MCU 引脚 | 当前配置 | 有效电平 |
| --- | --- | --- | --- |
| `KEY1` | PC9 | GPIO 输入、上拉 | 按下为低 |
| `KEY2` | PC8 | GPIO 输入、上拉 | 按下为低 |
| `KEY3` | PC7 | GPIO 输入、上拉 | 按下为低 |
| `KEY4` | PC6 | GPIO 输入、上拉 | 按下为低 |

四个按键均有板外上拉和滤波电容。当前 `.ioc` 已配置输入上拉，并为 PC9/PC8/PC7
设置 `KEY1`/`KEY2`/`KEY3` 标签；PC6 保留为未使用的输入。

按实物从左到右计入复位键：复位、KEY1 使能/取消使能、KEY2 运行/停止、
KEY3 模式切换、KEY4 保留。KEY3 循环为 **转矩 → 速度 → 位置 → 无感转速 → 转矩**；
若当前为开环、辨识或寻相，按一次回到转矩。

面板线程每 5 ticks 扫描，连续 3 次采样稳定后确认按下；长按只触发一次，释放后重新接收。
模式切换仅允许 DISABLED，ENABLED/RUN 时按下被忽略，也不会留待取消使能后执行。
电机线程通过现有参数入口切换模式，保留目标值，不自动使能或运行。
KEY1/KEY2/KEY3 同一轮扫描触发时按此顺序优先处理。
验证条件与结果见 [LCD 显示与实时性验证记录](../tests/system/LCD显示与实时性验证记录.md#8-key3-四模式循环)。

### 6.2 RGB 指示灯

| 原理图信号 | MCU 引脚 | 当前配置 | 器件 |
| --- | --- | --- | --- |
| `RGB` | PA5 | 未配置 | XL-3528RGBW-WS2812B |

该 RGB 灯为 WS2812B 类单线可寻址器件，使用 WS2812B 时序的单线波形驱动。

## 7. LCD 与 SPI3

| 原理图信号 | MCU 引脚 | 当前配置 | 说明 |
| --- | --- | --- | --- |
| `SPI3_SCK` | PC10 | SPI3_SCK | SPI3 时钟 |
| `SPI3_MISO` | PC11 | SPI3_MISO | SPI3 数据输入 |
| `SPI3_MOSI` | PC12 | SPI3_MOSI | SPI3 数据输出 |
| `SPI3_CSN` | PA15 | GPIO 输出 | SPI3 片选，固件名为 `SPI3_CS`，默认高电平 |
| `LCD_RES` | PC14 | GPIO 输出 | LCD 复位，当前上电置高 |
| `LCD_DC` | PC15 | GPIO 输出 | LCD 数据/命令选择，当前上电置高 |
| `LCD_CS` | PB6 | GPIO 输出 | LCD 片选，默认高电平，仅传输时拉低 |
| `LCD_BLK` | PB7 | GPIO 输出，上拉、默认高 | 经 R21 驱动 S8550 PNP 管 Q4，低电平开启背光、高电平关闭 |

## 8. 其他通信与调试接口

| 原理图信号 | MCU 引脚 | 当前配置 | 说明 |
| --- | --- | --- | --- |
| `TX_RX` | PB10 | USART3_TX | 串口发送 |
| `RX_TX` | PB11 | USART3_RX | 串口接收 |
| `COM` | PB2 | 未配置 | 原理图通信控制信号 |
| `BDM` | PA11 | USB_DM | USB D- |
| `BDP` | PA12 | USB_DP | USB D+ |
| `DIO` | PA13 | SWDIO | SWD 调试数据 |
| `CLK` | PA14 | SWCLK | SWD 调试时钟 |
| `O_IN` | PF0 | RCC_OSC_IN | 外部高速晶振输入 |
| `O_OUT` | PF1 | RCC_OSC_OUT | 外部高速晶振输出 |
| `NRST` | NRST | 系统复位 | 低电平复位 MCU |

## 9. 软件维护约定

1. 原理图信号名称与 `.ioc` GPIO Label 应尽量保持一致。
2. 修改引脚配置时，同时检查 `.ioc`、生成代码和本文档。
3. 不能根据旧版硬件报告图片推断 V1.3 引脚。
4. `R_EN` 只由 CAN 功能管理，不得加入电机使能或 PWM 开关流程。
5. TIM1 CH1～CH5 时基关系和电机控制时序由电机控制文档说明，本文档只记录硬件引脚归属。

LCD 显示配置与实测结果见[LCD 显示与实时性验证记录](../tests/system/LCD显示与实时性验证记录.md)。
