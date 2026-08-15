# AxDrive-L V1.3 引脚说明

## 1. 适用范围

本文档适用于 **AxDrive-L V1.3** 开发板，用于统一说明原理图信号、STM32G431RBT6 引脚以及当前固件配置之间的对应关系。

引脚依据：

- V1.3 原理图工程：`docs/文档/开发板工程/ProDoc_axdr_v1.3_2026-08-15.epro2`
- 原理图标题：`axdr_v1.3`
- 固件配置：`AxDr_L_Motor.ioc`
- GPIO 宏定义：`Core/Inc/main.h`

`docs/文档/2. AxDrive-L 硬件设计报告.md` 中的原理图和 PCB 图片来自旧版设计，不能作为 V1.3 引脚定义依据。

当前软件状态一栏仅表示本工程现有配置，不代表硬件功能限制：

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
| `IC` | PA0 | ADC1_IN1 | C 相电流采样 |
| `IB` | PA1 | ADC1_IN2 | B 相电流采样 |
| `IA` | PA2 | ADC1_IN3 | A 相电流采样 |
| `IBUS` | PA3 | 未配置 | 母线电流采样 |
| `DAC` | PA4 | 未配置 | 板上模拟调试信号 |
| `VA` | PC0 | ADC2_IN6 | A 相电压采样 |
| `VB` | PC1 | ADC2_IN7 | B 相电压采样 |
| `VC` | PC2 | ADC2_IN8 | C 相电压采样 |
| `APH4` | PC3 | 未配置 | 原理图保留模拟信号 |
| `ADSPE` | PC4 | ADC2_IN5 | 原理图模拟采样信号 |
| `VBUS` | PC5 | ADC2_IN11 | 母线电压采样 |
| `NTC1` | PB1 | ADC1_IN12 | 温度采样 1 |
| `NTC3` | PB12 | ADC1_IN11 | 温度采样 3 |

## 4. 编码器与 Hall

### 4.1 SPI1 编码器

| 原理图信号 | MCU 引脚 | 当前配置 | 说明 |
| --- | --- | --- | --- |
| `SPI1_SCK` | PB3 | SPI1_SCK | 编码器时钟 |
| `SPI1_MISO` | PB4 | SPI1_MISO | 编码器数据输入 |
| `SPI1_MOSI` | PB5 | SPI1_MOSI | 编码器数据输出 |
| `SPI1_CSN` | PD2 | GPIO 输出 | 片选，当前上电置高 |

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

一条 CAN 总线通常只在物理两端接入终端电阻。后续通信功能应根据本板在总线中的位置设置 `R_EN`，不能把它作为普通状态灯随意翻转。

## 6. 按键与 RGB

### 6.1 用户按键

| 原理图信号 | MCU 引脚 | 当前配置 | 有效电平 |
| --- | --- | --- | --- |
| `KEY1` | PC9 | GPIO 输入、上拉 | 按下为低 |
| `KEY2` | PC8 | GPIO 输入、上拉 | 按下为低 |
| `KEY3` | PC7 | GPIO 输入、上拉 | 按下为低 |
| `KEY4` | PC6 | GPIO 输入、上拉 | 按下为低 |

四个按键均有板外上拉和滤波电容。当前 `.ioc` 已配置输入上拉，但尚未设置 `KEY1`～`KEY4` GPIO 标签，也没有按键处理逻辑。

### 6.2 RGB 指示灯

| 原理图信号 | MCU 引脚 | 当前配置 | 器件 |
| --- | --- | --- | --- |
| `RGB` | PA5 | 未配置 | XL-3528RGBW-WS2812B |

该 RGB 灯为 WS2812B 类单线可寻址器件，不是三路普通 PWM LED。后续需要通过符合 WS2812B 时序的单线波形驱动，可使用定时器配合 DMA 或其他确定性发送方式。

## 7. LCD 与 SPI3

| 原理图信号 | MCU 引脚 | 当前配置 | 说明 |
| --- | --- | --- | --- |
| `SPI3_SCK` | PC10 | SPI3_SCK | SPI3 时钟 |
| `SPI3_MISO` | PC11 | SPI3_MISO | SPI3 数据输入 |
| `SPI3_MOSI` | PC12 | SPI3_MOSI | SPI3 数据输出 |
| `SPI3_CSN` | PA15 | GPIO 输出 | SPI3 片选，固件名为 `SPI3_CS` |
| `LCD_RES` | PC14 | GPIO 输出 | LCD 复位，当前上电置高 |
| `LCD_DC` | PC15 | GPIO 输出 | LCD 数据/命令选择，当前上电置高 |
| `LCD_CS` | PB6 | GPIO 输出 | LCD 片选，当前上电置低 |
| `LCD_BLK` | PB7 | GPIO 输出 | LCD 背光控制，低电平关闭、高电平开启 |

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
