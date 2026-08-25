# Architecture

本文描述 `AxDr_L_Motor` 当前代码实际运行架构，不作为未来规划文档。

## 1. 项目定位

本仓库以 PMSM/FOC 控制、参数辨识、Observer 与无感控制实验为核心，同时提供 AxDrive-L + STM32G474 的参考固件实现。

工程不追求将所有代码抽象成平台无关库。纯算法模块尽量保持可独立理解和 Host Test；ADC、PWM、Encoder、ThreadX、USBX 等明确属于 STM32G4 参考实现。

## 2. 控制状态与模式

顶层电机状态：

```text
DISABLED
ENABLED
RUN
```

`Motor_State_e` 表示功率授权和控制执行状态。

控制模式：

```text
TORQUE
SPEED
POSITION
OPEN_LOOP
IDENT
SENSORLESS_SPEED
```

`Motor_Mode_e` 只允许在 `DISABLED` 状态修改。`Motor_Enable()` 根据当前模式绑定 20 kHz Fast Path，运行期间不在每个快环周期重复分派模式。

快环执行模式：

```text
FAST_OFF
FAST_CURRENT
FAST_VOLTAGE
```

它不是用户控制模式，而是 20 kHz Fast Loop 的执行合同：

- `FAST_OFF`：本周期不输出控制电压；
- `FAST_CURRENT`：使用 `Theta_e + Id_Ref/Iq_Ref` 运行电流环；
- `FAST_VOLTAGE`：直接使用 `Ualpha/Ubeta`，绕过电流环。

该设计允许辨识等特殊流程在同一顶层模式内部切换电流控制与直接电压激励。

## 3. 实时执行关系

### 3.1 20 kHz Fast Loop

ADC Injected JEOS IRQ 是快环入口：

```text
ADC sample
    ↓
Motor_Fast_Run()
    ↓
Fast Path
    ↓
FAST_CURRENT ──> Current_Loop()
FAST_VOLTAGE ──> direct Ualpha/Ubeta
FAST_OFF     ──> output off
    ↓
SVPWM_Calc()
    ↓
PWM_Update()
    ↓
Fast Plot sample
```

`Fast_Loop()` 负责：

- 读取 ADC injected 结果并换算物理量；
- 调用当前绑定的 Fast Path；
- 在 `FAST_CURRENT` 下发布本周期 FOC 使用的 `Motor_Run.Theta_e`；
- 运行 Current Loop；
- 运行 SVPWM；
- 更新 TIM1 PWM；
- 记录快环 Profile 与 FAST Plot 数据。

ThreadX 不参与该 20 kHz 路径。

### 3.2 2 kHz Motor Control

TIM1 Update ISR 按 `20 kHz / 2 kHz` 分频释放 `Motor_Sem`。

Motor Thread 每次被唤醒后依次执行：

```text
Motor_Cmd_Run()
Motor_Control()
USB_Tx_Poll()
```

因此命令处理、速度控制、模式慢状态机和 USB TX 位于线程上下文，而不是 ADC IRQ。

### 3.3 1 kHz Position Loop

位置模式在 2 kHz `Motor_Control()` 内部再二分执行 Position Loop：

```text
1 kHz Position Loop
      ↓
Wm_Ref
      ↓
2 kHz Speed Loop
      ↓
Iq_Ref
```

当前主要控制频率为：

```text
Current / Fast Loop : 20 kHz
Speed / Motor Ctrl  : 2 kHz
Position Loop       : 1 kHz
```

## 4. Fast Path

`Motor_Enable()` 调用 `Fast_Path_Bind()`，根据当前 `Motor_Mode` 固定快环执行函数：

| Motor Mode | Fast Path |
|---|---|
| TORQUE / SPEED / POSITION | `Servo_Fast_Run` |
| OPEN_LOOP | `Open_Fast_Run` |
| IDENT | `Ident_Fast_Run` |
| SENSORLESS_SPEED | `Sensorless_Fast_Run` |

`Motor_Disable()` 将 Fast Path 恢复为 `Fast_Off_Run`。

这种绑定方式避免 20 kHz ISR 每周期执行顶层 Mode switch，同时保持模式生命周期清晰。

## 5. 电角度所有权

`Motor_Run.Theta_e` 始终表示当前 FOC 实际使用的电角度。

各路径只产生候选角度：

```text
Servo       -> Encoder_Theta_e()
Open Loop   -> Open_Loop()
Ident       -> Identification_Fast_Run()
Sensorless  -> Sensorless_Run()
```

Encoder 只更新机械反馈：

```text
Motor_Run.Theta_m
Motor_Run.Turn
Motor_Run.Wm
```

它不直接写 `Motor_Run.Theta_e`。

Fast Loop 在运行 `Current_Loop()` 之前唯一发布：

```text
Motor_Run.Theta_e = Theta_e
```

因此候选角度产生与 FOC 最终角度所有权分离。

## 6. Servo 控制路径

### Torque

```text
Te_Target
   ↓ Kt mapping
Iq_Ref
   ↓
Current Loop
```

### Speed

```text
Wm_Target
   ↓ Speed_Profile
Wm_Ref
   ↓ Pp
We_Ref
   ↓ Speed Loop
Iq_Ref
   ↓
Current Loop
```

### Position

```text
Position Target
    ↓ Position Loop @ 1 kHz
Wm_Ref
    ↓ Speed Loop @ 2 kHz
Iq_Ref
    ↓
Current Loop
```

Servo 三种模式统一通过 `Servo_Fast_Run()` 使用编码器电角度与 `Current_Ref` 进入 20 kHz Current Loop。

## 7. Open Loop

`OPEN_LOOP` 使用慢环生成的带符号 `We_Ref`，在 Fast Path 中由 `Open_Loop()` 生成：

```text
Theta_e
Id_Ref
Iq_Ref
```

随后继续走统一 `FAST_CURRENT -> Current_Loop -> SVPWM -> PWM` 路径。

## 8. Identification

Identification 分为慢环流程控制与快环物理激励两部分。

### 2 kHz

```text
Identification_Control()
```

负责：

- 监视当前辨识模式；
- 判断完成/失败；
- 管理辨识顶层状态。

### 20 kHz

```text
Identification_Fast_Run()
```

根据当前辨识阶段返回：

```text
FAST_CURRENT
FAST_VOLTAGE
FAST_OFF
```

`Rs_Ls` 和 `Flux` 模块负责各自具体辨识流程。

辨识结果通过显式 `Identification_Apply()` 写入 `Motor_Para`。当前 `Motor_Para_Changed()` 仍会同步刷新部分派生控制参数；该行为属于已知 Parameter Flow 架构债务，后续参数/Flash 系统整理时处理，不属于当前主控制架构。

## 9. Sensorless

`Sensorless.c` 是无感控制集成层，而不是纯算法库。

它组合：

```text
Align
IF_Start
Flux_Observer
PLL
Speed Loop
IF <-> Observer state transition
angle blend / current transition
```

当前主要状态为：

```text
ALIGN
IF
IF_TO_OBS
OBS
OBS_TO_IF
```

纯算法部分如 `Flux_Observer.c`、`PLL.c`、`IF_Start.c` 尽量保持较少硬件依赖；`Sensorless.c` 本身允许依赖 Motor 参数、Speed Controller 和 Fast Profile，因为其职责是系统级控制整合。

## 10. STM32G4 Reference Platform

以下模块明确属于 AxDrive-L / STM32G474 参考实现：

```text
Motor/Motor_ADC.c
Motor/Motor_PWM.c
Motor/Encoder.c
Core/
AZURE_RTOS/
USBX/
Drivers/
ThirdParty/
```

其中：

- ADC Fast Loop 直接读取 ADC injected data register；
- Encoder 使用 SPI1 + DMA + DWT；
- PWM 直接操作 TIM1 CCR/CCER/MOE；
- ThreadX 负责慢控制与通信调度；
- USBX 提供 USB CDC transport。

这些硬件路径不为了 Host portability 增加额外 abstraction layer。

## 11. 软件责任模型

当前工程可以按责任理解为四层，但不要求物理目录按四层重排：

```text
Command / System
Protocol / Thread / Motor State
            ↓
Control Integration
Motor_Control / Identification / Sensorless / Open Loop
            ↓
Algorithms
PID / Math / SVPWM / IF / PLL / Flux Observer / Profiles
            ↓
STM32G4 Reference Platform
ADC / PWM / Encoder / HAL / ThreadX / USBX / DMA
```

`Motor_Control`、`Identification`、`Sensorless` 属于允许跨多个算法对象的集成模块，不为了形式上的“纯算法层”继续拆分 manager/service/interface。

## 12. 构建边界

- ST Arm Clang 是主要本地开发工具链；
- GNU Arm GCC 由 CI 持续检查兼容性；
- CubeMX 负责 MCU/HAL/RTOS glue 与 `cmake/stm32cubemx/`；
- 根 `CMakeLists.txt`、`CMakePresets.json` 和 `cmake/` 下项目工具链/中间件重映射属于项目构建层；
- `ThirdParty/` 是实际参与构建的 ThreadX/USBX 来源；
- CubeMX regenerate 后应重新执行 ST Arm Clang 与 GNU Arm GCC 构建验证。

## 13. 相关文档

- [VALIDATION.md](VALIDATION.md)：当前实现的验证状态；
- [测试与问题记录索引](测试与问题记录索引.md)：各专题实机测试和问题记录入口；
- [CODE_STYLE.md](CODE_STYLE.md)：代码风格与架构约束；
- [伺服电机控制软件架构设计_阶段性定稿.md](伺服电机控制软件架构设计_阶段性定稿.md)：历史设计讨论文档，不作为当前实现事实的唯一依据。
