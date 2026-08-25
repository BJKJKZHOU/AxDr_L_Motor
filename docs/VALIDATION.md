# Validation Status

本文记录当前 `main` 分支各功能的验证状态。它用于区分“代码已实现”与“当前版本已重新上板验证”，避免将旧版本测试结论直接沿用到后续重构后的代码。

## 1. 状态定义

| 状态 | 含义 |
|---|---|
| `VALIDATED` | 当前相关实现已有对应实机验证证据 |
| `REGRESSION_REQUIRED` | 历史版本曾通过验证，但之后相关控制路径发生实质修改，需要重新上板回归 |
| `EXPERIMENTAL` | 已有实机结果，但性能、稳定性或适用工作区间尚未收敛 |
| `UNTESTED` | 实现已存在，但当前没有对应的完整实机验证证据 |

CI 编译通过只说明工程能够构建，不等价于电机控制功能完成实机验证。

## 2. 当前验证矩阵

| 功能 | 当前状态 | 说明 |
|---|---|---|
| GNU Arm GCC firmware build | `VALIDATED` | GitHub CI 当前通过 |
| ST Arm Clang local build | `VALIDATED` | 主要本地开发工具链 |
| ADC injected sampling / current conversion | `VALIDATED` | 20 kHz Fast Loop 基础路径已有实机时序与数据验证 |
| TIM1 PWM / Enable / Disable / High-Z | `VALIDATED` | 功率级关断与 PWM 输出语义已有实机验证 |
| MT6816 SPI + DMA encoder path | `VALIDATED` | SPI/DMA 快速路径与角度反馈已实测 |
| Encoder-based Current FOC baseline | `VALIDATED` | 编码器角度 + dq Current Loop + SVPWM 基础链路已实测 |
| Rs/Ls identification core flow | `VALIDATED` | 历史测试重复性与 Apply 流程已验证 |
| Flux identification | `REGRESSION_REQUIRED` | 历史测试已通过，但后续 Fast Path / angle ownership / motion flow 有改动 |
| Torque mode | `REGRESSION_REQUIRED` | 基础控制链已有实测历史，近期顶层控制流重构后建议统一回归 |
| Speed mode | `REGRESSION_REQUIRED` | signed speed flow、shared speed profile 与 Fast Path 近期调整 |
| Position mode | `REGRESSION_REQUIRED` | 位置/速度级联保留，但近期 Motor Control/Fast Path 调整后需回归 |
| Open-loop signed-speed path | `REGRESSION_REQUIRED` | 最近统一了 signed speed command flow 和 Fast Path |
| Flux Observer shadow operation | `EXPERIMENTAL` | 已有 BASIC PASS；速度平均值正确，但质量和工作区间仍需继续验证 |
| Observer `2fe` ripple | `EXPERIMENTAL` | 已确认问题位于 PLL 前，逆变器非线性/模型误差仍在排查 |
| Sensorless initial ALIGN + I/F | `REGRESSION_REQUIRED` | 历史 I/F 有实测，但最近增加双向流程并重构状态 |
| I/F -> Observer takeover | `REGRESSION_REQUIRED` | 历史 observer/takeover 测试基础存在，最近 angle blend/current transition 重构 |
| Observer -> I/F low-speed fallback | `UNTESTED` | 新增低速回切路径后尚未完成完整实机验证 |
| Sensorless reversal / through-zero operation | `UNTESTED` | 新增双向与穿零运行逻辑后尚未完成完整实机验证 |
| USB FAST/NORMAL Plot transport | `VALIDATED` | 已验证 USB 唤醒对快环长尾问题修复，历史记录中 `Deadline_Miss` 已归零 |
| New motor commissioning workflow | `EXPERIMENTAL` | `tools/new_motor_commission.py` 已建立流程，但当前仍存在不同电机适配与参数自动化缺口 |

## 3. 近期需要统一回归的范围

2026-08-24 前后对以下控制路径做过实质性重构：

- signed mechanical speed command flow；
- Sensorless 顶层状态；
- 双向 I/F 与穿零逻辑；
- I/F -> Observer 接管；
- Observer -> I/F 回切；
- shared mechanical speed profile；
- Motor parameter derived update flow；
- FOC electrical angle ownership；
- Fast Path binding 与 20 kHz 执行路径。

因此这些改动涉及的功能即使历史上已经测试过，也不直接沿用旧 `PASS`，统一标为 `REGRESSION_REQUIRED` 或 `UNTESTED`，直到当前代码版本完成实机回归。

## 4. 已有测试证据入口

测试记录按物理对象与责任层维护：

- [测试与问题记录索引](测试与问题记录索引.md)
- [辨识测试记录](辨识测试记录.md)
- [功率级关断问题记录](功率级关断问题记录.md)
- [NLOB_PLL观测器测试记录](NLOB_PLL观测器测试记录.md)
- [20kHz快环时序调试记录](20kHz快环时序调试记录.md)
- [USB_Plot_分阶段测试记录](USB_Plot_分阶段测试记录.md)
- [电机控制测试历史总记录](电机控制测试历史总记录.md)

专题文档与历史总记录冲突时，以更新、对应当前问题且有明确实测证据的专题记录为准。

## 5. 验证层级

当前工程的验证体系按三层理解：

```text
Build / Static
    ST Arm Clang
    GNU Arm GCC CI
        ↓
Host-side Algorithm Tests
    PID / Math / SVPWM / PLL / Flux Observer / IF / profiles
        ↓
Hardware Validation
    AxDrive-L + STM32G474 + real motor
```

Host-side Algorithm Tests 目前仍是需要逐步补充的能力。它用于覆盖纯算法与状态边界，但不能替代 ADC/PWM/Encoder/时序/功率级等实机验证。

## 6. 更新规则

当某个功能对应的控制路径发生实质修改时：

```text
VALIDATED
    ↓ code behavior changed
REGRESSION_REQUIRED
    ↓ current revision hardware test passes
VALIDATED
```

若新增功能尚未完成实机测试，应标记为 `UNTESTED`；已有部分实测但性能边界未确定，则使用 `EXPERIMENTAL`。

不要因为编译通过、Host Test 通过或相邻功能验证通过而自动提升实机验证状态。

## 7. 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md)：当前实际运行架构与模块责任；
- [测试与问题记录索引](测试与问题记录索引.md)：详细实验记录入口；
- [README](../README.md)：项目入口、构建和硬件说明。
