# Host Test / CI 验证体系设计

日期：2026-08-25

## 1. 目标

使用 Unity + Ceedling 为 `AxDr_L_Motor` 建立长期维护的 Host 端算法验证体系。

测试的期望行为必须来源于控制理论、数学公式和物理不变量，而不是逐行复刻当前 C 实现。测试体系也不能为了提高覆盖率而人为构造大量硬件 Mock。

最终验证体系分为三类彼此独立的证据：

```text
Firmware Build
    GNU Arm GCC 编译 / 链接

Host Algorithm Test
    Unity + Ceedling
    native host compiler
    基于理论公式 / 独立参考模型

Hardware Validation
    AxDrive-L + STM32G474 + 实际电机
```

这三类证据必须保持语义分离。Host Test 通过不能单独把某项功能提升为 Hardware `VALIDATED`。

## 2. 初始范围

第一阶段 Host Test 覆盖具有明确平台无关理论、数学关系或状态方程的确定性算法：

```text
Algo/Math.c
Algo/PID.c
Algo/Voltage_Mod.c
Motor/Motion_Loop.c       -> 仅 Speed_Profile
Sensorless/IF_Start.c
Observer/PLL.c
Observer/Flux_Observer.c
```

第一阶段不对以下模块做整模块 Host Test：

```text
Motor/Motor_Control.c
Sensorless/Sensorless.c
Identification/Identification.c
Motor/Motor_ADC.c
Motor/Motor_PWM.c
Motor/Encoder.c
ThreadX / USBX / HAL / DMA / peripheral IRQ paths
```

这些模块属于系统集成层或硬件参考实现层，不因为 Host Test 覆盖率而强行重构。

## 3. 测试框架

正式 Host Test 框架使用 Unity + Ceedling。

仓库结构：

```text
project.yml
Gemfile

test/
├── test_math.c
├── test_pid.c
├── test_svpwm.c
├── test_motion.c
├── test_if_start.c
├── test_pll.c
├── test_flux_observer.c
└── support/
    ├── test_math_ref.c
    ├── test_math_ref.h
    ├── motor_model.c
    └── motor_model.h
```

`test/support/` 只保存独立的测试数学工具和参考模型，不作为生产代码，也不参与固件构建。

`Gemfile` 必须固定 CI 使用的 Ceedling 相关 Ruby 依赖版本，使本地和 CI 的测试环境可复现。

Ceedling 自带的 CMock 可以保留，但第一阶段测试不以 Mock 为核心。优先直接调用真实算法代码，并使用理论生成的输入进行验证。

## 4. 测试 Oracle 原则

测试期望值必须来自数学定义、控制规范或独立参考模型：

```text
理论 / 控制规范 / 物理模型
            ↓
        期望行为
            ↓
     生产代码实现结果
```

测试代码不能把生产实现的 if/else、状态切换和中间变量结构原样复制成“参考结果”。

如果未来生产代码更换实现方法但保持相同控制律，测试应继续成立。

## 5. 数值比较规则

Host Test 使用共享的数值比较辅助函数。

### 5.1 标量比较

根据算法的量纲和数值敏感度，同时使用绝对误差与相对误差：

```text
|a - b| <= abs_tol + rel_tol * max(|a|, |b|)
```

不使用一个全局固定 epsilon 覆盖所有算法。

### 5.2 角度比较

角度误差必须使用 `[-π, π]` 范围内的包角差：

```text
angle_error = wrap_to_pi(a - b)
```

再判断：

```text
|angle_error| <= angle_tol
```

不能直接比较跨越 `0 / 2π` 边界的原始角度差。

### 5.3 动态测试

动态算法通过序列输入验证。允许检查：

- 瞬态边界；
- 收敛时间窗口；
- 最终稳态误差；
- 物理不变量。

不要求不同 Host 编译器得到逐采样点 bit-identical 的浮点轨迹。

## 6. Math 测试

### 6.1 `Angle_Wrap`

数学定义：

```text
theta_out = theta mod 2π
0 <= theta_out < 2π
```

覆盖：

- `0`、`2π`、`-2π`；
- wrap 边界上下的小偏移；
- 多圈正角度和负角度；
- 输入与输出只相差整数个 `2π` 的不变量。

### 6.2 `Limit_Value`

参考公式：

```text
y = min(max(x, min), max)
```

覆盖低于范围、范围内、高于范围三种情况，并验证函数返回的限幅方向。

### 6.3 `Vector2_Limit`

当 `L > 0`：

```text
if sqrt(x² + y²) <= L:
    output = input
else:
    output = input * L / sqrt(x² + y²)
```

验证：

- 模长上限；
- 超限后方向保持不变；
- 未超限时输出不变；
- `L <= 0` 时输出为零。

## 7. PID 测试

PID 测试规范直接来自离散控制律：

```text
e[k] = r[k] - y[k]
I*[k] = I[k-1] + Ki * e[k] * Ts
D[k] = -Kd * (y[k] - y[k-1]) / Ts
u*[k] = Kp * e[k] + I[k] + D[k]
```

控制器还包含：

- Integrator saturation；
- Output saturation；
- conditional anti-windup。

测试覆盖：

- 纯比例响应；
- 恒定误差下的积分累积；
- 正负积分限幅；
- 输出饱和；
- derivative-on-measurement 的符号和幅值；
- 输出饱和时积分不能继续向错误方向增长；
- 误差反向后能退出饱和；
- 零误差稳态。

测试 Oracle 由上述离散方程计算，不复制 `PID_Run()` 内部控制流。

## 8. SVPWM 测试

对输入电压矢量 `(Ualpha, Ubeta)`：

```text
Ua = Ualpha
Ub = -0.5 * Ualpha + sqrt(3)/2 * Ubeta
Uc = -0.5 * Ualpha - sqrt(3)/2 * Ubeta
Uoff = -0.5 * (max(Ua, Ub, Uc) + min(Ua, Ub, Uc))
Da = 0.5 + (Ua + Uoff) / Vbus
Db = 0.5 + (Ub + Uoff) / Vbus
Dc = 0.5 + (Uc + Uoff) / Vbus
```

参考模型只在最后一步将 Duty 限制到 `[0, 1]`。

测试覆盖：

- 零矢量 -> `0.5 / 0.5 / 0.5`；
- `Vbus <= 0` -> 三相中性 Duty `0.5`；
- 代表性的 alpha/beta 轴方向和六个扇区；
- 三相 Duty 始终位于 `[0, 1]`；
- 相反电压矢量的对称性；
- 在一组网格化输入点上与独立公式一致。

Host SVPWM 测试只验证调制数学，不验证定时器 preload、Dead Time 或实际 Gate 时序。

## 9. Speed Profile 测试

`Speed_Profile` 按机械加速度和减速度约束定义。

同方向加速：

```text
|Wm_ref[k+1] - Wm_ref[k]| <= Acc * SPD_TS
```

同方向减速：

```text
|Wm_ref[k+1] - Wm_ref[k]| <= Dec * SPD_TS
```

当目标与当前参考异号时，参考速度必须先按照 `Dec` 接近零，达到零后才能按照 `Acc` 加速进入反方向。

测试覆盖：

- `0 -> positive`；
- `0 -> negative`；
- 正负方向加速对称性；
- 同方向减速；
- 正转到反转；
- 反转到正转；
- 不允许越过目标值；
- 剩余差值小于一个 step 时准确停在目标。

测试只验证运动规律，不依赖内部 helper 的具体实现。

## 10. I/F Start 测试

I/F 测试以电角速度、电角度和电流轨迹为主要验证对象，不以 enum 状态本身作为核心 Oracle。

### 10.1 电角速度

每周期满足：

```text
|We[k+1] - We[k]| <= IF_ACC_RAD_S2 * CUR_TS
```

速度不能越过目标值。

### 10.2 电角度

积分规律：

```text
Theta[k+1] = wrap(Theta[k] + We[k] * CUR_TS)
```

测试覆盖多次正向和反向跨越 `0 / 2π`。

### 10.3 电流幅值与方向

额定电流幅值：

```text
ratio = min(|We| / IF_WE_TARGET_RAD_S, 1)
Iq_abs = IF_IQ_START_A +
         (IF_IQ_TARGET_A - IF_IQ_START_A) * ratio
```

电流 slew 约束：

```text
|Iq[k+1] - Iq[k]| <= IF_IQ_SLEW_A_S * CUR_TS
```

`Iq` 符号跟随当前实际旋转方向；在精确零速处，如果需要启动，则允许使用目标方向确定起步电流符号。

测试覆盖：

- `0 -> positive`；
- `0 -> negative`；
- 正负方向加速；
- 同号目标变化；
- 正转到反转并穿零；
- 反转到正转并穿零；
- 电流 slew 上限；
- 过零附近的电流方向；
- 达到目标后保持配置时间才 Ready；
- 修改目标后 Hold readiness 被清除。

## 11. PLL 测试

PLL 输入由解析旋转矢量生成：

```text
X = Mag * cos(theta)
Y = Mag * sin(theta)
theta(t) = theta0 + we * t
```

测试覆盖：

- Reset 时角度包络；
- `Mag_Ref <= 0` 或 `Ts <= 0` 时不更新；
- 零速锁定；
- 正恒速跟踪；
- 负恒速跟踪；
- 多次跨越 `0 / 2π`；
- 非零初始角度误差；
- 非零初始速度误差。

动态断言使用包角误差和速度误差。收敛时间窗口和稳态误差边界由测试使用的 PLL 参数决定，不要求瞬态逐采样点复制生产实现。

## 12. Flux Observer 测试

Flux Observer 的 Oracle 使用 `test/support/motor_model.c` 中独立实现的理想表贴式 PMSM alpha/beta 模型。

给定电角度：

```text
Psi_alpha = Flux * cos(theta)
Psi_beta  = Flux * sin(theta)

Lambda_alpha = Ls * Ialpha + Psi_alpha
Lambda_beta  = Ls * Ibeta  + Psi_beta

Ualpha = Rs * Ialpha + d(Lambda_alpha)/dt
Ubeta  = Rs * Ibeta  + d(Lambda_beta)/dt
```

参考模型独立产生：

```text
Ialpha / Ibeta / Ualpha / Ubeta
```

并将其输入真实 `Flux_Observer_Run()`。

测试覆盖：

- 已知磁链角和电流下的 Reset；
- 静止已知磁链；
- 正向旋转磁链；
- 反向旋转磁链；
- 磁链幅值收敛；
- 估算磁链角与理论角度的包角误差；
- alpha/beta 方向对称性。

可以增加组合测试：

```text
理想 PMSM 模型
    ↓
Flux Observer
    ↓
生产 PLL
    ↓
Theta / We estimate
```

再与已知合成轨迹比较。

这仍然属于 Host Algorithm Test，不能等价为完整 Sensorless 实机验证。

## 13. 硬件边界

不为了覆盖率给 STM32 外设行为增加 Host Mock。

以下内容不属于 Host Test 的真实性范围：

```text
ADC injected trigger timing
ADC register acquisition
TIM1 preload / UEV / MOE / CCER behavior
PWM physical output
SPI / DMA / MT6816 timing
DWT cycle timing
ThreadX scheduling
USBX transport timing
interrupt priority and deadline behavior
power-stage safety behavior
real motor startup and observer robustness
```

这些内容由 Firmware Build 与已有/后续 Hardware Validation 覆盖。

## 14. CI 集成

保留现有 GNU Arm GCC Firmware Build job，作为独立的嵌入式编译/链接兼容性检查。

增加独立的 Host Test job：

```text
checkout
setup Ruby
bundle install
bundle exec ceedling test:all
```

Host Test job 使用 Ubuntu native compiler，不要求 ARM toolchain，也不依赖与这些纯算法无关的 STM32 middleware 子模块。

从第一版开始配置 Ceedling gcov，使覆盖率报告可查看，但覆盖率百分比不作为 CI pass/fail 门槛。

CI 语义必须保持明确：

```text
Firmware Build PASS
    -> 固件能被 GNU Arm GCC 完整编译和链接

Host Test PASS
    -> 被覆盖的数学 / 控制行为符合测试规范

Hardware VALIDATED
    -> 当前固件版本已经通过对应实机测试
```

## 15. 生产代码修改原则

引入 Host Test 不能成为大规模重构生产代码的理由。

只允许为了 native 编译确定性算法源码而进行必要且行为保持的最小修改。

如果某个模块需要大量 HAL、寄存器、RTOS 或全局状态 Mock 才能测试，则第一阶段直接排除该模块，而不是为测试强行增加 abstraction layer。

`Sensorless.c`、`Motor_Control.c` 和硬件模块继续作为系统集成 / 参考平台代码。只有未来真实维护问题需要时，才考虑进一步抽出纯算法边界。

## 16. 完成标准

第一阶段实现完成时必须满足：

- Unity + Ceedling 可以通过固定的 Ruby 环境在本地运行；
- CI 增加独立 Host Test job；
- 七个初始算法区域都有基于理论 / 独立参考模型的测试；
- 有统一的标量和角度比较辅助函数；
- 理想 PMSM alpha/beta 参考模型只存在于 `test/support/`；
- gcov 可生成报告，但不作为门禁阈值；
- 不引入 STM32 外设 Mock 层；
- 现有 firmware build 行为和 GCC CI 保持不变；
- `docs/VALIDATION.md` 继续明确区分 Host Test 证据与 Hardware Validation。
