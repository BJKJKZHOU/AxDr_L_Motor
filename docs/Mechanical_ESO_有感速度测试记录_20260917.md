# Mechanical ESO 有感速度测试记录（2026-09-17）

## 1. 目的

本记录用于固化 `feat/mechanical-eso` 分支的首轮实机验证结论。

Mechanical ESO 的当前职责边界：

- 20 kHz 快环运行；
- 输入编码器连续机械位置、实际 `Iq` 和已辨识 `J/B/Flux/Pp`；
- `Wm_hat` 作为有感 Torque / Speed / Position 模式的速度反馈来源；
- `Theta_hat`、`Td_hat` 和观测误差仅用于诊断；
- FOC 电角度与 Position Loop 位置反馈仍直接使用编码器；
- 暂不加入 disturbance compensation、notch、encoder periodic correction 等额外控制功能。

测试电机本轮使用 MT6835，固件按完整 21-bit 角度读取。

## 2. 首轮 Position 闭环结果

沿用 RAM 电机参数与寻相结果，完成无断点 SWD 测试。

首个 0.25 turn 位置动作：

| 检查项 | 结果 |
| --- | --- |
| Enable / Run | 正常进入 `ENABLED / RUN` |
| 方向 | 正指令产生正机械位置变化 |
| 机械速度 | 限速 5 rad/s，采样峰值约 6.26 rad/s |
| 位置收敛 | 目标附近约 ±0.23 rad 往复，10 s 超时 |
| 电流 | SWD 采样相电流峰值约 0.97 A |
| 保护 | 无保护故障 |
| STOP | 从约 4.08 rad/s 停至接近 0 |
| Disable | PWM 关闭，编码器位置稳定 |

由于 Position Loop 绕过后纯 Speed 测试仍出现明显速度波动，本轮不将该现象归因于 Position Kp。

## 3. ESO Bandwidth 扫描

固定：

- Speed BW = 50 Hz；
- current limit = 1 A；
- 相同 Motor 参数与 Servo Phase 结果。

测试 `ESO BW = 100 / 150 / 200 Hz`，每个非零速度保持 5 s，再回零 3 s。

非零段最后 2 s 的 `Wm_hat`：

| 速度指令 | ESO 100 Hz | ESO 150 Hz | ESO 200 Hz |
| --- | ---: | ---: | ---: |
| +1 rad/s | +0.881 ± 0.687 | +0.872 ± 0.794 | +1.091 ± 0.671 |
| -1 rad/s | -1.155 ± 0.636 | -0.937 ± 0.777 | -0.630 ± 0.691 |
| +2 rad/s | +1.968 ± 0.700 | +1.905 ± 0.738 | +1.969 ± 0.728 |
| -2 rad/s | -1.945 ± 0.705 | -1.973 ± 0.670 | -1.905 ± 0.719 |

结论：

- 提高 ESO BW 没有一致改善速度纹波；
- 旧编码器差分速度与 ESO 速度高度相关，相关系数约 0.99；
- 零速段均可稳定，末段平均速度绝对值低于 0.009 rad/s；
- ESO Error 随 BW 提高有所降低，但 `Td_hat` 波动没有明显改善；
- `Iq Ref` 采样峰值约 0.466 A，未触及 1 A 限幅；
- 本轮不继续通过提高 ESO BW 处理该速度纹波。

ESO BW 恢复为 100 Hz。

## 4. 50 RPM / 200 RPM 定速测试

固定：

- Speed BW = 50 Hz；
- ESO BW = 100 Hz；
- current limit = 1 A。

### 50 RPM

去掉启动前 2 s：

| 项目 | 结果 |
| --- | ---: |
| 平均转速 | 49.79 RPM |
| 速度标准差 | 6.77 RPM（0.709 rad/s） |
| 采样范围 | 35.24 ~ 66.05 RPM |
| 相电流 SWD 采样峰值 | 1.05 A |

### 200 RPM

去掉启动前 2 s：

| 项目 | 结果 |
| --- | ---: |
| 平均转速 | 199.81 RPM |
| 速度标准差 | 6.74 RPM（3.37%） |
| 采样范围 | 185.07 ~ 215.51 RPM |
| 相电流 SWD 采样峰值 | 1.01 A |

平均速度跟踪准确。50 RPM 到 200 RPM 时绝对速度标准差基本不变，而相对纹波明显下降。

MT6835 使用 21-bit 角度，因此该约 6.7 RPM 纹波不能用基本编码器 count 分辨率直接解释。

## 5. Speed Bandwidth 扫描

固定：

- target = +200 RPM；
- ESO BW = 100 Hz；
- current limit = 1 A；
- 每组运行 10 s；
- 统计最后 8 s。

| Speed BW | 平均转速 | ESO 速度标准差 | 旧差分速度标准差 | Iq Ref 标准差 |
| --- | ---: | ---: | ---: | ---: |
| 10 Hz | 199.06 RPM | 8.91 RPM | 7.61 RPM | 0.0402 A |
| 20 Hz | 199.55 RPM | 8.50 RPM | 7.91 RPM | 0.0781 A |
| 50 Hz | 199.88 RPM | 6.73 RPM | 7.64 RPM | 0.1544 A |

从 50 Hz 降到 10 Hz：

- `Iq Ref` 纹波下降约 74%；
- 旧编码器差分速度纹波基本不变；
- ESO 速度纹波反而增大。

因此可确认 Speed PI 会追踪现有周期速度信号并产生对应电流补偿，但不能据此判断该信号是编码器周期误差还是真实机械转矩纹波。

Speed BW 测试后恢复为 50 Hz。

## 6. 机械阶次与正反向测试

对已有数据按机械角度分箱并进行机械阶次 FFT；随后完成 `+200 RPM / -200 RPM` 各 10 s 测试。

新增测试采样率约 194 Hz。

稳定主峰：

- 约 14 阶；
- 约 12 阶。

原 50 RPM 数据中旧差分速度：

- 14.03 阶，单边峰值约 7.28 RPM；
- 12.07 阶，单边峰值约 4.04 RPM。

原 200 RPM 约 99 Hz 数据中的约 6.24 阶峰在更高采样率正反向测试中不再是主峰，因此不作为确定机械阶次。

新增正反向统计：

| 项目 | +200 RPM | -200 RPM |
| --- | ---: | ---: |
| ESO 平均转速 | +199.74 RPM | -199.60 RPM |
| ESO 速度标准差 | 7.06 RPM | 7.14 RPM |
| 旧差分速度标准差 | 7.95 RPM | 8.01 RPM |
| Iq Ref 标准差 | 0.159 A | 0.161 A |

正反向按同一机械角位置对齐后，波形不是简单重合或反相。

当前证据只能确认：

> 存在稳定的约 12 / 14 阶机械角相关速度与电流成分。

尚不能唯一判断来源是：

- MT6835 / 磁铁 / 安装造成的周期角度误差；
- 电机齿槽或其他真实周期转矩纹波；
- 二者叠加。

由于控制闭环、采样相位和 SWD 采样率都会影响相位，本版本不针对 12/14 阶现象新增 observer/filter/notch/补偿逻辑。

## 7. Host 速度语义修复

测试发现 Disable 后 Mechanical ESO 停算，但内部 `State.Wm` 会保留最后值。

`Motor_Wm_Get()` 已修正：

- `DISABLED` 时 Host `ωm` 返回 0；
- Servo 运行时 `PARAM_RUN_WM` 继续表示控制实际采用的 ESO 机械速度；
- ESO 内部状态保留本身不视为运行反馈。

该行为已在多组 Speed 测试结束后确认。

## 8. 新增 Host diagnostics

为后续 NMIXX Control Tuning / Plot 使用，增加只读诊断量：

| Parameter | Label | 含义 |
| --- | --- | --- |
| `PARAM_MECH_ESO_THETA` | `θm ESO` | Mechanical ESO 连续机械角度 |
| `PARAM_MECH_ESO_WM` | `ωm ESO` | Mechanical ESO 机械速度 |
| `PARAM_MECH_ESO_TD` | `Td` | Mechanical ESO 扰动/负载转矩估计 |
| `PARAM_MECH_ESO_ERROR` | `ESO Error` | 编码器位置与 ESO 角度状态误差 |
| `PARAM_ENCODER_WM` | `ωm Encoder` | 原编码器差分 + LPF 速度 |

这些量均为只读诊断，不改变控制路径。

`PARAM_RUN_WM` 仍保留“当前控制实际使用的用户机械速度反馈”语义。

## 9. 回归工具

新增：

```bash
python3 tools/sensored_speed_test.py \
    --port /dev/ttyACM1 \
    --speed-rpm 200 \
    --hold-seconds 5 \
    --zero-seconds 3 \
    --current-limit 1.0 \
    --speed-bw 50 \
    --eso-bw 100 \
    --run
```

标准序列：

```text
prepare
  -> Enable
  -> Run
  -> +target hold
  -> zero
  -> -target hold
  -> zero
  -> Stop
  -> Disable
```

工具记录 Host 侧低速率 telemetry：

- `ωm Ref`;
- active `ωm`;
- `ωm ESO`;
- `ωm Encoder`;
- `Iq Ref`;
- `Iq`;
- `ESO Error`;
- `Td`.

此工具用于回归和 Control Tuning 前期验证，不替代高采样率 Plot / SWD / 专用阶次分析。

## 10. 当前结论

Mechanical ESO V1 当前可保留：

- 20 kHz 运行路径已建立；
- 正反方向平均速度跟踪正确；
- 零速可稳定；
- Speed Loop 使用 ESO `Wm_hat` 后无持续发散或保护故障；
- ESO BW 100~200 Hz 扫描没有解决现有周期速度纹波；
- 约 12/14 阶周期成分来源暂不下结论；
- 不因该未决问题继续扩大 observer/control 架构。

后续如需要定位 12/14 阶来源，应优先使用更高采样率和独立机械角度参考，而不是继续在当前 ESO 上叠加补偿算法。
