# NLOB + PLL 观测器测试记录

更新时间：2026-08-22
分支：`feat/sensorless-open-loop`

## 1. 记录范围

本文记录 NLOB + PLL Shadow 的符号、平均速度、角度关系、磁链模型误差、
`2fe` 纹波及 Observer Takeover 门槛。Observer 计算耗时归入
[20 kHz 快环时序调试记录](20kHz快环时序调试记录.md)。

## 2. Shadow 基本结果

`3073c88` 中 NLOB 和 PLL 在 I/F 运行期间执行，但 FOC 电角度仍由 `Theta_IF`
提供，Observer 没有接管。两次正转和一次反转的 HOLD 最后 `2 s` 统计为：

| 测试 | We_obs 均值 | RMS 纹波 | Theta_obs - Theta_IF | PLL_Err RMS | Flux_Err / Flux² | 主纹波 |
|---|---:|---:|---:|---:|---:|---:|
| 正转 1 | `+119.9820 rad/s` | `6.5558 rad/s` | `+81.088 deg` | `0.016989` | `-3.163%` | `37.98 Hz` |
| 反转 | `-120.0336 rad/s` | `5.1410 rad/s` | `-81.157 deg` | `0.013732` | `-3.013%` | `37.98 Hz` |
| 正转 2 | `+119.9829 rad/s` | `4.4857 rad/s` | `+81.110 deg` | `0.012382` | `-3.156%` | `37.98 Hz` |

```text
NLOB 状态积分       PASS
正反转符号          PASS
PLL 平均速度         PASS
磁链幅值            基本 PASS
角度连续性          PASS
Observer takeover   NOT READY
```

## 3. `+/-81 deg` 的正确解释

当前 I/F 使用 `Theta_IF` 作为 Park 角，且 `Id_ref=0`。Iq 对应的定子电流矢量本来就相对 Park 角旋转约 `+/-90 deg`：

```text
正转：Theta_Is ~= Theta_IF + 90 deg
      Theta_obs ~= Theta_IF + 81 deg
      Theta_Is - Theta_obs ~= +9 deg

反转：Theta_Is ~= Theta_IF - 90 deg
      Theta_obs ~= Theta_IF - 81 deg
      Theta_Is - Theta_obs ~= -9 deg
```

因此磁链与定子电流矢量的夹角幅值约为 `9 deg`。正反转高度对称支持坐标和方向符号正确，不支持“Observer 固定错 `90 deg`”的判断。

未来不能直接将 `Motor_Run.Theta_e` 切换为 `Theta_obs`。必须设计角度渐变和
Id/Iq 重投影，否则 Park 坐标会瞬间跳变约 `81 deg`。

## 4. Flux Error 定义

当前定义：

```text
Flux_Err = Flux_ref^2 - PsiAlpha^2 - PsiBeta^2
```

它是平方磁链误差，不是磁链幅值的直接百分比。例如
`Flux_Err/Flux_ref^2=-3.16%` 对应磁链幅值约偏高 `1.57%`。正反转偏差接近，更像固定模型误差，不是观测器发散。

## 5. `2fe` 专项定位

在 `abs(We_IF)=120 rad/s` 时：

```text
fe  ~= 19.10 Hz
2fe ~= 38.20 Hz
```

实测主峰为 `37.981 Hz`，且在 PLL 前后的多个信号中同时存在：

| 信号 | 正转峰值 | 反转峰值 | 含义 |
|---|---:|---:|---|
| `We_obs` | `10.8938 rad/s` | `5.8812 rad/s` | 速度纹波 |
| `PLL_Err` | `0.026488` | `0.014303` | PLL 输入已含 `2fe` |
| `DeltaTheta` | `3.840 deg` | `2.073 deg` | Observer/I-F 角差纹波 |
| `U_Mag` | `0.042808 V` | `0.020859 V` | 指令电压中二次项最强 |
| `Flux_Mag` | `10.638 uWb` | `2.499 uWb` | PLL 前已有确定性二倍频 |

alpha-beta 椭圆拟合的最强特征是：`Ualpha/Ubeta` 指令的二次项在正反转之间相位差约
`181.9 deg`，幅值相差约两倍。

当前排查优先级：

1. 逆变器死区、MOS/二极管压降和电流方向相关的端电压误差；
2. alpha-beta 电流比例或偏置不对称；
3. `Ld != Lq`、饱和或转子位置相关电感；
4. PLL 带宽对已存在相位纹波的传递。

`Motor_Run.Ualpha/Ubeta` 是指令电压，不是实测端电压，因此目前只能将逆变器非线性作为强线索，不能判定为唯一根因。

## 6. 当前门槛

- 冻结 Gamma、PLL 带宽和 NLOB 数学模型，不用调参掩盖 `2fe`；
- 先定位端电压与指令电压误差；
- 完成快环余量优化；
- 设计无扰角度过渡和负载角/电流指令重组后，再进入 Takeover。

完整采集表、正反转原始文件路径和频谱拟合结果保留在
[电机控制测试历史总记录](电机控制测试历史总记录.md)第 13～14 节。
