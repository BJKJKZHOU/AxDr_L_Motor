# NLOB + PLL 观测器测试记录

更新时间：2026-09-17
分支：`feat/sensorless-open-loop`（第 1～6 节历史记录）；`feat/jb-identification`（第 7 节）

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

<a id="jb-dft-pll-speed"></a>

## 7. 2026-09-16：J/B DFT 使用的 PLL 速度信号

J/B 在固定 Rs/Ls/Flux 下执行 `1～5 Hz` 扫频，原 DFT 使用
`Ident_PLL.State.We / Pp`。该量为 PLL 的积分状态，未包含比例通道。
仓库 `Observer/PLL.c` 中的角度更新为：

```c
We_Next = Pll->State.We + Pll->Para.Ki * Err * Ts;
Theta_Next = Pll->State.Theta + (We_Next + Pll->Para.Kp * Err) * Ts;
```

随后保存 `State.We=We_Next`、`State.Err=Err`。因此，在已完成一次 PLL 更新后，
`State.We + Kp * State.Err` 对应本次角度积分使用的电角速度。
本次仅将 J/B DFT 的机械速度采样改为：

```c
Wm = (Ident_PLL.State.We +
      Ident_PLL.Para.Kp * Ident_PLL.State.Err) /
     (float)Motor_Para.Pp;
```

速度环反馈仍使用 `State.We`，Handover、Observer/PLL 算法及参数均未修改。
此改动是选择 PLL 中已有的速度量，未添加经验延时或相位补偿。

在归一化相位误差、锁定附近的连续小信号近似下，以输入相位导数为速度输入：

```text
State.We 通道：       H_I(s)  = Ki / (s² + Kp·s + Ki)
角度积分速度通道：   H_PI(s) = (Ki + Kp·s) / (s² + Kp·s + Ki)
```

第二个通道包含比例项，在远低于 PLL 带宽的频率下可减小速度采样的相位滞后。
这只是 PLL 局部模型，不代表 Observer、电流测量及机械系统的完整频率响应。

固定参数实测中，修改前 2～5 Hz 计算出负 B，修改后五点均通过，
J 仍约 `1.65～1.74e-5 kg·m²`，B 均为正，总相位为 `85.85°～88.49°`。
该轮 1 Hz 的 B 偏低，每频点只有一次测试，不能据此宣称全部频率相关误差已消除。
这组 J/B 结果也不替代第 1～6 节历史 Shadow 与 `2fe` 专项问题的结论。

固定参数、修改前后完整五项结果、测试方法和 CSV 路径见
[辨识测试记录](辨识测试记录.md)第 8 节。

### 7.1 2026-09-17 重复性与激励幅值复测

保持修正后的 DFT 速度信号及相同 Rs/Ls/Flux，10% 激励下 1/2/3 Hz 各五次均通过。
三组 B 均值分别为 `12.26 / 13.44 / 13.43 × 10^-6 N·m·s`，
CV 分别为 `21.51% / 26.51% / 38.56%`。这组结果不支持“1 Hz 稳定在约 5e-6”，
也没有出现“2/3 Hz 很集中、只有 1 Hz 散布大”的特征。

随后仅将 3 Hz 激励幅值提高至 20%，五次均通过。与 10% 的 3 Hz 组相比，
B 均值由 `1.34262e-5` 变为 `1.28261e-5 N·m·s`，
样本标准差由 `5.17691e-6` 降至 `8.57662e-7 N·m·s`；
相位标准差由 `0.915668°` 降至 `0.142425°`。
结果支持增大激励有助于降低辨识散布，但分时测试尚未排除时间变化的影响，
不能将改善唯一归因于 Observer、PLL 或某种漂移机制。

据此将 J/B 默认值设为 `3 Hz、20%`，保留 Host 可调参数；
DFT 速度采样处补充实际相位推进速度的注释，控制反馈及观测器参数不变。
完整 20 次结果、统计口径和原始记录见
[辨识测试记录第 9 节](辨识测试记录.md#jb-repeat-amplitude)。
