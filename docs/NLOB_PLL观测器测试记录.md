# NLOB + PLL 观测器测试记录

更新时间：2026-10-06
分支：`feat/sensorless-open-loop`（第 1～6 节历史记录）；`feat/jb-identification`（第 7 节）；`fix/sensorless-speed-loop`（第 8 节）

最新无感加减速及 IF 双向切换实测见第 8 节。第 1～6 节的 Shadow 与 Takeover 门槛保留为历史记录。

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

<a id="sensorless-accdec-20261006"></a>

## 8. 2026-10-06：无感转速加减速与 IF 双向切换

固件：04807f0，已重新下载当前 Release ELF 并校验。无控制代码修改，未写参数 Flash。

### 8.1 工况与采集

- 指令：0 → 1000 RPM（12 s）→ 150 RPM（8 s）→ STOP。
- RAM 临时用户限速 1200 RPM，加减速度 100 rad/s²；测试后恢复原值。
- 7 极对，Flux = 0.00228119478561 Wb；电流限值 2 A，IF 电流 1 A。
- 电流环 2000 Hz，速度环 30 Hz，编码器机械 ESO 100 Hz；无感磁链观测器 200 Hz、PLL 50 Hz。
- FAST 20 kHz，NORMAL 1 kHz；SWD 状态间隔中位数约 42 ms，各字段逐个读取，不能视为严格同步快照。

### 8.2 切换规则与本次阈值

启动时按母线电压计算，单次运行中不更新：

`We_enter = 0.10 × (Vbus / √3 × 0.95) / Flux`

`We_exit = 0.05 × (Vbus / √3 × 0.95) / Flux`

机械 RPM = `We × 60 / (2π × Pp)`。

本次 RAM 阈值：352.28305 / 176.141525 电气 rad/s，即 480.579 / 240.290 RPM。

加速：指令绝对值大于进入阈值、IF 频率到达阈值后，IF 暂停继续升频并等待观测器稳定。滤波后的 PLL 与 IF 电气转速差 ≤ 5 rad/s、PLL 误差绝对值 ≤ 0.08、磁链幅值在模型值的 80–120%，连续 200 ms 满足后混合角度 150 ms；随后速度环接管，Id 以 5 A/s 归零。

减速：PLL 速度绝对值 ≤ 退出阈值时，用当前 PLL 角度与速度初始化 IF，立即退出速度闭环；电流参考在 150 ms 内过渡到 IF。此方向没有连续 200 ms 稳定判据。IF 加减速共用其 Acc，本次机械值约 71.37 rad/s²。

### 8.3 结果

两个方向都完成切换，STOP 正常结束。NORMAL 统计取 1000 RPM 阶段末约 3.4 s、150 RPM 阶段末约 3.4 s；原始编码器角度差分取对应阶段末 60000 个 FAST 样本（约 3 s），使用 10 ms 窗口。

| 稳态区间 | 编码器 ESO 平均 / 标准差 | 无感 PLL 平均 / 标准差 | 原始编码器角度 10 ms 差分 |
| --- | --- | --- | --- |
| 1000 RPM 闭环 | 999.934 / 4.684 RPM | 999.968 / 2.617 RPM | 平均 999.952，范围 992.922–1006.496 RPM |
| 150 RPM IF | 149.963 / 17.569 RPM | 150.019 / 15.241 RPM | 平均 149.988，范围 111.345–181.628 RPM |

减速退出闭环附近，ESO 最低约 48.9 RPM、PLL 最低约 56.4 RPM；这些是观测值，不能视为无误差的瞬时真实速度。低速稳态的原始编码器角度差分也有明显波动，证明现象不只是 ESO 估计波动。150 RPM 时 IF 内部频率恒定，Iq 参考固定 1 A；此时接口返回的 IF 速度是生成旋转磁场的频率，不是实际转子速度闭环反馈。

相电流峰值 2.90 A，发生在 150 RPM IF 运行末段；并非切换瞬间。2 A 是控制电流限幅，实测瞬态仍存在超出限值的尖峰，本次未满足保护触发条件。峰值记录来自三相 Ia、Ib、Ic=−Ia−Ib。

编码器 CRC/错误/漏采、快环 Deadline_Miss 均无新增；累计快环最大 7680 周期（160 MHz 下 48.0 µs）。USB FAST/NORMAL 序号丢帧均 0，但固件 Plot 丢弃计数分别增加 12/11，不能据此声称采样链路完全无丢弃。

结束状态：DISABLED、PWM MOE=0、Ready=1、Fault/Error/Trip=0、速度目标=0。完整持久化参数在 RAM 中与测试前 Flash 参数记录一致；未执行参数保存。

### 8.4 判断

本次证明当前参数下 1000 RPM 无感闭环能够启动、运行并减速退出；单次试验不能代表全部负载与转速范围。

退回 IF 后失去速度闭环，固定 IF 电流与开环频率不能直接抑制实际转速振荡。数据支持优先分析 OBS→IF 交接及低速 IF 行为，但尚不足以将根因归结为某个参数或交接公式。

本机原始数据目录：`/tmp/eso_continuous_20261006/sensorless_accdec/`。
测试脚本：`/tmp/eso_continuous_20261006/sensorless_accdec.py`。
原始文件：normal.raw、fast.raw、fast_frames.csv、timing.json；统计：analysis.json；状态切换采样：transitions.json；全过程与恢复信息：result.json。
