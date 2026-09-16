# CCMRAM 执行时间优化与 Flux 测试记录

测试日期：2026-09-13  
目标：缩短 20 kHz 电流快环及 Flux 观测相关函数的执行时间，同时验证辨识流程仍能完成。

## 版本与优化操作

- 对照版本：`origin/main` 的 `47b5562`，含已合并的辨识流程；本次用相同的 ST Arm Clang 21.1.1 Release 工具链单独构建。
- 优化版本：从上述 `origin/main` 移植原 `feat/ccmram-fast-code` 的代码放置方案，本地提交为 `c49dfbb`、`a05b312`。没有修改控制公式、固定 I/F 电流策略或 Flux 流程定时。
- `c49dfbb`：将 `Current_Loop`、`PID_Run`、限幅和角度运算、`SinCos`、`SVPWM_Calc` 等快环热点放入 `.ccmram_text`。
- `a05b312`：将 `Flux_Observer_Run`、`PLL_Run`、I/F 运行与目标设置、无感运行和交接路径中的热点放入同一段。
- `FAST_CODE` 指定 CCMRAM 代码段；启动阶段通过 `.preinit_array` 在 `main()` 前把该段从 Flash 复制到 CCMRAM。优化版 Release ELF 的 `.ccmram_text` 位于 `0x10000000`，大小 `0x1070 = 4208 B`；对照版该段为 `0 B`。CCMRAM 总容量为 `32 KiB`。

原功能分支早于最新辨识流程，故这里移植代码放置改动，而非将旧分支的控制逻辑一并带入。两个版本均通过 Release 构建，板上固件烧录后均完成校验。

## 测量方法与条件

同一电机、供电约 `14.5 V`，极对数 `7`，辨识限流 `2 A`，固定 I/F 电流由 `--if-current 1` 下发。每轮先辨识一次 Rs/Ls 并应用到 RAM，再进行 Flux。使用不中断目标运行的 SWD 读取 DWT 周期计数；只取 `Flux` 观测等待阶段、辨识运行中、观测器工作且 PWM 开启的采样。时钟为 `160 MHz`，`1 μs = 160` 周期。`Fast_Cyc` 表示 TIM1 下溢到 ADC ISR 结束的总时间，`ADC_ISR_Cyc` 是 ADC ISR 时间；函数周期数由各自的计时字段读取。

完整 5+5 验证使用以下命令。1+1 轮次将正反次数各改为 `1` 并启用下文所述的 SWD 采样；对照版和首次优化版的单次超时为 `60 s`，低采样率复测及完整 5+5 为 `25 s`。缩短超时是为了避免失步后长时间维持 I/F；成功轮次约 `11.5 s` 即结束。

```bash
python3 tools/identification_test.py \
  --port /dev/serial/by-id/usb-STMicroelectronics_STM32_USB_Device_000000000001-if00 \
  --pole-pairs 7 --current-limit 2 --if-current 1 \
  --rs-ls-count 1 --apply-rl \
  --flux-forward-count 5 --flux-reverse-count 5 \
  --ident-timeout 25 --interval 0.5 --stop-on-error --run --apply-flux \
  --output /tmp/ccmram_full_flux_test.json
```

对照版和首次优化版均以 `20 Hz` 采集 SWD，计时字段每 5 个样本更新一次。首次优化版发生失步后，又以 `5 Hz` 采样进行成功复测。因此表中的中位数用于判断量级和方向，不代表无采样干扰时的最坏执行时间。

| 观测等待阶段指标 | 对照版 20 Hz | 优化版 20 Hz（失步轮） | 优化版 5 Hz（通过轮） | 对照至优化 5 Hz |
|---|---:|---:|---:|---:|
| `Fast_Cyc` | 9143 周期 / 57.14 μs | 7784 / 48.65 μs | 7807 / 48.79 μs | 缩短 14.6% |
| `ADC_ISR_Cyc` | 5153 / 32.21 μs | 3803 / 23.77 μs | 3819 / 23.87 μs | 缩短 25.9% |
| Flux 观测器 | 1202 / 7.51 μs | 781 / 4.88 μs | 781 / 4.88 μs | 缩短 35.0% |
| I/F 路径 | 420 / 2.62 μs | 198 / 1.24 μs | 212 / 1.32 μs | 缩短 49.5% |

两轮优化版数据接近，说明所选代码路径的执行时间确实下降。20 kHz 周期为 `50 μs`；这几次抽样中，优化版 `Fast_Cyc` 中位数低于一个周期，但累计最大值仍可能超过周期，不能据此宣称所有 Deadline 均已满足。单次 Flux 总耗时仍约 `11.5 s`，流程中的定时阶段不会因函数执行变快而等比例缩短。

## Flux 功能验证

| 固件与采样方式 | Rs / Ls | Flux 结果 | Flux 中值 | 最大相电流峰值 | 结果 |
|---|---|---|---:|---:|---|
| 对照版，SWD 20 Hz | `0.286193 Ω / 72.803 μH` | 正 1、反 1 | `0.002269856 Wb` | `2.498 A` | 2/2 PASS |
| 优化版，SWD 20 Hz | `0.281360 Ω / 88.598 μH` | 首次正转后停转，观测等待未完成 | — | — | 人工中止 |
| 优化版，SWD 5 Hz | `0.287919 Ω / 78.009 μH` | 正 1、反 1 | `0.002271316 Wb` | `2.497 A` | 2/2 PASS |
| 优化版，无持续 SWD | `0.282287 Ω / 63.714 μH` | 正 5、反 5 | `0.002268961 Wb` | `2.497 A` | 10/10 PASS |

完整 10 次优化版测试的 Flux 范围为 `0.002260729–0.002278234 Wb`，相对中值的最大偏差 `0.41%`，正反方向差 `0.08%`；单次耗时范围 `11.427–11.625 s`，中位数 `11.512 s`。10 次的 FAST/NORMAL 捕获丢失数均为 `0`。供对照，合并后的默认分支此前同参数 5 正转＋5 反转全部通过，Flux 中值 `0.002267144 Wb`，见 `build/Release/identification_test_20260913_165314.json`。

首次优化版 SWD 20 Hz 测试中，现场观察为“转起来后停了”；随后 I/F 指令速度到达上限约 `2199 rad/s`，反电势比值仍约 `0.03`，Flux 未进入完成阶段。测试在约 `57 s` 时人工中止，确认 PWM 关闭。该轮不能记作 Flux PASS，也不能单凭一次现象把失步归因于 CCMRAM；采样负载与启动条件均可能影响结果。后续低采样率 2/2 及无持续采样 10/10 通过，说明问题并非每次必现，但偶发启动风险尚未排除。完整测试结束后再次确认 `Motor_State=0`、TIM1 `MOE=0`。

## 原始记录

- 对照版 1+1：`/tmp/ccmram_baseline_test.json`，DWT 采样 `/tmp/ccmram_baseline_trace.csv`。
- 优化版首次失步：DWT 采样 `/tmp/ccmram_enabled_trace.csv`；该轮人工中止，未形成有效 Flux 结果。
- 优化版低采样率 1+1：`/tmp/ccmram_enabled_retry_test.json`，DWT 采样 `/tmp/ccmram_enabled_retry_trace.csv`。
- 优化版无持续采样 5+5：`/tmp/ccmram_full_flux_test.json`。

`/tmp` 中的原始文件是本机临时记录；本节与上表保留了复核结论所需的版本、条件及数值。快环计时字段的定义另见 [20 kHz 快环时序调试记录](20kHz快环时序调试记录.md)。
