# LCD 显示与实时性验证记录

日期：2026-10-11。硬件：AxDrive-L V1.3，STM32G474RE。

## 1. 实装屏幕与显示方向

硬件来源为 `文档/开发板工程/ProDoc_1.axdr_v1.3_2026-10-02.epro2`。
其中 `IPSLCD1` 标注 `TFT_1.14_135x240_13pin`；实装面板为 135×240，
并且横向安装。原软件的 240×240、零偏移不适用。

最终配置：

| 项目 | 设置/验证 |
|---|---|
| 物理像素 | 135×240 |
| 逻辑坐标 | 横屏 240×135，X=0…239、Y=0…134 |
| 控制器/颜色 | ST7789、RGB565，高字节先传 |
| MADCTL | `0xA0`（MY、MV） |
| 显存偏移 | X=40、Y=52；窗口终点为 X=279、Y=186 |
| SPI3 | Mode 0、8 bit、170 MHz / 8 = 21.25 Mbit/s |
| 视觉确认 | 现场目视确认文字正向，TL/TR/BL/BR 四角及四边完整 |

135×240 面板在 240×320 控制器显存中的居中偏移随旋转方向变化，奇数宽度导致
52/53 的区别。配置参考 [Adafruit ST7789 驱动的 init/setRotation](https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7789.cpp)，
最终方向以本次实板反馈为准：`0x60` 横屏上下颠倒，改为 `0xA0` 后确认正常。

顶部信息压缩为两行、32 像素高：第一行模式/单位/状态，第二行 REF/FBK 及数值。
下方 240×103（Y=32…134）全部显示五秒波形，删除底部 `-5S`、`NOW` 文字。
2 bit 缓存每行 60 字节；240 像素宽度对应 15 个完整列块，传输块为 16×8，底部块裁切到 7 行。
全屏恰好 64,800 字节，17 次矩形 DMA，避免漏掉第 135 行或写出屏幕边界。

## 2. 背光黑屏原因

初次运行时 DMA 计数正常，但现场观察到屏幕和背光均黑。
读取 GPIO 确认 PB7 为高电平；重新核对原理图发现 Q4 为 **S8550 PNP**，
PB7 经过 R21 驱动基极，R22 将基极上拉到 VCC。
因此 PB7 **低电平开背光，高电平关背光**，原代码和引脚文档写反。

通过 SWD 拉低 PB7 后，现场目视确认背光点亮、四角可见。
已同步修正 `.ioc` 的 PB7 上拉/默认高、CubeMX 生成 GPIO，以及 ST7789 初始化：
初始化期间保持高，清屏和 DISPON 完成后拉低。重启后自动点亮已实板确认。

## 3. CubeMX 实际生成结果

工具：本机 STM32CubeMX 6.18.1、数据库 DB.6.0.181、STM32Cube FW_G4 V1.6.3。
将根目录 `.ioc` 和现有 `Core/AZURE_RTOS/USBX/cmake` 复制到临时目录，执行：

```text
config load /tmp/<副本>/AxDr_L_Motor.ioc
project generate /tmp/<副本>
exit
```

使用 `STM32CubeMX -q <脚本>`，生成日志返回 `OK`。随后审查并同步相关结果：

- `Core/Src/spi.c`：`hdma_spi3_tx`、`DMA2_Channel1`、`DMA_REQUEST_SPI3_TX`、
  Memory→Peripheral、字节对齐、MemInc、Normal、Low priority、`__HAL_LINKDMA`；
  PC10/PC12 变为 Very High speed。
- `Core/Src/dma.c`：DMA2 时钟使能，DMA2 Channel1 IRQ 优先级 8 并使能。
- `Core/Src/stm32g4xx_it.c` / `Core/Inc/stm32g4xx_it.h`：DMA2 Channel1 中断入口、
  声明及 `HAL_DMA_IRQHandler(&hdma_spi3_tx)`。
- `Core/Src/gpio.c`：LCD/外部 SPI3 片选默认高，PB7 上拉且默认高。
- `AZURE_RTOS/App/app_azure_rtos_config.h`：应用池 3 KB，与 `.ioc` 一致。

上述生成文件与临时副本逐字节核对；唯一保留差异是原有四个中断的 `FAST_CODE`：
DMA1 Channel5、ADC1_2、TIM1_UP_TIM16、TIM1_CC。CubeMX 会删除函数定义前的此属性，
同步时恢复，以保持电机实时路径的 CCMRAM 放置。未覆盖项目级构建配置及其他外设代码。

上板还确认了 `hspi3.hdmatx` 关联、DMA2 RCC 时钟、NVIC 使能/优先级 8，
以及 DMAMUX 的 SPI3 TX 请求号 15。完成帧计数持续增加，实际走到 DMA 完成回调。

## 4. 栈与内存

原 Panel 栈仅 512 B。本次先调整为 1024 B，但首次烧录停在
`App_ThreadX_Init` 失败分支：2 KB 应用池容不下 Motor/Panel 两个 1024 B 栈、
命令队列及分配开销。将 `.ioc` 的 `TX_APP_MEM_POOL_SIZE` 改为 `3*1024`，
再经 CubeMX 生成后启动正常。

最终横屏、初始化、测试图和正反转刷新工况下：

- 分配的 Panel 栈：1024 B；ThreadX 控制块报告有效大小 1020 B。
- `tx_thread_stack_highest_ptr` 与栈底 `0xEF` 填充扫描均得到峰值 **640 B**，
  有效剩余 **380 B**。旧 512 B 容量不足。
- 原有 `TX_ENABLE_STACK_CHECKING` 保持开启。
- Clang Release（两行布局）：RAM 52,840 B / 96 KB，CCMSRAM 8,824 B / 32 KB，Flash 112,220 B / 496 KB。
- GCC Release（两行布局）：RAM 53,264 B，CCMSRAM 8,912 B，Flash 115,776 B。

栈峰值属于本次 Clang Release 实测；GCC 完成构建验证，未作为上板镜像。

## 5. 刷新率和 FOC 运行态复测

本节六组数据对应顶部压缩前的横屏布局；两行布局的增量检查见第 7 节。

供电约 24.8 V；有感 SPEED 模式，目标 ±200 RPM，Iq 参考限流 1 A，
速度带宽 50 Hz、ESO 带宽 100 Hz、加减速度 100 rad/s²。
编码器校准有效、保护状态为零。每组先稳定 1 s，随后采集约 5 s，
正反转之间归零 1 s。运行中使用 USB 参数轮询检查保护/编码器和电流。

CPU/TIM1 为 170 MHz，TIM1 ARR=4250，中心对齐周期 **50 μs / 20 kHz**。
所有 SWD 采样均在 MCU 连续运行下进行；GDB 只离线解析 ELF 地址。
每组重新启动现有 Fast_Snap 窗口，ISR 在第 2000 周期清 Max/错误计数，
第 4000 周期冻结快照。下表使用覆盖本次观测窗的 **Fast_Time 累计最大值**，
而非只覆盖 100 ms 的冻结快照。

| LCD 工况 | 方向 | 完成帧/约5s | 完成更新率 FPS | FOC 总耗时 Max μs | ADC ISR Max μs |
|---|---|---:|---:|---:|---:|
| 暂停渲染/传输 | 正转 | 0 | 0 | 42.041 | 12.894 |
| 暂停渲染/传输 | 反转 | 0 | 0 | 42.176 | 12.871 |
| 正常局部刷新 | 正转 | 152 | 30.396 | 42.429 | 13.529 |
| 正常局部刷新 | 反转 | 151 | 30.193 | 42.724 | 13.059 |
| 每帧全屏刷新 | 正转 | 101 | 20.195 | 42.318 | 12.941 |
| 每帧全屏刷新 | 反转 | 101 | 20.194 | 42.153 | 13.106 |

六组 `Deadline_Miss=0`、`Enc_Miss=0`、`Enc_Late=0`；采集前后状态均为 RUN。
最坏 7263 cycles / 170 MHz = **42.724 μs**，距 50 μs 周期 **7.276 μs**。
LCD 关闭组最大为 42.176 μs，正常刷新组最大增加约 0.547 μs。
这些是有限测试窗中的观测最大值，不能当作所有工况的最坏执行时间上界。

计数口径是**有像素变化且最后一块 DMA 已完成**的整次屏幕更新；无变化的静态画面
不增加帧数。它不是面板内部扫描频率。5 s 窗口存在一帧计数量化，故局部刷新结果
约为 **30.3 FPS**，与 33 tick 调度一致。全屏纯 SPI 字节时间理论下限为
`64800*8/21.25e6 = 24.395 ms`，叠加渲染、像素转换、矩形命令、线程调度与 FOC 抢占后，
实测约 **20.2 FPS**；本轮全屏完成延迟最大 50 ticks。

局部刷新传输约 239～249 KB/s，全屏约 1.309 MB/s。
低速遥测平均速度约 +199～201 / −197～−199 RPM；各组速度标准差约 6.7～8.4 RPM。
全组低速轮询观察到的最大 |Iq| 为 1.251 A（参考限流不等于瞬时实测电流限幅）。
本轮验证显示负载对有感低速 FOC 的影响；没有覆盖满载高速、无感和高速 USB Plot 并发。

完成后已逐项读回确认测试参数恢复，电机为 DISABLED，Error/Trip 均为零，
LCD 恢复正常波形模式，未保存临时测试参数到 Flash。

## 6. 复测与构建

已通过 ST Arm Clang / GNU Arm GCC Release 构建。
从实际 LCD 函数提取的主机测试用 ASan/UBSan 检查了缓存行寻址、64,800 字节全屏覆盖、
底部 7 行裁切、最后一块 DMA 完成后计数、无变化帧和边框绘制，均通过。

新增 `tools/lcd_foc_check.py`，要求 OpenOCD 已启动且 ELF 与板上镜像完全一致：

```bash
openocd -f openocd.cfg
python3 tools/lcd_foc_check.py --mode pause --seconds 5
python3 tools/lcd_foc_check.py --mode normal --seconds 5
python3 tools/lcd_foc_check.py --mode full --seconds 5
python3 tools/lcd_foc_check.py --mode pattern --seconds 1 --keep-mode
python3 tools/lcd_foc_check.py --mode normal --seconds 1 --keep-mode
```

该工具不使能电机，仅切换显示诊断模式、重开 FOC 计时窗口并读数；运行态测试需先
通过现有控制工具建立工况。默认退出恢复原显示模式，`--keep-mode` 用于目视检查。
SWD 各字段读数不是原子快照，不用于重建 20 kHz 电流波形。

临时生成日志、六组 JSON 和电机轮询 CSV 位于 `build/lcd-check-20261011/`；
按项目归档约定，正式记录只保留本专题条件、数值和结论。

## 7. 两行信息与全高波形布局

顶部布局压缩为 32 像素内的两行，删除底部时间文字。
波形区由 240×56 扩大为 **240×103**，覆盖 Y=32…134，网格间隔同步调整。
模式、单位、状态位于首行；REF/FBK 标签和数值位于第二行，均使用一倍字号。

Clang/GCC Release 构建通过，Clang 镜像已烧录。电机保持 DISABLED，3 s 全屏
传输检查完成 65 帧（约 21.66 FPS），DMA2 时钟/中断及请求号验证通过。
Panel 栈峰值仍为 640 B；该未使能工况快环最大 31.994 μs，Deadline/编码器错误均为零。
检查结束已恢复正常局部刷新。本次布局调整没有重新执行第 5 节的电机正反转六组测试。
临时数据：`build/lcd-check-20261011/compact-full-disabled.json`。

## 8. KEY3 四模式循环

实物第 4 键为 KEY3/PC7。CubeMX 6.18.1 从更新的 `.ioc` 在临时工程实际生成
`KEY3_Pin`/`KEY3_GPIO_Port` 及 GPIO 分组，保持输入上拉配置。新增面板扫描和
`MOTOR_CMD_MODE_NEXT` 队列命令，电机线程通过 `Parameter_Write` 切换：
TORQUE → SPEED → POSITION → SENSORLESS_SPEED → TORQUE。OPEN_LOOP、IDENT、PHASE_SEARCH
按一次返回 TORQUE；未知模式值不切换。
仅 DISABLED 接收切换，目标值保持，FOC 中断代码未改。

ST Arm Clang / GNU Arm GCC Release 均构建通过，Clang 镜像已烧录。
通过 SWD 临时将 PC7 设为开漏输出，仅拉低模拟按下、释放为高阻，不驱动高电平；
MCU 持续运行，通过真实按键扫描、消抖、队列和参数入口验证，并经 USB 读回模式/状态。
这验证了电气输入到软件的链路，不替代人工操作实体按键的触感与接触可靠性检查。

验证通过：

- 连续两轮四模式循环；每次按下、持续保持、释放分别读回，长按不连跳。
- OPEN_LOOP、IDENT、PHASE_SEARCH 分别按一次返回 TORQUE。
- ENABLED、RUN 时按下保持原模式；在 ENABLED 按住后取消使能，不补触发切换。
- 模式循环前后转矩、速度、位置目标完全保持。
- 从实际 `Key_Press` 提取的主机测试检查输入抖动、长按和释放后再次按下，
  ASan/UBSan 检查通过。

母线约 24.8 V，以 TORQUE、零转矩目标、1 A 参考限流进入 RUN，分别采样 3 s：

| 显示负载 | 完成帧数 | 刷新率 | 快环最大 | Panel 栈峰值 | Deadline / Enc Miss / Enc Late |
|---|---:|---:|---:|---:|---|
| 正常局部刷新 | 90 | 29.99 FPS | 39.959 μs | 640 B | 0 / 0 / 0 |
| 强制全屏刷新 | 58 | 19.33 FPS | 39.959 μs | 640 B | 0 / 0 / 0 |

DMA2 时钟、中断优先级 8、SPI3 TX 请求号 15 与 DMA 链接检查通过。
本轮运行态计时为零转矩工况，没有验证无感启动或无感带转。
结束后恢复 PC7 输入配置、测试参数和正常显示，电机 DISABLED/TORQUE，Error/Trip 为零，
未保存临时参数到 Flash。临时数据：`build/key3-check-20261011/`。

## 9. 审查后的精简

删除电机命令分支中与参数入口重复的禁用检查，保留面板接收条件和参数入口校验。
显式列出合法模式回环，未知模式忽略该命令并继续处理队列。
强制全屏更新跳过缓存比较；按固定 240 像素宽度简化行步长与列块计算，删除末列裁切，
保留 135 行高度所需的底部裁切。

Clang/GCC Release 构建通过。抽取实际 LCD 函数、模式切换分支和参数入口状态检查的
主机 ASan/UBSan 回归通过：64,800 字节完整覆盖、底部 7 行、DMA 完成计数、无变化帧、
强制全屏不比较缓存，以及 7 种合法模式 × 3 种状态、未知值跳过和两轮循环。
本次精简未重新烧录或上板测试，第 8 节板上数据对应精简前版本。
