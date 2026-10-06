# AxDr_L_Motor

[![Firmware Build](https://github.com/BJKJKZHOU/AxDr_L_Motor/actions/workflows/ci.yml/badge.svg)](https://github.com/BJKJKZHOU/AxDr_L_Motor/actions/workflows/ci.yml)

基于 **STM32G474 + AxDrive-L** 的 PMSM/FOC 电机控制工程。

当前包含电流/速度/位置控制、电机参数辨识、I/F 启动、磁链 Observer、PLL 和无感控制相关实现。部分功能仍处于实验和调整阶段。

## 主要内容

- **20 kHz FOC**：三相电流采样、Clarke/Park、dq 电流环、SVPWM 与 PWM 更新
- **Servo 控制(有感)**：Torque / Speed / Position 控制路径
- **运动轨迹**：速度斜坡与梯形位置轨迹
- **编码器反馈**：MT6816 / MT6835 SPI 编码器与相位标定
- **电机参数辨识**：Rs / Ls、永磁体磁链、转动惯量 J 与粘性阻尼 B 辨识
- **无感控制**：`ALIGN → I/F → Observer` 启动与接管
- **Observer**：非线性磁链 Observer + PLL 电角度 / 电角速度估计，机械 ESO 速度与扰动转矩估计
- **Open Loop**：用于 bring-up、测试和部分辨识流程
- **上位机通信**：USB CDC 承载 AxDr CAN-FD 风格应用层消息
- **Python 工具**：测试、数据采集、参数辨识和新电机 commissioning
- **双工具链构建**：ST Arm Clang 本地开发 + GNU Arm GCC GitHub CI

## 硬件平台

当前固件运行于 **STM32G474RET6**，主要基于 AxDrive-L 硬件进行开发和测试。

相关硬件和参考工程：

- [AxDrive-L 硬件](https://oshwhub.com/lylssy/foc_driver)
- [AxDrive-L 软件参考](https://github.com/disnox/AxDr_L)

本仓库并非上述项目的官方固件仓库。

## 快速开始

### 1. 克隆

推荐直接初始化 Git 子模块：

```bash
git clone --recursive https://github.com/BJKJKZHOU/AxDr_L_Motor.git
cd AxDr_L_Motor
```

如果已经完成普通 clone：

```bash
git submodule update --init --recursive
```

### 2. GNU Arm GCC 构建

需要 CMake 3.22 或更新版本、Ninja 和 `arm-none-eabi-gcc`：

```bash
cmake --preset gcc-release
cmake --build --preset gcc-release
```

GitHub Actions 使用同一套 GCC 构建路径，并生成 ELF / BIN / HEX / MAP 产物。

### 3. ST Arm Clang 构建

本地主要开发工具链为 ST Arm Clang：

```bash
cmake --preset Release
cmake --build --preset Release
```

## 工程与 CubeMX

`AxDr_L_Motor.ioc` 由 STM32CubeMX 维护 MCU、HAL、ThreadX 和 USBX 基础配置。

CubeMX 重新生成后，需要注意：

- `Core/`、`AZURE_RTOS/`、`USBX/` 和 `cmake/stm32cubemx/` 中包含 CubeMX 生成或维护的 glue code
- `Middlewares/` 下可能重新生成中间件副本，但该目录不参与实际构建
- 实际 ThreadX / USBX portable code 使用 `ThirdParty/` 下的依赖
- 根目录 `CMakeLists.txt`、`CMakePresets.json` 和项目级 `cmake/` 配置不应被 CubeMX 结果覆盖

重新生成工程后建议重新执行 ST Arm Clang 和 GNU Arm GCC 构建。

## 上位机与通信

固件使用 AxDr CAN-FD 风格的应用层协议，USB CDC 使用同一套消息结构。

VOFA+ 的 JustCANFD 协议支持和相关上位机代码维护在：

- [Vodka / JustCANFD](https://github.com/BJKJKZHOU/Vodka/tree/master/dataengines/justcanfd)

`tools/` 下的 Python 脚本也使用同一套协议与固件通信。

## 目录结构

```text
Algo/            基础数学、PID、Sin LUT、SVPWM
Motor/           ADC、PWM、Encoder、电流环和运动控制
Motion/          速度斜坡与梯形位置轨迹
Identification/  Rs/Ls、磁链与 J/B 辨识
Observer/        Flux Observer、PLL 与机械 ESO
Sensorless/      I/F 启动与无感控制集成
Comm/            USB、Protocol 与 Plot
Parameter/       参数定义、访问与生成文件
Storage/         Flash 与 NVS 参数存储
User/            电机和控制参数
BSP/             板级外设
ThirdParty/      ThreadX / USBX / Zephyr NVS / STM32 USBX DCD 等外部依赖
tools/           算法测试、实机采集、commissioning 与参数生成脚本
docs/            实机测试记录、设计记录和代码风格说明
```

设计与实测记录见 `docs/`，可从[测试与问题记录索引](docs/测试与问题记录索引.md)进入；源码风格见 [CODE_STYLE.md](docs/CODE_STYLE.md)。

## 第三方依赖

主要外部依赖包括：

- Eclipse ThreadX
- Eclipse USBX
- Zephyr NVS / CRC
- STM32Cube HAL / CMSIS
- STM32 USBX device-controller adaptation

ThreadX、USBX portable code 与 Zephyr 使用 Git 子模块管理，其中 Zephyr 仅编译 NVS / CRC 相关源码。各第三方组件继续遵循其原始许可证，不受本项目 Apache-2.0 许可证覆盖。

详细说明见 [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)。

## Safety

该固件可以直接驱动电机功率级。上电测试前必须确认目标硬件、电流限制、PWM 配置、电机参数和独立保护措施。

仓库中的参数、测试脚本和自动 commissioning 流程不能替代实际硬件保护与安全测试流程。

## License

项目自有源码和文档采用 [Apache License 2.0](LICENSE)。

STM32Cube、Eclipse ThreadX、Eclipse USBX、Zephyr、STM32 USBX DCD 等第三方组件继续遵循各自的许可证和版权声明，详见 [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)。
