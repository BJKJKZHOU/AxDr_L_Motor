# AxDr_L_Motor

基于 AxDrive-L 开发板的 PMSM/FOC 控制工程，运行于 STM32G474RET6。

## 当前功能

- 20 kHz 三相电流采样、dq 电流环与 SVPWM
- Rs/Ls 和永磁体磁链辨识
- 磁链 Observer 与 PLL 转速/角度估计
- `ALIGN → I/F → Observer` 无感启动接管

## 硬件

本工程基于 AxDr_L(AxDrive-L) 硬件平台进行开发和实机验证。

相关硬件开源与参考工程：

- [AxDr_L 硬件](https://oshwhub.com/lylssy/foc_driver) (GPL 3.0)
- [AxDr_L 软件参考](https://github.com/disnox/AxDr_L)

本仓库并非上述硬件项目的官方固件仓库。

## 上位机与通信协议

本工程使用 AxDr CAN-FD 应用层协议，USB CDC 同样承载 CAN-FD 风格的消息帧。

VOFA+ 上位机的 JustCANFD 协议支持与协议说明维护在：

- [Vodka / JustCANFD](https://github.com/BJKJKZHOU/Vodka/tree/master/dataengines/justcanfd)

`tools/` 下的 Python 脚本使用同一套协议，主要用于自动测试、参数辨识和新电机 commissioning。

## 依赖

ThreadX 与 USBX 使用 Git 子模块管理：

- `ThirdParty/Eclipse/threadx/`：Eclipse ThreadX 6.5.1
- `ThirdParty/Eclipse/usbx/`：Eclipse USBX 6.5.0 portable core 与 CDC ACM device class
- `ThirdParty/ST/usbx_stm32_dcd/`：STM32 USBX device-controller adaptation

克隆后需要初始化子模块：

```bash
git submodule update --init --recursive
```

STM32CubeMX 重新生成工程时可能会在 `Middlewares/` 下产生中间件副本。该目录不参与实际构建，项目通过 CMake 重映射使用 `ThirdParty/` 下的依赖。

## 构建

使用 VS Code 的 `STMicroelectronics.stm32-vscode-extension` 工具链：

```bash
cmake --preset Release
cmake --build --preset Release
```

## 目录

- `Motor/`：PWM、采样、电流环和运动控制
- `Identification/`：Rs/Ls 与磁链辨识
- `Observer/`：磁链 Observer 和 PLL
- `Sensorless/`：I/F 启动与 Observer 接管
- `Comm/`：USB 控制协议和 Plot
- `User/`：电机与控制参数
- `ThirdParty/`：外部依赖与 STM32 USBX DCD 适配层
- `tools/`：构建、辨识和实机测试脚本

## Project status

This project is under active development. Some functions have been validated on hardware, while other control paths and operating ranges remain experimental.

## Safety

This firmware can directly drive a motor power stage. Verify the target hardware, current limits, PWM configuration, motor parameters, and protection settings before energizing the inverter. Hardware test scripts are not a substitute for independent protection and safe test procedures.

## License

Project-owned source code and documentation are licensed under the Apache License 2.0 unless a file states otherwise.

Third-party components, including STM32Cube, Eclipse ThreadX, Eclipse USBX, and the STM32 USBX device-controller adaptation, remain under their respective licenses. See `THIRD_PARTY_LICENSES.md` and the license files shipped with those components.
