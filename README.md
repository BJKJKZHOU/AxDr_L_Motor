# AxDr_L_Motor

基于 AxDrive-L 开发板的 PMSM/FOC 控制工程，运行于 STM32G474RET6。

## 当前功能

- 20 kHz 三相电流采样、dq 电流环与 SVPWM
- Rs/Ls 和永磁体磁链辨识
- 磁链 Observer 与 PLL 转速/角度估计
- `ALIGN → I/F → Observer` 无感启动接管

## 默认电机

- 32 极外转子电机，极对数 `Pp = 16`
- `Rs = 0.08471736 Ω`
- `Ld = Lq = 17.836 µH`
- `Flux = 0.0031835556 Wb`

正常运行直接使用固件当前参数。参数辨识是独立流程，只有明确执行
`IDENT_APPLY` 后才会更新 RAM 中的电机参数。

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
- `tools/`：构建、辨识和实机测试脚本

## Project status

This project is under active development. Some functions have been validated on hardware, while other control paths and operating ranges remain experimental.

## Safety

This firmware can directly drive a motor power stage. Verify the target hardware, current limits, PWM configuration, motor parameters, and protection settings before energizing the inverter. Hardware test scripts are not a substitute for independent protection and safe test procedures.

## License

Project-owned source code and documentation are licensed under the Apache License 2.0 unless a file states otherwise.

Third-party components, including STM32Cube, Azure RTOS ThreadX, and USBX sources, remain under their respective licenses. See `THIRD_PARTY_LICENSES.md` and the license files shipped with those components.
