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
