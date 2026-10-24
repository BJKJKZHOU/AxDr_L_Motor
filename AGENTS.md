# Project Instructions for Codex

Before modifying C/C++ source in this repository, read `docs/CODE_STYLE.md` and follow it.

## 文件变更规范（必须遵守）

 - 每次修改前，需要说明原因，变更计划，取得同意后才能修改。

## 文档与测试记录

- 以实测数据、结论和参数调整依据为主，保留影响结果解释的必要条件；重复测量和对比优先用表格。
- 正常结束状态、零故障、零丢帧、构建通过及参数恢复等常规信息默认省略；出现异常或本身就是测试对象时记录。
- 不记录无归档用途的固件哈希、冗长临时文件清单及过时命令；必要的数据和复现入口简要保留。
- 避免重复的“不能据此”“不代表”等泛化说明，直接写明观察结果、具体差异和待定位问题，区分事实与推测。

## Core rules

- Preserve the existing motor-control behavior and timing unless the task explicitly requires a behavior change.
- Prefer Simulink/control-engineering style naming and signal flow over enterprise-software abstractions.
- Use short domain names such as `Ia`, `Iq`, `Ud`, `Theta_e`, `Wm`, `PID_Run`, `Motor_Ctrl`, `Enc_Cal`.
- Do not introduce long generic software names when a short control-domain name is clear.
- Do not add lifecycle boilerplate such as `Init`, `Reset`, `Clear`, `Start`, `Stop`, or `DeInit` merely for interface symmetry.
- A function should exist only when it has an independent, stable algorithm or system meaning. Do not wrap one or two obvious assignments only to reduce duplication.
- Keep temporary calculations local. Store values in structs only when they must survive across cycles, are externally observed, or represent a real module interface.
- Do not duplicate values that can be directly derived from existing data or read from hardware.
- Do not add extra state machines or status fields when the state can be inferred from existing servo state, control mode, feedback, or hardware registers.
- Do not create Target/Cmd/Ref copies unless they correspond to real processing stages that exist in the implementation.
- Algorithm objects may use `Para`, `Sig`, and `State`; keep those parts in the same algorithm header.
- Split files by algorithm family or real functional responsibility, not by struct layer.
- In ADC/PWM hot paths, prioritize deterministic execution and direct signal flow. Direct register access, local variables, and expanded control formulas are acceptable.
- Do not add wrapper/manager/service/interface layers unless the current task demonstrates a concrete need.
- Keep changes narrow. Do not perform unrelated cleanup or architectural refactoring.
- Comments should explain non-obvious control choices, units, signs, timing, and assumptions; do not narrate obvious assignments.

## Before adding an abstraction

Ask internally:

1. Does this solve a problem that exists now?
2. Does it represent a real and stable control/system concept?
3. Is the code clearer without it?
4. Is it only wrapping a few assignments?
5. Is it only being added for symmetry or future speculation?
6. Does it duplicate data already available elsewhere?

If the abstraction is mainly justified by symmetry, generic software convention, or possible future use, do not add it.

## Change review

Before finishing a change, check the diff for:

- unnecessary new structs or fields;
- unnecessary `Init/Reset/Clear/Start/Stop` helpers;
- duplicated physical quantities;
- long software-style names replacing concise control notation;
- new layers in the current-loop ISR path;
- unrelated refactors.
