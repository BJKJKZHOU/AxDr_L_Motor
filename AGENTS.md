# Project Instructions for Codex

Before modifying C/C++ source in this repository, read `docs/CODE_STYLE.md` and follow it.

## 文件变更规范（必须遵守）

 - 每次修改前，需要说明原因，变更计划，取得同意后才能修改。

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
