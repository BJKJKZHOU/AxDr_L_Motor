# AxDr_L_Motor

**Language / 语言:** [简体中文](../README.md) | English

[![Firmware Build](https://github.com/BJKJKZHOU/AxDr_L_Motor/actions/workflows/ci.yml/badge.svg)](https://github.com/BJKJKZHOU/AxDr_L_Motor/actions/workflows/ci.yml)

**AxDr_L_Motor** is permanent-magnet synchronous motor (PMSM) field-oriented control (FOC) firmware for the AxDrive-L driver board, powered by an STM32G474RET6. It supports encoder-based torque, speed, and position control, as well as sensorless speed control using a nonlinear flux observer (NLOB) and PLL.

Encoder-based control uses a **20 kHz current loop, 2 kHz speed loop, and 1 kHz position loop**. The current loop supports PI / LADRC, alongside mechanical ESO speed estimation, dq decoupling, and back-EMF feedforward. The firmware also provides trapezoidal and S-curve motion profiles, motor parameter identification, encoder calibration and compensation, and control parameter configuration and storage.

**Software foundation:** C / STM32 HAL / ThreadX / USBX.

![AxDrive-L driver board, LCD, and motor running at 6000 RPM](images/axdrive-l-6000rpm-20261024.jpg)

Hardware run on 2026-10-24: approximately 24.8 V supply, encoder-based speed mode, PI current control, and a 6000 RPM target. The LCD shows speed reference, feedback, and live waveforms.

## Main features

| Feature | Details |
| --- | --- |
| Encoder-based control | Torque, speed, and position modes with mechanical ESO speed estimation, 2 kHz speed PI, and 1 kHz position P; includes MT6816 and MT6835 SPI encoder drivers |
| Motion profiles | Torque ramps, trapezoidal speed and position profiles, and S-curve profiles; tested with target changes during operation, direction reversals, position round trips, and deceleration to a stop |
| Current control | 20 kHz dq current loop; encoder-based operation can switch between PI / LADRC while retaining the same outer speed and position loops for comparison; supports dq decoupling and back-EMF feedforward |
| Sensorless control | I/F startup, nonlinear flux observer (NLOB), and PLL; firmware handles rotor alignment, startup, closed-loop handover, and return to I/F during deceleration |
| Motor parameter identification | Identifies stator resistance and inductance (Rs/Ls), permanent-magnet flux linkage, inertia, and viscous friction (J/B); results can be read and applied to the motor model, with scripts for recording repeated tests |
| Encoder calibration and compensation | Encoder direction and zero-offset alignment, plus angular error calibration and compensation; calibration results are saved automatically |
| Tuning and storage | Current and speed PI gains can be calculated from the model and bandwidth or set manually through Kp/Ki; manual gains are stored separately and retained when switching tuning methods |
| Data and display | USB capture of selectable current, speed, angle, and other signals, with waveform, Bode, and FFT/PSD analysis; the onboard LCD shows operating mode, state, references, feedback, and live waveforms |

## Control structure

In encoder-based operation, position, speed, and torque commands enter the control chain at their respective points.

~~~mermaid
flowchart LR
    P["Position profile"] --> PC["Position P + profile-speed feedforward<br/>1 kHz"]
    E["SPI encoder"] --> PC
    E --> ESO["Mechanical ESO<br/>20 kHz"]
    PC --> SC["Speed PI<br/>2 kHz"]
    S["Speed profile"] --> SC
    ESO --> SC
    SC --> IQ["Iq reference"]
    T["Torque command / Kt"] --> IQ
    IQ --> CC["dq current PI / LADRC<br/>20 kHz"]
    CC --> M["SVPWM / inverter / PMSM"]
~~~

Current sensing, coordinate transforms, limiting, and protection are omitted from the diagram. Control loops and motion profiles are implemented in [Motor/](../Motor/) and [Motion/](../Motion/).

Current sensing, coordinate transforms, current control, and modulation execute in the ADC interrupt path, with the current loop running once per PWM cycle. ThreadX threads handle motion-control scheduling, command processing, USB communication, and LCD updates; high-frequency control runs separately from communication and display tasks. The current, speed, and position loops run at **20 kHz / 2 kHz / 1 kHz**, respectively.

Sensorless operation follows **rotor alignment → I/F open-loop startup → NLOB + PLL → closed-loop handover**, with a return to I/F available during deceleration to low speed. Startup and transition logic is in [Sensorless/](../Sensorless/); the flux observer, PLL, and mechanical ESO are in [Observer/](../Observer/).

## Build

Install Git, CMake 3.22+, Ninja, and the GNU Arm Embedded toolchain before building.

~~~bash
git clone --recurse-submodules https://github.com/BJKJKZHOU/AxDr_L_Motor.git &&
cd AxDr_L_Motor &&
cmake --preset gcc-release &&
cmake --build --preset gcc-release --parallel &&
arm-none-eabi-objcopy -O binary build/gcc-release/AxDr_L_Motor.elf build/gcc-release/AxDr_L_Motor.bin &&
arm-none-eabi-objcopy -O ihex build/gcc-release/AxDr_L_Motor.elf build/gcc-release/AxDr_L_Motor.hex
~~~

Build outputs are in `build/gcc-release/`: `AxDr_L_Motor.elf`, `AxDr_L_Motor.bin`, and `AxDr_L_Motor.hex`.

## Measured results

The following results apply to the motors, firmware, and operating conditions documented in the linked records.

**Encoder calibration and compensation (2026-10-20)**

At 6000 RPM with encoder-based PI speed control, approximately 10 seconds of data were captured before and after automatic calibration, with the current-loop bandwidth and outer-loop gains unchanged:

| Metric | Without compensation | Automatic compensation |
| --- | ---: | ---: |
| ESO speed standard deviation / RPM | 11.1395 | 1.7472 |
| Iq second mechanical harmonic peak / A | 0.42668 | 0.02162 |

In this comparison, ESO speed standard deviation decreased by 84.3%, and the Iq second mechanical harmonic peak decreased by 94.9%. The tests also verified calibration storage, loading after reset, and aborting calibration with a stop command during acceleration. ESO speed is derived from the same encoder, so these results do not establish independently measured angular accuracy. Full conditions and raw waveforms are in the [encoder calibration and compensation test record (Chinese)](Mechanical_ESO_有感速度测试记录_20260917.md#encoder-auto-cal-20261020).

**PI / LADRC current-loop and three-loop comparison (2026-10-17)**

With identical speed PI, position P, and motion-profile parameters, PI and LADRC were compared as the inner current loop through speed reversals, position round trips, steady-state holding, electrical disturbances, and Rs/Ls model mismatches of ±30%. Both current controllers completed operation with all three loops. The second comparison included 24 repeated positioning segments, 24 steady-state holding segments, and 96 electrical disturbance segments.

Under the tested settings, LADRC had lower RMS position-holding error and lower peak position displacement under electrical disturbances. Manually tuned PI had lower maximum position-holding error and lower speed fluctuations with the nominal model. Parameters, methods, and results are in the [PI/LADRC three-loop comparison record (Chinese)](PI与LADRC电流环及三环回归测试记录_20261017.md#manual-pi-tuning-20261017).

**Current-loop frequency response (2026-10-10)**

[![D/Q current-loop closed-loop frequency responses across seven PI settings](data/20261010_current_frf/bode_comparison.png)](data/20261010_current_frf/bode_comparison.png)

At 20 kHz sampling, seven PI parameter sets × D/Q axes × feedforward decoupling on/off produced 28 closed-loop frequency sweeps. Higher-bandwidth settings show gain peaking. The PI design bandwidth is not the measured −3 dB bandwidth, and inferred margins are not independent open-loop measurements. Test conditions, validity assessment, and raw data are in the [current-loop frequency-response record (Chinese)](电流环频响测试.md#frf-hw-20261010).

Other hardware tests:

- [Motion profiles (Chinese)](轨迹规划测试记录.md): target changes, direction reversals, position round trips, S-curves, and stop behavior across the three control modes.
- [Speed-loop frequency response (Chinese)](速度环与位置环调参记录.md#speed-frf-hw-20261010): eight PI configurations at a 100 rad/s operating point; linear bandwidth and stability margins remain unconfirmed.
- [Parameter identification (Chinese)](辨识测试记录.md): Rs/Ls, flux linkage, and J/B test conditions, repeatability, and open questions.

## Communication and tools

The firmware communicates with a computer over a USB virtual serial port (CDC) using a protocol defined by this project. It supports parameter reads and writes, motor start and stop, parameter identification, encoder calibration, status and fault reporting, and live waveform capture. Python tools are in [tools/](../tools/).

| Purpose | Tool |
| --- | --- |
| Sequential identification of a new motor with an optional sensorless trial run | [new_motor_commission.py](../tools/new_motor_commission.py) |
| Rs/Ls, flux linkage, and J/B identification and result logging | [identification_test.py](../tools/identification_test.py) |
| Encoder alignment | [servo_phase_search.py](../tools/servo_phase_search.py) |
| Sensorless operation | [sensorless_run.py](../tools/sensorless_run.py) |
| USB waveform capture, file archiving, and FFT/PSD analysis | [fft.py](../tools/fft.py) |
| Replotting saved waveforms and fundamental-component analysis | [current_waveform.py](../tools/current_waveform.py) |
| Current-loop and speed-loop frequency sweeps | [current_frf.py](../tools/current_frf.py) |

The [test and issue index (Chinese)](测试与问题记录索引.md) lists the tools and topic records. Use `--help` to view script options. Waveform and spectrum data can be saved for offline replotting and comparison across control settings.

## Hardware sources and licensing

The current hardware is [AxDrive-L](https://oshwhub.com/lylssy/foc_driver), with [AxDr_L](https://github.com/disnox/AxDr_L) as the reference software. This repository is not that project's official firmware.

This firmware directly drives a motor power stage. Verify the hardware, motor parameters, and independent protection before experiments.

Project-owned code and documentation use [Apache-2.0](../LICENSE). See [THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md) for third-party components.
