# AxDr_L_Motor

**Language / 语言:** [简体中文](../README.md) | English

[![Firmware Build](https://github.com/BJKJKZHOU/AxDr_L_Motor/actions/workflows/ci.yml/badge.svg)](https://github.com/BJKJKZHOU/AxDr_L_Motor/actions/workflows/ci.yml)

Permanent-magnet synchronous motor (PMSM) field-oriented control (FOC) firmware for **STM32G474RET6 + AxDrive-L**. The project evolves through hardware experiments, motor identification, and controller comparisons, with measured data and test conditions retained for review.

**Development environment:** C / STM32 HAL / ThreadX / USBX, STM32CubeMX configuration, CMake with ST Arm Clang / GNU Arm GCC, and Python for frequency-response and noise-spectrum analysis.

## Current progress

| Area | Stage | Measured scope and limitations |
| --- | --- | --- |
| Position-sensor-based servo control | **Basic functions tested on hardware** | Torque-command, speed PI, position P, motion profiles, and Stop behavior; results apply to the tested no-load conditions. [Test record (Chinese)](三环基本功能测试记录_20261009.md) |
| Sensorless speed control | **Verified in a limited operating range** | 1000 RPM closed-loop operation and transitions between I/F startup and observer control; speed fluctuations remain at 150 RPM in I/F mode. [Test record (Chinese)](NLOB_PLL观测器测试记录.md#sensorless-accdec-20261006) |
| Motor parameter identification | **Measured; accuracy still under evaluation** | Stator resistance/inductance (Rs/Ls), magnet flux linkage, inertia and viscous friction (J/B); five flux measurements yielded a 0.145% coefficient of variation. Rs/Ls consistency across operating points remains open. [Test record (Chinese)](辨识测试记录.md) |
| Control-loop frequency-response testing | **Measurement and comparison ongoing** | 28 Id/Iq current-loop and eight speed-loop Bode datasets; high-bandwidth anomalies and speed-loop linearity require further verification. [Record index (Chinese)](测试与问题记录索引.md) |

## Control structure

The primary signal path for **encoder-based** operation is shown below. Position, speed, and torque refer to **alternative command modes**, not three independent loops operating simultaneously.

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
    IQ --> CC["dq current PI<br/>20 kHz"]
    CC --> M["SVPWM / inverter / PMSM"]
~~~

The position P controller adds a speed correction to the planned velocity. Speed PI uses electrical angular velocity (electrical speed = pole pairs × mechanical speed) and produces the Iq reference. The mechanical extended state observer (ESO) estimates speed and disturbance torque using encoder position and other signals; disturbance-torque estimation is currently diagnostic only. Current sensing, coordinate transforms, and limits are omitted from the diagram. See [Motor/](../Motor/) and [Motion/](../Motion/).

Sensorless operation follows **rotor alignment → I/F open-loop startup → nonlinear flux observer + PLL → closed-loop handover**, with a return to I/F at lower speeds. See [Sensorless/](../Sensorless/) and [Observer/](../Observer/).

## Measured frequency responses

Click either preview to view its full-resolution Bode plot. Both measurements are **closed-loop frequency responses**; the configured PI design bandwidth is not necessarily the measured −3 dB bandwidth, and inferred stability margins are not independently measured open-loop margins.

<table>
<tr>
<td width="50%" valign="top">
<a href="data/20261010_current_frf/bode_comparison.png"><img src="data/20261010_current_frf/bode_comparison.png" width="460" alt="D and Q current-loop frequency-response comparison across seven PI settings" /></a>
<p><strong>Id / Iq current loops</strong><br />20 kHz sampling; seven PI parameter sets × D/Q axes × feedforward decoupling on/off, for 28 datasets. Higher-bandwidth settings show gain peaking.<br /><a href="电流环频响测试.md#frf-hw-20261010">Test record and data (Chinese)</a></p>
</td>
<td width="50%" valign="top">
<a href="data/20261010_speed_frf/Speed-Wm100radps-Manual-Kp0.04-Ki0.3-FF0-2026-10-10.bode.png"><img src="data/20261010_speed_frf/Speed-Wm100radps-Manual-Kp0.04-Ki0.3-FF0-2026-10-10.bode.png" width="460" alt="Encoder-based speed-loop Bode result with manual PI" /></a>
<p><strong>Encoder-based speed loop</strong><br />Eight PI configurations at a 100 rad/s operating point, with 1–150 Hz excitation. The example uses manual Kp=0.04 and Ki=0.3. Linear operating conditions are not yet verified; bandwidth and stability margins are N/A.<br /><a href="速度环与位置环调参记录.md#speed-frf-hw-20261010">Test record and data (Chinese)</a></p>
</td>
</tr>
</table>

## Build and documentation

The target is [AxDrive-L](https://oshwhub.com/lylssy/foc_driver) (see its [reference firmware](https://github.com/disnox/AxDr_L)). This repository is not the official firmware for that project. Clone with Git submodules; an existing clone can run git submodule update --init --recursive.

~~~bash
git clone --recursive https://github.com/BJKJKZHOU/AxDr_L_Motor.git
cd AxDr_L_Motor

# GNU Arm GCC (CMake 3.22+, Ninja, arm-none-eabi-gcc)
cmake --preset gcc-release
cmake --build --preset gcc-release

# ST Arm Clang
cmake --preset Release
cmake --build --preset Release
~~~

USB CDC currently carries AxDr Parameter / Action / Event messages and Plot data; the application layer uses CAN-FD-style messages. See the [frequency-response tool](../tools/current_frf.py), [FFT/PSD tool](../tools/fft.py), and [test index (Chinese)](测试与问题记录索引.md). Test scripts evolve with experiments; check their compatibility with the firmware revision. Coding conventions are in [CODE_STYLE.md (Chinese)](CODE_STYLE.md).

**Safety and licensing:** The firmware can directly drive a motor power stage; verify the hardware, motor parameters, and independent protection before energizing it. Project-owned code and documentation use [Apache-2.0](../LICENSE); third-party dependencies retain their respective licenses as documented in [THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md).
