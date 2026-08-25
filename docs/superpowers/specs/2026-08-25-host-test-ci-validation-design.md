# Host Test / CI Validation Design

Date: 2026-08-25

## 1. Goal

Establish a permanent host-side validation system for the motor-control algorithms in `AxDr_L_Motor` using Unity + Ceedling.

The tests define expected behavior from control theory, mathematical equations and physical invariants. They must not mirror the current C implementation line by line, and they must not create hardware mocks merely to raise coverage.

The resulting validation model is:

```text
Firmware Build
    GNU Arm GCC compile/link

Host Algorithm Test
    Unity + Ceedling on native host compiler
    theory/reference-model based

Hardware Validation
    AxDrive-L + STM32G474 + real motor
```

These three kinds of evidence remain separate. A Host Test pass never upgrades a hardware validation state by itself.

## 2. Scope

The first Host Test system covers the deterministic algorithm modules that have meaningful platform-independent theory or state equations:

```text
Algo/Math.c
Algo/PID.c
Algo/Voltage_Mod.c
Motor/Motion_Loop.c       -> Speed_Profile only
Sensorless/IF_Start.c
Observer/PLL.c
Observer/Flux_Observer.c
```

The initial Host Test system does not attempt whole-module tests for:

```text
Motor/Motor_Control.c
Sensorless/Sensorless.c
Identification/Identification.c
Motor/Motor_ADC.c
Motor/Motor_PWM.c
Motor/Encoder.c
ThreadX / USBX / HAL / DMA / peripheral IRQ paths
```

Those modules are integration or hardware-reference layers. They must not be refactored only to satisfy Host Test coverage.

## 3. Test framework

Use Unity + Ceedling as the formal C Host Test framework.

Repository layout:

```text
project.yml
Gemfile

test/
├── test_math.c
├── test_pid.c
├── test_svpwm.c
├── test_motion.c
├── test_if_start.c
├── test_pll.c
├── test_flux_observer.c
└── support/
    ├── test_math_ref.c
    ├── test_math_ref.h
    ├── motor_model.c
    └── motor_model.h
```

`test/support/` contains independent test mathematics and reference models. It is not a general-purpose production library and is not compiled into firmware.

The `Gemfile` must pin the exact Ceedling-related Ruby dependencies used by CI so local and CI test environments remain reproducible.

CMock may be available through Ceedling, but it is not the basis of the initial suite. The initial suite should call real algorithm code with theory-generated inputs.

## 4. Oracle rule

Expected results are derived from equations, mathematical properties or independently written reference models:

```text
Theory / control specification / physical model
                    ↓
              expected behavior
                    ↓
          production implementation
```

Tests must not copy the implementation structure into the expected-value calculation. If production code changes implementation while preserving the same control law, the tests should remain valid.

## 5. Numerical comparison rules

Host tests use shared helpers for numerical comparison.

### 5.1 Scalar comparison

Use both absolute and relative tolerance where appropriate:

```text
|a - b| <= abs_tol + rel_tol * max(|a|, |b|)
```

Tests choose tolerances based on the scale and numerical sensitivity of the algorithm, not on a single global epsilon.

### 5.2 Angle comparison

Angle error must use wrapped difference:

```text
angle_error = wrap_to_pi(a - b)
```

and compare `|angle_error|` against an angle tolerance. Raw subtraction across `0 / 2π` is not a valid angle comparison.

### 5.3 Dynamic tests

Dynamic algorithms are evaluated over sequences. Tests may assert transient bounds, convergence, final error and physical invariants; they should not require bit-identical trajectories across different host compilers.

## 6. Math tests

### 6.1 `Angle_Wrap`

The mathematical definition is:

```text
theta_out = theta mod 2π
0 <= theta_out < 2π
```

Coverage includes:

- `0`, `2π`, `-2π`;
- values immediately above and below wrap boundaries;
- multiple positive and negative revolutions;
- invariant that input and output differ by an integer multiple of `2π` within numerical tolerance.

### 6.2 `Limit_Value`

Reference equation:

```text
y = min(max(x, min), max)
```

Verify below-range, in-range and above-range values together with the function's direction return value.

### 6.3 `Vector2_Limit`

For limit `L > 0`:

```text
if sqrt(x² + y²) <= L:
    output = input
else:
    output = input * L / sqrt(x² + y²)
```

Verify magnitude bound, direction preservation, in-limit identity and zero output for non-positive limits.

## 7. PID tests

The test specification follows the implemented discrete control law, expressed independently as equations:

```text
e[k] = r[k] - y[k]
I*[k] = I[k-1] + Ki * e[k] * Ts
D[k] = -Kd * (y[k] - y[k-1]) / Ts
u*[k] = Kp * e[k] + I[k] + D[k]
```

The controller also applies integrator limits, output limits and conditional anti-windup.

Tests cover:

- proportional-only response;
- integral accumulation from a constant error sequence;
- positive and negative integral saturation;
- output saturation;
- derivative-on-measurement sign and magnitude;
- prevention of further integral growth into an active output saturation;
- release from saturation when error direction changes;
- zero-error steady state.

The test oracle is calculated from these equations, not by duplicating `PID_Run()` control flow.

## 8. SVPWM tests

For input voltage vector `(Ualpha, Ubeta)`:

```text
Ua = Ualpha
Ub = -0.5 * Ualpha + sqrt(3)/2 * Ubeta
Uc = -0.5 * Ualpha - sqrt(3)/2 * Ubeta
Uoff = -0.5 * (max(Ua, Ub, Uc) + min(Ua, Ub, Uc))
Da = 0.5 + (Ua + Uoff) / Vbus
Db = 0.5 + (Ub + Uoff) / Vbus
Dc = 0.5 + (Uc + Uoff) / Vbus
```

The reference model clamps duty to `[0, 1]` only at the final step.

Tests cover:

- zero vector -> `0.5 / 0.5 / 0.5`;
- non-positive bus voltage -> neutral `0.5` duties;
- representative alpha/beta axes and sextants;
- output duty always in `[0, 1]`;
- symmetry for opposite vectors;
- agreement with the independent formula over a grid of in-range input vectors.

Host SVPWM tests validate modulation mathematics, not timer preload, dead time or physical gate timing.

## 9. Speed profile tests

`Speed_Profile` is specified by mechanical acceleration and deceleration limits.

For same-direction acceleration:

```text
|Wm_ref[k+1] - Wm_ref[k]| <= Acc * SPD_TS
```

For same-direction deceleration:

```text
|Wm_ref[k+1] - Wm_ref[k]| <= Dec * SPD_TS
```

For a target with opposite sign, the reference must first approach zero using the deceleration limit. Only after reaching zero may it accelerate into the opposite direction.

Tests cover:

- `0 -> positive`;
- `0 -> negative`;
- positive/negative acceleration symmetry;
- same-direction deceleration;
- positive-to-negative reversal;
- negative-to-positive reversal;
- no overshoot of target;
- exact settling to target when the remaining delta is smaller than one step.

Tests assert motion-law properties over sequences rather than internal helper behavior.

## 10. I/F start tests

I/F testing is based on its commanded electrical-speed and current trajectories rather than primarily on enum values.

### 10.1 Electrical speed

Per cycle:

```text
|We[k+1] - We[k]| <= IF_ACC_RAD_S2 * CUR_TS
```

The speed must not overshoot the target.

### 10.2 Electrical angle

The angle integration law is:

```text
Theta[k+1] = wrap(Theta[k] + We[k] * CUR_TS)
```

Tests include multiple positive and negative wraps.

### 10.3 Current magnitude and direction

The nominal current magnitude is:

```text
ratio = min(|We| / IF_WE_TARGET_RAD_S, 1)
Iq_abs = IF_IQ_START_A +
         (IF_IQ_TARGET_A - IF_IQ_START_A) * ratio
```

The current slew constraint is:

```text
|Iq[k+1] - Iq[k]| <= IF_IQ_SLEW_A_S * CUR_TS
```

Current sign follows the active rotation direction, with the target direction used at exact zero where needed to start moving.

Tests cover:

- `0 -> positive`;
- `0 -> negative`;
- positive/negative acceleration;
- same-sign target changes;
- positive-to-negative reversal through zero;
- negative-to-positive reversal through zero;
- current slew bound;
- current sign around zero crossing;
- hold readiness after the configured hold duration;
- target changes resetting hold readiness.

## 11. PLL tests

PLL test input uses an analytically generated rotating vector:

```text
X = Mag * cos(theta)
Y = Mag * sin(theta)
theta(t) = theta0 + we * t
```

Tests cover:

- reset angle wrapping;
- no update when `Mag_Ref <= 0` or `Ts <= 0`;
- zero-speed lock;
- positive constant speed tracking;
- negative constant speed tracking;
- repeated `0 / 2π` wrap crossings;
- non-zero initial angle error;
- non-zero initial speed error.

Dynamic assertions use wrapped angle error and speed error. The test defines convergence windows and steady-state bounds based on the configured PLL gains used by that test case; it does not require identical transient samples to a copied implementation.

## 12. Flux observer tests

The Flux Observer oracle uses an independent ideal surface-PMSM alpha/beta model in `test/support/motor_model.c`.

For a selected electrical angle:

```text
Psi_alpha = Flux * cos(theta)
Psi_beta  = Flux * sin(theta)

Lambda_alpha = Ls * Ialpha + Psi_alpha
Lambda_beta  = Ls * Ibeta  + Psi_beta

Ualpha = Rs * Ialpha + d(Lambda_alpha)/dt
Ubeta  = Rs * Ibeta  + d(Lambda_beta)/dt
```

The model generates synthetic `Ialpha/Ibeta/Ualpha/Ubeta` sequences independently of `Flux_Observer_Run()`.

Tests cover:

- reset state from known flux angle and current;
- stationary known-flux cases;
- positive rotating flux;
- negative rotating flux;
- flux magnitude convergence;
- estimated flux angle agreement using wrapped angle error;
- symmetry between alpha/beta orientations.

A combined reference test may feed the observer output into the production PLL and verify estimated angle/speed against the known synthetic trajectory. This remains a Host Algorithm test, not a Sensorless hardware validation.

## 13. Hardware boundary

Do not add host mocks for STM32 peripheral behavior solely for coverage.

The following remain outside Host Test truth claims:

```text
ADC injected trigger timing
ADC register acquisition
TIM1 preload / UEV / MOE / CCER behavior
PWM physical output
SPI / DMA / MT6816 timing
DWT cycle timing
ThreadX scheduling
USBX transport timing
interrupt priority and deadline behavior
power-stage safety behavior
real motor startup and observer robustness
```

These are validated by firmware build plus existing or future hardware tests.

## 14. CI integration

Keep the existing GNU Arm GCC firmware-build job as an independent compile/link compatibility check.

Add a separate Host Test job on Ubuntu:

```text
checkout
setup Ruby
bundle install
bundle exec ceedling test:all
```

The Host Test job must not require the ARM toolchain or STM32 submodules that are unrelated to the selected algorithm sources.

Ceedling gcov support should be configured from the beginning so coverage reports are available for inspection. Coverage percentage is informational and is not a CI pass/fail threshold.

The CI meaning remains explicit:

```text
Firmware Build PASS
    -> embedded project compiles and links with GNU Arm GCC

Host Test PASS
    -> tested mathematical/control behavior matches its specification

Hardware VALIDATED
    -> current firmware revision passed corresponding real-hardware tests
```

## 15. Production-code change policy

Host Test introduction must not trigger broad production refactoring.

Allowed changes are limited to what is required to compile deterministic algorithm sources natively while preserving firmware behavior. If a module requires extensive HAL, register, RTOS or global-state mocking, it is excluded from the initial Host Test scope instead of being abstracted for testability.

`Sensorless.c`, `Motor_Control.c` and hardware modules remain integration/reference-platform code unless future maintenance problems create an independent reason to extract a pure algorithm boundary.

## 16. Success criteria

The first implementation is complete when:

- Unity + Ceedling can run locally through the pinned Ruby environment;
- CI has a separate Host Test job;
- the seven scoped algorithm areas have theory/reference-model based tests;
- shared scalar and angle comparison helpers exist;
- the ideal PMSM alpha/beta reference model exists only under `test/support/`;
- gcov reporting is available but non-gating;
- no STM32 peripheral mock layer is introduced;
- existing firmware build behavior and GCC CI remain intact;
- `docs/VALIDATION.md` continues to distinguish Host Test evidence from hardware validation.
