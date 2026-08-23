# Third-Party Licenses

This repository contains third-party software that is not covered by the project-level Apache-2.0 license. Those components remain under their original license terms and copyright notices.

## STMicroelectronics STM32Cube components

The following directories contain STM32Cube-provided components and retain their original notices and license files:

- `Drivers/STM32G4xx_HAL_Driver/`
- `Drivers/CMSIS/`

Refer to the `LICENSE.txt` files shipped in those component directories for the applicable terms.

## Microsoft Azure RTOS ThreadX

`Middlewares/ST/threadx/` contains the STM32Cube-distributed Azure RTOS ThreadX 6.2.0 source currently used by this firmware. It retains the Microsoft Azure RTOS license text and source headers distributed with that version.

Relevant files include:

- `Middlewares/ST/threadx/LICENSE.txt`
- `Middlewares/ST/threadx/LICENSED-HARDWARE.txt`

The current firmware targets the STM32G4 series, which is listed in the accompanying licensed-hardware file.

## Microsoft Azure RTOS USBX

`Middlewares/ST/usbx/` contains the STM32Cube-distributed Azure RTOS USBX 6.2.0 source currently used by this firmware. It retains the Microsoft Azure RTOS license text and source headers distributed with that version.

Relevant files include:

- `Middlewares/ST/usbx/LICENSE.txt`
- `Middlewares/ST/usbx/LICENSED-HARDWARE.txt`

## Project-owned code

Except for third-party files and generated/vendor files that carry their own license or copyright notices, project-owned source code and documentation are licensed under the Apache License 2.0. See the repository root `LICENSE` file.

If a file has its own license or copyright notice, that notice takes precedence for that file.
