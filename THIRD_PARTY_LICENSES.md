# Third-Party Licenses

This repository contains third-party software that is not covered by the project-level Apache-2.0 license. Those components remain under their original license terms and copyright notices.

## STMicroelectronics STM32Cube components

The following directories contain STM32Cube-provided components and retain their original notices and license files:

- `Drivers/STM32G4xx_HAL_Driver/`
- `Drivers/CMSIS/`

Refer to the `LICENSE.txt` files shipped in those component directories for the applicable terms.

## Eclipse ThreadX

`ThirdParty/Eclipse/threadx/` is a Git submodule that provides Eclipse ThreadX 6.5.1 used by this firmware.

The submodule points to the upstream Eclipse ThreadX repository and remains under the license distributed by that project. Refer to the license files and notices inside the submodule for the applicable terms.

## Eclipse USBX

`ThirdParty/Eclipse/usbx/` is a Git submodule that provides Eclipse USBX 6.5.0 portable core and CDC ACM device-class sources used by this firmware.

The submodule points to the upstream Eclipse USBX repository and remains under the license distributed by that project. Refer to the license files and notices inside the submodule for the applicable terms.

## STMicroelectronics USBX STM32 device controller adaptation

`ThirdParty/ST/usbx_stm32_dcd/` contains the STM32 USBX device-controller adaptation retained from the STM32Cube distribution.

This code is separate from the Eclipse USBX portable sources and retains its original source headers and accompanying license files, including:

- `ThirdParty/ST/usbx_stm32_dcd/LICENSE.txt`
- `ThirdParty/ST/usbx_stm32_dcd/LICENSED-HARDWARE.txt`

## Generated middleware copies

STM32CubeMX may regenerate dependency copies under `Middlewares/`. Those generated copies are ignored by Git and are not used by the project build. The firmware build uses the dependencies under `ThirdParty/` described above.

## Project-owned code

Except for third-party files and generated/vendor files that carry their own license or copyright notices, project-owned source code and documentation are licensed under the Apache License 2.0. See the repository root `LICENSE` file.

If a file has its own license or copyright notice, that notice takes precedence for that file.
