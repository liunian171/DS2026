# HtiCar

STM32F103C8T6 control firmware for the 2024 H-task vehicle functional
reproduction.

The project is generated from `HtiCar.ioc` with STM32CubeMX and built with
CMake, Ninja, and GNU Arm Embedded Toolchain. OpenOCD uses a CMSIS-DAP/DAPLink
probe over SWD.

Current scope: stage 1 board bring-up is complete. Stage 2 adds a portable
TB6612 motor layer, 20 kHz PWM on TIM3, safe default GPIO states, and a
one-shot left-motor test that does not require the debug UART.

See `docs/stage2-motor-test.md` for the current hardware test procedure.
