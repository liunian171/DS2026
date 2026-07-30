# Stage 2 motor output test

## Wiring

All grounds must be connected together: battery negative, TB6612 GND,
STM32 GND, and DAPLink GND.

| TB6612 | STM32F103C8T6 | Purpose |
| --- | --- | --- |
| VCC | 3.3 V from the STM32 board LDO | Logic supply |
| VM | 7.4 V battery positive | Motor supply |
| GND | Common GND | Logic and motor reference |
| STBY | 3.3 V | Permanently enabled |
| PWMA | PB4 / TIM3_CH1 | Right motor PWM, 20 kHz |
| AIN1 | PB12 | Right motor direction 1 |
| AIN2 | PB13 | Right motor direction 2 |
| AO1/AO2 | Right MG310 motor | Right motor output |
| PWMB | PB5 / TIM3_CH2 | Left motor PWM, 20 kHz |
| BIN1 | PB14 | Left motor direction 1 |
| BIN2 | PB15 | Left motor direction 2 |
| BO1/BO2 | Left MG310 motor | Left motor output |

The debug UART is not required for this test.

## Firmware behavior

The automatic motor test is disabled after stage 2 passes. The parameters
below document the last test sequence.

1. Immediately after reset, both PWM channels are zero and all four direction
   pins are low.
2. PC13 blinks for 3 seconds while both motors remain stopped.
3. PC13 stays on while both motors receive 50% reverse PWM for 2 seconds.
4. Both motors are then placed in coast mode and PC13 turns off permanently.
5. The sequence runs only once per reset.

## Safe download and test order

1. Lift both wheels clear of the table.
2. Disconnect the 7.4 V wire from TB6612 VM. Keep DAPLink and logic power
   connected.
3. Download the firmware with `tools/flash.ps1`.
4. Do not use GDB breakpoints while VM is powered because STBY is permanently
   high.
5. Connect the 7.4 V wire to VM.
6. Press RESET once and observe the left wheel.
7. Disconnect VM immediately if the motor does not stop after the short pulse.

## Report these results

- Did PC13 blink for about 3 seconds?
- Did only the left wheel move?
- Which direction did it rotate relative to vehicle forward?
- Was motion weak, normal, or violent?
- Did both wheels stop automatically after about 2 seconds?
