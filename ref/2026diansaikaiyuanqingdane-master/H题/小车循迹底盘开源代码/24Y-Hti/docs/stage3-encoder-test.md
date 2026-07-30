# Stage 3 encoder test

## Pin map

| Wheel | Encoder A | Encoder B | Timer |
| --- | --- | --- | --- |
| Left | PA15 | PB3 | TIM2, partial remap 1 |
| Right | PB6 | PB7 | TIM4, default mapping |

Both encoders are powered from 3.3 V and share ground with the STM32,
TB6612, battery, and DAPLink.

## Firmware behavior

- Motor PWM remains zero; this test never drives either motor.
- TIM2 and TIM4 run in quadrature encoder mode TI12 with digital filter 8.
- The 16-bit hardware counters are accumulated into signed 32-bit counts.
- Forward wheel rotation is normalized positive on both sides.
- The nominal calibration is 1061 counts per wheel revolution
  (`13 lines * x4 quadrature * 20.4 gearbox`).
- PC13 turns on briefly whenever either encoder count changes.
- OpenOCD can inspect the counts without resetting or halting the MCU.

## Manual test

1. Rotate only the left wheel in the vehicle-forward direction by exactly one
   wheel revolution. PC13 should show activity.
2. Rotate only the right wheel in the vehicle-forward direction by exactly one
   wheel revolution. PC13 should show activity.
3. Run `tools/read-encoders.ps1` and record both signed counts.
4. Reverse each wheel slightly and confirm that its count moves toward zero.

If manual rotation produces zero counts, a diagnostic build drives both motors
forward at 50% for 1 second while continuing to accumulate encoder counts.
