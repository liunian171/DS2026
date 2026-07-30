# Stage 1 bring-up test

## Expected firmware behavior

- The onboard PC13 LED is active-low.
- The LED changes state every 500 ms, producing a 1 s full blink period.
- USART3 uses PB10 (TX) and PB11 (RX), 115200 baud, 8 data bits, no parity,
  one stop bit, and no flow control.
- The serial port prints three startup lines and then `heartbeat` every 500 ms.

## DAPLink UART wiring

| DAPLink | STM32F103C8T6 |
| --- | --- |
| UART RX | PB10 / USART3 TX |
| UART TX | PB11 / USART3 RX |
| GND | GND |

The current DAPLink virtual serial port is COM11.

## Test commands

Run these commands from the project root in PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\flash.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-monitor.ps1
```

## Pass criteria

1. PC13 LED blinks continuously with a 1 s full period.
2. COM11 shows the startup banner after reset.
3. COM11 shows two `heartbeat` lines per second.
4. Resetting the board restarts both the LED pattern and startup banner.
