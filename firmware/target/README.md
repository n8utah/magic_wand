# Target firmware

ESP32 firmware for the physical target (hit detection, lights, servos, etc.).

## WS2812 color cycle (current)

Drives one **WS2812** on **D23** (GPIO 23) through a rainbow color cycle.

Settings in `include/config.h`: `kWs2812DataPin`, `kWs2812Brightness`, `kColorStepMs`.

## Build / upload

From the repo root:

```powershell
pio run -e target
pio run -e target -t upload
pio device monitor -e target
```
