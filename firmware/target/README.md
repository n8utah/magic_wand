# Target firmware

ESP32 spell target: BLE peripheral, D2 status LED, WS2812 on D23.

## LEDs

- **D2 (GPIO2)** — blinks white during spell activation
- **D23 (GPIO23)** — WS2812 rainbow cycle while idle; blinks white during activation

Settings in `include/config.h`: `kStatusLedPin`, `kWs2812DataPin`, `kWs2812Brightness`, `kColorStepMs`.

NeoPixel init runs **after** BLE comes up to avoid RMT/radio conflicts on ESP32.

## Build / upload

From the repo root:

```powershell
pio run -e target
pio run -e target -t upload
pio device monitor -e target
```
