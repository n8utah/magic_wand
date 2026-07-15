# Wand firmware

ESP32 + MPU6050 gesture wand. Streams strokes and orientation over BLE, runs TFLite digit recognition.

## Build / upload

From the repo root:

```powershell
pio run -e wand
pio run -e wand -t upload
pio device monitor -e wand
```

## Web app

https://petewarden.github.io/magic_wand/website/index.html
