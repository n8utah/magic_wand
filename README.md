# Halloween Prop (Magic Wand + Target)

ESP32 firmware for a gesture wand and a physical target, in one PlatformIO repo.

## Layout

```
firmware/
  wand/     ESP32 + MPU6050 — gestures, BLE, TFLite digits
  target/   ESP32 target controller (scaffold)
website/    Web BLE recorder / 3D orientation debug (wand)
train/      Colab notebook for retraining the wand model
magic_wand.ino   Legacy Arduino Nano sketch (reference only)
```

## Requirements

- [PlatformIO](https://platformio.org/)
- ESP32 dev board(s)

## Build and upload

Pick an environment (`wand` or `target`):

```powershell
pio run -e wand
pio run -e wand -t upload

pio run -e target
pio run -e target -t upload
```

Serial monitor:

```powershell
pio device monitor -e wand
```

Default environment is `wand` (`pio run` with no `-e` builds the wand).

## Wand quick start

1. Upload `wand` firmware.
2. Hold still at boot for IMU calibration.
3. Open the [web app](https://petewarden.github.io/magic_wand/website/index.html) or watch Serial for ASCII strokes.

See `firmware/wand/README.md` for wand-specific notes.

## Target

The `target` env is a placeholder. Add hit detection, BLE pairing with the wand, actuators, etc. under `firmware/target/`.

See `firmware/target/README.md`.

## Original Magic Wand (Nano 33 BLE)

The upstream TensorFlow Lite Magic Wand docs and `magic_wand.ino` still apply to the original Arduino board; this repo’s active port lives under `firmware/wand/`.
