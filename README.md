## pioarduino LVGL demo (ESP32-S3 Touch AMOLED 1.64)

![Waveshare ESP32-S3-Touch-AMOLED-1.64 board image](https://www.waveshare.com/img/devkit/ESP32-S3-Touch-AMOLED-1.64/ESP32-S3-Touch-AMOLED-1.64-1_460.jpg)

This folder is a **PlatformIO** project that ports the repo’s Arduino LVGL demo (`Arduino/examples/06_LVGL_Test`) to **pioarduino**.

### Board links

- **Product page**: [Waveshare ESP32-S3-Touch-AMOLED-1.64](https://www.waveshare.com/product/arduino/boards-kits/esp32-s3/esp32-s3-touch-amoled-1.64.htm)
- **Wiki**: [Waveshare ESP32-S3-Touch-AMOLED-1.64 Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.64)

### What it uses

- **Board**: Waveshare ESP32‑S3 Touch AMOLED 1.64 (custom board JSON in `boards/`)
- **Display**: SH8601 AMOLED over QSPI (`GPIO9/10/11/12/13/14`, reset `GPIO21`)
- **Touch**: FT3168 over I2C (`SDA GPIO47`, `SCL GPIO48`, addr `0x38`)
- **LVGL**: standard PlatformIO Registry library `lvgl/lvgl@8.4.0` + local config in `include/lv_conf.h`
- **UI**: a small in-project demo screen (`src/ui_demo.c`) instead of `lv_demo_widgets()` (the registry `lvgl` package doesn’t ship demo sources)

### How to build

- Install **pioarduino** (a community fork of PlatformIO IDE for VS Code): https://marketplace.visualstudio.com/items?itemName=pioarduino.pioarduino-ide
- Open this folder in **VS Code** (with the pioarduino extension)
- Select the `pioarduino-lvgl-amoled164` project folder
- Build/Upload using the `amoled164` environment

### If your board definition differs

If your AMOLED board needs a different PlatformIO `board` (USB settings, flash/psram variant, etc), edit:

- `platformio.ini` → change `board = ...`

---

Author: Mark Evans  
For more information about the project, visit [markevans.info](https://markevans.info/).


