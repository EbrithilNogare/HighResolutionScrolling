# High Resolution Scrolling Mouse

Ultra-smooth Bluetooth scrolling device using ESP32-C3 and AS5600 magnetic encoder with intelligent power management.

![Device Photo](https://github.com/user-attachments/assets/1cda570d-2695-4311-adfa-4bde7830b25f)

## Features

- **High-resolution scrolling** with configurable multiplier
- **Battery monitoring** with voltage-based percentage and charging detection
- **Smart power management** with automatic deep sleep and wake on rotation
- **Bluetooth LE HID** - appears as "Smooth scroller" to any BLE-compatible device

## Hardware

### Required Components

- [Seeed XIAO ESP32C3](https://www.aliexpress.com/item/1005005382287176.html) - Main controller
- [AS5600 Magnetic Encoder](https://www.aliexpress.com/item/1005009122468349.html) - Position sensor
- [LiPo Battery 503040 (500mAh)](https://www.aliexpress.com/item/1005008218024646.html)
- Ball Bearing 45x58x7mm - [Standard](https://www.aliexpress.com/item/1005007420073930.html) or [Ceramic (smoother)](https://www.aliexpress.com/item/32437944825.html)

### Optional Components

- [Brass Weight 45mm](https://www.aliexpress.com/item/1005007804047419.html)

### Wiring

| AS5600 | ESP32-C3     | Note                      |
| ------ | ------------ | ------------------------- |
| VCC    | GPIO20       | Power controlled by ESP32 |
| GND    | GND          |                           |
| SCL    | GPIO7        | I2C Clock                 |
| SDA    | GPIO6        | I2C Data                  |
| DIR    | VCC (AS5600) | Direction control         |

Battery voltage divider connected to GPIO4 (220kΩ/220kΩ divider - calibrate in code).

## Quick Start

```bash
pio run --target upload
```

Device appears as "Smooth scroller" in Bluetooth settings. Tested and working on Windows. Other platforms may have varying compatibility.

## Configuration

Key parameters are in `src/main.cpp`:

## Libraries

- Modified [ESP32-BLE-Mouse](https://github.com/T-vK/ESP32-BLE-Mouse)
- [AS5600](https://github.com/RobTillaart/AS5600)

## Tools

[Scrolling Visualizer](https://ebrithilnogare.github.io/HighResolutionScrolling/debugger.html)
