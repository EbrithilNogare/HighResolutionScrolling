# High Resolution Scrolling Mouse

Ultra-smooth Bluetooth scrolling device using ESP32-C3 and AS5600 magnetic encoder with intelligent power management.

![Device Photo](https://github.com/user-attachments/assets/1cda570d-2695-4311-adfa-4bde7830b25f)

## Features

- **High-resolution scrolling**\* with configurable multiplier
- **Battery monitoring** with voltage-based percentage and charging detection
- **Smart power management** with automatic deep sleep and wake-up on rotation
- **Bluetooth LE HID** - appears as "Smooth scroller" to any BLE-compatible device

\*Works only on Windows; Mac does not support resolution multiplier, so it must be set to 1 in code to work properly (but not smoothly)

## Hardware

- [Seeed XIAO ESP32C3](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html) - Main controller
- [AS5600 Magnetic Encoder](https://www.aliexpress.com/item/1005008401280982.html?spm=a2g0o.order_list.order_list_main.59.695018026o9Pzf) - Rotation sensor
- [LiPo Battery 503040 (500mAh)](https://www.aliexpress.com/item/1005008218024646.html)
- Micro JST PH2.0 2.0mm Plug Socket Wire Connector 26AWG 2Pin - Connector for battery
- Ball Bearing 45x58x7mm - [Standard](https://www.aliexpress.com/item/1005007420073930.html) or [Ceramic (smoother)](https://www.aliexpress.com/item/32437944825.html)
- [Brass Weight 45mm](https://www.aliexpress.com/item/1005007804047419.html)
- 2x $220\ \mathsf{k\Omega}$ resistors (for measuring battery level)
- Brass Threaded Insert Nut (M3x5x4.2) + Bolt M3x10
- Soldering wires, it's better to get a stiff single wire
- Some adhesive matter, for example, Blu Tack

### Wiring

| AS5600 | ESP32-C3     | Note                      |
| ------ | ------------ | ------------------------- |
| VCC    | GPIO20       | Power controlled by ESP32 |
| GND    | GND          |                           |
| SCL    | GPIO7        | I2C Clock                 |
| SDA    | GPIO6        | I2C Data                  |

- AS5600 DIR <---> AS5600 VCC (Direction control)
- Battery voltage divider connected to GPIO4 (220kΩ/220kΩ divider - calibrate in code).

<img width="1440" height="1600" alt="as5600_xiao_esp32c3_wiring" src="https://github.com/user-attachments/assets/fcd34052-c547-4cfb-9ad3-5635dfeab781" />

## Quick Start

- Install [Visual Studio Code](https://code.visualstudio.com/)
- Install [Platformio extension](https://docs.platformio.org/en/latest/integration/ide/vscode.html) for Visual Studio Code
- Install [Python3](https://www.python.org/downloads/) (I used 3.11.14)
- Run build task _Ctrl+Shift+B_ or run build & upload manually `pio run --target upload`

Device appears as "Smooth scroller" in Bluetooth settings. Tested and working on Windows. Other platforms may have varying compatibility.

## Configuration

Key parameters are in `src/main.cpp`:

## Libraries

- Modified [ESP32-BLE-Mouse](https://github.com/T-vK/ESP32-BLE-Mouse)
- [AS5600](https://github.com/RobTillaart/AS5600)

## Inspiration

Engineer Bo: [Wireless High Resolution Scrolling is Amazing](https://www.youtube.com/watch?v=FSy9G6bNuKA)

## Tools

[Scrolling Visualizer](https://ebrithilnogare.github.io/HighResolutionScrolling/debugger.html)
