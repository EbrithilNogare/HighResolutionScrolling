# High Resolution Scrolling Mouse

An ESP32-C3 based Bluetooth mouse that uses an AS5600 magnetic encoder for high-resolution scrolling.

![](https://github.com/user-attachments/assets/1cda570d-2695-4311-adfa-4bde7830b25f)

## Hardware

- Main parts
  - **ESP32-C3 Board**: [Seeed Studio XIAO ESP32C3](https://www.aliexpress.com/item/1005005382287176.html)
  - **Magnetic Encoder**: [AS5600 Module](https://www.aliexpress.com/item/1005009122468349.html)
- Optional parts that I used
  - [Brass weight (2mm thick, 45mm diameter)](https://www.aliexpress.com/item/1005007804047419.html)
  - [Ball bearing v1 45x58x7mm (and cleaned brass)](https://www.aliexpress.com/item/1005007420073930.html) OR [Ceramic ball bearing v2 45x58x7mm](https://www.aliexpress.com/item/32437944825.html)
  - [LiPo battery (503040, 3.7 V ~500mAh)](https://www.aliexpress.com/item/1005008218024646.html)
  - [Thermal pad .5 mm](https://www.aliexpress.com/item/32988894487.html)
  - [Rubber legs](https://www.aliexpress.com/item/1005002478823169.html)
  - [Any wires that are tuff](https://www.aliexpress.com/item/1005007079142852.html)

## Wiring

Connect the AS5600 to your ESP32-C3 as follows:

| AS5600 Pin | ESP32-C3 Pin | Description                          |
| ---------- | ------------ | ------------------------------------ |
| VCC        | GPIO20       | Power supply (controlled by ESP32)   |
| GND        | GND          | Ground                               |
| SCL        | GPIO7        | I2C Clock (default for XIAO ESP32C3) |
| SDA        | GPIO6        | I2C Data (default for XIAO ESP32C3)  |
| DIR        | VCC (AS5600) | Direction pin                        |

**Note**: The AS5600 VCC is connected to GPIO20 instead of 3V3 to allow the ESP32 to control power to the encoder for better power management during sleep modes.

## Usage

1. **Build and Upload**:

   ```bash
   pio run --target upload --target monitor
   ```

2. **Pairing**:

   - The device will appear as "Smooth scroller" in Bluetooth settings
   - Pair with your computer/device (Windows and Android working on 100%, MacOS not that great and Linux mostly not)

## Configuration

You can adjust the following parameters in `main.cpp`:

- `DEGREES_PER_SCROLL_STEP`: Degrees of rotation needed for one scroll step
- `ENCODER_READ_INTERVAL_MS`: How often to read the encoder in milliseconds (must be at least 7.5 ms)

## Troubleshooting

1. **AS5600 not found**: Check I2C wiring and ensure the sensor has power
2. **No Bluetooth connection**: Ensure your device supports Bluetooth HID and try re-pairing
3. **Scrolling too sensitive/insensitive**: Adjust the `DEGREES_PER_SCROLL_STEP` value in code

## Libraries Used

- modified [ESP32-BLE-Mouse](https://github.com/T-vK/ESP32-BLE-Mouse): For Bluetooth HID mouse functionality
- [AS5600](https://github.com/RobTillaart/AS5600): For magnetic encoder communication
