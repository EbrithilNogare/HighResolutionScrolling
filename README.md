# High Resolution Scrolling Mouse

An ESP32-C3 based Bluetooth mouse that uses an AS5600 magnetic encoder for high-resolution scrolling.

## Hardware

- **ESP32-C3 Board**: [Seeed Studio XIAO ESP32C3](https://www.aliexpress.com/item/1005005382287176.html)
- **Magnetic Encoder**: [AS5600 Module](https://www.aliexpress.com/item/1005009122468349.html)

## Wiring

Connect the AS5600 to your ESP32-C3 as follows:

| AS5600 Pin | ESP32-C3 Pin | Description                          |
| ---------- | ------------ | ------------------------------------ |
| VCC        | 3V3          | Power supply (3.3V)                  |
| GND        | GND          | Ground                               |
| SCL        | GPIO7        | I2C Clock (default for XIAO ESP32C3) |
| SDA        | GPIO6        | I2C Data (default for XIAO ESP32C3)  |
| DIR        | GPIO4        | Direction pin (optional)             |

## Features

1. **Bluetooth Mouse**: Connects as a standard Bluetooth HID mouse device
2. **Serial Monitoring**:
   - Sends "alive" heartbeat every second
   - Reports Bluetooth connection status
   - Shows current magnetic encoder angle (0-360°)
3. **High Resolution Scrolling**: Converts rotary encoder movement to smooth mouse scroll events

## Usage

1. **Build and Upload**:

   ```bash
   pio run --target upload --target monitor
   ```

2. **Pairing**:

   - The device will appear as "ESP32-C3 Scroll Mouse" in Bluetooth settings
   - Pair with your computer/device

3. **Operation**:
   - Rotate the magnet near the AS5600 sensor to scroll
   - Monitor serial output at 9600 baud for status information
   - Adjust `degreesPerScroll` in code to change sensitivity

## Configuration

You can adjust the following parameters in `main.cpp`:

- `degreesPerScroll`: Degrees of rotation needed for one scroll step (default: 15°)
- `ENCODER_READ_INTERVAL`: How often to read the encoder in milliseconds (default: 50ms)
- `HEARTBEAT_INTERVAL`: How often to send status via serial (default: 1000ms)

## Serial Output Example

```
=== ESP32-C3 High Resolution Scrolling Mouse ===
✓ AS5600 magnetic encoder connected successfully
Initial angle: 123.4°
✓ BLE Mouse service started
Waiting for Bluetooth connection...

[5s] Alive - Bluetooth: CONNECTED | Encoder: 125.2° | Total rotation: 0.0°
Scroll: 1 steps (UP)
[6s] Alive - Bluetooth: CONNECTED | Encoder: 140.8° | Total rotation: 0.8°
Scroll: -2 steps (DOWN)
```

## Troubleshooting

1. **AS5600 not found**: Check I2C wiring and ensure the sensor has power
2. **No Bluetooth connection**: Ensure your device supports Bluetooth HID and try re-pairing
3. **Scrolling too sensitive/insensitive**: Adjust the `degreesPerScroll` value in code

## Libraries Used

- [ESP32-BLE-Mouse](https://github.com/T-vK/ESP32-BLE-Mouse): For Bluetooth HID mouse functionality
- [AS5600](https://github.com/RobTillaart/AS5600): For magnetic encoder communication
