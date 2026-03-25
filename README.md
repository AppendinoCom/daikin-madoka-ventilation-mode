# Daikin Madoka BLE Bridge — ESPHome Component

Custom [ESPHome](https://esphome.io/) component for controlling **Daikin Madoka BRC1H** thermostats via Bluetooth Low Energy (BLE) using an **ESP32** as a wireless bridge to Home Assistant.

Built specifically for **ventilation / recuperation units** (Daikin Sky Air with BRC1H52W controller).

## Features

| Entity | Type | Description |
|--------|------|-------------|
| Ventilation | `fan` | On/Off + Low/High speed control |
| Indoor Temperature | `sensor` | Current room temperature (°C) |
| Outdoor Temperature | `sensor` | Outdoor unit temperature (°C) |
| Cooling Setpoint | `sensor` | Cooling target temperature |
| Heating Setpoint | `sensor` | Heating target temperature |
| Filter Alert | `binary_sensor` | Indicates when the air filter needs cleaning |
| Reset Filter | `button` | Clears the filter warning and resets the timer |
| Eye Brightness | `number` | Controls the front LED brightness (0–19) |
| Firmware Version | `text_sensor` | BRC1H firmware version (diagnostic) |
| Restart | `button` | Restart the ESP32 bridge device |

## Hardware

### Supported Boards

| Board | Chip | Example Config |
|-------|------|----------------|
| **Seeed XIAO ESP32-S3** | ESP32-S3 | [`example-seeed-xiao-esp32s3.yaml`](example-seeed-xiao-esp32s3.yaml) |
| **M5Stack Atom Echo** | ESP32-PICO | [`example-m5stack-atom-echo.yaml`](example-m5stack-atom-echo.yaml) |

Any ESP32 board with BLE support should work — the configs above are tested and ready to use.

### Target Device

- **Daikin BRC1H** thermostat (tested with BRC1H52W, firmware 1.10.3)

The ESP32 connects to the BRC1H over BLE using LE Secure Connections with automatic Numeric Comparison pairing. No manual pairing steps needed — the firmware handles everything.

> **Note:** The M5Stack Atom Echo config uses the board only as a BLE bridge — its microphone and speaker are not used.

## Installation

### 1. Add the component to your ESPHome config

Copy the example config for your board to your ESPHome config directory and modify it for your setup:

- **Seeed XIAO ESP32-S3:** [`example-seeed-xiao-esp32s3.yaml`](example-seeed-xiao-esp32s3.yaml)
- **M5Stack Atom Echo:** [`example-m5stack-atom-echo.yaml`](example-m5stack-atom-echo.yaml)

The key section that pulls the component from GitHub:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/AppendinoCom/daikin-madoka-ventilation-mode
      ref: main
    components: [madoka_ble]
```

### 2. Set your secrets

Add these entries to your ESPHome `secrets.yaml`:

```yaml
wifi_ssid: "YourWiFi"
wifi_password: "YourPassword"
api_encryption_key: "your-base64-key"
ota_password: "your-ota-password"
ap_password: "your-fallback-ap-password"
madoka_mac: "AA:BB:CC:DD:EE:FF"  # Your BRC1H MAC address
```

> **Finding your MAC address:** Use a BLE scanner app (e.g. nRF Connect) on your phone to find devices named `UE878...` or `Madoka...` near your thermostat.

### 3. Flash

```bash
esphome run madoka_bridge.yaml
```

For subsequent updates, use OTA:

```bash
esphome run madoka_bridge.yaml --device <ESP32_IP>
```

### ESPHome Builder (Home Assistant Add-on)

If you use the ESPHome Builder add-on in Home Assistant:

1. Create a new device config based on the example for your board:
   - `example-seeed-xiao-esp32s3.yaml` for Seeed XIAO ESP32-S3
   - `example-m5stack-atom-echo.yaml` for M5Stack Atom Echo
2. Add your BRC1H MAC address to the ESPHome secrets
3. Install the firmware via the add-on UI

## How It Works

```
Home Assistant ←── ESPHome Native API ←── ESP32 ←── BLE ──→ BRC1H Thermostat
```

The ESP32 acts as a BLE-to-WiFi bridge:
1. Connects to the BRC1H thermostat over BLE (LE Secure Connections)
2. Implements the Madoka TLV protocol (chunked commands over GATT)
3. Polls the device every 60 seconds for state updates
4. Exposes all entities to Home Assistant via the ESPHome native API

## Protocol

The component implements the Daikin Madoka TLV (Tag-Length-Value) protocol:
- BLE GATT Service: `2141e110-213a-11e6-b67b-9e71128cae77`
- Commands are chunked into 20-byte BLE ATT packets
- Ventilation speed control uses command `0x4031` with parameter `0x21`

## Credits

This project is based on and inspired by the work of:

- **Manuel Durán** ([@mduran80](https://github.com/mduran80)) — original [daikin_madoka](https://github.com/mduran80/daikin_madoka) Home Assistant integration and the [pymadoka](https://github.com/mduran80/pymadoka) library that first reverse-engineered the BLE protocol
- **zobylamouche** ([@zobylamouche](https://github.com/zobylamouche)) — [v2.0 rewrite](https://github.com/zobylamouche/daikin_madoka) with native HA BLE stack and clean-room TLV protocol implementation

This ESPHome component is a derivative work that re-implements the protocol in C++ for the ESP-IDF BLE stack, adding ventilation-specific features for recuperation units.

## License

MIT — see [LICENSE](LICENSE) for details.
