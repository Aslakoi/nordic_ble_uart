# Nordic BLE UART Temperature Logger

A complete BLE-based environmental monitoring system using Nordic nRF54L15 DK boards with real-time web dashboard.

**Based on:** [Nordic nRF Connect SDK Bluetooth Samples](https://github.com/nrfconnect/sdk-nrf/tree/main/samples/bluetooth) - specifically the `ble_app_uart` peripheral_uart and central_uart samples, extended with BME280 temperature sensing and a Python web dashboard.

## System Overview

This project demonstrates:
- **Bluetooth Low Energy (BLE)** communication between two nRF54L15 boards
- **Environmental sensing** using BME280 temperature sensor via I2C
- **Real-time data logging** and visualization with a Python web server
- **Interactive dashboard** with temperature statistics and location mapping

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Computer                            │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Python Logger (ble_logger.py)                         │ │
│  │  - Reads serial data from Central board                │ │
│  │  - Parses BLE messages                                 │ │
│  │  - Converts to JSON                                    │ │
│  │  - Serves web dashboard on http://localhost:8000       │ |
│  └────────────────────────────────────────────────────────┘ │
└───────────────────┬─────────────────────────────────────────┘
                    │ Serial (COM14)
                    ┴──────────┐
                               │
  ┌───────▼───────┐    ┌────────▼────────┐
  │  Peripheral   │    │    Central      │
  │  Board #1     │    │    Board #2     │
  │ ───────────── │    | ─────────────── │
  │ • BME280 I2C  │    |  • BLE Receiver │
  │ • Reads Temp  │    |  • Serial Output│
  │ • BLE TX      │--->|  • BLE RX       │
  └───────────────┘    └─────────────────┘
        BLE Link (Wireless)
```

## Quick Start

### Prerequisites
- 2x nRF54L15 DK boards
- 1x BME280 sensor breakout
- USB cables for both boards
- Python 3.7+
- nRF Connect SDK v3.1.1

### Hardware Setup

**Peripheral Board (Board #1) - I2C BME280 Connections:**
- P1.11 → SCL
- P1.12 → SDA
- GND → GND
- VCC → VCC

**Central Board (Board #2):**
- Connected to PC via USB

### Building & Flashing

```bash
# Peripheral Board
cd peripheral_uart
west build -b nrf54l15dk_nrf54l15_cpuapp
west flash

# Central Board
cd ../central_uart
west build -b nrf54l15dk_nrf54l15_cpuapp
west flash
```

### Running the System

```bash
# Install Python dependencies
pip install pyserial

# Run the logger
python ble_logger.py

# Open dashboard in browser
http://localhost:8000
```

## How It Works

**Peripheral Board:** Reads BME280 temperature via I2C, broadcasts BLE message every 5 seconds:
```
LED: OFF, Temp: 24.10°C
```

**Central Board:** Receives BLE messages, outputs to serial port (COM14)

**Python Logger:** 
- Reads serial data from COM14 (115200 baud)
- Parses temperature and LED state
- Serves JSON API at `/api/data`
- Displays interactive web dashboard

## Dashboard Features

- **Real-time Temperature Display** - Current reading with LED status
- **Statistics** - Average, minimum, maximum temperatures
- **Recent Readings** - Last 10 measurements with timestamps
- **Interactive Map** - Location visualization (Trondheim, Norway)
- **Auto-refresh** - Updates every 2 seconds

## Web API

```bash
# Get current readings
curl http://localhost:8000/api/data
```

Response:
```json
{
  "readings": [
    {"timestamp": "2025-12-15T14:30:45", "led_state": "OFF", "temperature": 24.10, "unit": "°C"}
  ],
  "count": 42,
  "last_update": "2025-12-15T14:35:22"
}
```

## Configuration

Edit `ble_logger.py` to customize:
- `SERIAL_PORT` - COM port for Central board (default: COM14)
- `BAUD_RATE` - Serial baud rate (default: 115200)
- `WEB_PORT` - Web server port (default: 8000)

## Customization

**Change location:** Edit coordinates in `ble_logger.py`
```python
trondheimCoords = [63.4305, 10.3951]  # latitude, longitude
```

**Change colors:** Update CSS in the HTML section

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "COM port in use" | Close other serial terminals, restart logger |
| "Failed to open serial port" | Check board connection, verify correct COM port |
| No BLE connection | Ensure both boards are flashed and powered |
| Wrong temperature values | Verify BME280 wiring, check I2C pull-up resistors |

## Project Structure

- `peripheral_uart/` - BLE peripheral with BME280 temperature sensor
- `central_uart/` - BLE central receiver with serial output
- `ble_logger.py` - Python web server and data logger

## Features Implemented

- [x] BLE UART communication
- [x] BME280 temperature sensor with calibration
- [x] Real-time serial parsing
- [x] JSON API endpoint
- [x] Interactive web dashboard
- [x] Temperature statistics
- [x] Location mapping
- [x] Auto-refreshing display

## Resources

- [Nordic nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/)
- [BME280 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/)
- [Zephyr RTOS](https://docs.zephyrproject.org/)
- `boards/` - Board-specific configuration files

## License

Nordic Semiconductor proprietary