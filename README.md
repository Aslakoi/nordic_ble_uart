# Nordic BLE UART

A Bluetooth Low Energy UART bridge application for Nordic Semiconductor nRF5340 and nRF54x development boards, featuring a central and peripheral implementation with button-triggered LED control.

## Features

- **Central UART**: Connects to peripheral devices and relays data over UART
- **Peripheral UART**: Advertises BLE service, manages button/LED control, and transmits state via BLE
- **Multi-board support**: Ideally we want to tansmit sensordata with the thingy91:X. This can be done by creating another build for the board in the peripheral_uart application
- **Button Integration**: Press to toggle LED and transmit state over BLE
- **Periodic Updates**: Sends LED state periodically via BLE

## Requirements

- **nRF Connect SDK** (tested with recent versions)
- **Zephyr RTOS**
- Supported development board (see Features)
- **west** build tool

## Building

```bash
# Build peripheral application for nRF5340 DK
west build -b nrf5340dk_nrf5340_cpuapp peripheral_uart

# Build central application for nRF5340 DK
west build -b nrf5340dk_nrf5340_cpuapp central_uart
```

## Flashing

```bash
# Flash peripheral
west flash

# Or specify board
west flash -b nrf5340dk_nrf5340_cpuapp
```

## Usage

1. Flash the **peripheral** application to one board
2. Flash the **central** application to another board (or use a phone with BLE terminal app)
3. Press the button on the peripheral board to toggle the LED and send state
4. Central device receives and displays the messages over UART

## Project Structure

- `peripheral_uart/` - Peripheral BLE device with button/LED control
- `central_uart/` - Central device that connects to peripherals
- `boards/` - Board-specific configuration files

## License

Nordic Semiconductor proprietary