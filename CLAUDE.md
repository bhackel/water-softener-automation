# Water Softener Automation - Claude Context

## Project Overview
This is an Arduino-based water softener automation system using the Arduino Nano 33 BLE. The system monitors water quality through TDS (Total Dissolved Solids) and temperature sensors, with Bluetooth Low Energy connectivity for remote monitoring.

## Architecture
- **Hardware**: Arduino Nano 33 BLE
- **Sensors**: TDS sensor, Temperature sensor (Dallas Temperature), Ultrasonic sensor
- **Connectivity**: Bluetooth Low Energy (BLE)
- **Data Storage**: Local sensor data logging
- **Client**: iOS app for real-time monitoring and data visualization

## Key Components

### Core Files
- `src/main.cpp` - Main Arduino loop with cooperative multitasking
- `src/sensor_manager.cpp` - Core sensor and automation logic
- `src/bluetooth_service.cpp` - BLE service implementation
- `src/data_logger.cpp` - Data logging operations

### Headers
- `include/sensor_manager.h` - Core system definitions and shared variables
- `include/bluetooth_service.h` - BLE service interface
- `include/data_logger.h` - Data logging interface

## Build System
- **Platform**: PlatformIO (Arduino framework)
- **Target**: Nordic nRF52 (Arduino Nano 33 BLE)

## Dependencies
### PlatformIO Libraries
- ArduinoJson (^7.4.1)
- OneWire (^2.3.8) 
- DallasTemperature (^4.0.4)
- ArduinoBLE (^1.4.1)


## Build Commands
```bash
pio run                  # Build
pio run --target upload  # Upload to board
```

## System Architecture
The system uses cooperative multitasking with these main tasks:
- `body_button()` - Button handling
- `body_sequence()` - Main automation sequence
- `body_led()` - LED status indicators  
- `body_ultrasonic()` - Ultrasonic distance sensing
- `body_tds()` - TDS water quality monitoring
- `body_persistent_temperature()` - Temperature logging
- `body_persistent_log_to_database()` - Data logging
- `handleBluetoothEvents()` - BLE communication

## BLE Service
- Device Name: "Water"
- Exposes TDS and temperature data
- Handles client connections and data requests

## Development Notes
- Single-threaded cooperative loop (no threads)
- Non-blocking task implementations
- Real-time sensor data collection
- Sensor data logging
- iOS app with real-time BLE connectivity and time-series graphing

## iOS App Plans
- **Features**: Real-time BLE connectivity, historical data storage, time-series graphing
- **Storage**: Core Data for local sensor reading persistence
- **Charts**: Dual y-axis overlay charts (TDS + Temperature) using Swift Charts
- **BLE**: Connects to "Water" device, reads TDS and temperature characteristics
- **CI/CD**: GitHub Actions for build, test, and IPA generation

## Recent Changes
- Migrated from Raspberry Pi to Arduino Nano 33 BLE
- Implemented BLE-based communication
- Updated pin configurations
- Removed SD storage and SQLite dependencies
- Renamed source files: `assignment1` → `sensor_manager`, `database` → `data_logger`