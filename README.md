# MLX90614 Temperature Sensor with ESP32

A C++ library for interfacing the MLX90614 infrared temperature sensor with ESP32 using ESP-IDF.

## Features

- 🌡️ Read ambient temperature
- 🎯 Read object temperature (non-contact)
- 🔌 I2C communication interface
- 📦 Modular component architecture
- ✨ Modern C++17 implementation

## Hardware Requirements

- ESP32 development board(I have used ESP32-C6 for this project)
- MLX90614 infrared temperature sensor
- Pull-up resistors (4.7kΩ) for I2C lines (if not built-in)

## Wiring

| MLX90614 Pin | ESP32 Pin | Description |
|--------------|-----------|-------------|
| VCC          | 3.3V      | Power supply |
| GND          | GND       | Ground |
| SDA          | GPIO 3    | I2C Data line |
| SCL          | GPIO 2    | I2C Clock line |

**Note:** Make sure to use 4.7kΩ pull-up resistors on SDA and SCL lines to 3.3V.

## Software Requirements

- ESP-IDF v4.4 or later
- CMake 3.16 or later
- C++17 compatible compiler

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/somasus/MLX90614-ESP32.git
cd MLX90614
```

### 2. Set up ESP-IDF Environment

```bash
# If you haven't set up ESP-IDF, follow the official guide:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
# or u can follow this easy guide from embeddedtutorials.com
# https://embeddedtutorials.com/eps32/get-started-with-esp-idf-windows/
```

## Project Structure

```
MLX90614/
├── CMakeLists.txt              # Root CMake configuration
├── main/
│   ├── CMakeLists.txt          # Main component configuration
│   └── main.cpp                # Application entry point
└── components/
    ├── CPPI2C/                 # I2C wrapper library
    │   ├── CMakeLists.txt
    │   ├── include/CPPI2C/
    │   │   └── cppi2c.h
    │   └── src/CPPI2C/
    │       └── cppi2c.cpp
    └── mlx90614_i2c/           # MLX90614 driver
        ├── CMakeLists.txt
        ├── include/mlx90614_i2c/
        │   └── mlx90614_i2c.h
        └── src/mlx90614_i2c/
            └── mlx90614_i2c.cpp
```

## Components

### CPPI2C
A C++ wrapper for ESP-IDF's I2C driver, providing:
- Easy I2C initialization
- Device scanning
- Register read/write operations
- Multi-byte read/write support

### mlx90614_i2c
MLX90614 sensor driver providing:
- Temperature reading (ambient and object)
- I2C communication handling
- Simple initialization

## Troubleshooting

### Sensor Not Found
- Check wiring connections
- Verify pull-up resistors are present (4.7kΩ)
- Ensure sensor is powered with 3.3V
- Try different GPIO pins

### Invalid Temperature Readings
- Check I2C clock speed 
- Verify sensor address (default is 0x5A for MLX90614)
- Ensure proper initialization sequence
- Check for I2C bus conflicts

### Compilation Errors
- Ensure ESP-IDF is properly installed and activated
- Check CMake version (3.16+)
- Verify C++17 support

## License

This project is open source and available under the [MIT License](LICENSE).

## References

- [Get started with esp idf windows](https://embeddedtutorials.com/eps32/get-started-with-esp-idf-windows/)
- [ESP-IDF C++ with CMake for ESP32](https://embeddedtutorials.com/eps32/esp-idf-cpp-with-cmake-for-esp32/)
- [MLX90614 Datasheet](https://www.melexis.com/en/product/MLX90614/Digital-Plug-Play-Infrared-Thermometer-TO-Can)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 I2C Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)
