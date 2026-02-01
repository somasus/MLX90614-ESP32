# MLX90614 Temperature Sensor with ESP32

A C++ library for interfacing the MLX90614 infrared temperature sensor with ESP32 using ESP-IDF.

## Features

- 🌡️ Read ambient temperature
- 🎯 Read object temperature (non-contact)
- 🔌 I2C communication interface
- 📦 Modular component architecture
- ✨ Modern C++17 implementation

## Hardware Requirements

- ESP32 development board
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
git clone https://github.com/YOUR_USERNAME/MLX90614.git
cd MLX90614
```

### 2. Set up ESP-IDF Environment

```bash
# If you haven't set up ESP-IDF, follow the official guide:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

# Activate ESP-IDF environment
. $HOME/esp/esp-idf/export.sh  # Linux/Mac
# OR
%userprofile%\esp\esp-idf\export.bat  # Windows
```

### 3. Configure the Project (Optional)

```bash
idf.py menuconfig
```

### 4. Build the Project

```bash
idf.py build
```

### 5. Flash to ESP32

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with your ESP32's serial port (e.g., `/dev/ttyUSB0` on Linux, `COM3` on Windows).

## Usage

### Basic Example

```cpp
#include "CPPI2C/cppi2c.h"
#include "mlx90614_i2c/mlx90614_i2c.h"

constexpr static int I2C_SDA = 3;
constexpr static int I2C_SCL = 2;
constexpr static uint32_t I2C_CLK_SPEED_HZ = 100000;  // 100kHz

CPPI2C::I2c i2c {I2C_NUM_0};

extern "C" void app_main(void)
{    
    CPPMLX90614::MLX90614I2C mlx90614i2c;

    // Initialize I2C
    i2c.InitMaster(I2C_SDA, I2C_SCL, I2C_CLK_SPEED_HZ, true, true);

    // Initialize MLX90614
    mlx90614i2c.InitI2c(&i2c, 0x5A);
    
    // Check if sensor is connected
    esp_err_t ret = i2c.Scan(0x5A);
    if (ret == ESP_OK) {
        std::cout << "MLX90614 found!\n";
    } else {
        std::cout << "MLX90614 not found!\n";
        return;
    }
    
    mlx90614i2c.Init();
    mlx90614i2c.SetMode(1);
    
    // Read temperatures continuously
    while(true) {
        double ambientTemp = mlx90614i2c.readAmbientTempC();
        double objectTemp = mlx90614i2c.readObjectTempC();
        
        std::cout << "Ambient Temperature: " << ambientTemp << "°C\n";
        std::cout << "Object Temperature: " << objectTemp << "°C\n";
        std::cout << "---\n";
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
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

## API Reference

### CPPI2C::I2c

```cpp
// Initialize I2C in master mode
esp_err_t InitMaster(int sda_io_num, int scl_io_num, uint32_t clk_speed,
                     bool sda_pullup_en = false, bool scl_pullup_en = false);

// Scan for device at address
esp_err_t Scan(uint8_t dev_addr);

// Read single register
uint8_t ReadRegister(uint8_t dev_addr, uint8_t reg_addr);

// Write single register
esp_err_t WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t txData);

// Read multiple bytes
esp_err_t ReadRegisterMultipleBytes(uint8_t dev_addr, uint8_t reg_addr, 
                                    uint8_t *rx_data, int length);
```

### CPPMLX90614::MLX90614I2C

```cpp
// Initialize I2C interface
void InitI2c(CPPI2C::I2c *i_i2c, const uint8_t dev_addr = 0x5A);

// Initialize sensor
void Init(void);

// Set operating mode
void SetMode(uint8_t mode);

// Read ambient temperature in Celsius
double readAmbientTempC(void);

// Read object temperature in Celsius
double readObjectTempC(void);
```

## Troubleshooting

### Sensor Not Found
- Check wiring connections
- Verify pull-up resistors are present (4.7kΩ)
- Ensure sensor is powered with 3.3V
- Try different GPIO pins

### Invalid Temperature Readings
- Check I2C clock speed (should be 100kHz or 400kHz)
- Verify sensor address (default is 0x5A)
- Ensure proper initialization sequence
- Check for I2C bus conflicts

### Compilation Errors
- Ensure ESP-IDF is properly installed and activated
- Check CMake version (3.16+)
- Verify C++17 support

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is open source and available under the [MIT License](LICENSE).

## Acknowledgments

- ESP-IDF framework by Espressif Systems
- MLX90614 datasheet by Melexis

## Contact

Your Name - [@your_twitter](https://twitter.com/your_twitter)

Project Link: [https://github.com/YOUR_USERNAME/MLX90614](https://github.com/YOUR_USERNAME/MLX90614)

## References

- [MLX90614 Datasheet](https://www.melexis.com/en/product/MLX90614/Digital-Plug-Play-Infrared-Thermometer-TO-Can)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 I2C Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)
