#pragma once

#include "CPPI2C/cppi2c.h"
// MLX90614 default register addresses
#define MLX90614_TA      0x06
#define MLX90614_TOBJ1   0x07
#define MLX90614_I2CADDR_DEFAULT 0x5A

namespace CPPMLX90614
{
    class MLX90614I2C 
    {
    private:
        CPPI2C::I2c *i2c;
        uint8_t _devAddress{};
        float readTemp(uint8_t reg);
        uint16_t read16(uint8_t reg);
        public:
        void InitI2c(CPPI2C::I2c *i_i2c, const uint8_t dev_addr = MLX90614_I2CADDR_DEFAULT);
        double readObjectTempC(void);
        double readAmbientTempC(void);
    }; // namespace CPPMLX90614
} // namespace CPPMLX90614