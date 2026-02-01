#include "mlx90614_i2c/mlx90614_i2c.h"

namespace CPPMLX90614
{
    void MLX90614I2C::InitI2c(CPPI2C::I2c *i_i2c, const uint8_t dev_addr)
    {
        i2c = i_i2c;
        _devAddress = dev_addr;
    }

    double MLX90614I2C::readObjectTempC(void)
    {
        return readTemp(MLX90614_TOBJ1);
    }

    double MLX90614I2C::readAmbientTempC(void){
        return readTemp(MLX90614_TA);
    }

    float MLX90614I2C::readTemp(uint8_t reg) {
        float temp = read16(reg);
        temp *= 0.02;
        temp -= 273.15;
        return temp;
    }
    uint16_t MLX90614I2C::read16(uint8_t reg) {
        uint8_t buffer_i2c[3];

        if (i2c->ReadRegisterMultipleBytes(_devAddress, reg, buffer_i2c, 3) != ESP_OK) {
            return 0xFFFF;
        }

        uint16_t result = buffer_i2c[0] | (buffer_i2c[1] << 8);
        // buffer[2] is PEC, not used here
        return result;
    }

} // namespace CPPBME280