#include <iostream>
#include "CPPI2C/cppi2c.h"
#include "mlx90614_i2c/mlx90614_i2c.h"


constexpr static int I2C_SDA = 3;
constexpr static int I2C_SCL = 2;
constexpr static uint32_t I2C_CLK_SPEED_HZ = 115200;  // 100kHz standard I2C speed

CPPI2C::I2c i2c {I2C_NUM_0};

extern "C" void app_main(void)
{    

    CPPMLX90614::MLX90614I2C mlx90614i2c;

    // Initialise the I2C
    i2c.InitMaster(I2C_SDA, I2C_SCL, I2C_CLK_SPEED_HZ, true, true);

    // Initialize the MLX90614 device
    mlx90614i2c.InitI2c(&i2c, 0x5A);
    
    while(true){
        double ambientemp = mlx90614i2c.readAmbientTempC();
        double temp = mlx90614i2c.readObjectTempC();
        
        std::cout << "Ambient Temperature: " << ambientemp << "°C\n";
        std::cout << "Object Temperature: " << temp << "°C\n";
        std::cout << "---\n";
        
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
    

}
