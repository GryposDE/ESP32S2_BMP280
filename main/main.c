
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "../BMP280/BMP280.h"

void app_main(void)
{
    // Sensor configuration.
    sBMP280_Config_t config;
    config.ctrl_meas = ( BMP280_TEMP_MODE_NORMAL | BMP280_PRESSURE_MODE_NORMAL | BMP280_FORCED_MODE );
    config.I2C_DeviceAddr = 0x76;
    config.I2C_SCL_Pin = 5;
    config.I2C_SDA_Pin = 6;
    
    // Sensor initialization.
    sBMP280_t mySensor;
    vBMP280_Init(&mySensor, &config);

    while(1)
    {
        printf("\n");
        printf("BMP280:: current pressure: %.2f\n", f32BMP280_GetPressure(&mySensor));
        printf("BMP280:: current temp: %.2f\n", f32BMP280_GetTemperature(&mySensor));

        vTaskDelay(300);
    }

    vTaskDelay(10000);
}