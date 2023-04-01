/**
 * @file    : BMP280.h
 * @author  : Christian Roth.
 * @brief   : Implements the BMP280 Digital Pressure (and Temperature) Sensor for the ESP32S2.
 * 
 * @version : 0.1
 * @date    : 01.04.2023
 * 
 * @copyright Copyright (c) 2023
 */

#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
typedef float  f32_t;
typedef double f64_t;



// BMP280 TEMPERATURE -----------------------------------------------------
#define BMP280_POS_CONTROL_TEMP          5U
#define BMP280_MASK_CONTROL_TEMP        (7U << BMP280_POS_CONTROL_TEMP)

#define BMP280_TEMP_MODE_OFF            (0U << BMP280_POS_CONTROL_TEMP)
#define BMP280_TEMP_MODE_NORMAL         (1U << BMP280_POS_CONTROL_TEMP)
#define BMP280_TEMP_MODE_ULTRA          (7U << BMP280_POS_CONTROL_TEMP)
    // ToDo: theoretical 4 more modes, but maybe later (p.13 datasheet)

// BMP280 PRESSURE --------------------------------------------------------
#define BMP280_POS_CONTROL_PRESSURE      2U
#define BMP280_MASK_CONTROL_PRESSURE    (7U << BMP280_POS_CONTROL_PRESSURE)

#define BMP280_PRESSURE_MODE_OFF        (0U << BMP280_POS_CONTROL_PRESSURE)
#define BMP280_PRESSURE_MODE_NORMAL     (1U << BMP280_POS_CONTROL_PRESSURE)
#define BMP280_PRESSURE_MODE_ULTRA      (7U << BMP280_POS_CONTROL_PRESSURE)
    // ToDo: theoretical 4 more modes, but maybe later (p.25 datasheet)

// BMP280 DEVICE MODE -----------------------------------------------------
#define BMP280_POS_CONTROL_MODE          0U
#define BMP280_MASK_CONTROL_MODE        (3U << BMP280_POS_CONTROL_MODE)

#define BMP280_SLEEP_MODE               (0U << BMP280_POS_CONTROL_MODE)
#define BMP280_NORMAL_MODE              (1U << BMP280_POS_CONTROL_MODE)
#define BMP280_FORCED_MODE              (3U << BMP280_POS_CONTROL_MODE)



typedef struct
{
    union
    {
        struct
        {
            uint8_t osrs_t : 3; // temperature configuration [5::7]
            uint8_t osrs_p : 3; // pressure configuration    [2::4]  
            uint8_t mode   : 2; // device mode               [0::1]
        };
        uint8_t ctrl_meas;
    };
    uint8_t I2C_DeviceAddr; // 7bit i2c device address.

    uint8_t I2C_SDA_Pin;
    uint8_t I2C_SCL_Pin;
    
} sBMP280_Config_t;


typedef struct
{
    // Device specific temperature calibration.
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    // Device specific pressure calibration.
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    // Device configuration from initialization.
    sBMP280_Config_t* deviceConfig;

} sBMP280_t;



/**
 * @brief: Initialized the BMP280 device with the given configuration.
 * 
 * @param _this : The BMP280 device handler used for other functions.
 *                 Will be loaded with the device calibration data.
 * @param config: The configuration for the BMP280 device.
 */
void vBMP280_Init(sBMP280_t* const _this, sBMP280_Config_t* const config);

/**
 * @brief: Measures the current temperature and returns the current temperature in Celsius.
 *         Note: Accuracy depends on pressure configuration at initialization.
 *               For more info look into the datasheet p.13
 * 
 * @param _this: The BMP280 device handler.
 * 
 * @return f32_t: The current temperature in Celsius.
 */
f32_t f32BMP280_GetTemperature(sBMP280_t const* const _this);

/**
 * @brief: Measures the current pressure and returns the current pressure in Pa.
 *         Note: Accuracy depends on pressure configuration at initialization.
 *               For more info look into the datasheet p.12
 * 
 * @param _this: The BMP280 device handler.
 * 
 * @return f32_t: The current pressure in Pa.
 */
f32_t f32BMP280_GetPressure(sBMP280_t const* const _this);


#endif /* BMP280_H */
