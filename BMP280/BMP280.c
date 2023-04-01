/**
 * @file    : BMP280.c
 * @author  : Christian Roth.
 * @brief   : Implements the BMP280 Digital Pressure (and Temperature) Sensor for the ESP32S2.
 * 
 * @version : 0.1
 * @date    : 01.04.2023
 * 
 * @copyright Copyright (c) 2023
 */

#include "BMP280.h"
#include "driver/i2c.h"


// BMP280 GENERAL ADDRESS REGISTERS ---------------------------------------
#define BMP280_REGISTER_ID              0xD0
#define BMP280_REGISTER_CONTROL         0xF4
#define BMP280_CALIBRATION_DATA         0x88

// BMP280 TEMPERATURE -----------------------------------------------------
#define BMP280_TEMP_DATA_XLSB           0xFC
#define BMP280_TEMP_DATA_LSB            0xFB
#define BMP280_TEMP_DATA_MSB            0xFA

// BMP280 PRESSURE --------------------------------------------------------
#define BMP280_PRESSURE_DATA_LSB        0xF8
#define BMP280_PRESSURE_DATA_MSB        0xF7


/**
 *  Used for I2C data transmission.
 */
static int i2c_master_port;


/**
 *  @brief: Writes data to a register of a specified BMP280 device.
 *          Can also be used to write multiple registers if "length > 1".
 *            This will increase the register address automatically and writes the corresponding data.
 * 
 *  @param: I2C_DeviceAddr: The I2C address of the BMP280 device.
 *  @param: registerAddr  : The register that should get written to.
 *  @param: data[in]      : The data that should get stored.
 *                            data[0] = data to "registerAddr".
 *                            data[1] = data to (registerAddr + 1).
 *                            data[N] = data to (registerAddr + N).
 *  @param: length        : How many bytes should get written
 *                           and the length of the given "data" array. 
 */
static void readData(uint8_t I2C_DeviceAddr, uint8_t registerAddr, uint8_t* const data, uint8_t length);

/**
 *  @brief: Reads data from a register of a specified BMP280 device.
 *          Can also be used to read multiple registers if "length > 1".
 *            This will increase the register address automatically and stores the data into the next "data" index.
 * 
 *  @param: I2C_DeviceAddr: The I2C address of the BMP280 device.
 *  @param: registerAddr  : The register that should get read.
 *  @param: data[out]     : The data that got read.
 *                            data[0] = data from "registerAddr".
 *                            data[1] = data from (registerAddr + 1).
 *                            data[N] = data from (registerAddr + N).
 *  @param: length        : How many bytes should get read
 *                           and the length of the given "data" array. 
 */
static void writeData(uint8_t I2C_DeviceAddr, uint8_t registerAddr, uint8_t const* const data, uint8_t length);

/**
 *  @brief: Reads the calibration data of the BMP280 device and saves that data into the sBMP280_t structure.
 * 
 *  @param: _this: The structure that represents a BMP280 device.
 */
static void vBMP280_LoadCalibration(sBMP280_t* const _this);

/**
 * Transform given adc temperature data from BMP280 device to degree C.
 * Function is from datasheet p.22
 */
static int32_t bmp280_compensate_T_int32(sBMP280_t const* const _this, int32_t adc_T);

/**
 * Transform given adc pressure data from BMP280 device to degree C.
 * Function is from datasheet p.22
 */
static uint32_t bmp280_compensate_P_int64(sBMP280_t const* const _this, int32_t adc_P);



/* *** public functions *** */

void vBMP280_Init(sBMP280_t* const _this, sBMP280_Config_t* const config)
{
    // Initialize I2C
    i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->I2C_SDA_Pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = config->I2C_SCL_Pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    // Save config for BMP280 device.
    _this->deviceConfig = config;

    // Configure BMP280 with given device config.
    writeData(config->I2C_DeviceAddr, BMP280_REGISTER_CONTROL, &config->ctrl_meas, 1U);

    // Load device calibration.
    vBMP280_LoadCalibration(_this);
}

f32_t f32BMP280_GetTemperature(sBMP280_t const* const _this)
{
    uint16_t u16TemperatureADC = 0;
    uint8_t  data[2];
    
    // Read temp_lsb and temp_msb from device
    readData(_this->deviceConfig->I2C_DeviceAddr, BMP280_TEMP_DATA_MSB, data, 2U);
    ((uint8_t*)&u16TemperatureADC)[0] = data[1];
    ((uint8_t*)&u16TemperatureADC)[1] = data[0];

    // Read temp_xlsb and add the four msbit to u16TemperatureADC.
    readData(_this->deviceConfig->I2C_DeviceAddr, BMP280_TEMP_DATA_XLSB, data, 1U);

    // Calculate degree C with given function.
    int32_t s32TemperatureDegreeC = bmp280_compensate_T_int32( _this, ((u16TemperatureADC << 4U) | (data[0] >> 4U)) );
    return (s32TemperatureDegreeC / 100.0F);
}

f32_t f32BMP280_GetPressure(sBMP280_t const* const _this)
{
    uint16_t u16PressureADC = 0;
    uint8_t data[2];

    // Read press_lsb and press_msb from device
    readData(_this->deviceConfig->I2C_DeviceAddr, BMP280_PRESSURE_DATA_MSB, data, 2U);
    ((uint8_t*)&u16PressureADC)[0] = data[1];
    ((uint8_t*)&u16PressureADC)[1] = data[0];

    // Read press_xlsb and add the four msbit to u16PressureADC.
    readData(_this->deviceConfig->I2C_DeviceAddr, BMP280_TEMP_DATA_XLSB, data, 1U);

    uint32_t u32Pressure = bmp280_compensate_P_int64( _this, ((u16PressureADC << 4U) | (data[0] >> 4U)) );
    return (u32Pressure / 256.0F); // Need to divide the value by 256 to get Pa value
}



/* *** private functions *** */

static void vBMP280_LoadCalibration(sBMP280_t* const _this)
{
    // Get calibrated data from device and save it inside _this.
    readData(_this->deviceConfig->I2C_DeviceAddr, BMP280_CALIBRATION_DATA, (uint8_t*)_this, 24U);
} 

static void readData(uint8_t I2C_DeviceAddr, uint8_t registerAddr, uint8_t* const data, uint8_t length)
{
    // Send which register should get read
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (I2C_DeviceAddr << 1U) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, registerAddr, I2C_MASTER_ACK);
    
    // Read data from register
    i2c_master_start(i2c_cmd);     
    i2c_master_write_byte(i2c_cmd, (I2C_DeviceAddr << 1U) | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(i2c_cmd, data, length, I2C_MASTER_LAST_NACK);
    
    // End data transfer
    i2c_master_stop(i2c_cmd);
    i2c_master_cmd_begin(i2c_master_port, i2c_cmd, portMAX_DELAY);
    i2c_cmd_link_delete(i2c_cmd);
}

static void writeData(uint8_t I2C_DeviceAddr, uint8_t registerAddr, uint8_t const* const data, uint8_t length)
{
    // Send which register should get written to
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (I2C_DeviceAddr << 1U) | I2C_MASTER_WRITE, I2C_MASTER_ACK);

    // Send data
    for(int i=0; i<length; ++i)
    {
        i2c_master_write_byte(i2c_cmd, registerAddr, I2C_MASTER_ACK);
        i2c_master_write_byte(i2c_cmd, data[i], I2C_MASTER_ACK);
    }

    // End data transfer
    i2c_master_stop(i2c_cmd);
    i2c_master_cmd_begin(i2c_master_port, i2c_cmd, portMAX_DELAY);
    i2c_cmd_link_delete(i2c_cmd);
}



/* *** Datasheet defines functions *** */

static int32_t t_fine;
static int32_t bmp280_compensate_T_int32(sBMP280_t const* const _this, int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)_this->dig_T1 << 1))) * ((int32_t)_this->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)_this->dig_T1)) * ((adc_T >> 4) - ((int32_t)_this->dig_T1))) >> 12) * ((int32_t)_this->dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t bmp280_compensate_P_int64(sBMP280_t const* const _this, int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_this->dig_P6;
    var2 = var2 + ((var1 * (int64_t)_this->dig_P5) << 17);
    var2 = var2 + (((int64_t)_this->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_this->dig_P3) >> 8) + ((var1 * (int64_t)_this->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_this->dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_this->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_this->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)_this->dig_P7) << 4);
    return (uint32_t)p;
}
