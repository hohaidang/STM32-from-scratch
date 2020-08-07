/*
 * bme280.h
 *
 *  Created on: Jul 31, 2020
 *      Author: prnsoft
 */

#ifndef INC_BME280_DRIVER_H_
#define INC_BME280_DRIVER_H_
#include "stm32f4xx.h"
#include <vector>
#include <functional>
using namespace std;

#define BME280_CHIP_ID                  (u8)(0x60)
										
#define BME280_CTRL_MEAS_ADDR           (u8)(0xF4)
#define BME280_STATUS_REG_ADDR          (u8)(0xF3)
#define BME280_CHIP_ID_ADDR             (u8)(0xD0)
#define BME280_SOFT_RST_ADDR            (u8)(0xE0)
										
#define BME280_SPI_R                    (u8)(0x80)
#define BME280_SPI_W                    (u8)(0x7F)
#define BME280_STATUS_IM_UPDATE         (u8)(0x01)
										
/**\name Sensor power modes */          
#define BME280_SLEEP_MODE               (u8)(0x00)
#define BME280_FORCED_MODE              (u8)(0x01)
#define BME280_NORMAL_MODE              (u8)(0x03)

#define BME280_RST_COMMAND              (u8)(0xB6)

typedef enum  {
    SENSOR_OK       = 0x00U,
    SENSOR_NOT_OK   = 0x01U
} BME280_Stat;

class BMESensor_Handler {
    using read_fnc = function<u8(const u8, u8 *, u32)>;
    using write_fnc = function<u8(const u8, const u8 *, u32)>;
    using delay_ms_fnc = function<void(u32)>;

    /*!
     * @brief bme280 sensor structure which comprises of temperature, pressure and
     * humidity data
     */
    typedef struct {
        /*< Compensated pressure */
        double pressure;

        /*< Compensated temperature */
        double temperature;

        /*< Compensated humidity */
        double humidity;
    } bme280_data;

    typedef struct {
        uint8_t chipID;
        read_fnc user_read;
        write_fnc user_write;
        BME280_Stat status;
        delay_ms_fnc delay_ms;
    } bme280;

private:
    bme280 dev_ = {};
    bme280_data data_ = {};
public:
    BMESensor_Handler(read_fnc user_read, write_fnc user_write, delay_ms_fnc user_delay);
    ~BMESensor_Handler();
    BME280_Stat init_BME280();
    uint8_t getChipID();
    BME280_Stat softReset();
    BME280_Stat setSensorMode(const uint8_t sensorMode);
    BME280_Stat getSensorMode(uint8_t &mode);
    BME280_Stat putDeviceToSleep();

private:
    BME280_Stat getRegData(u8 regAddr, u8 *regData, const u32 len);
    BME280_Stat setRegData(u8 regAddr, const u8 *setData, const u32 len);
};


#endif /* INC_BME280_DRIVER_H_ */
