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

#define BME280_CHIP_ID                  0x60

#define BME280_CHIP_ID_ADDR             0xD0

#define BME280_SPI_R                    0x80

typedef enum  {
    SENSOR_OK       = 0x00U,
    SENSOR_NOT_OK   = 0x01U
} BME280_Stat;

class BMESensor_Handler {
    using read_fnc = function<u8(const u8, u8 *, u32)>;
    using write_fnc = function<u8(const u8, const u8 *, u32)>;

    typedef struct {
        uint8_t chipID;
        read_fnc user_read;
        write_fnc user_write;
        BME280_Stat status;
        function<void(uint32_t)> user_delay;
    } bme280;

private:
    bme280 dev_ = {};
public:
    BMESensor_Handler(read_fnc user_read,
                      write_fnc user_write);
    ~BMESensor_Handler();
    BME280_Stat init_BME280();
    uint8_t getChipID();
    uint8_t testFunctionPtr();

private:
    BME280_Stat getRegData(u8 regAddr, u8 *regData, u32 len);
};


#endif /* INC_BME280_DRIVER_H_ */
