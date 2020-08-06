/*
 * bme280.c
 *
 *  Created on: Jul 31, 2020
 *      Author: prnsoft
 */

#ifndef SRC_BME280_C_
#define SRC_BME280_C_
#include "../inc/bme280_driver.h"



BMESensor_Handler::BMESensor_Handler(read_fnc user_read,
                                     write_fnc user_write) {
    dev_.user_read = user_read;
    dev_.user_write = user_write;
    dev_.status = init_BME280();
}

BME280_Stat BMESensor_Handler::init_BME280() {
    BME280_Stat retStatus = SENSOR_OK;
    // read sensor ID
    uint8_t regAddr = BME280_CHIP_ID_ADDR;
    getRegData(regAddr, &dev_.chipID, 1);
    if(BME280_CHIP_ID == dev_.chipID) {
        // soft reset sensor

    }
    else {
        retStatus = SENSOR_NOT_OK;
    }
    return retStatus;
}

BME280_Stat BMESensor_Handler::getRegData(u8 regAddr, u8 *regData, u32 len) {
    regAddr |= BME280_SPI_R;
    dev_.user_read(regAddr, &regData[0], len);
    return SENSOR_OK;
}


uint8_t BMESensor_Handler::getChipID() {
    return dev_.chipID;
}

uint8_t BMESensor_Handler::testFunctionPtr() {
    uint8_t data_reg = 0xD0;
    u8 data[1] = {0x00};
    return dev_.user_read(data_reg, data, 1);
}




#endif /* SRC_BME280_C_ */
