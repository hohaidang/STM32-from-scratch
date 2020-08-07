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
                                     write_fnc user_write,
                                     delay_ms_fnc user_delay) {
    dev_.user_read = user_read;
    dev_.user_write = user_write;
    dev_.delay_ms = user_delay;
    dev_.status = init_BME280();
}

BME280_Stat BMESensor_Handler::init_BME280() {
    BME280_Stat retStatus = SENSOR_OK;
    // read sensor ID
    uint8_t regAddr = BME280_CHIP_ID_ADDR;
    getRegData(regAddr, &dev_.chipID, 1);
    if(BME280_CHIP_ID == dev_.chipID) {
        // soft reset sensor
        retStatus = softReset();
    }
    else {
        retStatus = SENSOR_NOT_OK;
    }
    return retStatus;
}

BME280_Stat BMESensor_Handler::softReset() {
    uint8_t regAddr = BME280_SOFT_RST_ADDR;
    uint8_t softRstVal = BME280_RST_COMMAND;
    uint8_t regStatus = 0, tryRun = 5;
    BME280_Stat retStatus = SENSOR_OK;
    /* write soft reset command into the sensor */
    setRegData(regAddr, &softRstVal, 1);
    do
    {
        /* As per data sheet - Table 1, startup time is 2 ms. */
        dev_.delay_ms(2);
        getRegData(BME280_STATUS_REG_ADDR, &regStatus, 1);

    } while ((tryRun--) && (regStatus & BME280_STATUS_IM_UPDATE));

    if (regStatus & BME280_STATUS_IM_UPDATE)
    {
        retStatus = SENSOR_NOT_OK;
    }
    return retStatus;
}

BME280_Stat BMESensor_Handler::setSensorMode(const uint8_t sensorMode) {
    uint8_t currentMode = BME280_SLEEP_MODE;
    BME280_Stat retStatus = SENSOR_OK;
    retStatus = getSensorMode(currentMode);
    if(currentMode != BME280_SLEEP_MODE) {
        retStatus = putDeviceToSleep();
    }
    //TODO: Continue write power mode.
    return retStatus;
}

BME280_Stat BMESensor_Handler::getSensorMode(uint8_t &mode) {
    return getRegData(BME280_CTRL_MEAS_ADDR, &mode, 1);
}

BME280_Stat BMESensor_Handler::getRegData(u8 regAddr, u8 *regData, const u32 len) {
    regAddr |= BME280_SPI_R;
    dev_.user_read(regAddr, &regData[0], len);
    return SENSOR_OK;
}

BME280_Stat BMESensor_Handler::setRegData(u8 regAddr, const u8 *setData, const u32 len) {
    regAddr &= BME280_SPI_W;
    dev_.user_write(regAddr, &setData[0], len);
    return SENSOR_OK;
}

uint8_t BMESensor_Handler::getChipID() {
    return dev_.chipID;
}




#endif /* SRC_BME280_C_ */
