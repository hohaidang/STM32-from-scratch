/*
 * bme280.c
 *
 *  Created on: Jul 31, 2020
 *      Author: prnsoft
 */

#ifndef SRC_BME280_C_
#define SRC_BME280_C_
#include "../inc/bme280_driver.h"
#include <vector>

#define BME280_CHIP_ID                  (u8)(0x60)

#define BME280_DATA_ADDR                (u8)(0xF7)
#define BME280_CTRL_MEAS_ADDR           (u8)(0xF4)
#define BME280_STATUS_REG_ADDR          (u8)(0xF3)
#define BME280_CTRL_HUM_ADDR            (u8)(0xF2)
#define BME280_CHIP_ID_ADDR             (u8)(0xD0)
#define BME280_SOFT_RST_ADDR            (u8)(0xE0)

#define BME280_SPI_R                    (u8)(0x80)
#define BME280_SPI_W                    (u8)(0x7F)
#define BME280_STATUS_IM_UPDATE         (u8)(0x01)
#define BME280_RST_COMMAND              (u8)(0xB6)

#define BME280_SENSOR_MODE_POS          (u8)(0x00)
#define BME280_SENSOR_MODE_MSK          (u8)(0x03)

#define BME280_CTRL_HUM_MSK             (u8)(0x07)
#define BME280_CTRL_HUM_POS             (u8)(0x00)
#define BME280_CTRL_TEMPERATURE_MSK     (u8)(0xE0)
#define BME280_CTRL_TEMPERATURE_POS     (u8)(0x05)
#define BME280_CTRL_PRESSURE_MSK        (u8)(0x1C)
#define BME280_CTRL_PRESSURE_POS        (u8)(0x02)
#define BME280_FILTER_MSK               (u8)(0x1C)
#define BME280_FILTER_POS               (u8)(0x02)
#define BME280_STANDBY_MSK              (u8)(0xE0)
#define BME280_STANDBY_POS              (u8)(0x05)

#define BME280_TEMP_PRESS_CALIB_DATA_LEN          (u8)(28u)
#define BME280_HUMIDITY_CALIB_DATA_LEN            (u8)(7u)
#define BME280_P_T_H_DATA_LEN                     (u8)(8u)

#define BME280_TEMP_PRESS_CALIB_DATA_ADDR         (u8)(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR           (u8)(0xE1)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

static inline u8 BME280_SET_BITS(const u8, const u8, const u8, const u8);
static inline u8 BME280_GET_BITS(const u8, const u8, const u8);



BMESensor_Handler::BMESensor_Handler(read_fnc user_read,
                                     write_fnc user_write,
                                     delay_ms_fnc user_delay) {
    if(user_read == nullptr || user_write == nullptr || user_delay == nullptr) {
        dev_.status = SENSOR_NOT_OK;
        return;
    }
    dev_.user_read = user_read;
    dev_.user_write = user_write;
    dev_.delay_ms = user_delay;
    dev_.status = init_BME280();
}

BME280_Stat BMESensor_Handler::init_BME280() {
    BME280_Stat ret = SENSOR_OK;
    // read sensor ID
    uint8_t regAddr = BME280_CHIP_ID_ADDR;
    getRegData(regAddr, &dev_.chipID, 1);
    if(BME280_CHIP_ID == dev_.chipID) {
        // soft reset sensor
        ret = softReset();
        if(SENSOR_OK == ret) {
            /* Read the calibration data */
            ret = get_calib_data();
        }
    }
    else {
        ret = SENSOR_NOT_OK;
    }
    return ret;
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

BME280_Stat BMESensor_Handler::get_calib_data() {
    u8 calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = { 0 };
    getRegData(BME280_TEMP_PRESS_CALIB_DATA_ADDR, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN);
    parse_temp_calib_data(calib_data);
    getRegData(BME280_HUMIDITY_CALIB_DATA_ADDR, calib_data, BME280_HUMIDITY_CALIB_DATA_LEN);
    parse_humidity_calib_data(calib_data);
    return SENSOR_OK;
}

void BMESensor_Handler::parse_temp_calib_data(u8 *reg_data) {
    dev_.calib_data.dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    dev_.calib_data.dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    dev_.calib_data.dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    dev_.calib_data.dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    dev_.calib_data.dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    dev_.calib_data.dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    dev_.calib_data.dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    dev_.calib_data.dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    dev_.calib_data.dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    dev_.calib_data.dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    dev_.calib_data.dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    dev_.calib_data.dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    dev_.calib_data.dig_h1 = reg_data[25];
}

void BMESensor_Handler::parse_humidity_calib_data(u8 *reg_data) {
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    dev_.calib_data.dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    dev_.calib_data.dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    dev_.calib_data.dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    dev_.calib_data.dig_h5 = dig_h5_msb | dig_h5_lsb;
    dev_.calib_data.dig_h6 = (int8_t)reg_data[6];
}

bme280_data BMESensor_Handler::get_sensor_data() {
    bme280_uncomp_data uncomp_data = { 0 };
    u8 reg_data[BME280_P_T_H_DATA_LEN] = { 0 };
    getRegData(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN);
    parse_sensor_data(reg_data, uncomp_data);
    compensate_data(uncomp_data);
    return data_;
}

void BMESensor_Handler::compensate_data(const bme280_uncomp_data &uncomp_data) {
    data_.temperature = compensate_temperature(uncomp_data);
    data_.pressure = compensate_pressure(uncomp_data);
    data_.humidity = compensate_humidity(uncomp_data);
}

void BMESensor_Handler::print_sensor_data() {
#ifdef DEBUG_EN
    float temp, hum, pres;
    temp = data_.temperature;
    pres = 0.01 * data_.pressure;
    hum = data_.humidity;

    cout << fixed << setprecision(2) << temp << " deg C, "
                                     << pres << " hPa, "
                                     << hum << " % " << endl;
#endif
}

BME280_Stat BMESensor_Handler::setSensorMode(const u8 sensorMode) {
    u8 currentMode = BME280_SLEEP_MODE;
    u8 ctrl_means_reg = 0x00;
    BME280_Stat ret = SENSOR_OK;
    /* get ctrl means reg */
    ret = getRegData(BME280_CTRL_MEAS_ADDR, &ctrl_means_reg, 1);
    currentMode = BME280_GET_BITS(ctrl_means_reg, BME280_SENSOR_MODE_MSK, BME280_SENSOR_MODE_POS);


    if(currentMode != BME280_SLEEP_MODE) {
        ret = put_device_to_sleep(ctrl_means_reg);
    }

    if(SENSOR_OK == ret) {
        ret = write_power_mode(sensorMode, ctrl_means_reg);
    }
    return ret;
}

BME280_Stat BMESensor_Handler::put_device_to_sleep(u8 &ctrl_means_reg) {
    u8 reg_set = BME280_SET_BITS(ctrl_means_reg,
                                 BME280_SLEEP_MODE,
                                 BME280_SENSOR_MODE_MSK,
                                 BME280_SENSOR_MODE_POS);
    return setRegData(BME280_CTRL_MEAS_ADDR, &reg_set, 1);
}

BME280_Stat BMESensor_Handler::get_status() {
    return dev_.status;
}

BME280_Stat BMESensor_Handler::write_power_mode(const u8 mode_set, u8 ctrl_means_reg) {
    ctrl_means_reg = BME280_SET_BITS(ctrl_means_reg,
                                     mode_set,
                                     BME280_SENSOR_MODE_MSK,
                                     BME280_SENSOR_MODE_POS);

    return setRegData(BME280_CTRL_MEAS_ADDR, &ctrl_means_reg, 1);
}

BME280_Stat BMESensor_Handler::getSensorMode(u8 &sensor_mode) {
    u8 ctrl_means_reg = 0x00;
    BME280_Stat ret = SENSOR_OK;
    ret = getRegData(BME280_CTRL_MEAS_ADDR, &ctrl_means_reg, 1);
    sensor_mode = BME280_GET_BITS(ctrl_means_reg, BME280_SENSOR_MODE_MSK, BME280_SENSOR_MODE_POS);
    return ret;
}

BME280_Stat BMESensor_Handler::set_sensor_settings(const bme280_settings &desired_settings) {
    u8 ctrl_means_reg = 0x00;
    BME280_Stat ret = SENSOR_OK;
    /* get ctrl means reg */
    ret = getRegData(BME280_CTRL_MEAS_ADDR, &ctrl_means_reg, 1);
    if(BME280_SLEEP_MODE != BME280_GET_BITS(ctrl_means_reg, BME280_SENSOR_MODE_MSK, BME280_SENSOR_MODE_POS)) {
        ret = put_device_to_sleep(ctrl_means_reg);
    }

    if(SENSOR_OK == ret) {
        // change humidity osr
        if(desired_settings.osr_h != dev_.settings.osr_h) {
            set_osr_humidity_settings(desired_settings.osr_h);
        }

        // change temperature and pressure osr
        if(desired_settings.osr_t != dev_.settings.osr_t || desired_settings.osr_p != dev_.settings.osr_p) {
            set_osr_temp_pres_settings(ctrl_means_reg, desired_settings);
        }

        // Add filter settings here
    }

    // update new settings
    dev_.settings.osr_h = desired_settings.osr_h;
    dev_.settings.osr_t = desired_settings.osr_t;
    dev_.settings.osr_p = desired_settings.osr_p;
    return ret;
}

void BMESensor_Handler::set_osr_temp_pres_settings(u8 &cur_ctrl_means_reg, const bme280_settings settings) {
    cur_ctrl_means_reg = BME280_SET_BITS(cur_ctrl_means_reg, settings.osr_t, BME280_CTRL_TEMPERATURE_MSK, BME280_CTRL_TEMPERATURE_POS);
    cur_ctrl_means_reg = BME280_SET_BITS(cur_ctrl_means_reg, settings.osr_p, BME280_CTRL_PRESSURE_MSK, BME280_CTRL_PRESSURE_POS);
    setRegData(BME280_CTRL_MEAS_ADDR, &cur_ctrl_means_reg, 1);
}

void BMESensor_Handler::set_osr_humidity_settings(const u8 osr_h) {
    uint8_t ctrl_hum = 0x00, ctrl_means = 0x00;

    ctrl_hum = osr_h & BME280_CTRL_HUM_MSK;
    /* Write the humidity control value in the register */
    setRegData(BME280_CTRL_HUM_ADDR, &ctrl_hum, 1);
    /* Humidity related changes will be only effective after a
     * write operation to ctrl_meas register
     */
    getRegData(BME280_CTRL_MEAS_ADDR, &ctrl_means, 1);
    setRegData(BME280_CTRL_MEAS_ADDR, &ctrl_means, 1);
}

BME280_Stat BMESensor_Handler::get_sensor_settings(bme280_settings &settings) {
    u8 reg_data[4] = {};
    BME280_Stat ret = SENSOR_OK;
    /* get 4 registers, ctrl_hum, status, ctrl_means, config */
    ret = getRegData(BME280_CTRL_HUM_ADDR, reg_data, 4);
    parse_device_settings(reg_data, settings);
    return ret;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void BMESensor_Handler::parse_sensor_data(u8 *reg_data, bme280_uncomp_data &uncomp_data) {
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (uint32_t)reg_data[0] << 12;
    data_lsb = (uint32_t)reg_data[1] << 4;
    data_xlsb = (uint32_t)reg_data[2] >> 4;
    uncomp_data.pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t)reg_data[3] << 12;
    data_lsb = (uint32_t)reg_data[4] << 4;
    data_xlsb = (uint32_t)reg_data[5] >> 4;
    uncomp_data.temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb = (uint32_t)reg_data[6] << 8;
    data_lsb = (uint32_t)reg_data[7];
    uncomp_data.humidity = data_msb | data_lsb;
}

void BMESensor_Handler::parse_device_settings(const u8 *reg_data, bme280_settings &settings) {
    settings.osr_h = BME280_GET_BITS(reg_data[0], BME280_CTRL_HUM_MSK, BME280_CTRL_HUM_POS);
    settings.osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESSURE_MSK, BME280_CTRL_PRESSURE_POS);
    settings.osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMPERATURE_MSK, BME280_CTRL_TEMPERATURE_POS);
    settings.filter = BME280_GET_BITS(reg_data[3], BME280_FILTER_MSK, BME280_FILTER_POS);
    settings.standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY_MSK, BME280_STANDBY_POS);
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

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
double BMESensor_Handler::compensate_temperature(const bme280_uncomp_data &uncomp_data)
{
    double var1;
    double var2;
    double temperature { 0 };
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = ((double)uncomp_data.temperature) / 16384.0 - ((double)dev_.calib_data.dig_t1) / 1024.0;
    var1 = var1 * ((double)dev_.calib_data.dig_t2);
    var2 = (((double)uncomp_data.temperature) / 131072.0 - ((double)dev_.calib_data.dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)dev_.calib_data.dig_t3);
    dev_.calib_data.t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
double BMESensor_Handler::compensate_pressure(const bme280_uncomp_data &uncomp_data)
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)dev_.calib_data.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dev_.calib_data.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)dev_.calib_data.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dev_.calib_data.dig_p4) * 65536.0);
    var3 = ((double)dev_.calib_data.dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)dev_.calib_data.dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dev_.calib_data.dig_p1);

    /* avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) uncomp_data.pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)dev_.calib_data.dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)dev_.calib_data.dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)dev_.calib_data.dig_p7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
double BMESensor_Handler::compensate_humidity(const bme280_uncomp_data &uncomp_data)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)dev_.calib_data.t_fine) - 76800.0;
    var2 = (((double)dev_.calib_data.dig_h4) * 64.0 + (((double)dev_.calib_data.dig_h5) / 16384.0) * var1);
    var3 = uncomp_data.humidity - var2;
    var4 = ((double)dev_.calib_data.dig_h2) / 65536.0;
    var5 = (1.0 + (((double)dev_.calib_data.dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)dev_.calib_data.dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)dev_.calib_data.dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

static inline u8 BME280_SET_BITS(const u8 reg, const u8 data, const u8 msk, const u8 pos) {
    return (u8)( ( (reg & ~(msk)) | ((data << pos) & msk) ) );
}

static inline u8 BME280_GET_BITS(const u8 reg, const u8 msk, const u8 pos) {
    return (u8)((reg & msk) >> pos);
}


#endif /* SRC_BME280_C_ */
