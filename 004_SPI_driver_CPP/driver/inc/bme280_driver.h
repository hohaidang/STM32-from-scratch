/*
 * bme280.h
 *
 *  Created on: Jul 31, 2020
 *      Author: prnsoft
 */

#ifndef INC_BME280_DRIVER_H_
#define INC_BME280_DRIVER_H_
#include "stm32f4xx.h"

#include <functional>
using namespace std;

/* Maximum size of SPI read or write each time is 28 bytes */
#define BME280_MAX_SIZE_WR              (28U)

/**@SENSOR_MODE Sensor power modes */
#define BME280_SLEEP_MODE               static_cast<u8>(0x00)
#define BME280_FORCED_MODE              static_cast<u8>(0x01)
#define BME280_NORMAL_MODE              static_cast<u8>(0x03)

typedef enum {
    SENSOR_OK = 0x00U,
    SENSOR_NOT_OK = 0x01U
} BME280_Stat;

/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */
struct bme280_settings {
    /*< pressure oversampling */
    uint8_t osr_p;

    /*< temperature oversampling */
    uint8_t osr_t;

    /*< humidity oversampling */
    uint8_t osr_h;

    /*< filter coefficient */
    uint8_t filter;

    /*< standby time */
    uint8_t standby_time;
};

/*!
 * @brief Calibration data
 */
struct bme280_calib_data {
    /*< Calibration coefficient for the temperature sensor */
    uint16_t dig_t1;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t2;

    /*< Calibration coefficient for the temperature sensor */
    int16_t dig_t3;

    /*< Calibration coefficient for the pressure sensor */
    uint16_t dig_p1;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p2;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p3;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p4;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p5;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p6;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p7;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p8;

    /*< Calibration coefficient for the pressure sensor */
    int16_t dig_p9;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h1;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h2;

    /*< Calibration coefficient for the humidity sensor */
    uint8_t dig_h3;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h4;

    /*< Calibration coefficient for the humidity sensor */
    int16_t dig_h5;

    /*< Calibration coefficient for the humidity sensor */
    int8_t dig_h6;

    /*< Variable to store the intermediate temperature coefficient */
    int32_t t_fine;
};

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
typedef struct {
    /*< un-compensated pressure */
    uint32_t pressure;

    /*< un-compensated temperature */
    uint32_t temperature;

    /*< un-compensated humidity */
    uint32_t humidity;
} bme280_uncomp_data;

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

class BMESensor_Handler {
    using read_fnc = function<u8(const u8, u8 *, u32)>;
    using write_fnc = function<u8(const u8, const u8 *, u32)>;
    using delay_ms_fnc = function<void(u32)>;

    typedef struct {
        uint8_t chipID;
        BME280_Stat status;
        bme280_calib_data calib_data;
        bme280_settings settings;
        read_fnc user_read;
        write_fnc user_write;
        delay_ms_fnc delay_ms;
    } bme280;

private:
    bme280 dev_ = { };
    bme280_data data_ = { };
public:
    BMESensor_Handler(read_fnc user_read, write_fnc user_write,
            delay_ms_fnc user_delay);
    ~BMESensor_Handler() = default;
    uint8_t getChipID();
    BME280_Stat softReset();
    BME280_Stat get_calib_data();
    BME280_Stat setSensorMode(const uint8_t sensorMode);
    BME280_Stat getSensorMode(uint8_t &sensor_mode);
    BME280_Stat set_sensor_settings(const bme280_settings &settings);
    BME280_Stat get_sensor_settings(bme280_settings &settings);
    bme280_data get_sensor_data();
    BME280_Stat get_status();
    void print_sensor_data();

private:
    BME280_Stat init_BME280();
    BME280_Stat getRegData(u8 regAddr, u8 *regData, const u32 len);
    BME280_Stat setRegData(u8 regAddr, const u8 *setData, const u32 len);
    BME280_Stat write_power_mode(const u8 mode_set, u8 &ctrl_means_reg);
    BME280_Stat put_device_to_sleep(u8 &ctrl_means_reg);
    void set_osr_humidity_settings(const u8 osr_h);
    void set_osr_temp_pres_settings(u8 &cur_ctrl_means_reg, const bme280_settings settings);
    void parse_device_settings(const u8 *reg_data, bme280_settings &settings);
    void parse_temp_calib_data(const u8 *reg_data);
    void parse_humidity_calib_data(const u8 *reg_data);
    void parse_sensor_data(u8 *reg_data, bme280_uncomp_data &uncomp_data);
    double compensate_temperature(const bme280_uncomp_data &uncomp_data);
    double compensate_pressure(const bme280_uncomp_data &uncomp_data);
    double compensate_humidity(const bme280_uncomp_data &uncomp_data);

};

#endif /* INC_BME280_DRIVER_H_ */
