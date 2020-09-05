/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
//#include "driver/inc/stm32f4xx.h"
#include "../driver/stm32f4xx.h"
#include "../driver/bme280_driver.h"
#include "spi_readSensordata.h"
#include <array>

using namespace std;


void InitilizePeripheral(void);
void user_delay_us(u32);
u8 user_spi_read (const u8, u8 *, u32);
u8 user_spi_write(const u8, const u8 *, u32);



unique_ptr<SysTick> SystemTick;
unique_ptr<SPI_Handler> spi1;
unique_ptr<BMESensor_Handler> bme280;


int main(void)
{
    bme280_settings temp = { 0 };
    InitilizePeripheral();

    if (bme280->get_status() == SENSOR_OK) {
        bme280->setSensorMode(BME280_NORMAL_MODE);

        bme280_settings settings;
        settings.osr_h = 0x07; // x16
        settings.osr_t = 0x07; // x16
        settings.osr_p = 0x07; // x16
        bme280->set_sensor_settings(settings);
        bme280->get_sensor_settings(temp);
        while(1) {
            // get sensor data
            bme280->get_sensor_data();

#ifdef DEBUG_EN
			bme280->print_sensor_data();
#endif
			SystemTick->delay_ms(1000);
       }
    }
    else {
#ifdef DEBUG_EN
    	cout << "Sensor Init failed. Please check again hardware connection pins" << endl;
#endif
    }


   return 0;
}

void InitilizePeripheral(void) {
    // HSI Clock 16 MHz
    SystemTick.reset( new SysTick() );

    spi1.reset( new SPI_Handler(SPI1,
                                        SPI_DEVICE_MODE_MASTER,
                                        SPI_BUS_CONFIG_FD,
                                        SPI_SCLK_SPEED_DIV2,
                                        SPI_DFF_8BITS,
                                        SPI_CPOL_LOW,
                                        SPI_CPHA_LOW,
                                        SPI_SSM_EN) );

    spi1->spi_ir_config<SPI1_IRQn, ENABLE>();
    spi1->spi_ir_prio_config<SPI1_IRQn, IRQ_Prio_NO_15>();
    spi1->spi_init_nss_sw(GPIOB, GPIO_PIN_NO_6);
    spi1->spi_nss_->gpio_write_to_output_pin(SET);

    bme280.reset( new BMESensor_Handler(user_spi_read,
                                        user_spi_write,
                                        user_delay_us) );
}

void user_delay_us(u32 period)
{
    SystemTick->delay_ms(period);
}

u8 user_spi_read (const u8 reg_addr, u8 *reg_data, u32 len) {
    array<u8, BME280_MAX_SIZE_WR> txBuffer{};
    array<u8, BME280_MAX_SIZE_WR> rxBuffer{};

    txBuffer[0] = reg_addr;

    spi1->spi_nss_->gpio_write_to_output_pin(RESET);

    spi1->spi_transmit_receive_data_it(txBuffer.data(), rxBuffer.data(), len + 1);

    spi1->spi_nss_->gpio_write_to_output_pin(SET);
    // copy to reg_data
    for(u32 i = 0; i < len; ++i) {
        reg_data[i] = rxBuffer[i + 1];
    }
    return 0;
}

u8 user_spi_write(const u8 reg_addr, const u8 *reg_data, u32 len) {
    array<u8, BME280_MAX_SIZE_WR> txBuffer{};
    txBuffer[0] = reg_addr;
    for(u32 i = 0; i < len; ++i) {
        txBuffer[i + 1] = reg_data[i];
    }

    spi1->spi_nss_->gpio_write_to_output_pin(RESET);

    spi1->spi_transmit_data_it(txBuffer.data(), len + 1);

    spi1->spi_nss_->gpio_write_to_output_pin(SET);

    return 0;
}


//u8 user_spi_read (const u8 reg_addr, u8 *reg_data, u32 len) {
//    array<u8, BME280_MAX_SIZE_WR> txBuffer = {};
//    array<u8, BME280_MAX_SIZE_WR> rxBuffer = {};
//
//    txBuffer[0] = reg_addr;
//
//    spi1->spi_nss_->gpio_write_to_output_pin(RESET);
//
//    spi1->spi_transmit_receive_data(txBuffer.data(), rxBuffer.data(), len + 1);
//
//    spi1->spi_nss_->gpio_write_to_output_pin(SET);
//
//    // copy to reg_data
//    for(u32 i = 0; i < len; ++i) {
//        reg_data[i] = rxBuffer[i + 1];
//    }
//    return 0;
//}
//
//u8 user_spi_write(const u8 reg_addr, const u8 *reg_data, u32 len) {
//    array<u8, BME280_MAX_SIZE_WR> txBuffer = {};
//
//    txBuffer[0] = reg_addr;
//    for(u32 i = 0; i < len; ++i) {
//        txBuffer[i + 1] = reg_data[i];
//    }
//
//    spi1->spi_nss_->gpio_write_to_output_pin(RESET);
//    spi1->spi_transmit_data(txBuffer.data(), len + 1);
//    spi1->spi_nss_->gpio_write_to_output_pin(SET);
//
//    return 0;
//}

extern "C" {
    void SPI1_IRQHandler(void) {
        // handle the interrupt
        spi1->spi_irq_handling();
    }
}