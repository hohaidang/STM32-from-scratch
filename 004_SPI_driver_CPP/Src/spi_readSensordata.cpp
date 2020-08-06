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

#include "../driver/inc/stm32f4xx.h"
#include <memory>
//#include <stdio.h>
#include <iostream>
#include "temp.h"
using namespace std;

void InitilizePeripheral(void);

//SPI1, AHB/APB2
//PA5 - SPI1_SCK
//PA6 - SPI1_MISO
//PA7 - SPI1_MOSI
//PA4 - slave select
// Alternate function 5
SPI_Handler *SPI1_Handler;
GPIO_Handler *PB6;
struct bme280_dev dev;

void user_delay_us(uint32_t period, void *intf_ptr)
{
    // TODO: Design system tick in here
	for(int i = 0; i < 25000; ++i) {

	}
}

int8_t user_spi_read (uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t txBuffer[28] = {};
    uint8_t rxBuffer[28] = {};
    txBuffer[0] = reg_addr;

    PB6->GPIO_WriteToOutputPin(RESET);

    SPI1_Handler->SPI_SendAndReceiveData(&txBuffer[0], &rxBuffer[0], len + 1);

    PB6->GPIO_WriteToOutputPin(SET);

    // copy to reg_data
    for(uint32_t i = 0; i < len; ++i) {
        reg_data[i] = rxBuffer[i + 1];
    }
    return 0;
}

int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t txBuffer[28] = {};
    txBuffer[0] = reg_addr;
    for(uint32_t i = 0; i < len; ++i) {
        txBuffer[i + 1] = reg_data[i];
    }

    PB6->GPIO_WriteToOutputPin(RESET);
    SPI1_Handler->SPI_SendData(&txBuffer[0], len + 1);
    PB6->GPIO_WriteToOutputPin(SET);

    return 0;
}


int main(void)
{
    InitilizePeripheral();

    int8_t rslt = BME280_OK;

    /* Sensor_0 interface over SPI with native chip select line */
    uint8_t dev_addr = 0;

    dev.intf_ptr = &dev_addr;
    dev.intf = BME280_SPI_INTF;
    dev.read = &user_spi_read;
    dev.write = &user_spi_write;
    dev.delay_us = &user_delay_us;

    rslt = bme280_init(&dev);

    // configure
    uint8_t ctrl_hum = 0x03;
    user_spi_write(0x72, &ctrl_hum, 1, dev.intf_ptr);
    uint8_t ctrl_means = 0xFF;
    user_spi_write(0x74, &ctrl_means, 1, dev.intf_ptr);

    // read configure
    user_spi_read(0xF2, &ctrl_hum, 1, dev.intf_ptr);
    user_spi_read(0xF4, &ctrl_means, 1, dev.intf_ptr);

    user_delay_us(5, dev.intf_ptr);
    // get sensor data
    struct bme280_data comp_data;
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    SPI1_Handler->SPI_PeripheralControl(DISABLE);
    while(1);
    return 0;
}

void InitilizePeripheral(void) {
    // HSI Clock 16 Mhz
    SPI1_Handler = new SPI_Handler(SPI1,
                                        SPI_DEVICE_MODE_MASTER,
                                        SPI_BUS_CONFIG_FD,
                                        SPI_SCLK_SPEED_DIV32,
                                        SPI_DFF_8BITS,
                                        SPI_CPOL_LOW,
                                        SPI_CPHA_LOW,
                                        SPI_SSM_EN);

    // TODO: design interrupt for SPI connect with sensor
//  SPI1_Handler->SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);
//  SPI1_Handler->SPI_IRQPriorityConfig(IRQ_NO_SPI1, IRQ_Prio_NO_15);

    PB6 = new GPIO_Handler(GPIOB,
                           GPIO_PIN_NO_6,
                           GPIO_MODE_OUT,
                           GPIO_SPEED_FAST,
                           GPIO_OP_TYPE_PP,
                           GPIO_NO_PUPD);
    PB6->GPIO_WriteToOutputPin(SET);
}

extern "C" {
    void SPI1_IRQHandler(void) {
        // handle the interrupt
        SPI1_Handler->SPI_IRQHandling();
    }
}
