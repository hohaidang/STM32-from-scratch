/*
 * 006spi_tx_testing.c
 *
 *  Created on: Feb 10, 2019
 *      Author: admin
 */

#include<string.h>
#include "../driver/inc/stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

//SPI1, AHB/APB2
//PA5 - SPI1_SCK
//PA6 - SPI1_MISO
//PA7 - SPI1_MOSI
//PA4 - slave select
// Alternate function 5

void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPOI_SPEED_HIGH;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);


//	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//	GPIO_Init(&SPIPins);


}

void SPI1_Inits(void)
{

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//generates sclk of 2MHz
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI1handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	GPIO_Handle_t SPI_NSS;
	SPI_NSS.pGPIOx = GPIOB;
	SPI_NSS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	SPI_NSS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SPI_NSS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_NSS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPI_NSS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&SPI_NSS);
	GPIO_WriteToOutputPin(SPI_NSS.pGPIOx, GPIO_PIN_NO_6, 0);
}


int main(void)
{
	uint8_t dummyByte = 0x00;
	uint8_t dummyRead = 0x00;
//	uint8_t temp_data[3];
	uint8_t chipID = 0;

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI1_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI1_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	//SPI_SSOEConfig(SPI1, ENABLE);

//	while(1)
//	{
		//wait till button is pressed
//		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );
//
//		//to avoid button de-bouncing related issues 200ms of delay
//		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);


		//to send data
		uint8_t tx_buffer[1] = {0xD0};
		uint8_t rx_buffer[1] = {0};
		SPI_SendData(SPI1, tx_buffer, 1);
//		delay();
//		SPI_ReceiveData(SPI1, &dummyRead, 1);

		SPI_SendData(SPI1, &dummyByte, 1);
		SPI_ReceiveData(SPI1, rx_buffer, 1);


		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI1,DISABLE);
//	}

	while(0);
	return 0;

}
