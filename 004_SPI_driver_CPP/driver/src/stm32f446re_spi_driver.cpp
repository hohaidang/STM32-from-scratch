/*
 * stm32f446re_spi_driver.cpp
 *
 *  Created on: Jul 16, 2020
 *      Author: prnsoft
 */

#include "../inc/stm32f446re_spi_driver.h"


/*********************************************************************
 * @class      		  - GPIO_Handler Constructor
 *
 * @brief             -
 **********************************************************************/
SPI_Handler::SPI_Handler(){

    SPI_PeriClockControl();
    SPI_Init();
}

/*********************************************************************
 * @fn      		  - SPI_Handler De-constructor
 *
 * @brief             -
 *
 * @param None
 *
 * @return None
 *
 * @Node: Be careful when using many Peripheral device in the same PORT
 **********************************************************************/
SPI_Handler::~SPI_Handler(){
	SPI_DeInit();
}

// peripheral clock setup
void SPI_Handler::SPI_PeriClockControl() {
	if (SPI1 == SPIx_.pSPIx) {
		SPI1_PCLK_EN();
	}
	else if (SPI2 == SPIx_.pSPIx) {
		SPI2_PCLK_EN();
	}
	else if (SPI3 == SPIx_.pSPIx) {
		SPI3_PCLK_EN();
	}
	else if (SPI4 == SPIx_.pSPIx) {
		SPI4_PCLK_EN();
	}
}


/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param None
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_Init() {
    // Lets configure the SPI_CR1 register
    uint32_t tempReg = 0;

    // 1. configure the deivce mode
    tempReg |= SPIx_.SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
    // 2. configure the bus config
    if(SPI_BUS_CONFIG_FD == SPIx_.SPIConfig.SPI_BusConfig) {
        // bidi mode should be cleared
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if(SPI_BUS_CONFIG_HD == SPIx_.SPIConfig.SPI_BusConfig) {
        // bidi mode should be set
        tempReg |= (1 << SPI_CR1_BIDIMODE);
        // bidioe mode should be set
        tempReg |= (1 << SPI_CR1_BIDIOE);
    }
    else if(SPI_BUS_CONFIG_SIMPLEX_RXONLY == SPIx_.SPIConfig.SPI_BusConfig) {
        // bidi mode should be cleared, for active SCLK, because SLCK is only activate when MOSI is activate
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
        // RXONLY should be set
        tempReg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. configure for clock speed
    tempReg |= (SPIx_.SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    // 4. configure for data format
    tempReg |= (SPIx_.SPIConfig.SPI_DFF << SPI_CR1_DFF);

    // 5. configure CPOL
    tempReg |= (SPIx_.SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // 6. configure CPOA
    tempReg |= (SPIx_.SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    SPIx_.pSPIx->CR1 = tempReg;
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param None
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_DeInit() {
	if (SPI1 == SPIx_.pSPIx) {
		SPI1_REG_RESET();
	}
	else if (SPI2 == SPIx_.pSPIx) {
		SPI2_REG_RESET();
	}
	else if (SPI3 == SPIx_.pSPIx) {
		SPI3_REG_RESET();
	}
	else if (SPI4 == SPIx_.pSPIx) {
		SPI4_REG_RESET();
	}
}

/*********************************************************************
 * @fn                -
 *
 * @brief             -
 *
 * @param None
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_SendData() {

}
