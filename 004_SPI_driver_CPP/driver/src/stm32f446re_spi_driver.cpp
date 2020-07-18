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
SPI_Handler::SPI_Handler(SPI_RegDef_t *SPIx_ADDR,
                         uint8_t DeviceMode,
                         uint8_t BusConfig,
                         uint8_t SclkSpeed,
                         uint8_t DFF,
                         uint8_t CPOL,
                         uint8_t CPHA,
                         uint8_t SSM) {
    SPIx_.pSPIx = SPIx_ADDR;
    SPIx_.SPIConfig.SPI_DeviceMode = DeviceMode;
    SPIx_.SPIConfig.SPI_BusConfig = BusConfig;
    SPIx_.SPIConfig.SPI_SclkSpeed = SclkSpeed;
    SPIx_.SPIConfig.SPI_DFF = DFF;
    SPIx_.SPIConfig.SPI_CPOL = CPOL;
    SPIx_.SPIConfig.SPI_CPHA = CPHA;
    SPIx_.SPIConfig.SPI_SSM = SSM;
    SPI_PeriClockControl();
    SPI_Init();
//    SPI_SSIConfig(ENABLE);
    SPI_SSOEConfig(ENABLE);
    // Enable SPI peripheral
    //SPI_PeripheralControl(ENABLE);
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

    tempReg |= (SPIx_.SPIConfig.SPI_SSM << SPI_CR1_SSM);

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
void SPI_Handler::SPI_PeripheralControl(uint8_t EnOrDi) {
    if(ENABLE == EnOrDi) {
        SPIx_.pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else {
        SPIx_.pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
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
uint8_t SPI_Handler::SPI_GetFlagStatus(uint8_t FlagName) {
    if(SPIx_.pSPIx->SR & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*********************************************************************
 * @fn                -
 *
 * @brief             -
 *
 * @param None
 * @Node              - This is blocking call
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_SendData(uint8_t *pTxBuffer, uint32_t Len) {
    while(Len > 0) {
        // 1. wait util tx buffer empty
        while(SPI_GetFlagStatus(SPI_TXE_FLAG) == FLAG_RESET);

        // 2. check the DFF bit in CR1
        if(SPIx_.pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16 BIT DFF
            // 1. load the data into the DR
            SPIx_.pSPIx->DR = *((uint16_t *) pTxBuffer);
            Len -= 2;
            (uint16_t *)pTxBuffer++;
        }
        else {
            SPIx_.pSPIx->DR = *pTxBuffer;
            Len -= 1;
            pTxBuffer++;
        }
    }
}

//uint16_t SPI_Handler::SPI_ReceiveData() {
//	uint16_t answer = 0;
//	while(SPI_GetFlagStatus(SPI_RXNE_FLAG) == FLAG_RESET) {}
//	answer = SPIx_.pSPIx->DR;
//	return answer;
//}


void SPI_Handler::SPI_SSOEConfig(uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		SPIx_.pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else {
		SPIx_.pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*********************************************************************
 * @fn                - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_Handler::SPI_SSIConfig(uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        SPIx_.pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
    }else
    {
        SPIx_.pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
    }
}


void SPI_Handler::SPI_ReceiveData(uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXNE is set
			while(SPI_GetFlagStatus(SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

			//2. check the DFF bit in CR1
			if( (SPIx_.pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data from DR to Rxbuffer address
				 *((uint16_t*)pRxBuffer) = SPIx_.pSPIx->DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = SPIx_.pSPIx->DR ;
				Len--;
				pRxBuffer++;
			}
		}

}
