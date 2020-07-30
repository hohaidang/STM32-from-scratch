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
    SPI_GPIOs_Init();
    SPI_PeriClockControl();
    SPI_Init();
    SPI_SSIConfig(ENABLE);
    if(SPI_SSM_EN == SPIx_.SPIConfig.SPI_SSM) {
    	SPI_SSOEConfig(DISABLE);
    }
    else {
    	SPI_SSOEConfig(ENABLE);
    }
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
 * @fn      		  - SPI_GPIOs_Init
 *
 * @brief             - Initial Sck, MOSI, MISO, NSS GPIOs for SPI
 *
 * @param None
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_GPIOs_Init() {
    if (SPIx_.pSPIx == SPI1) {
        SPI_Sck.reset( new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_5,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        IRQ_Prio_NO_15,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        5) );

        SPI_MOSI.reset( new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_7,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        IRQ_Prio_NO_15,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        5) );

        SPI_MISO.reset( new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_6,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        IRQ_Prio_NO_15,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        5) );



        SPI_NSS.reset( new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_4,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        IRQ_Prio_NO_15,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        5) );
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

    // 1. configure the device mode
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
 * @fn                - SPI_SendData
 *
 * @brief             - Send data via SPI protocol.
 *
 * @param[in]: pTxBuffer: pointer to Tx buffer
 * @param[in]: Len: length of data transfer
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_SendData(const uint8_t *pTxBuffer, uint32_t Len) {
	if ((SPIx_.pSPIx->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
		SPI_PeripheralControl(ENABLE);
	}

    while(Len > 0) {
        // 1. wait util tx buffer empty
        while(SPI_GetFlagStatus(SPI_TXE_FLAG) == FLAG_RESET);

        // 2. check the DFF bit in CR1
        if(SPIx_.pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16 BIT DFF
            // 1. load the data into the DR
            SPIx_.pSPIx->DR = *((uint16_t *)pTxBuffer);
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
 * @param[in]         - EnOrDi: Enable or disable
 *
 * @return            - None

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

/*********************************************************************
 * @fn                - SPI_ReceiveData
 *
 * @brief             - Receive data via SPI protocol.
 *
 * @param[out]: pRxBuffer: pointer to Rx buffer
 * @param[in]: Len: length of data receive
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_ReceiveData(uint8_t *pRxBuffer, uint32_t Len)
{
	if ((SPIx_.pSPIx->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
		SPI_PeripheralControl(ENABLE);
	}

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


/*********************************************************************
 * @fn                - SPI_IRQInterruptConfig
 *
 * @brief             - Enable Interrupt for NVIC
 *
 * @param[in] IRQNumber     - IRQNumber IRQ Position number
 * @param[in] EnorDi        - Enable/Disable Flag
 *
 * @return None
 */
void SPI_Handler::SPI_IRQInterruptConfig(const uint8_t IRQNumber, const uint8_t EnorDi) {
    if(EnorDi == ENABLE) {
        if (IRQNumber <= 31) {
            //  program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            // program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // program ISER2 register
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else {
        if (IRQNumber <= 31) {
            // program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            // program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // program ICE2 register
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }

}


/*********************************************************************
 * @fn                - SPI_IRQPriorityConfig
 *
 * @brief - Configure priority for GPIO, 0->15, lower number higher priority
 *
 * @param[in] IRQNumber: IRQ position for EXIT
 * @param[in] IRPriority: 0->15
 * @return None
 */
void SPI_Handler::SPI_IRQPriorityConfig(const uint8_t IRQNumber, const uint8_t IRQPriority) {
    // 1. first lets find out the ipr register
    uint8_t iprx = IRQNumber >> 2;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


uint8_t SPI_Handler::SPI_SendDataIT(const uint8_t *pTxBuffer, uint32_t Len) {
    if(SPIx_.TxState != SPI_BUSY_IN_TX) {
        // 1. Save the Tx buffer address and Len information in some global variables
        SPIx_.pTxBuffer = const_cast<uint8_t *>(pTxBuffer);
        SPIx_.TxLen = Len;

        // 2. Mark the SPI state as busy in transmission so that no other code
        // can take over same SPI peripheral until transmission is over
        SPIx_.TxState = SPI_BUSY_IN_TX;

        // 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
        SPIx_.pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }

    return SPIx_.TxState;
}

uint8_t SPI_Handler::SPI_ReceiveDataIT(const uint8_t *pRxBuffer, uint32_t Len) {
    if(SPIx_.RxState != SPI_BUSY_IN_RX) {
        // 1. Save the Tx buffer address and Len information in some global variables
        SPIx_.pRxBuffer = const_cast<uint8_t *>(pRxBuffer);
        SPIx_.RxLen = Len;

        // 2. Mark the SPI state as busy in transmission so that no other code
        // can take over same SPI peripheral until transmission is over
        SPIx_.RxState = SPI_BUSY_IN_RX;

        // 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
        SPIx_.pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return SPIx_.TxState;
}

void SPI_Handler::SPI_IRQHandling() {
    uint8_t temp1 = 0, temp2 = 0;
    // First lets check for TXE
    temp1 = SPIx_.pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = SPIx_.pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
    if(temp1 && temp2) {
        // Handle TXE
        spi_txe_interrupt_handle();
    }

    // check for RXNE
    temp1 = SPIx_.pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = SPIx_.pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
    if(temp1 && temp2) {
        spi_rxne_interrupt_handle();
    }

    // check for ovr flag
    temp1 = SPIx_.pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = SPIx_.pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
    if(temp1 && temp2) {
        spi_ovr_err_interrupt_handle();
    }
}

void SPI_Handler::spi_txe_interrupt_handle() {
	if ((SPIx_.pSPIx->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }

    // 2. check the DFF bit in CR1
    if (SPIx_.pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16 BIT DFF
        // 1. load the data into the DR
        SPIx_.pSPIx->DR = *((uint16_t*) SPIx_.pTxBuffer);
        SPIx_.TxLen -= 2;
        (uint16_t*) SPIx_.pTxBuffer++;
    } else {
        SPIx_.pSPIx->DR = *SPIx_.pTxBuffer;
        SPIx_.TxLen -= 1;
        SPIx_.pTxBuffer++;
    }

    if(!SPIx_.TxLen) {
        // TxLen is zero, close the spi transmission and inform the application
        // Tx is over
        SPI_CloseTransmission();
        SPI_ApplicationEventCallback(&SPIx_, SPI_EVENT_TX_CMPLT);
    }
}

void SPI_Handler::spi_rxne_interrupt_handle() {
	if ((SPIx_.pSPIx->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }


    //2. check the DFF bit in CR1
    if ((SPIx_.pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
        //16 bit DFF
        //1. load the data from DR to Rxbuffer address
        *((uint16_t*) SPIx_.pRxBuffer) = SPIx_.pSPIx->DR;
        SPIx_.RxLen -= 2;
        (uint16_t*) SPIx_.pRxBuffer++;
    } else {
        //8 bit DFF
        *(SPIx_.pRxBuffer) = SPIx_.pSPIx->DR;
        SPIx_.RxLen--;
        SPIx_.pRxBuffer++;
    }

    if(!SPIx_.RxLen) {
        // TxLen is zero, close the spi transmission and inform the application
        // Tx is over
        SPI_CloseReception();
        SPI_ApplicationEventCallback(&SPIx_, SPI_EVENT_RX_CMPLT);
    }
}

void SPI_Handler::spi_ovr_err_interrupt_handle() {
    uint8_t temp = 0;
    // 1. Clear the ovr flag
    if(SPIx_.TxState != SPI_BUSY_IN_TX) {
        temp = SPIx_.pSPIx->DR;
        temp = SPIx_.pSPIx->SR;
    }
    // 2. Inform the application
    SPI_ApplicationEventCallback(&SPIx_, SPI_EVENT_OVR_ERR);
    static_cast<void>(temp);
}


void SPI_Handler::SPI_ClearOVRFlag() {
    uint8_t temp = 0;
    temp = SPIx_.pSPIx->DR;
    temp = SPIx_.pSPIx->SR;
    static_cast<void>(temp);
}

void SPI_Handler::SPI_CloseTransmission() {
    SPIx_.pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    SPIx_.pTxBuffer = nullptr;
    SPIx_.TxLen = 0;
    SPIx_.TxState = SPI_READY;
}

void SPI_Handler::SPI_CloseReception() {
    SPIx_.pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    SPIx_.pRxBuffer = nullptr;
    SPIx_.RxLen = 0;
    SPIx_.RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {
    //This is a weak implementation . the user application may override this function.
}










