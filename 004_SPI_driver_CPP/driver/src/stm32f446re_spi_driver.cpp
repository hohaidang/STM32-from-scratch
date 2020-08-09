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
 * @brief             - Initialize SPI, GPIOs, Interrupt, etc.
 **********************************************************************/
SPI_Handler::SPI_Handler(SPI_RegDef_t *SPIx_ADDR,
                         u8 DeviceMode,
                         u8 BusConfig,
                         u8 SclkSpeed,
                         u8 DFF,
                         u8 CPOL,
                         u8 CPHA,
                         u8 SSM) {
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
 * @brief             - Disable all SPIs feature
 *
 * @param None
 *
 * @return None
 *
 * @Node:
 **********************************************************************/
SPI_Handler::~SPI_Handler(){
	SPI_DeInit();
}

/*********************************************************************
 * @fn                - SPI_Handler peripheral clock setup
 *
 * @brief             - peripheral clock setup for SPI
 *
 * @param None
 *
 * @return None
 *
 * @Node:
 **********************************************************************/
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


        if(SPIx_.SPIConfig.SPI_SSM == SPI_SSM_DI) {
            // Hardware NSS enable, configure for PA4
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
}

/*********************************************************************
 * @fn      		  - Initialize SPI
 *
 * @brief             -
 *
 * @param None
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_Init() {
    // Lets configure the SPI_CR1 register
    u32 tempReg = 0;

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
 * @fn                - Enable or Disable SPI
 *
 * @brief             -
 *
 * @param None
 *
 * @return None
 **********************************************************************/
void SPI_Handler::SPI_PeripheralControl(u8 EnOrDi) {
    if(ENABLE == EnOrDi) {
        SPIx_.pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else {
        SPIx_.pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/*********************************************************************
 * @fn                - Get Flag register status
 *
 * @brief             -
 *
 * @param[in]         - FlagName: check @FLAG_NAME_STATUS
 *
 * @return None
 **********************************************************************/
inline u8 SPI_Handler::SPI_GetFlagStatus(const u8 FlagName) {
    return (SPIx_.pSPIx->SR & FlagName) ? FLAG_SET : FLAG_RESET;
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
void SPI_Handler::SPI_SendData(const u8 *pTxBuffer, u32 Len) {
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
    while(SPI_GetFlagStatus(SPI_BSY_FLAG)); // w8 until SPI done
    SPI_ClearOVRFlag(); // because in 2 lines mode, write data does not need to read
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
void SPI_Handler::SPI_ReceiveData(u8 *pRxBuffer, u32 Len)
{
    if ((SPIx_.pSPIx->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }

    while (Len > 0) {
        //1. wait until RXNE is set
        while (SPI_GetFlagStatus(SPI_RXNE_FLAG) == (u8) FLAG_RESET);

        //2. check the DFF bit in CR1
        if ((SPIx_.pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((uint16_t*) pRxBuffer) = SPIx_.pSPIx->DR;
            Len--;
            Len--;
            (uint16_t*) pRxBuffer++;
        }
        else {
            //8 bit DFF
            *(pRxBuffer) = SPIx_.pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
    while(SPI_GetFlagStatus(SPI_BSY_FLAG)); // w8 until SPI done
}

/*!
 * @brief This is API is used for send and receive SPI data concurrently
 *
 * @param[in] pTxBuffer : Transmit buffer data
 * @param[in] pRxBuffer : Receive buffer data
 * @param[in] len: len of transmit and receive
 *
 * @return None
 *
 */
void SPI_Handler::SPI_SendAndReceiveData(const u8 *pTxBuffer, u8 *pRxBuffer, u32 len) {
    if ((SPIx_.pSPIx->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }
    while(len > 0) {
        // 1. wait util tx buffer empty
        while(SPI_GetFlagStatus(SPI_TXE_FLAG) == FLAG_RESET);

        // 2. check the DFF bit in CR1
        if(SPIx_.pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16 BIT DFF
            // 1. load the data into the DR
            SPIx_.pSPIx->DR = *((uint16_t *)pTxBuffer);
            len -= 2;
            (uint16_t *)pTxBuffer++;
        }
        else {
            SPIx_.pSPIx->DR = *pTxBuffer;
            len -= 1;
            pTxBuffer++;
        }

        //1. wait until RXNE is set
        while (SPI_GetFlagStatus(SPI_RXNE_FLAG) == (u8) FLAG_RESET);

        //2. check the DFF bit in CR1
        if ((SPIx_.pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((uint16_t*) pRxBuffer) = SPIx_.pSPIx->DR;
            (uint16_t*) pRxBuffer++;
        }
        else {
            //8 bit DFF
            *(pRxBuffer) = SPIx_.pSPIx->DR;
            pRxBuffer++;
        }
    }
    while(SPI_GetFlagStatus(SPI_BSY_FLAG)); // w8 until SPI done
}


void SPI_Handler::SPI_SSOEConfig(u8 EnOrDi) {
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
void SPI_Handler::SPI_SSIConfig(u8 EnOrDi)
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
 * @fn                - SPI_IRQInterruptConfig
 *
 * @brief             - Enable Interrupt for NVIC
 *
 * @param[in] IRQNumber     - IRQNumber IRQ Position number
 * @param[in] EnorDi        - Enable/Disable Flag
 *
 * @return None
 */
void SPI_Handler::SPI_IRQInterruptConfig(const u8 IRQNumber, const u8 EnorDi) {
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
 * @brief - Configure priority for GPIO, 0->15, lower SPI_CR2_ERRIE higher priority
 *
 * @param[in] IRQNumber: IRQ position for EXIT
 * @param[in] IRPriority: 0->15
 * @return None
 */
void SPI_Handler::SPI_IRQPriorityConfig(const u8 IRQNumber, const u8 IRQPriority) {
    // 1. first lets find out the ipr register
    u8 iprx = IRQNumber >> 2;
    u8 iprx_section = IRQNumber % 4;
    u8 shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


u8 SPI_Handler::SPI_SendDataIT(const u8 *pTxBuffer, u32 Len) {
	if ((SPIx_.pSPIx->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
		SPI_PeripheralControl(ENABLE);
	}

	while(SPIx_.TxState != SPI_READY || SPIx_.RxState != SPI_READY);
	// 1. Save the Tx buffer address and Len information in some global variables
	SPIx_.pTxBuffer = const_cast<u8*>(pTxBuffer);
	SPIx_.TxLen = Len;

	// 2. Mark the SPI state as busy in transmission so that no other code
	// can take over same SPI peripheral until transmission is over
	SPIx_.TxState = SPI_BUSY_IN_TX;

	SPIx_.pSPIx->CR2 |= (1 << SPI_CR2_TXEIE | 1 << SPI_CR2_ERRIE);

    return SPIx_.TxState;
}

u8 SPI_Handler::SPI_ReceiveDataIT(u8 *pRxBuffer, u32 Len) {
	while (SPIx_.TxState != SPI_READY || SPIx_.RxState != SPI_READY);
	SPI_ClearOVRFlag();
	// 1. Save the Tx buffer address and Len information in some global variables
	SPIx_.pTxBuffer = pRxBuffer; // transmit dummy
	SPIx_.TxLen = Len;

	SPIx_.pRxBuffer = pRxBuffer;
	SPIx_.RxLen = Len;

	// 2. Mark the SPI state as busy in transmission so that no other code
	// can take over same SPI peripheral until transmission is over
	SPIx_.RxState = SPI_BUSY_IN_RX;
	SPIx_.TxState = SPI_BUSY_IN_TX;

	// 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
	SPIx_.pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE | 1 << SPI_CR2_ERRIE | 1 << SPI_CR2_TXEIE);

    return SPIx_.TxState;
}

void SPI_Handler::SPI_IRQHandling() {
    volatile u8 temp1 = 0, temp2 = 0;
	// check for RXNE
	temp1 = SPIx_.pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = SPIx_.pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2) {
		spi_rxne_interrupt_handle();
		return;
	}

    // First lets check for TXE
    temp1 = SPIx_.pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = SPIx_.pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
    if(temp1 && temp2) {
        // Handle TXE
        spi_txe_interrupt_handle();
        return;
    }

	// check for ovr flag
	temp1 = SPIx_.pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = SPIx_.pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) {
		spi_ovr_err_interrupt_handle();
		return;
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
    // 1. Clear the ovr flag
    SPI_ClearOVRFlag();
    // 2. Inform the application
    SPI_ApplicationEventCallback(&SPIx_, SPI_EVENT_OVR_ERR);
}


void SPI_Handler::SPI_ClearOVRFlag() {
    volatile u8 temp = 0;
    temp = SPIx_.pSPIx->DR;
    temp = SPIx_.pSPIx->SR;
    static_cast<void>(temp);
}

void SPI_Handler::SPI_CloseTransmission() {
	while((SPIx_.pSPIx->SR & (1 << SPI_SR_TXE)) == RESET);
    SPIx_.pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE | 1 << SPI_CR2_ERRIE);
    SPIx_.pTxBuffer = nullptr;
    SPIx_.TxLen = 0;
    SPIx_.TxState = SPI_READY;
    SPI_ClearOVRFlag(); // TODO: loi o day vi clear sau khi goi dummy
}

void SPI_Handler::SPI_CloseReception() {
    SPIx_.pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE | 1 << SPI_CR2_ERRIE);
    SPIx_.pRxBuffer = nullptr;
    SPIx_.RxLen = 0;
    SPIx_.RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, u8 AppEv) {
    //This is a weak implementation . the user application may override this function.
}









