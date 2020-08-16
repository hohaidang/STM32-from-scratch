/*
 * stm32f446re_spi_driver.cpp
 *
 *  Created on: Jul 16, 2020
 *      Author: hohaidang
 */

#include "../inc/stm32f446re_spi_driver.h"


/*!
 * @brief Initialize SPI, GPIOs, Interrupt, etc.
 *
 * @param: None
 *
 * @return None
 *
 */
SPI_Handler::SPI_Handler(SPI_RegDef_t *SPIx_ADDR,
                        u8 device_mode,
                        u8 bus_config,
                        u8 sclk_speed,
                        u8 DFF,
                        u8 CPOL,
                        u8 CPHA,
                        u8 SSM) : SPIx_(SPIx_ADDR),
                                  config_({device_mode, bus_config, sclk_speed, DFF, CPOL, CPOL, SSM}){
    SPI_GPIOs_Init();
    SPI_PeriClockControl();
    SPI_Init();
    SPI_SSIConfig(ENABLE);
    if (SPI_SSM_EN == config_.SPI_SSM) {
        SPI_SSOEConfig(DISABLE);
    } else {
        SPI_SSOEConfig(ENABLE);
    }
}


/*!
 * @brief Disable all SPIs feature
 *
 * @param: None
 *
 * @return None
 *
 */
SPI_Handler::~SPI_Handler() {
    SPI_DeInit();
}


/*!
 * @brief peripheral clock setup for SPI
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_PeriClockControl() {
    if (SPI1 == SPIx_) {
        SPI1_PCLK_EN();
    } else if (SPI2 == SPIx_) {
        SPI2_PCLK_EN();
    } else if (SPI3 == SPIx_) {
        SPI3_PCLK_EN();
    } else if (SPI4 == SPIx_) {
        SPI4_PCLK_EN();
    }
}


/*!
 * @brief Initial Sck, MOSI, MISO, NSS GPIOs for SPI
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_GPIOs_Init() {
    if (SPIx_ == SPI1) {
        SPI_Sck.reset(new GPIO_Handler( GPIOA,
                                        GPIO_PIN_NO_5,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        IRQ_Prio_NO_15,
                                        GPIO_ALT_5));

        SPI_MOSI.reset(new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_7,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        IRQ_Prio_NO_15,
                                        GPIO_ALT_5));

        SPI_MISO.reset(new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_6,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        IRQ_Prio_NO_15,
                                        GPIO_ALT_5));

        if (config_.SPI_SSM == SPI_SSM_DI) {
            // Hardware NSS enable, configure for PA4
            SPI_NSS.reset(new GPIO_Handler( GPIOA,
                                            GPIO_PIN_NO_4,
                                            GPIO_MODE_ALTFN,
                                            GPIO_SPEED_HIGH,
                                            GPIO_OP_TYPE_PP,
                                            GPIO_NO_PUPD,
                                            IRQ_Prio_NO_15, 5));
        }
    }
}


/*!
 * @brief Initialize SPI
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_Init() {
    // Lets configure the SPI_CR1 register
    u32 tempReg = 0;

    // 1. configure the device mode
    tempReg |= config_.SPI_DeviceMode << SPI_CR1_MSTR;
    // 2. configure the bus config
    if (SPI_BUS_CONFIG_FD == config_.SPI_BusConfig) {
        // bidi mode should be cleared
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
    } else if (SPI_BUS_CONFIG_HD == config_.SPI_BusConfig) {
        // bidi mode should be set
        tempReg |= (1 << SPI_CR1_BIDIMODE);
        // bidioe mode should be set
        tempReg |= (1 << SPI_CR1_BIDIOE);
    } else if (SPI_BUS_CONFIG_SIMPLEX_RXONLY == config_.SPI_BusConfig) {
        // bidi mode should be cleared, for active SCLK, because SLCK is only activate when MOSI is activate
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
        // RXONLY should be set
        tempReg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. configure for clock speed
    tempReg |= (config_.SPI_SclkSpeed << SPI_CR1_BR);

    // 4. configure for data format
    tempReg |= (config_.SPI_DFF << SPI_CR1_DFF);

    // 5. configure CPOL
    tempReg |= (config_.SPI_CPOL << SPI_CR1_CPOL);

    // 6. configure CPOA
    tempReg |= (config_.SPI_CPHA << SPI_CR1_CPHA);

    tempReg |= (config_.SPI_SSM << SPI_CR1_SSM);

    SPIx_->CR1 = tempReg;
}


/*!
 * @brief Reset register of SPI
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_DeInit() {
    if (SPI1 == SPIx_) {
        SPI1_REG_RESET();
    } else if (SPI2 == SPIx_) {
        SPI2_REG_RESET();
    } else if (SPI3 == SPIx_) {
        SPI3_REG_RESET();
    } else if (SPI4 == SPIx_) {
        SPI4_REG_RESET();
    }
}


/*!
 * @brief peripheral clock control
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_PeripheralControl(u8 EnOrDi) {
    if (ENABLE == EnOrDi) {
        SPIx_->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        SPIx_->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}


/*!
 * @brief Get Flag register status
 *
 * @param[in]         - FlagName: check @FLAG_NAME_STATUS
 *
 * @return None
 *
 */
inline u8 SPI_Handler::SPI_GetFlagStatus(const u8 FlagName) {
    return (SPIx_->SR & FlagName) ? FLAG_SET : FLAG_RESET;
}


/*!
 * @brief Send data via SPI protocol.
 *
 * @param[in]: pTxBuffer: pointer to Tx buffer
 * @param[in]: Len: length of data transfer
 *
 * @return None
 *
 */
void SPI_Handler::SPI_SendData(const u8 *pTxBuffer, u32 Len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }

    while (Len > 0) {
        // 1. wait util tx buffer empty
        while (SPI_GetFlagStatus(SPI_TXE_FLAG) == FLAG_RESET)
            ;

        // 2. check the DFF bit in CR1
        if (SPIx_->CR1 & (1 << SPI_CR1_DFF)) {
            // 16 BIT DFF
            // 1. load the data into the DR
            SPIx_->DR = *((uint16_t*) pTxBuffer);
            Len -= 2;
            (uint16_t*) pTxBuffer++;
        } else {
            SPIx_->DR = *pTxBuffer;
            Len -= 1;
            pTxBuffer++;
        }
    }
    while (SPI_GetFlagStatus(SPI_BSY_FLAG))
        ; // w8 until SPI done
    SPI_ClearOVRFlag(); // because in 2 lines mode, write data does not need to read
}


/*!
 * @brief Receive data via SPI protocol.
 *
 * @param[out]: pRxBuffer: pointer to Rx buffer
 * @param[in]: Len: length of data receive
 *
 * @return None
 *
 */
void SPI_Handler::SPI_ReceiveData(u8 *pRxBuffer, u32 Len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }

    while (Len > 0) {
        //1. wait until RXNE is set
        while (SPI_GetFlagStatus(SPI_RXNE_FLAG) == (u8) FLAG_RESET)
            ;

        //2. check the DFF bit in CR1
        if ((SPIx_->CR1 & (1 << SPI_CR1_DFF))) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((uint16_t*) pRxBuffer) = SPIx_->DR;
            Len--;
            Len--;
            (uint16_t*) pRxBuffer++;
        } else {
            //8 bit DFF
            *(pRxBuffer) = SPIx_->DR;
            Len--;
            pRxBuffer++;
        }
    }
    while (SPI_GetFlagStatus(SPI_BSY_FLAG))
        ; // w8 until SPI done
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
void SPI_Handler::SPI_SendAndReceiveData(const u8 *pTxBuffer, u8 *pRxBuffer,
        u32 len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }
    while (len > 0) {
        // 1. wait util tx buffer empty
        while (SPI_GetFlagStatus(SPI_TXE_FLAG) == FLAG_RESET)
            ;

        // 2. check the DFF bit in CR1
        if (SPIx_->CR1 & (1 << SPI_CR1_DFF)) {
            // 16 BIT DFF
            // 1. load the data into the DR
            SPIx_->DR = *((uint16_t*) pTxBuffer);
            len -= 2;
            (uint16_t*) pTxBuffer++;
        } else {
            SPIx_->DR = *pTxBuffer;
            len -= 1;
            pTxBuffer++;
        }

        //1. wait until RXNE is set
        while (SPI_GetFlagStatus(SPI_RXNE_FLAG) == (u8) FLAG_RESET)
            ;

        //2. check the DFF bit in CR1
        if ((SPIx_->CR1 & (1 << SPI_CR1_DFF))) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((uint16_t*) pRxBuffer) = SPIx_->DR;
            (uint16_t*) pRxBuffer++;
        } else {
            //8 bit DFF
            *(pRxBuffer) = SPIx_->DR;
            pRxBuffer++;
        }
    }
    while (SPI_GetFlagStatus(SPI_BSY_FLAG))
        ; // w8 until SPI done
}

/*!
 * @brief Configure software slave output enable
 *
 * @param[in]: Enable/Disable
 *
 * @return None
 *
 */
void SPI_Handler::SPI_SSOEConfig(const u8 EnOrDi) {
    if (EnOrDi == ENABLE) {
        SPIx_->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        SPIx_->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}


/*!
 * @brief Configure internal slave select
 *
 * @param[in]: Enable/Disable
 *
 * @return None
 *
 */
void SPI_Handler::SPI_SSIConfig(const u8 EnOrDi) {
    if (EnOrDi == ENABLE) {
        SPIx_->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        SPIx_->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}


/*!
 * @brief Enable Interrupt for NVIC
 *
 * @param[in] IRQNumber     - IRQNumber IRQ Position number
 * @param[in] EnorDi        - Enable/Disable Flag
 *
 * @return None
 *
 */
void SPI_Handler::SPI_IRQInterruptConfig(const u8 IRQNumber, const u8 EnorDi) {
    if (EnorDi == ENABLE) {
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
    } else {
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


/*!
 * @brief Configure priority for GPIO, 0->15, lower SPI_CR2_ERRIE higher priority
 *
 * @param[in] IRQNumber: IRQ position for EXIT
 * @param[in] IRPriority: 0->15
 *
 * @return None
 *
 */
void SPI_Handler::SPI_IRQPriorityConfig(const u8 IRQNumber,
        const u8 IRQPriority) {
    // 1. first lets find out the ipr register
    u8 iprx = IRQNumber >> 2;
    u8 iprx_section = IRQNumber % 4;
    u8 shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*!
 * @brief Send data out via SPI protocol using interrupt. Interrupt will be enable if hw shift buffer is empty
 *
 * @param[in] pTxBuffer: pointer to Txbuffer
 * @param[in] Len: length of data
 *
 * @return u8: Trasmition state
 *
 */
u8 SPI_Handler::SPI_SendDataIT(const u8 *pTxBuffer, u32 Len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }

    while (handle_.TxState != SPI_READY || handle_.RxState != SPI_READY);

    // 1. Save the Tx buffer address and Len information in some global variables
    handle_.pTxBuffer = const_cast<u8*>(pTxBuffer);
    handle_.TxLen = Len;

    // 2. Mark the SPI state as busy in transmission so that no other code
    // can take over same SPI peripheral until transmission is over
    handle_.TxState = SPI_BUSY_IN_TX;

    SPIx_->CR2 |= (1 << SPI_CR2_TXEIE | 1 << SPI_CR2_ERRIE);

    return handle_.TxState;
}

/*!
 * @brief Receive data out via SPI protocol (sending padding data) using interrupt.
 *        Interrupt will be enable if hw shift buffer is NOT empty
 *
 * @param[out] pRxBuffer: pointer to receive buffer
 * @param[in] Len: length of data
 *
 * @return u8: Trasmition state
 *
 */
u8 SPI_Handler::SPI_ReceiveDataIT(u8 *pRxBuffer, u32 Len) {
    while (handle_.TxState != SPI_READY || handle_.RxState != SPI_READY);

    SPI_ClearOVRFlag();
    // 1. Save the Tx buffer address and Len information in some global variables
    handle_.pTxBuffer = pRxBuffer; // transmit dummy
    handle_.TxLen = Len;

    handle_.pRxBuffer = pRxBuffer;
    handle_.RxLen = Len;

    // 2. Mark the SPI state as busy in transmission so that no other code
    // can take over same SPI peripheral until transmission is over
    handle_.RxState = SPI_BUSY_IN_RX;
    handle_.TxState = SPI_BUSY_IN_TX;

    // 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
    SPIx_->CR2 |= (1 << SPI_CR2_RXNEIE | 1 << SPI_CR2_ERRIE
            | 1 << SPI_CR2_TXEIE);

    return handle_.TxState;
}

/*!
 * @brief IRQ handling. This function will be called in interrupt service routine
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_IRQHandling() {
    volatile u8 temp1 = 0, temp2 = 0;
    // check for RXNE
    temp1 = SPIx_->SR & (1 << SPI_SR_RXNE);
    temp2 = SPIx_->CR2 & (1 << SPI_CR2_RXNEIE);
    if (temp1 && temp2) {
        spi_rxne_interrupt_handle();
        return;
    }

    // First lets check for TXE
    temp1 = SPIx_->SR & (1 << SPI_SR_TXE);
    temp2 = SPIx_->CR2 & (1 << SPI_CR2_TXEIE);
    if (temp1 && temp2) {
        // Handle TXE
        spi_txe_interrupt_handle();
        return;
    }

    // check for ovr flag
    temp1 = SPIx_->SR & (1 << SPI_SR_OVR);
    temp2 = SPIx_->CR2 & (1 << SPI_CR2_ERRIE);
    if (temp1 && temp2) {
        spi_ovr_err_interrupt_handle();
        return;
    }

}

/*!
 * @brief This function should be call in interrupt service routine for handling txe.
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::spi_txe_interrupt_handle() {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }

    // 2. check the DFF bit in CR1
    if (SPIx_->CR1 & (1 << SPI_CR1_DFF)) {
        // 16 BIT DFF
        // 1. load the data into the DR
        SPIx_->DR = *((uint16_t*) handle_.pTxBuffer);
        handle_.TxLen -= 2;
        (uint16_t*) handle_.pTxBuffer++;
    } else {
        SPIx_->DR = *handle_.pTxBuffer;
        handle_.TxLen -= 1;
        handle_.pTxBuffer++;
    }

    if (!handle_.TxLen) {
        // TxLen is zero, close the spi transmission and inform the application
        // Tx is over
        SPI_CloseTransmission();
        SPI_ApplicationEventCallback(&handle_, SPI_EVENT_TX_CMPLT);
    }
}

/*!
 * @brief This function should be call in interrupt service routine for handling rxe.
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::spi_rxne_interrupt_handle() {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        SPI_PeripheralControl(ENABLE);
    }

    //2. check the DFF bit in CR1
    if ((SPIx_->CR1 & (1 << SPI_CR1_DFF))) {
        //16 bit DFF
        //1. load the data from DR to Rxbuffer address
        *((uint16_t*) handle_.pRxBuffer) = SPIx_->DR;
        handle_.RxLen -= 2;
        (uint16_t*) handle_.pRxBuffer++;
    } else {
        //8 bit DFF
        *(handle_.pRxBuffer) = SPIx_->DR;
        handle_.RxLen--;
        handle_.pRxBuffer++;
    }

    if (!handle_.RxLen) {
        // TxLen is zero, close the spi transmission and inform the application
        // Tx is over
        SPI_CloseReception();
        SPI_ApplicationEventCallback(&handle_, SPI_EVENT_RX_CMPLT);
    }
}

/*!
 * @brief This function should be call in interrupt service routine for handling overrun error.
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::spi_ovr_err_interrupt_handle() {
    // 1. Clear the ovr flag
    SPI_ClearOVRFlag();
    // 2. Inform the application
    SPI_ApplicationEventCallback(&handle_, SPI_EVENT_OVR_ERR);
}

/*!
 * @brief Clear SPI overrun flag
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_ClearOVRFlag() {
    volatile u8 temp = 0;
    temp = SPIx_->DR;
    temp = SPIx_->SR;
    static_cast<void>(temp);
}

/*!
 * @brief Close Interrupt SPI transmission
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_CloseTransmission() {
    while ((SPIx_->SR & (1 << SPI_SR_TXE)) == RESET)
        ;
    SPIx_->CR2 &= ~(1 << SPI_CR2_TXEIE | 1 << SPI_CR2_ERRIE);
    handle_.pTxBuffer = nullptr;
    handle_.TxLen = 0;
    handle_.TxState = SPI_READY;
    SPI_ClearOVRFlag();
}

/*!
 * @brief Close Interrupt SPI Reception
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::SPI_CloseReception() {
    SPIx_->CR2 &= ~(1 << SPI_CR2_RXNEIE | 1 << SPI_CR2_ERRIE);
    handle_.pRxBuffer = nullptr;
    handle_.RxLen = 0;
    handle_.RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, u8 AppEv) {
    //This is a weak implementation . the user application may override this function.
}

