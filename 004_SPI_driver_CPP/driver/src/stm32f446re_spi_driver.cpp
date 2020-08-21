/*
 * stm32f446re_spi_driver.cpp
 *
 *  Created on: Jul 16, 2020
 *      Author: hohaidang
 */

#include "../inc/stm32f446re_spi_driver.h"
<<<<<<< HEAD
using namespace std;

// TODO: build driver o ngoai
=======
#include <iostream>
using namespace std;

//void printQueue(queue<u8> q) {
//    while(!q.empty()) {
//        cout << "  " << hex << unsigned(q.front());
//        q.pop();
//    }
//    cout << " end" << endl;
//}
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt

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
    spi_gpios_init();
    spi_peripheral_clock_init();
    spi_init();
    spi_ssi_config(ENABLE);
    if (SPI_SSM_EN == config_.SPI_SSM) {
        spi_ssoe_config(DISABLE);
    } else {
        spi_ssoe_config(ENABLE);
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
    spi_deinit();
}


/*!
 * @brief peripheral clock setup for SPI
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::spi_peripheral_clock_init() {
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
void SPI_Handler::spi_gpios_init() {
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
void SPI_Handler::spi_init() {
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
void SPI_Handler::spi_deinit() {
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
// TODO: consider inline
void SPI_Handler::spi_peripheral_control(u8 EnOrDi) {
    if (ENABLE == EnOrDi) {
        SPIx_->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        SPIx_->CR1 &= ~(1 << SPI_CR1_SPE);
    }
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
void SPI_Handler::spi_transmit_data(const u8 *pTxBuffer, u32 Len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        spi_peripheral_control(ENABLE);
    }
    while (Len > 0) {
        // 1. wait util tx buffer empty
        while (RESET == spi_get_SR_reg(SPI_SR_TXE));

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
    while (spi_get_SR_reg(SPI_SR_BSY)); // w8 until SPI done
    spi_clear_OVR_flag(); // because in 2 lines mode, write data does not need to read
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
void SPI_Handler::spi_receive_data(u8 *pRxBuffer, u32 Len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        spi_peripheral_control(ENABLE);
    }

    while (Len > 0) {
        //1. wait until RXNE is set
        while (RESET == spi_get_SR_reg(SPI_SR_RXNE));

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
    while (spi_get_SR_reg(SPI_SR_BSY)); // w8 until SPI done
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
void SPI_Handler::spi_transmit_receive_data(const u8 *pTxBuffer, u8 *pRxBuffer, u32 len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        spi_peripheral_control(ENABLE);
    }
    while (len > 0) {
        // 1. wait util tx buffer empty
        while (RESET == spi_get_SR_reg(SPI_SR_TXE));

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
        while (RESET == spi_get_SR_reg(SPI_SR_RXNE));

        //2. check the DFF bit in CR1
        if ((SPIx_->CR1 & (1 << SPI_CR1_DFF))) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((uint16_t*) pRxBuffer) = SPIx_->DR;
            (uint16_t*) pRxBuffer++;
        } else {
            //8 bit DFF
<<<<<<< HEAD

            *(pRxBuffer) = spix_->DR;
=======
            *(pRxBuffer) = SPIx_->DR;
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
            pRxBuffer++;
        }
    }
    while (spi_get_SR_reg(SPI_SR_BSY)); // w8 until SPI done
}

/*!
 * @brief Configure software slave output enable
 *
 * @param[in]: Enable/Disable
 *
 * @return None
 *
 */
void SPI_Handler::spi_ssoe_config(const u8 EnOrDi) {
    if (EnOrDi == ENABLE) {
        SPIx_->CR2 |= SPI_CR2_SSOE;
    } else {
        SPIx_->CR2 &= ~(SPI_CR2_SSOE);
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
// TODO: consider to be inline function
void SPI_Handler::spi_ssi_config(const u8 EnOrDi) {
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
void SPI_Handler::spi_ir_config(const u8 IRQNumber, const u8 EnorDi) {
    if (EnorDi == ENABLE) {
        if (IRQNumber <= 31) {
            //  program ISER0 register
            NVIC->ISER[0] |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            // program ISER1 register
            NVIC->ISER[1] |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // program ISER2 register
            NVIC->ISER[2] |= (1 << (IRQNumber % 64));
        }
    } else {
        if (IRQNumber <= 31) {
            // program ICER0 register
            NVIC->ICER[0] |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            // program ICER1 register
            NVIC->ICER[1] |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // program ICE2 register
            NVIC->ICER[2] |= (1 << (IRQNumber % 64));
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
void SPI_Handler::spi_ir_prio_config(const u8 IRQNumber, const u8 IRQPriority) {
    // 1. first lets find out the ipr register
    u8 iprx = IRQNumber >> 2;
    u8 iprx_section = IRQNumber % 4;
    u8 shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    NVIC->IPR[iprx] |= (IRQPriority << shift_amount);
}

/*!
 * @brief Send data out via SPI protocol using interrupt. Interrupt will be enable if hw shift buffer is empty
 *
 * @param[in] p_tx_buf: pointer to Txbuffer
 * @param[in] len: length of data
 *
 * @return u8: Transmission state
 *
 */
<<<<<<< HEAD
void SPI_Handler::spi_transmit_data_it(u8 *p_tx_buf, const u32 len) {
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
=======
u8 SPI_Handler::spi_transmit_data_it(vector<u8> &TxBuffer, const u32 Len) {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
        spi_peripheral_control(ENABLE);
    }

    while (handle_.TxState != SPI_READY or handle_.RxState != SPI_READY);

<<<<<<< HEAD
    /* 1. Pass pointer to tx buffer and assign length of transmission */
    handle_.tx_buf = p_tx_buf;

    handle_.tx_len = len;
    handle_.rx_len = 0UL; // no receive

    if(spix_->CR1 & SPI_CR1_DFF) {
        /* Not Support 16bit yet */
        handle_.transmit_fnc = nullptr;
    }
    else {
        handle_.transmit_fnc = [=](){this->spi_tx_8bit_it();};
=======
    // 1. Save the Tx buffer address and Len information in some global variables
    for(auto it = TxBuffer.begin(); it != TxBuffer.end(); ++it) {
        handle_.TxBuffer.push(*it);
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
    }
//    handle_.pTxBuffer = TxBuffer.begin();

    handle_.TxLen = Len;

    // 2. Mark the SPI state as busy in transmission so that no other code
    // can take over same SPI peripheral until transmission is over
    handle_.TxState = SPI_BUSY_IN_TX;

    SPIx_->CR2 |= (SPI_CR2_TXEIE | SPI_CR2_ERRIE);

    return handle_.TxState;
}

/*!
 * @brief Receive data out via SPI protocol (sending padding data) using interrupt.
 *        Interrupt will be enable if hw shift buffer is NOT empty
 *
 * @param[out] pRxBuffer: pointer to receive buffer
 * @param[in] len: length of data
 *
 * @return u8: Transmission state
 *
 */
<<<<<<< HEAD
void SPI_Handler::spi_receive_data_it(u8 *p_rx_buf, const u32 len) {
    /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    spi_transmit_receive_data_it(p_rx_buf, p_rx_buf, len);
=======
u8 SPI_Handler::spi_receive_data_it(vector<u8> &RxBuffer, const u32 Len) {
    /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    return spi_transmit_receive_data_it(RxBuffer, RxBuffer, Len);
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
}

/*!
 * @brief Send and Receive data out via SPI protocol (sending padding data) using interrupt.
 *
 * @param[in] pTxBuffer: pointer to transmit buffer
 * @param[out] pRxBuffer: pointer to receive buffer
 * @param[in] len: length of data
 *
 * @return u8: Transmission State
 *
 */
<<<<<<< HEAD
void SPI_Handler::spi_transmit_receive_data_it(u8 *p_tx_buf, u8 *p_rx_buf, const u32 len) {
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control(ENABLE);
    }

    /* 1. Pass pointer to tx buffer and assign length of transmission */
    handle_.tx_buf = p_tx_buf;
    handle_.rx_buf = p_rx_buf;
    handle_.tx_len = len;
    handle_.rx_len = len;

    if(spix_->CR1 & SPI_CR1_DFF) {
        /* Not Support 16bit yet */
        handle_.transmit_fnc = nullptr;
        handle_.receive_fnc = nullptr;
    }
    else {
        handle_.transmit_fnc = [=](){this->spi_tx_8bit_it();};
        handle_.receive_fnc = [=](){this->spi_rx_8bit_it();};
=======
u8 SPI_Handler::spi_transmit_receive_data_it(vector<u8> &pTxBuffer, vector<u8> &pRxBuffer, const u32 Len) {
    while (handle_.TxState != SPI_READY || handle_.RxState != SPI_READY);
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        spi_peripheral_control(ENABLE);
    }

    for(auto it = pTxBuffer.begin(); it != pTxBuffer.end(); ++it) {
        handle_.TxBuffer.push(*it);
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
    }
//    handle_.TxBuffer.assign(pTxBuffer.begin(), pTxBuffer.end());
//    handle_.pTxBuffer = pTxBuffer.begin();
    // TODO: considering replace Len with queue size
    handle_.TxLen = Len;

//    handle_.RxBuffer.assign(pRxBuffer.begin(), pRxBuffer.end());
//    handle_.pRxBuffer = pRxBuffer.begin();
    handle_.RxLen = Len;

    /* Mark the SPI state as busy in transmission so that no other code
     * can take over same SPI peripheral until transmission is over
     */
    handle_.RxState = SPI_BUSY_IN_RX;
    handle_.TxState = SPI_BUSY_IN_TX;

    /* Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR */
<<<<<<< HEAD
    spix_->CR2 |= (SPI_CR2_RXNEIE | SPI_CR2_ERRIE | SPI_CR2_TXEIE);

    while(handle_.rx_state != SPI_READY or handle_.tx_state != SPI_READY); /* w8 until SPI transmit completed*/
=======
    SPIx_->CR2 |= (SPI_CR2_RXNEIE | SPI_CR2_ERRIE | SPI_CR2_TXEIE);

    // TODO: considering remove later
    while(handle_.RxState != SPI_READY or handle_.TxState != SPI_READY);
//    printQueue(handle_.RxBuffer);
    while(!handle_.RxBuffer.empty()) {
        pRxBuffer.push_back(handle_.RxBuffer.front());
        handle_.RxBuffer.pop();
    }

    return handle_.TxState;
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
}



/*!
 * @brief IRQ handling. This function will be called in interrupt service routine
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::spi_irq_handling() {
<<<<<<< HEAD
    volatile u32 SR_reg = spix_->SR;
    volatile u32 CR2_reg = spix_->CR2;
//    cout << "sr_reg = " << hex << unsigned(SR_reg) << endl;
//    cout << "cr2_reg = " << hex << unsigned(CR2_reg) << endl;

    if(spi_check_flag(SR_reg, SPI_SR_BSY) == SET) {
        return;
    }
=======
    volatile u32 SR_reg = SPIx_->SR;
    volatile u32 CR2_reg = SPIx_->CR2;
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt

    /* Receive Interrupt
     * If overrun occurs, transmission may on going or overrun's previous transmission*/
    if (spi_check_flag(SR_reg, SPI_SR_RXNE) and spi_check_flag(CR2_reg, SPI_CR2_RXNEIE)
                                            and !spi_check_flag(SR_reg, SPI_SR_OVR)) {
//        cout << "rx interrupt is called\n";

        spi_rxne_interrupt_handle();
//        printQueue(handle_.RxBuffer);
        return;
    }

    /* Transmit Interrupt */
    if (spi_check_flag(SR_reg, SPI_SR_TXE) and spi_check_flag(CR2_reg, SPI_CR2_TXEIE)) {
//        cout << "tx interrupt is called\n";
//        printQueue(handle_.TxBuffer);
        spi_txe_interrupt_handle();

        return;
    }

    /* Overrun Interrupt */
    if (spi_check_flag(SR_reg, SPI_SR_OVR) and spi_check_flag(CR2_reg, SPI_CR2_ERRIE)){
//        cout << "overrun interruprt called\n" << endl;
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
<<<<<<< HEAD
void SPI_Handler::spi_tx_8bit_it() {
    cout << "tx:" << hex << unsigned(*handle_.tx_buf);
    *(volatile uint8_t *)&spix_->DR = *handle_.tx_buf;
    ++handle_.tx_buf;
    --handle_.tx_len;
//    cout << "tx len: " << handle_.tx_len << endl;

    if (!handle_.tx_len) {
        /* Receive done, transmit buffer empty */
        if(!handle_.rx_len) {
=======
void SPI_Handler::spi_txe_interrupt_handle() {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        spi_peripheral_control(ENABLE);
    }

    // 2. check the DFF bit in CR1
    if (SPIx_->CR1 & (1 << SPI_CR1_DFF)) {
        // TODO: Implement later
//        // 16 BIT DFF
//        SPIx_->DR = *(reinterpret_cast<u16 *>(handle_.pTxBuffer));
//        handle_.TxLen -= 2;
//        reinterpret_cast<u16 *>(handle_.pTxBuffer++);
    } else {
//        cout << "send 0x" << hex << unsigned(handle_.TxBuffer.front()) << endl;
        SPIx_->DR = handle_.TxBuffer.front();
        handle_.TxBuffer.pop();
        handle_.TxLen -= 1;
//        SPIx_->DR = *handle_.pTxBuffer;

//        handle_.pTxBuffer++;
    }
//    cout << "RxLen = " << unsigned(handle_.RxLen) << ", TxLen = " << handle_.TxLen << endl;
    if (!handle_.TxLen) {
        // TxLen is zero, close the spi transmission and inform the application
        // Tx is over
        if(handle_.RxLen == 0U) {
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
            spi_close_tx_rx_isr();
            SPI_ApplicationEventCallback(&handle_, SPI_EVENT_TX_CMPLT);
        }

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
<<<<<<< HEAD
void SPI_Handler::spi_rx_8bit_it() {
    *handle_.rx_buf = *((volatile uint8_t *)&spix_->DR);
//    cout << "rx:" << hex << unsigned(*handle_.rx_buf) << endl;
    ++handle_.rx_buf;
    --handle_.rx_len;
//    cout << "rx len: " << handle_.rx_len << endl;

    if (!handle_.rx_len) {
        /* Receive done, transmit buffer empty */
        if(!handle_.tx_len) {
=======
void SPI_Handler::spi_rxne_interrupt_handle() {
    if ((SPIx_->CR1 & SPI_CR1_SPE_MSK) != SPI_CR1_SPE_MSK) {
        spi_peripheral_control(ENABLE);
    }

    //2. check the DFF bit in CR1
    if ((SPIx_->CR1 & (1 << SPI_CR1_DFF))) {
        // TODO: Implement later
//        //16 bit DFF
//        //1. load the data from DR to Rxbuffer address
//        *((uint16_t*) handle_.pRxBuffer) = SPIx_->DR;
//        handle_.RxLen -= 2;
//        (uint16_t*) handle_.pRxBuffer++;
    } else {
        //8 bit DFF
        handle_.RxBuffer.push(SPIx_->DR);
//        cout << "Current Queue: ";
//        printQueue(handle_.RxBuffer);
        handle_.RxLen--;
//        *(handle_.pRxBuffer) = SPIx_->DR;
//        handle_.RxLen--;
//        handle_.pRxBuffer++;
    }
//    cout << "RxLen = " << unsigned(handle_.RxLen) << ", TxLen = " << handle_.TxLen << endl;
    if (!handle_.RxLen) {
        // TxLen is zero, close the spi transmission and inform the application
        // Tx is over
        if(handle_.TxLen == 0U) {
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
            spi_close_tx_rx_isr();
            SPI_ApplicationEventCallback(&handle_, SPI_EVENT_RX_CMPLT);
        }

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
    spi_clear_OVR_flag();
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
void SPI_Handler::spi_clear_OVR_flag() {
    volatile u32 temp = 0x00U;
    temp = SPIx_->DR;
    temp = SPIx_->SR;
    static_cast<void>(temp);
}

/*!
 * @brief Close Transmission and Receive in Interrupt mode
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::spi_close_tx_rx_isr() {
    while(spi_get_SR_reg(SPI_SR_BSY)); /* w8 until SPI transmit done */
<<<<<<< HEAD
    cout << "called close tx rx" << endl;
    spix_->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    handle_.tx_state = SPI_READY;
    handle_.rx_state = SPI_READY;
    spi_clear_ovr_flag();
=======
    SPIx_->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    handle_.TxState = SPI_READY;
    handle_.RxState = SPI_READY;
    spi_clear_OVR_flag();
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
}

/*!
 * @brief Get transmission state
 *
 * @param: None
 *
 * @return u8: transmission state
 *
 */
u8 SPI_Handler::spi_get_tx_state() {
    return handle_.TxState;
}

/*!
 * @brief Get receive state
 *
 * @param: None
 *
 * @return u8: receoption state
 *
 */
u8 SPI_Handler::spi_get_rx_state() {
    return handle_.RxState;
}

/*!
 * @brief Get Flag register status
 *
 * @param[in]         - FlagName: check @FLAG_NAME_STATUS
 *
 * @return None
 *
 */
inline u8 SPI_Handler::spi_get_SR_reg(const u32 FlagName) {
    return (SPIx_->SR & FlagName) ? SET : RESET;
}

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __REG__  copy of SPI registers.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_SR_RXNE: Receive buffer not empty flag
  *            @arg SPI_SR_TXE: Transmit buffer empty flag
  *            @arg SPI_SR_OVR: Overrun flag
  *            @arg SPI_SR_BSY: Busy flag
  * @retval SET or RESET.
  */
inline u8 SPI_Handler::spi_check_flag(const u32 __REG__, const u32 __FLAG__) {
    return ((__REG__ & __FLAG__) != RESET) ? SET : RESET;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, u8 AppEv) {
    //This is a weak implementation . the user application may override this function.
}

