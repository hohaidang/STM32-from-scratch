/*
 * stm32f446re_spi_driver.cpp
 *
 *  Created on: Jul 16, 2020
 *      Author: hohaidang
 */

#include "../inc/stm32f446re_spi_driver.h"
#include <algorithm>
using namespace std;

// TODO: build driver o ngoai
//void printQueue(queue<u8> q) {
//    while(!q.empty()) {
//        cout << "  " << hex << unsigned(q.front());
//        q.pop();
//    }
//    cout << " end" << endl;
//}

/*!
 * @brief Initialize SPI, GPIOs, Interrupt, etc.
 *
 * @param: None
 *
 * @return None
 *
 */
SPI_Handler::SPI_Handler(SPI_RegDef_t *spix_addr,
                        u8 device_mode,
                        u8 bus_config,
                        u8 sclk_speed,
                        u8 dff,
                        u8 cpol,
                        u8 cpha,
                        u8 ssm) : spix_(spix_addr),
                                  config_({device_mode, bus_config, sclk_speed, dff, cpol, cpha, ssm}){
    spi_gpios_init();
    spi_peripheral_clock_init();
    spi_init();
    spi_ssi_config(ENABLE);
    if (SPI_SSM_EN == config_.spi_ssm) {
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
    if (SPI1 == spix_) {
        SPI1_PCLK_EN();
    } else if (SPI2 == spix_) {
        SPI2_PCLK_EN();
    } else if (SPI3 == spix_) {
        SPI3_PCLK_EN();
    } else if (SPI4 == spix_) {
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
    if (spix_ == SPI1) {
        spi_sck_.reset(new GPIO_Handler( GPIOA,
                                        GPIO_PIN_NO_5,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        IRQ_Prio_NO_15,
                                        GPIO_ALT_5));

        spi_mosi_.reset(new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_7,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        IRQ_Prio_NO_15,
                                        GPIO_ALT_5));

        spi_miso_.reset(new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_6,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        IRQ_Prio_NO_15,
                                        GPIO_ALT_5));

        if (SPI_SSM_DI == config_.spi_ssm) {
            // Hardware NSS enable, configure for PA4
            // TODO: develop feature for NSS
            spi_nss_.reset(new GPIO_Handler( GPIOA,
                                            GPIO_PIN_NO_4,
                                            GPIO_MODE_ALTFN,
                                            GPIO_SPEED_HIGH,
                                            GPIO_OP_TYPE_PP,
                                            GPIO_NO_PUPD,
                                            IRQ_Prio_NO_15,
                                            5));
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
    u32 temp_reg = 0;

    // 1. configure the device mode
    temp_reg |= config_.spi_device_mode << SPI_CR1_MSTR_Pos;

    // 2. configure the bus config
    if (SPI_BUS_CONFIG_FD == config_.spi_bus_config) {
        // bidi mode should be cleared
        temp_reg &= ~(SPI_CR1_BIDIMODE);
    } else if (SPI_BUS_CONFIG_HD == config_.spi_bus_config) {
        // bidi mode should be set
        temp_reg |= SPI_CR1_BIDIMODE;
        // bidioe mode should be set
        temp_reg |= SPI_CR1_BIDIOE;
    } else if (SPI_BUS_CONFIG_SIMPLEX_RXONLY == config_.spi_bus_config) {
        // bidi mode should be cleared, for active SCLK, because SLCK is only activate when MOSI is activate
        temp_reg &= ~(SPI_CR1_BIDIMODE);
        // RXONLY should be set
        temp_reg |= SPI_CR1_RXONLY;
    }

    // 3. configure for clock speed
    temp_reg |= (config_.spi_sclk_speed << SPI_CR1_BR_Pos);

    // 4. configure for data format
    temp_reg |= (config_.spi_dff << SPI_CR1_DFF_Pos);

    // 5. configure CPOL
    temp_reg |= (config_.spi_cpol << SPI_CR1_CPOL_Pos);

    // 6. configure CPOA
    temp_reg |= (config_.spi_cpha << SPI_CR1_CPHA_Pos);

    temp_reg |= (config_.spi_ssm << SPI_CR1_SSM_Pos);

    spix_->CR1 = temp_reg;
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
    if (SPI1 == spix_) {
        SPI1_REG_RESET();
    } else if (SPI2 == spix_) {
        SPI2_REG_RESET();
    } else if (SPI3 == spix_) {
        SPI3_REG_RESET();
    } else if (SPI4 == spix_) {
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
inline void SPI_Handler::spi_peripheral_control(const u8 EnOrDi) {
    if (ENABLE == EnOrDi) {
        spix_->CR1 |= SPI_CR1_SPE;
    } else {
        spix_->CR1 &= ~SPI_CR1_SPE;
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
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control(ENABLE);
    }

    while (Len > 0) {
        // 1. wait util tx buffer empty
        while (RESET == spi_get_SR_reg(SPI_SR_TXE));

        // 2. check the DFF bit in CR1
        if (spix_->CR1 & SPI_CR1_DFF) {
            // 16 BIT DFF
            // 1. load the data into the DR
            spix_->DR = *((uint16_t*) pTxBuffer);
            Len -= 2;
            (uint16_t*) pTxBuffer++;
        } else {
            spix_->DR = *pTxBuffer;
            Len -= 1;
            pTxBuffer++;
        }
    }
    while (spi_get_SR_reg(SPI_SR_BSY)); // w8 until SPI done
    spi_clear_ovr_flag(); // because in 2 lines mode, write data does not need to read
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
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control(ENABLE);
    }

    while (Len > 0) {
        //1. wait until RXNE is set
        while (RESET == spi_get_SR_reg(SPI_SR_RXNE));

        //2. check the DFF bit in CR1
        if ((spix_->CR1 & SPI_CR1_DFF)) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((uint16_t*) pRxBuffer) = spix_->DR;
            Len--;
            Len--;
            (uint16_t*) pRxBuffer++;
        } else {
            //8 bit DFF
            *(pRxBuffer) = spix_->DR;
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
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control(ENABLE);
    }
    while (len > 0) {
        // 1. wait util tx buffer empty
        while (RESET == spi_get_SR_reg(SPI_SR_TXE));

        // 2. check the DFF bit in CR1
        if (spix_->CR1 & SPI_CR1_DFF) {
            // 16 BIT DFF
            // 1. load the data into the DR
            spix_->DR = *((uint16_t*) pTxBuffer);
            len -= 2;
            (uint16_t*) pTxBuffer++;
        } else {
            spix_->DR = *pTxBuffer;
            len -= 1;
            pTxBuffer++;
        }

        //1. wait until RXNE is set
        while (RESET == spi_get_SR_reg(SPI_SR_RXNE));

        //2. check the DFF bit in CR1
        if ((spix_->CR1 & SPI_CR1_DFF)) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((uint16_t*) pRxBuffer) = spix_->DR;
            (uint16_t*) pRxBuffer++;
        } else {
            //8 bit DFF
            *(pRxBuffer) = spix_->DR;
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
inline void SPI_Handler::spi_ssoe_config(const u8 EnOrDi) {
    if (EnOrDi == ENABLE) {
        spix_->CR2 |= SPI_CR2_SSOE;
    } else {
        spix_->CR2 &= ~(SPI_CR2_SSOE);
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
inline void SPI_Handler::spi_ssi_config(const u8 EnOrDi) {
    if (EnOrDi == ENABLE) {
        spix_->CR1 |= SPI_CR1_SSI;
    } else {
        spix_->CR1 &= ~SPI_CR1_SSI;
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
 * @param[in] pTxBuffer: pointer to Txbuffer
 * @param[in] Len: length of data
 *
 * @return u8: Transmission state
 *
 */
void SPI_Handler::spi_transmit_data_it(const vector<u8> &tx_buf, const u32 len) {
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control(ENABLE);
    }

    while (handle_.tx_state != SPI_READY or handle_.rx_state != SPI_READY);

    // 1. Pass data to queue
    for_each(tx_buf.begin(), tx_buf.end(), [=](const u8 &data){
        handle_.tx_buffer.push(data);
    });

    handle_.rx_len = 0U; // no receive

    if(spix_->CR1 & SPI_CR1_DFF) {
        /* Not Support 16bit yet */
        handle_.transmit_fnc = nullptr;
    }
    else {
        handle_.transmit_fnc = [=](){this->spi_tx_8bit_it();};
    }

    // 2. Mark the SPI state as busy in transmission so that no other code
    // can take over same SPI peripheral until transmission is over
    handle_.tx_state = SPI_BUSY_IN_TX;

    spix_->CR2 |= (SPI_CR2_TXEIE | SPI_CR2_ERRIE);

    while(handle_.tx_state != SPI_READY);
}

/*!
 * @brief Receive data out via SPI protocol (sending padding data) using interrupt.
 *        Interrupt will be enable if hw shift buffer is NOT empty
 *
 * @param[out] pRxBuffer: pointer to receive buffer
 * @param[in] Len: length of data
 *
 * @return u8: Transmission state
 *
 */
void SPI_Handler::spi_receive_data_it(vector<u8> &rx_buf, const u32 len) {
    /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    vector<u8> dummy_tx(len, 0xFF);
    spi_transmit_receive_data_it(dummy_tx, rx_buf, len);
}

/*!
 * @brief Send and Receive data out via SPI protocol (sending padding data) using interrupt.
 *
 * @param[in] pTxBuffer: pointer to transmit buffer
 * @param[out] pRxBuffer: pointer to receive buffer
 * @param[in] Len: length of data
 *
 * @return u8: Transmission State
 *
 */
void SPI_Handler::spi_transmit_receive_data_it(const vector<u8> &tx_buf, vector<u8> &rx_buf, const u32 len) {
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control(ENABLE);
    }

    /* 1. Pass data to queue */
    for_each(tx_buf.begin(), tx_buf.end(), [=](const u8 &data){
        handle_.tx_buffer.push(data);
    });


    handle_.rx_len = len;
    if(spix_->CR1 & SPI_CR1_DFF) {
        /* Not Support 16bit yet */
        handle_.transmit_fnc = nullptr;
        handle_.receive_fnc = nullptr;
    }
    else {
        handle_.transmit_fnc = [=](){this->spi_tx_8bit_it();};
        handle_.receive_fnc = [=](){this->spi_rx_8bit_it();};
    }

    /* Mark the SPI state as busy in transmission so that no other code
     * can take over same SPI peripheral until transmission is over
     */
    handle_.rx_state = SPI_BUSY_IN_RX;
    handle_.tx_state = SPI_BUSY_IN_TX;

    /* Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR */
    spix_->CR2 |= (SPI_CR2_RXNEIE | SPI_CR2_ERRIE | SPI_CR2_TXEIE);

    while(handle_.rx_state != SPI_READY or handle_.tx_state != SPI_READY); /* w8 until SPI transmit completed*/
    while(!handle_.rx_buffer.empty()) {
        rx_buf.push_back(handle_.rx_buffer.front());
        handle_.rx_buffer.pop();
    }
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
    volatile u32 SR_reg = spix_->SR;
    volatile u32 CR2_reg = spix_->CR2;

    /* Receive Interrupt
     * If overrun occurs, transmission may on going or overrun's previous transmission*/
    if (spi_check_flag(SR_reg, SPI_SR_RXNE) and spi_check_flag(CR2_reg, SPI_CR2_RXNEIE)
                                            and !spi_check_flag(SR_reg, SPI_SR_OVR)) {
        handle_.receive_fnc();
        return;
    }

    /* Transmit Interrupt */
    if (spi_check_flag(SR_reg, SPI_SR_TXE) and spi_check_flag(CR2_reg, SPI_CR2_TXEIE)) {
        handle_.transmit_fnc();
        return;
    }

    /* Overrun Interrupt */
    if (spi_check_flag(SR_reg, SPI_SR_OVR) and spi_check_flag(CR2_reg, SPI_CR2_ERRIE)){
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
void SPI_Handler::spi_tx_8bit_it() {
    spix_->DR = handle_.tx_buffer.front();
    handle_.tx_buffer.pop();

    if (handle_.tx_buffer.empty()) {
        /* Receive done, transmit buffer empty */
        if(handle_.rx_len == 0U) {
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
void SPI_Handler::spi_rx_8bit_it() {
    handle_.rx_buffer.push(spix_->DR);
    handle_.rx_len--;

    if (!handle_.rx_len) {
        /* Receive done, transmit buffer empty */
        if(handle_.tx_buffer.empty()) {
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
    /* 1. Clear the ovr flag */
    spi_clear_ovr_flag();
    /* 2. Inform the application */
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
void SPI_Handler::spi_clear_ovr_flag() {
    volatile u32 temp = 0x00UL;
    temp = spix_->DR;
    temp = spix_->SR;
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
    spix_->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    handle_.tx_state = SPI_READY;
    handle_.rx_state = SPI_READY;
    spi_clear_ovr_flag();
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
    return handle_.tx_state;
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
    return handle_.rx_state;
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
    return (spix_->SR & FlagName) ? SET : RESET;
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

