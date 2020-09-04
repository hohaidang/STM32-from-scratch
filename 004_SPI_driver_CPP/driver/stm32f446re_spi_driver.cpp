/*
 * stm32f446re_spi_driver.cpp
 *
 *  Created on: Jul 16, 2020
 *      Author: hohaidang
 */

#include "stm32f446re_spi_driver.h"
#include "../Src/spi_readSensordata.h"
using namespace std;


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
    spi_ssi_config<ENABLE>();
    if (SPI_SSM_EN == config_.spi_ssm) {
        spi_ssoe_config<DISABLE>();
    } else {
        spi_ssoe_config<ENABLE>();
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
                                            GPIO_ALT_5));
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
 * @brief Init nss pin for sw chip select control
 *
 * @param: None
 *
 * @return None
 *
 */
void SPI_Handler::spi_init_nss_sw(GPIO_RegDef_t *GPIOx_addr, const u8 pin_number) {
    spi_nss_.reset( new GPIO_Handler(GPIOx_addr,
                                pin_number,
                                GPIO_MODE_OUT,
                                GPIO_SPEED_FAST,
                                GPIO_OP_TYPE_PP,
                                GPIO_NO_PUPD) );
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
        spi_peripheral_control<ENABLE>();
    }
    while (Len > 0) {
        // Wait until tx buffer empty
        while (RESET == spi_get_SR_reg<SPI_SR_TXE>());

        if (spix_->CR1 & SPI_CR1_DFF) {
            // 16 BIT DFF
            spix_->DR = *((u16*) pTxBuffer);
            Len -= 2;
            (u16*) pTxBuffer++;
        } else {
            spix_->DR = *pTxBuffer;
            Len -= 1;
            pTxBuffer++;
        }
    }
    while (spi_get_SR_reg<SPI_SR_BSY>()); // w8 until SPI done
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
        spi_peripheral_control<ENABLE>();
    }

    while (Len > 0) {
        //1. wait until RXNE is set
        while (RESET == spi_get_SR_reg<SPI_SR_RXNE>());

        //2. check the DFF bit in CR1
        if ((spix_->CR1 & SPI_CR1_DFF)) {
            //16 bit DFF
            //1. load the data from DR to Rxbuffer address
            *((u16*) pRxBuffer) = spix_->DR;
            Len--;
            Len--;
            (u16*) pRxBuffer++;
        } else {
            //8 bit DFF
            *(pRxBuffer) = spix_->DR;
            Len--;
            pRxBuffer++;
        }
    }
    while (spi_get_SR_reg<SPI_SR_BSY>()); // w8 until SPI done
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
void SPI_Handler::spi_transmit_receive_data(const u8 *p_tx_buffer, u8 *p_rx_buffer, u32 len) {
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control<ENABLE>();
    }
    u32 init_len = len;

    while (len > 0) {
        while (RESET == spi_get_SR_reg<SPI_SR_TXE>()); /* wait until tx buffer empty */

        if (spix_->CR1 & SPI_CR1_DFF) {
            // 16 BIT DFF
            spix_->DR = *((u16*) p_tx_buffer);
            if(init_len == len) {
                /* First transmission is sensor's address Should delay if MCU run so fast, slave cannot load data to shift register on time*/
                SystemTick->delay_ms(1);
            }
            len -= 2;
            (u16*) p_tx_buffer++;
        } else {
            spix_->DR = *p_tx_buffer;
            if(init_len == len) {
                /* First transmission is sensor's address Should delay if MCU run so fast, slave cannot load data to shift register on time*/
                SystemTick->delay_ms(1);
            }
            len -= 1;
            p_tx_buffer++;

        }

        while (RESET == spi_get_SR_reg<SPI_SR_RXNE>()); /* Wait until RXNE is set */

        if ((spix_->CR1 & SPI_CR1_DFF)) {
            //16 bit DFF
            *((u16*) p_rx_buffer) = spix_->DR;
            (u16*) p_rx_buffer++;
        } else {
            //8 bit DFF
            *(p_rx_buffer) = spix_->DR;
            p_rx_buffer++;
        }
    }

    while (spi_get_SR_reg<SPI_SR_BSY>()); // w8 until SPI done
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
void SPI_Handler::spi_transmit_data_it(u8 *p_tx_buf, const u32 len) {
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control<ENABLE>();
    }

    while (handle_.tx_state != SPI_READY or handle_.rx_state != SPI_READY);

    /* 1. Pass pointer to tx buffer and assign length of transmission */
    handle_.tx_buf = p_tx_buf;

    handle_.tx_len = len;
    handle_.init_tx_len = len;
    handle_.rx_len = 0UL; // no receive

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
 * @param[in] len: length of data
 *
 * @return u8: Transmission state
 *
 */
void SPI_Handler::spi_receive_data_it(u8 *p_rx_buf, const u32 len) {
    /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    spi_transmit_receive_data_it(p_rx_buf, p_rx_buf, len);
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
void SPI_Handler::spi_transmit_receive_data_it(u8 *p_tx_buf, u8 *p_rx_buf, const u32 len) {
    if (RESET == (spix_->CR1 & SPI_CR1_SPE)) {
        spi_peripheral_control<ENABLE>();
    }

    /* 1. Pass pointer to tx buffer and assign length of transmission */
    handle_.tx_buf = p_tx_buf;
    handle_.rx_buf = p_rx_buf;
    handle_.tx_len = len;
    handle_.init_tx_len = len;
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

    if(spi_check_flag(SR_reg, SPI_SR_BSY) == SET) {
        return;
    }

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
    *(volatile uint8_t *)&spix_->DR = *handle_.tx_buf;
    ++handle_.tx_buf;
    if(handle_.tx_len == handle_.init_tx_len) {
        /* First transmission is sensor's address Should delay if MCU run so fast, slave cannot load data to shift register on time*/
        SystemTick->delay_ms(1);
    }
    --handle_.tx_len;

    if (!handle_.tx_len) {
        /* Receive done, transmit buffer empty */
        if(!handle_.rx_len) {
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
    *handle_.rx_buf = *((volatile uint8_t *)&spix_->DR);
    ++handle_.rx_buf;
    --handle_.rx_len;

    if (!handle_.rx_len) {
        /* Receive done, transmit buffer empty */
        if(!handle_.tx_len) {
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
    while(spi_get_SR_reg<SPI_SR_BSY>()); /* w8 until SPI transmit done */
    spix_->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    handle_.tx_state = SPI_READY;
    handle_.rx_state = SPI_READY;
    spi_clear_ovr_flag();
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, u8 AppEv) {
    //This is a weak implementation . the user application may override this function.
}

