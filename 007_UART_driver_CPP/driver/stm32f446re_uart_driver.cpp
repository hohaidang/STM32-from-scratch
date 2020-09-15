#include "stm32f446re_uart_driver.h"

void uart_handler::init() const {
  init_peripheral_clock();
  setup_control_regs();
}

/*!
 * @brief peripheral clock setup for UART
 *
 * @param: None
 *
 * @return None
 *
 */
void uart_handler::init_peripheral_clock() const {
  if (USART1 == uartx_) {
    USART1_PCLK_EN();
  } else if (USART2 == uartx_) {
    USART2_PCLK_EN();
  } else if (USART3 == uartx_) {
    USART3_PCLK_EN();
  } else if (UART4 == uartx_) {
    UART4_PCLK_EN();
  } else if (UART5 == uartx_) {
    UART5_PCLK_EN();
  } else if (USART6 == uartx_) {
    USART6_PCLK_EN();
  }
}

/*!
 * @brief set up control register for UART
 *
 * @param: None
 *
 * @return None
 *
 */
void uart_handler::setup_control_regs() const {
  /*********** Congigure CR1 ***********/
  u32 temp_reg = 0;
  /* Mode */
  if (UART_MODE_ONLY_TX == config_.mode) {
    temp_reg |= UART_CR1_TE;
  } else if (UART_MODE_ONLY_RX == config_.mode) {
    temp_reg |= UART_CR1_RE;
  } else if (UART_MODE_TXRX == config_.mode) {
    temp_reg |= (UART_CR1_TE | UART_CR1_RE);
  }
  /* Word Length*/
  temp_reg |= static_cast<u32>(config_.word_length << UART_CR1_M_Pos);

  /* Parity control */
  if (UART_PARITY_EN_EVEN == config_.parity_control) {
    temp_reg |= UART_CR1_PCE; /* PS = 0 */
  } else if (UART_PARITY_EN_EVEN == config_.parity_control) {
    temp_reg |= UART_CR1_PCE;
    temp_reg |= UART_CR1_PS;
  }

  uartx_->cr1 = temp_reg;

  /*********** Congigure CR2 ***********/
  temp_reg = 0;
  /* Number of STOP bits */
  temp_reg |= static_cast<u32>(config_.no_of_stop_bits << UART_CR2_STOP_Pos);
  uartx_->cr2 = temp_reg;

  /*********** Congigure CR3 ***********/
  temp_reg = 0;
  if (config_.hw_flow_control == UART_HW_FLOW_CTRL_CTS) {
    temp_reg |= UART_CR3_CTSE;
  } else if (config_.hw_flow_control == UART_HW_FLOW_CTRL_RTS) {
    temp_reg |= UART_CR3_RTSE;
  } else if (config_.hw_flow_control == UART_HW_FLOW_CTRL_CTS_RTS) {
    temp_reg |= (UART_CR3_CTSE | UART_CR3_RTSE);
  }
  uartx_->cr3 = temp_reg;

  /*********** Congigure BRR ***********/
  set_baud_rate();
}

/*!
 * @brief TODO:
 *
 * @param: None
 *
 * @return None
 *
 */
void uart_handler::set_baud_rate() const {}

/*!
 * @brief This is API is used for send UART data concurrently
 *
 * @param[in] tx_buffer : Transmit buffer data
 * @param[in] len: len of transmit
 *
 * @return None
 *
 */
void uart_handler::transmit_data(u8 *tx_buffer, u32 len) const {
  if (RESET == (uartx_->cr1 & UART_CR1_UE)) {
    peripheral_control<ENABLE>();
  }
  while (len > 0) {
    while (RESET == get_sr_reg<UART_SR_TXE>())
      ; /* wait until tx buffer empty */
    /*TODO: Word Length 9bit not supported yet*/
    uartx_->dr = *tx_buffer;
    --len;
    ++tx_buffer;
  }
  while (get_sr_reg<UART_SR_TC>())
    ; /* wait until transmission completed */
}

/*!
 * @brief This is API is used for receive UART data concurrently
 *
 * @param[out] rx_buffer : receive buffer data
 * @param[in] len: len of transmit
 *
 * @return None
 *
 */
void uart_handler::receive_data(u8 *rx_buffer, u32 len) const {
  if (RESET == (uartx_->cr1 & UART_CR1_UE)) {
    peripheral_control<ENABLE>();
  }
  for (u32 i = 0; i < len; ++i) {
    while (RESET == get_sr_reg<UART_SR_RXNE>())
      ; /* wait until received data is ready*/
    /*TODO: Word Length 9bit not supported yet */
    if (UART_PARITY_DISABLE == config_.parity_control) {
      *(rx_buffer) = (uartx_->dr & 0xFF);
    } else {
      *(rx_buffer) = (uartx_->dr & 0x7F); /* MSB is parity bit */
    }
    ++rx_buffer;
  }
};