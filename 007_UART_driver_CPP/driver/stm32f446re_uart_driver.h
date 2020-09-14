#include "stm32f4xx.h"

/*
  @UART_MODE
*/
#define UART_MODE_ONLY_TX (0u)
#define UART_MODE_ONLY_RX (1u)
#define UART_MODE_TXRX (2u)

/*
  @UART_BAUD
*/
#define UART_STD_BAUD_1200 (1200u)
#define UART_STD_BAUD_2400 (2400u)
#define UART_STD_BAUD_9600 (9600u)
#define UART_STD_BAUD_19200 (19200u)
#define UART_STD_BAUD_38400 (38400u)
#define UART_STD_BAUD_57600 (57600u)
#define UART_STD_BAUD_115200 (115200u)

/*
  @UART_ParityControl
*/
#define UART_PARITY_EN_ODD (2u)
#define UART_PARITY_EN_EVEN (1u)
#define UART_PARITY_DISABLE (0u)

/*
  @UART_WordLength
*/
#define UART_WORDLEN_8BITS (0u)
#define UART_WORDLEN_9BITS (1u)

/*
  @UART_NoOfStopBits
*/
#define UART_STOPBIT_1 (0u)
#define UART_STOPBIT_0_5 (1u)
#define UART_STOPBIT_2 (2u)
#define UART_STOPBIT_1_5 (3u)

/*
  @UART_HWFlowControl
*/
#define UART_HW_FLOW_CTRL_NONE (0u)
#define UART_HW_FLOW_CTRL_CTS (1u)
#define UART_HW_FLOW_CTRL_RTS (2u)
#define UART_HW_FLOW_CTRL_CTS_RTS (3u)

typedef struct {
  u32 baud;
  u8 mode;
  u8 no_of_stop_bits;
  u8 word_length;
  u8 parity_control;
  u8 hw_flow_control;
} uart_config_t;

class uart_handler {
public:
  constexpr uart_handler(uart_regdef_t *uartx_addr, u8 mode, u32 baud,
                         u8 no_of_stop_bits, u8 word_length, u8 parity_control,
                         u8 hw_flow_control)
      : uartx_(uartx_addr), config_{baud,
                                    mode,
                                    no_of_stop_bits,
                                    word_length,
                                    parity_control,
                                    hw_flow_control} {};
  void init() const;
  void transmit_data(u8 *tx_buffer, u32 len) const;
  void receive_data(u8 *rx_buffer, u32 len) const;

  void transmit_data_it() const;
  void receive_data_it() const;
  template <u32 flag_name> inline u8 get_sr_reg() const {
    return (uartx_->sr & flag_name) ? SET : RESET;
  };

private:
  template <u8 en_or_di> constexpr inline void peripheral_control() const {
    if (ENABLE == en_or_di) {
      uartx_->cr1 |= UART_CR1_UE;
    } else {
      uartx_->cr1 &= ~(UART_CR1_UE);
    }
  };
  void setup_control_regs() const;
  void set_baud_rate() const;
  void init_peripheral_clock() const;

protected:
  uart_regdef_t *uartx_;
  uart_config_t config_ = {};
};