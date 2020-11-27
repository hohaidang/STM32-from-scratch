/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: Jul 16, 2020
 *      Author: prnsoft
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "core_cm4.h"
#include "stm32f446re_gpio_driver.h"
#include "stm32f4xx.h"
#include <array>
#include <functional>

// @SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER static_cast<u8>(1) // choose SPI as Master
#define SPI_DEVICE_MODE_SLAVE static_cast<u8>(0)

// @SPI_BusConfig
#define SPI_BUS_CONFIG_FD static_cast<u8>(1) // full duplex
#define SPI_BUS_CONFIG_HD static_cast<u8>(2) // half duplex, transmit only
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY static_cast<u8>(3) // simplex Rx only

// @SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2 static_cast<u8>(0)
#define SPI_SCLK_SPEED_DIV4 static_cast<u8>(1)
#define SPI_SCLK_SPEED_DIV8 static_cast<u8>(2)
#define SPI_SCLK_SPEED_DIV16 static_cast<u8>(3)
#define SPI_SCLK_SPEED_DIV32 static_cast<u8>(4)
#define SPI_SCLK_SPEED_DIV64 static_cast<u8>(5)
#define SPI_SCLK_SPEED_DIV128 static_cast<u8>(6)
#define SPI_SCLK_SPEED_DIV256 static_cast<u8>(7)

// @SPI_DFF, data format
#define SPI_DFF_8BITS static_cast<u8>(0)
#define SPI_DFF_16BITS static_cast<u8>(1)

// @SPI_CPOL
// start clock with high, idle clock is high
#define SPI_CPOL_HIGH static_cast<u8>(1)
// start clock with low, idle clock is low
#define SPI_CPOL_LOW static_cast<u8>(0)

// @SPI_CPHA
#define SPI_CPHA_HIGH static_cast<u8>(1) // Capture data at second edge clock
#define SPI_CPHA_LOW static_cast<u8>(0)  // Capture data at first edge clock

// @SPI_SSM, slave management
// sw slave management free slave select pin, 1 master 1 slave
#define SPI_SSM_EN static_cast<u8>(1)
// hw slave management use multiple slave and multiple slave select
#define SPI_SSM_DI static_cast<u8>(0)

// SPI application states
typedef enum {
  SPI_READY = 0x00U,
  SPI_BUSY_IN_RX = 0x01U,
  SPI_BUSY_IN_TX = 0x02U
} SPI_Status_Def;

// SPI Application events
#define SPI_EVENT_TX_CMPLT static_cast<u8>(1)
#define SPI_EVENT_RX_CMPLT static_cast<u8>(2)
#define SPI_EVENT_OVR_ERR static_cast<u8>(3)

/*!<possible values from @SPI_DeviceMode>*/
/*!<possible values from @SPI_BusConfig>*/
/*!<possible values from @SPI_SclkSpeed>*/
/*!<possible values from @SPI_DFF>*/
/*!<possible values from @SPI_CPOL>*/
/*!<possible values from @SPI_CPHA>*/
/*!<possible values from @SPI_SSM>*/
typedef struct {
  u8 spi_device_mode;
  u8 spi_bus_config;
  u8 spi_sclk_speed;
  u8 spi_dff;
  u8 spi_cpol;
  u8 spi_cpha;
  u8 spi_ssm;
} spi_config_t;

/* !< To store the app. Tx buffer address > */
/* !< To store the app. Rx buffer address > */
/* !< To store Tx len > */
/* !< To store Tx len of initial transmission> */
/* !< To store Rx len > */
/* !< To store Tx state > */
/* !< To store Rx state > */
/* Function pointer for receiving data in interrupt */
/* Function pointer for transmission data in interrupt */
/* Function pointer for delay */
typedef struct {
  volatile u8 *tx_buf;
  volatile u8 *rx_buf;
  volatile u32 tx_len;
  volatile u32 init_tx_len;
  volatile u32 rx_len;
  volatile u8 tx_state;
  volatile u8 rx_state;
  std::function<void(void)> receive_fnc;
  std::function<void(void)> transmit_fnc;
  std::function<void(u32)> delay_fnc;
} spi_handle_t;

void SPI_ApplicationEventCallback(spi_handle_t *pSPIHandle, u8 AppEv);

class spi_handler {
protected:
  volatile SPI_RegDef_t *spix_; /*!< holds the base address of SPIx(x:0,1,2)>*/
  spi_config_t config_ = {};
  spi_handle_t handle_ = {};
  gpio_handler spi_sck_;
  gpio_handler spi_mosi_;
  gpio_handler spi_miso_;

public:
  gpio_handler spi_nss_;

public:
  void spi_init(SPI_RegDef_t *spix_addr, std::function<void(int)> delay_fnc,
                u8 device_mode, u8 bus_config, u8 sclk_speed, u8 dff, u8 cpol,
                u8 cpha, u8 ssm);
  void spi_deinit();
  void spi_init_nss_sw(GPIO_RegDef_t *GPIOx_addr, const uint8_t pin_number);

  // Data Send and Receive
  void spi_transmit_data(const u8 *pTxBuffer, u32 Len);
  void spi_receive_data(u8 *pRxBuffer, u32 Len);
  void spi_transmit_receive_data(const u8 *p_tx_buffer, u8 *p_rx_buffer,
                                 u32 len);

  // Data Send and Receive in interrupt mode
  void spi_transmit_data_it(u8 *p_tx_buf, const u32 len);
  void spi_receive_data_it(u8 *p_rx_buf, const u32 len);
  void spi_transmit_receive_data_it(u8 *p_tx_buf, u8 *p_rx_buf, const u32 len);
  void spi_irq_handling();

  u8 spi_get_tx_state() const { return handle_.tx_state; }
  u8 spi_get_rx_state() const { return handle_.rx_state; }

  template <u32 flag_name> constexpr inline u8 spi_get_SR_reg();

private:
  void spi_clear_ovr_flag();
  void spi_peripheral_clock_init();
  void spi_gpios_init();
  void spi_reg_config();

  void spi_rx_8bit_it();
  void spi_tx_8bit_it();
  void spi_ovr_err_interrupt_handle();
  void spi_close_tx_rx_isr();
  template <u8 en_or_di> constexpr inline void spi_ssi_config();
  template <u8 en_or_di> constexpr inline void spi_ssoe_config();
  template <u8 en_or_di> constexpr inline void spi_peripheral_control();
  inline u8 spi_check_flag(const u32 __REG__, u32 __FLAG__);
};

/*!
 * @brief peripheral clock control
 *
 * @param: None
 *
 * @return None
 *
 */
template <u8 en_or_di>
constexpr inline void spi_handler::spi_peripheral_control() {
  if (ENABLE == en_or_di) {
    spix_->CR1 |= SPI_CR1_SPE;
  } else {
    spix_->CR1 &= ~SPI_CR1_SPE;
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
template <u32 flag_name> constexpr inline u8 spi_handler::spi_get_SR_reg() {
  return (spix_->SR & flag_name) ? SET : RESET;
}

/*!
 * @brief Configure software slave output enable
 *
 * @param[in]: Enable/Disable
 *
 * @return None
 *
 */
template <u8 en_or_di> constexpr inline void spi_handler::spi_ssoe_config() {
  if (en_or_di == ENABLE) {
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
template <u8 en_or_di> constexpr inline void spi_handler::spi_ssi_config() {
  if (en_or_di == ENABLE) {
    spix_->CR1 |= SPI_CR1_SSI;
  } else {
    spix_->CR1 &= ~SPI_CR1_SSI;
  }
}

/**
 * @brief  Check whether the specified SPI flag is set or not.
 * @param  __REG__  copy of SPI registers.
 * @param  __FLAG__ specifies the flag to check.
 *         This parameter can be one of the following values:
 *            @arg SPI_SR_RXNE: Receive buffer not empty flag
 *            @arg SPI_SR_TXE: Transmit buffer empty flag
 *            @arg SPI_SR_OVR: Overrun flag
 *            @arg SPI_SR_BSY: Busy flag
 * @retval SET or RESET.
 */
inline u8 spi_handler::spi_check_flag(const u32 __REG__, const u32 __FLAG__) {
  return ((__REG__ & __FLAG__) != RESET) ? SET : RESET;
}
#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */