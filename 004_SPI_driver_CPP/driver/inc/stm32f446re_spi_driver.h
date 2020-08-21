/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: Jul 16, 2020
 *      Author: prnsoft
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "stm32f4xx.h"
#include "core_cm4.h"
#include <memory>
<<<<<<< HEAD
#include <functional>
=======
#include <vector>
#include <queue>
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
using namespace std;

// @SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER                  static_cast<u8>(1)   // choose SPI as Master
#define SPI_DEVICE_MODE_SLAVE                   static_cast<u8>(0)

// @SPI_BusConfig
#define SPI_BUS_CONFIG_FD                       static_cast<u8>(1)	// full duplex
#define SPI_BUS_CONFIG_HD                       static_cast<u8>(2)	// half duplex, transmit only
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY           static_cast<u8>(3)	// simplex Rx only

// @SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2                     static_cast<u8>(0)
#define SPI_SCLK_SPEED_DIV4                     static_cast<u8>(1)
#define SPI_SCLK_SPEED_DIV8                     static_cast<u8>(2)
#define SPI_SCLK_SPEED_DIV16                    static_cast<u8>(3)
#define SPI_SCLK_SPEED_DIV32                    static_cast<u8>(4)
#define SPI_SCLK_SPEED_DIV64                    static_cast<u8>(5)
#define SPI_SCLK_SPEED_DIV128                   static_cast<u8>(6)
#define SPI_SCLK_SPEED_DIV256                   static_cast<u8>(7)

// @SPI_DFF, data format
#define SPI_DFF_8BITS                           static_cast<u8>(0)
#define SPI_DFF_16BITS                          static_cast<u8>(1)

// @SPI_CPOL
#define SPI_CPOL_HIGH                           static_cast<u8>(1)   // start clock with high, idle clock is high
#define SPI_CPOL_LOW                            static_cast<u8>(0)   // start clock with low, idle clock is low

// @SPI_CPHA
#define SPI_CPHA_HIGH                           static_cast<u8>(1)   // Capture data at second edge clock
#define SPI_CPHA_LOW                            static_cast<u8>(0)   // Capture data at first edge clock

// @SPI_SSM, slave management
#define SPI_SSM_EN                              static_cast<u8>(1)   // sw slave management free slave select pin, 1 master 1 slave
#define SPI_SSM_DI                              static_cast<u8>(0)   // hw slave management use multiple slave and multiple slave select

// SPI application states
typedef enum {
    SPI_READY       = 0x00U,
    SPI_BUSY_IN_RX  = 0x01U,
    SPI_BUSY_IN_TX  = 0x02U
} SPI_Status_Def;

// SPI Application events
#define SPI_EVENT_TX_CMPLT  static_cast<u8>(1)
#define SPI_EVENT_RX_CMPLT  static_cast<u8>(2)
#define SPI_EVENT_OVR_ERR   static_cast<u8>(3)

typedef struct {
    u8 SPI_DeviceMode; /*!<possible values from @SPI_DeviceMode>*/
    u8 SPI_BusConfig; /*!<possible values from @SPI_BusConfig>*/
    u8 SPI_SclkSpeed; /*!<possible values from @SPI_SclkSpeed>*/
    u8 SPI_DFF; /*!<possible values from @SPI_DFF>*/
    u8 SPI_CPOL; /*!<possible values from @SPI_CPOL>*/
    u8 SPI_CPHA; /*!<possible values from @SPI_CPHA>*/
    u8 SPI_SSM; /*!<possible values from @SPI_SSM>*/
} SPI_Config_t;

typedef struct {
<<<<<<< HEAD
    volatile u8 *tx_buf; /* !< To store the app. Tx buffer address > */
    volatile u8 *rx_buf; /* !< To store the app. Rx buffer address > */
    volatile u32 tx_len; /* !< To store Tx len > */
    volatile u32 rx_len; /* !< To store Rx len > */
    volatile u8 tx_state; /* !< To store Tx state > */
    volatile u8 rx_state; /* !< To store Rx state > */
    std::function<void(void)> receive_fnc; /* Function pointer for receiving data in interrupt */
    std::function<void(void)> transmit_fnc; /* Function pointer for transmission data in interrupt */
=======
    queue<u8> TxBuffer; /* !< To store the app. Tx buffer address > */
    queue<u8> RxBuffer; /* !< To store the app. Rx buffer address > */
    u32 TxLen; /* !< To store Tx len > */
    u32 RxLen; /* !< To store Tx len > */
    u8 TxState; /* !< To store Tx state > */
    u8 RxState; /* !< To store Rx state > */
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt
} SPI_Handle_t;

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, u8 AppEv);

class GPIO_Handler;

class SPI_Handler {
protected:
    SPI_RegDef_t *SPIx_; /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
    SPI_Config_t config_ = { };
    SPI_Handle_t handle_ = { };
    std::unique_ptr<GPIO_Handler> SPI_Sck;
    std::unique_ptr<GPIO_Handler> SPI_MOSI;
    std::unique_ptr<GPIO_Handler> SPI_MISO;
    std::unique_ptr<GPIO_Handler> SPI_NSS;

public:
    SPI_Handler(SPI_RegDef_t *SPIx_ADDR, u8 device_mode, u8 bus_config,
                u8 sclk_speed, u8 DFF, u8 CPOL, u8 CPHA, u8 SSM);
    ~SPI_Handler();

    // peripheral clock setup

    void spi_init();
    void spi_deinit();

    inline u8 spi_get_SR_reg(const u32 FlagName);

    // Data Send and Receive
    void spi_transmit_data(const u8 *pTxBuffer, u32 Len);
    void spi_receive_data(u8 *pRxBuffer, u32 Len);
    void spi_transmit_receive_data(const u8 *pTxBuffer, u8 *pRxBuffer, u32 len);

    // Data Send and Receive in interrupt mode
<<<<<<< HEAD
    void spi_transmit_data_it(u8 *p_tx_buf, const u32 len);
    void spi_receive_data_it(u8 *p_rx_buf, const u32 len);
    void spi_transmit_receive_data_it(u8 *p_tx_buf, u8 *p_rx_buf, const u32 len);
=======
    // TODO: Remove u8
    u8 spi_transmit_data_it(vector<u8> &TxBuffer, const u32 Len);
    u8 spi_receive_data_it(vector<u8> &RxBuffer, const u32 Len);
    u8 spi_transmit_receive_data_it(vector<u8> &pTxBuffer, vector<u8> &pRxBuffer, const u32 Len);
>>>>>>> parent of d7aa2b3... Clean code SPI interrupt

    // IRQ Configuration and ISR Handling
    void spi_ir_config(const u8 IRQNumber, const u8 EnorDi);
    void spi_ir_prio_config(const u8 IRQNumber, const u8 IRQPriority);
    void spi_irq_handling();
    void spi_clear_OVR_flag();

    u8 spi_get_tx_state();
    u8 spi_get_rx_state();

private:
    void spi_peripheral_control(u8 EnOrDi); // EnorDi: enable or disable
    void spi_peripheral_clock_init();
    void spi_gpios_init();
    void spi_ssi_config(const u8 EnOrDi);
    void spi_ssoe_config(const u8 EnOrDi);
    void spi_txe_interrupt_handle();
    void spi_rxne_interrupt_handle();
    void spi_ovr_err_interrupt_handle();
    void spi_close_tx_rx_isr();
    inline u8 spi_check_flag(const u32 __REG__, u32 __FLAG__);
};

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
