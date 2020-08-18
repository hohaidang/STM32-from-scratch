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
#include <vector>
#include <queue>
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
    queue<u8> TxBuffer; /* !< To store the app. Tx buffer address > */
    queue<u8> RxBuffer; /* !< To store the app. Rx buffer address > */
    u32 TxLen; /* !< To store Tx len > */
    u32 RxLen; /* !< To store Tx len > */
    u8 TxState; /* !< To store Tx state > */
    u8 RxState; /* !< To store Rx state > */
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
    void SPI_PeriClockControl(); // EnorDi: enable or disable
    void SPI_Init();
    void SPI_DeInit();

    inline u8 SPI_GetFlagStatus(const u32 FlagName);
    void SPI_PeripheralControl(u8 EnOrDi);

    // Data Send and Receive
    void SPI_SendData(const u8 *pTxBuffer, u32 Len);
    void SPI_ReceiveData(u8 *pRxBuffer, u32 Len);
    void SPI_SendAndReceiveData(const u8 *pTxBuffer, u8 *pRxBuffer, u32 len);

    // Data Send and Receive in interrupt mode
    u8 SPI_SendDataIT(vector<u8> &TxBuffer, const u32 Len);
    u8 SPI_ReceiveDataIT(vector<u8> &RxBuffer, const u32 Len);
    u8 SPI_SendReceiveDataIT(vector<u8> &pTxBuffer, vector<u8> &pRxBuffer, const u32 Len);

    // IRQ Configuration and ISR Handling
    void SPI_IRQInterruptConfig(const u8 IRQNumber, const u8 EnorDi);
    void SPI_IRQPriorityConfig(const u8 IRQNumber, const u8 IRQPriority);
    void SPI_IRQHandling();
    void spi_clear_OVR_flag();
    void SPI_CloseTransmission();
    void SPI_CloseReception();
    u8 get_TxState();
    u8 get_RxState();

private:
    void SPI_GPIOs_Init();
    void SPI_SSIConfig(const u8 EnOrDi);
    void SPI_SSOEConfig(const u8 EnOrDi);
    void spi_txe_interrupt_handle();
    void spi_rxne_interrupt_handle();
    void spi_ovr_err_interrupt_handle();
    inline u8 spi_check_flag(const u32 __REG__, u32 __FLAG__);
};

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
