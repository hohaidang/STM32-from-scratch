/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: Jul 16, 2020
 *      Author: prnsoft
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "stm32f4xx.h"
#include <memory>

// @SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER					1   // choose SPI as Master
#define SPI_DEVICE_MODE_SLAVE					0

// @SPI_BusConfig
#define SPI_BUS_CONFIG_FD						1	// full duplex
#define SPI_BUS_CONFIG_HD						2	// half duplex, transmit only
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			3	// simplex Rx only

// @SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

// @SPI_DFF, data format
#define SPI_DFF_8BITS                           0
#define SPI_DFF_16BITS                          1

// @SPI_CPOL
#define SPI_CPOL_HIGH                           1   // start clock with high, idle clock is high
#define SPI_CPOL_LOW                            0   // start clock with low, idle clock is low

// @SPI_CPHA
#define SPI_CPHA_HIGH                           1   // Capture data at second edge clock
#define SPI_CPHA_LOW                            0   // Capture data at first edge clock

// @SPI_SSM, slave management
#define SPI_SSM_EN                              1   // sw slave management free slave select pin, 1 master 1 slave
#define SPI_SSM_DI                              0   // hw slave management use multiple slave and multiple slave select

// SPI related status flags definitions
#define SPI_TXE_FLAG    (1u << SPI_SR_TXE)
#define SPI_RXNE_FLAG   (1u << SPI_SR_RXNE)
#define SPI_BSY_FLAG    (1u << SPI_SR_BSY)

// SPI application states
#define SPI_READY           0
#define SPI_BUSY_IN_RX      1
#define SPI_BUSY_IN_TX      2

// SPI Application events
#define SPI_EVENT_TX_CMPLT  1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_ERR   3

typedef struct {
    uint8_t SPI_DeviceMode; /*!<possible values from @SPI_DeviceMode>*/
    uint8_t SPI_BusConfig;  /*!<possible values from @SPI_BusConfig>*/
    uint8_t SPI_SclkSpeed;  /*!<possible values from @SPI_SclkSpeed>*/
    uint8_t SPI_DFF;        /*!<possible values from @SPI_DFF>*/
    uint8_t SPI_CPOL;       /*!<possible values from @SPI_CPOL>*/
    uint8_t SPI_CPHA;       /*!<possible values from @SPI_CPHA>*/
    uint8_t SPI_SSM;        /*!<possible values from @SPI_SSM>*/
}SPI_Config_t;

typedef struct {
    SPI_RegDef_t    *pSPIx;      /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
    SPI_Config_t    SPIConfig;
    uint8_t         *pTxBuffer; /* !< To store the app. Tx buffer address > */
    uint8_t         *pRxBuffer; /* !< To store the app. Rx buffer address > */
    uint32_t        TxLen;      /* !< To store Tx len > */
    uint32_t        RxLen;      /* !< To store Tx len > */
    uint8_t         TxState;    /* !< To store Tx state > */
    uint8_t         RxState;    /* !< To store Rx state > */
}SPI_Handle_t;

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

class GPIO_Handler;



/*********************************************************************
 * @class      		  - SPI_Handler
 *
 * @brief             - Constructor, initialize SPI, clock, IRQ
 **********************************************************************/
class SPI_Handler{
protected:
	SPI_Handle_t SPIx_ = {};
	std::unique_ptr<GPIO_Handler> SPI_Sck;
	std::unique_ptr<GPIO_Handler> SPI_MOSI;
	std::unique_ptr<GPIO_Handler> SPI_MISO;
	std::unique_ptr<GPIO_Handler> SPI_NSS;

public:
	SPI_Handler(SPI_RegDef_t *SPIx_ADDR,
	            uint8_t DeviceMode,
	            uint8_t BusConfig,
	            uint8_t SclkSpeed,
	            uint8_t DFF,
	            uint8_t CPOL,
	            uint8_t CPHA,
	            uint8_t SSM);
	~SPI_Handler();

	// peripheral clock setup
	void SPI_PeriClockControl(); // EnorDi: enable or disable
	void SPI_Init();
	void SPI_DeInit();

	uint8_t SPI_GetFlagStatus(uint8_t FlagName);
	void SPI_PeripheralControl(uint8_t EnOrDi);

	// Data Send and Receive
	void SPI_SendData(const uint8_t *pTxBuffer, uint32_t Len);
	void SPI_ReceiveData(uint8_t *pRxBuffer, uint32_t Len);

    // Data Send and Receive in interrupt mode
    uint8_t SPI_SendDataIT(const uint8_t *pTxBuffer, uint32_t Len);
    uint8_t SPI_ReceiveDataIT(const uint8_t *pRxBuffer, uint32_t Len);

    // IRQ Configuration and ISR Handling
    void SPI_IRQInterruptConfig(const uint8_t IRQNumber, const uint8_t EnorDi);
    void SPI_IRQPriorityConfig(const uint8_t IRQNumber, const uint8_t IRQPriority);
    void SPI_IRQHandling();
    void SPI_ClearOVRFlag();
    void SPI_CloseTransmission();
    void SPI_CloseReception();

private:

	void SPI_GPIOs_Init();
	void SPI_GPIOs_DeInit();
	void SPI_SSIConfig(uint8_t EnOrDi);
	void SPI_SSOEConfig(uint8_t EnOrDi);
	void spi_txe_interrupt_handle();
	void spi_rxne_interrupt_handle();
	void spi_ovr_err_interrupt_handle();

};



#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
