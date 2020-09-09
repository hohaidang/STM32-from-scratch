/*
 * stm3f407xx.h
 *
 *  Created on: Jan 29, 2019
 *      Author: admin
 */

#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include <stddef.h>
#include <stdint.h>

#ifdef DEBUG_EN
#include <iostream>
#include <iomanip>
#endif

#define __vo volatile
#define __weak __attribute__((weak))

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;


/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  CEC_IRQn                    = 93,     /*!< CEC global Interrupt                                              */
  SPDIF_RX_IRQn               = 94,     /*!< SPDIF-RX global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */
} IRQn_Type;

typedef struct {
    __vo u32 MODER;                        /*!< GPIO port mode register,                       Address offset: 0x00      */
    __vo u32 OTYPER;                       /*!< ,                                          Address offset: 0x04      */
    __vo u32 OSPEEDR;
    __vo u32 PUPDR;
    __vo u32 IDR;
    __vo u32 ODR;
    __vo u32 BSRR;
    __vo u32 LCKR;
    __vo u32 AFR[2];                    /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register          Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

typedef struct {
      __vo u32 CR;            /*!< ,                                           Address offset: 0x00 */
      __vo u32 PLLCFGR;       /*!< ,                                           Address offset: 0x04 */
      __vo u32 CFGR;          /*!< ,                                           Address offset: 0x08 */
      __vo u32 CIR;           /*!< ,                                           Address offset: 0x0C */
      __vo u32 AHB1RSTR;      /*!< ,                                           Address offset: 0x10 */
      __vo u32 AHB2RSTR;      /*!< ,                                           Address offset: 0x14 */
      __vo u32 AHB3RSTR;      /*!< ,                                           Address offset: 0x18 */
      u32      RESERVED0;     /*!< Reserved, 0x1C                                                       */
      __vo u32 APB1RSTR;      /*!< ,                                           Address offset: 0x20 */
      __vo u32 APB2RSTR;      /*!< ,                                           Address offset: 0x24 */
      u32      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
      __vo u32 AHB1ENR;       /*!< ,                                           Address offset: 0x30 */
      __vo u32 AHB2ENR;       /*!< ,                                           Address offset: 0x34 */
      __vo u32 AHB3ENR;       /*!< ,                                           Address offset: 0x38 */
      u32      RESERVED2;     /*!< Reserved, 0x3C                                                       */
      __vo u32 APB1ENR;       /*!< ,                                           Address offset: 0x40 */
      __vo u32 APB2ENR;       /*!< ,                                           Address offset: 0x44 */
      u32      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
      __vo u32 AHB1LPENR;     /*!< ,                                           Address offset: 0x50 */
      __vo u32 AHB2LPENR;     /*!< ,                                           Address offset: 0x54 */
      __vo u32 AHB3LPENR;     /*!< ,                                           Address offset: 0x58 */
      u32      RESERVED4;     /*!< Reserved, 0x5C                                                       */
      __vo u32 APB1LPENR;     /*!< ,                                           Address offset: 0x60 */
      __vo u32 APB2LPENR;     /*!< R,                                          Address offset: 0x64 */
      u32      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
      __vo u32 BDCR;          /*!< ,                                           Address offset: 0x70 */
      __vo u32 CSR;           /*!< ,                                           Address offset: 0x74 */
      u32      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
      __vo u32 SSCGR;         /*!< ,                                           Address offset: 0x80 */
      __vo u32 PLLI2SCFGR;    /*!< ,                                           Address offset: 0x84 */
      __vo u32 PLLSAICFGR;    /*!< ,                                           Address offset: 0x88 */
      __vo u32 DCKCFGR;       /*!< ,                                           Address offset: 0x8C */
      __vo u32 CKGATENR;      /*!< ,                                           Address offset: 0x90 */
      __vo u32 DCKCFGR2;      /*!< , */
} RCC_RegDef_t;

typedef struct {
    __vo u32 MEMRMP;
    __vo u32 PMC;
    __vo u32 EXTICR[4];
    u32 RESERVED1[2];        /*!< Reserved, 0x18 - 0x1C  */
    __vo u32 CMPCR;
    u32 RESERVED2[2];        /*!< Reserved, 0x24 - 0x28  */
    __vo u32 CFGR;
} SYSCFG_RegDef_t;

// peripheral register definition for EXTI
typedef struct {
    __vo u32 IMR;
    __vo u32 EMR;
    __vo u32 RTSR;
    __vo u32 FTSR;
    __vo u32 SWIER;
    __vo u32 PR;
} EXTI_RegDef_t;

// peripheral register definition for SPI
typedef struct {
    __vo u32 CR1;              /*!< Offset, 0x00  */
    __vo u32 CR2;              /*!< Offset, 0x04  */
    __vo u32 SR;               /*!< Offset, 0x08  */
    __vo u32 DR;               /*!< Offset, 0x0C  */
    __vo u32 CRCPR;            /*!< Offset, 0x10  */
    __vo u32 RXCRCR;           /*!< Offset, 0x14  */
    __vo u32 TXCRCR;           /*!< Offset, 0x18  */
    __vo u32 I2SCFGR;          /*!< Offset, 0x1C  */
    __vo u32 I2SPR;            /*!< Offset, 0x20  */
}SPI_RegDef_t;




#define AHB1PERIPH_BASEADDR         0x40020000UL
#define APB2PERIPH_BASEADDR         0x40010000UL
#define APB1PERIPH_BASEADDR         0x40000000UL


#define GPIOA_BASEADDR              (AHB1PERIPH_BASEADDR)
#define GPIOB_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR              (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR               (APB2PERIPH_BASEADDR + 0x3400)

#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)



// RCC CLOCK ENABLE
#define GPIOA_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 0u))
#define GPIOB_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 1u))
#define GPIOC_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 2u))
#define GPIOD_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 3u))
#define GPIOE_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 4u))
#define GPIOF_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 5u))
#define GPIOG_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 6u))
#define GPIOH_PCLK_EN()             (RCC->AHB1ENR |=  (1 << 7u))

#define SYSCFG_PCLK_EN()            (RCC->APB2ENR |= (1 << 14u))

#define SPI1_PCLK_EN()              (RCC->APB2ENR |= (1 << 12u))
#define SPI2_PCLK_EN()              (RCC->APB1ENR |= (1 << 14u))
#define SPI3_PCLK_EN()              (RCC->APB1ENR |= (1 << 15u))
#define SPI4_PCLK_EN()              (RCC->APB2ENR |= (1 << 13u))

// RCC RESET
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

#define SPI1_REG_RESET()                do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0);
#define SPI2_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0);
#define SPI3_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0);
#define SPI4_REG_RESET()                do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0);

// @GPIO_IRQ_Priority
#define IRQ_Prio_NO_0           (static_cast<u8>(0U))
#define IRQ_Prio_NO_3           (static_cast<u8>(3U))
#define IRQ_Prio_NO_14          (static_cast<u8>(14U))
#define IRQ_Prio_NO_15          (static_cast<u8>(15U))

#define ENABLE          1u
#define DISABLE         0
#define SET             static_cast<u8>(1u)
#define RESET           static_cast<u8>(0u)

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA_Pos        (0U)
#define SPI_CR1_CPHA_Msk        (static_cast<u32>(1UL << SPI_CR1_CPHA_Pos))
#define SPI_CR1_CPHA            (SPI_CR1_CPHA_Msk)

#define SPI_CR1_CPOL_Pos        (1U)
#define SPI_CR1_CPOL_Msk        (static_cast<u32>(1UL << SPI_CR1_CPOL_Pos))
#define SPI_CR1_CPOL            (SPI_CR1_CPOL_Msk)

#define SPI_CR1_MSTR_Pos        (2U)
#define SPI_CR1_MSTR_Msk        (static_cast<u32>(1UL << SPI_CR1_MSTR_Pos))
#define SPI_CR1_MSTR            (SPI_CR1_MSTR_Msk)

#define SPI_CR1_BR_Pos          (3U)
#define SPI_CR1_BR_Msk          (static_cast<u32>(1UL << SPI_CR_BR_Pos))
#define SPI_CR1_BR              (SPI_CR1_BR_Msk)

#define SPI_CR1_SPE_Pos         (6U)
#define SPI_CR1_SPE_Msk         (static_cast<u32>(1UL << SPI_CR1_SPE_Pos))
#define SPI_CR1_SPE             (SPI_CR1_SPE_Msk)

#define SPI_CR1_LSBFIRST_Pos    (7U)
#define SPI_CR1_LSBFIRST_Msk    (static_cast<u32>(1UL << SPI_CR1_LSBFIRST_Pos))
#define SPI_CR1_LSBIRST         (SPI_CR1_LSBFIRST_Msk)

#define SPI_CR1_SSI_Pos         (8U)
#define SPI_CR1_SSI_Msk         (static_cast<u32>(1UL << SPI_CR1_SSI_Pos))
#define SPI_CR1_SSI             (SPI_CR1_SSI_Msk)

#define SPI_CR1_SSM_Pos         (9U)
#define SPI_CR1_SSM_Msk         (static_cast<u32>(1UL << SPI_CR1_SSM_Pos))
#define SPI_CR1_SSM             (SPI_CR1_SSM_Msk)

#define SPI_CR1_RXONLY_Pos      (10U)
#define SPI_CR1_RXONLY_Msk      (static_cast<u32>(1UL << SPI_CR1_RXONLY_Pos))
#define SPI_CR1_RXONLY          (SPI_CR1_RXONLY_Msk)

#define SPI_CR1_DFF_Pos         (11U)
#define SPI_CR1_DFF_Msk         (static_cast<u32>(1UL << SPI_CR1_DFF_Pos))
#define SPI_CR1_DFF             (SPI_CR1_DFF_Msk)

#define SPI_CR1_CRCNEXT_Pos     (12U)
#define SPI_CR1_CRCNEXT_Msk     (static_cast<u32>(1UL << SPI_CR1_CRCNEXT_Pos))
#define SPI_CR1_CRCNEXT         (SPI_CR1_CRCNEXT_Msk)

#define SPI_CR1_CRCEN_Pos       (13U)
#define SPI_CR1_CRCEN_Msk       (static_cast<u32>(1UL << SPI_CR1_CRCEN_Pos))
#define SPI_CR1_CRCEN           (SPI_CR1_CRCEN_Msk)

#define SPI_CR1_BIDIOE_Pos      (14U)
#define SPI_CR1_BIDIOE_Msk      (static_cast<u32>(1UL << SPI_CR1_BIDIOE_Pos))
#define SPI_CR1_BIDIOE          (SPI_CR1_BIDIOE_Msk)

#define SPI_CR1_BIDIMODE_Pos    (15U)
#define SPI_CR1_BIDIMODE_Msk    (static_cast<u32>(1UL << SPI_CR1_BIDIOE_Pos))
#define SPI_CR1_BIDIMODE        (SPI_CR1_BIDIMODE_Msk)

#define SPI_CR1_SPE_MSK     (1U << SPI_CR1_SPE)

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN_Pos                 (0U)
#define SPI_CR2_TXDMAEN_Pos                 (1U)

#define SPI_CR2_SSOE_Pos                    (2U)
#define SPI_CR2_SSOE_Msk                    (static_cast<u32>(1UL << SPI_CR2_SSOE_Pos))
#define SPI_CR2_SSOE                        (SPI_CR2_SSOE_Msk)

#define SPI_CR2_FRF_Pos                     (4U)

#define SPI_CR2_ERRIE_Pos                   (5U)
#define SPI_CR2_ERRIE_Msk                   (static_cast<u32>(1UL << SPI_CR2_ERRIE_Pos))
#define SPI_CR2_ERRIE                       (SPI_CR2_ERRIE_Msk)

#define SPI_CR2_RXNEIE_Pos                  (6U)
#define SPI_CR2_RXEIE_Msk                   (static_cast<u32>(1UL << SPI_CR2_RXNEIE_Pos))
#define SPI_CR2_RXNEIE                      (SPI_CR2_RXEIE_Msk)

#define SPI_CR2_TXEIE_Pos                   (7U)
#define SPI_CR2_TXEIE_Msk                   (static_cast<u32>(1UL << SPI_CR2_TXEIE_Pos))
#define SPI_CR2_TXEIE                       (SPI_CR2_TXEIE_Msk)

/*
 * Bit position definitions SPI_SR @FLAG_NAME_STATUS
 */
#define SPI_SR_RXNE_Pos                     (0U)
#define SPI_SR_RXNE_Msk                     (static_cast<u32>(1UL << SPI_SR_RXNE_Pos))
#define SPI_SR_RXNE                         (SPI_SR_RXNE_Msk)

#define SPI_SR_TXE_Pos                      (1U)
#define SPI_SR_TXE_Msk                      (static_cast<u32>(1UL << SPI_SR_TXE_Pos))
#define SPI_SR_TXE                          (SPI_SR_TXE_Msk)

#define SPI_SR_CHSIDE_Pos                   (2U)
#define SPI_SR_UDR_Pos                      (3U)
#define SPI_SR_CRCERR_Pos                   (4U)
#define SPI_SR_MODF_Pos                     (5U)

#define SPI_SR_OVR_Pos                      (6u)
#define SPI_SR_OVR_Msk                      (static_cast<u32>(1UL << SPI_SR_OVR_Pos))
#define SPI_SR_OVR                          (SPI_SR_OVR_Msk)

#define SPI_SR_BSY_Pos                      (7U)
#define SPI_SR_BSY_Msk                      (static_cast<u32>(0x1UL << SPI_SR_BSY_Pos))
#define SPI_SR_BSY                          SPI_SR_BSY_Msk

#define SPI_SR_FRE_Pos                      (8U)

//#include "core_cm4.h"

#ifndef GOOGLE_UNIT_TEST
// @GPIOx_ADDR
#define GPIOA                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOA_BASEADDR) )
#define GPIOB                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOB_BASEADDR) )
#define GPIOC                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOC_BASEADDR) )
#define GPIOD                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOD_BASEADDR) )
#define GPIOE                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOE_BASEADDR) )
#define GPIOF                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOF_BASEADDR) )
#define GPIOG                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOG_BASEADDR) )
#define GPIOH                       ( reinterpret_cast<GPIO_RegDef_t *>     (GPIOH_BASEADDR) )
#define RCC                         ( reinterpret_cast<RCC_RegDef_t *>      (RCC_BASEADDR) )
#define EXTI                        ( reinterpret_cast<EXTI_RegDef_t *>     (EXTI_BASEADDR) )
#define SYSCFG                      ( reinterpret_cast<SYSCFG_RegDef_t *>   (SYSCFG_BASEADDR) )

#define SPI1                        ( reinterpret_cast<SPI_RegDef_t *>      (SPI1_BASEADDR) )
#define SPI2                        ( reinterpret_cast<SPI_RegDef_t *>      (SPI2_BASEADDR) )
#define SPI3                        ( reinterpret_cast<SPI_RegDef_t *>      (SPI3_BASEADDR) )
#define SPI4                        ( reinterpret_cast<SPI_RegDef_t *>      (SPI4_BASEADDR) )


#else

#endif


#endif /* INC_STM3F407XX_H_ */
