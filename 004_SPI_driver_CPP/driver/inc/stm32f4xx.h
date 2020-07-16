/*
 * stm3f407xx.h
 *
 *  Created on: Jan 29, 2019
 *      Author: admin
 */

#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

// Check ARM Cortex - M4 device generic User Guide
// ARM Cortex Mx Processor NVIC ISERx Register Address
#define NVIC_ISER0			((__vo uint32_t *) 0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *) 0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *) 0xE000E108)
#define NVIC_ISER3			((__vo uint32_t *) 0xE000E10C)

// ARM Cortex Mx Processor NVIC ICERx Register Address
#define NVIC_ICER0			((__vo uint32_t *) 0xE000E180)
#define NVIC_ICER1			((__vo uint32_t *) 0xE000E184)
#define NVIC_ICER2			((__vo uint32_t *) 0xE000E188)
#define NVIC_ICER3			((__vo uint32_t *) 0xE000E18C)

#define NVIC_PR_BASE_ADDR   ((__vo uint32_t *) 0xE000E400)

// Cortex M4 just have 16 priorities NVIC (4 most significant bits)
// https://community.arm.com/developer/ip-products/system/b/embedded-blog/posts/cutting-through-the-confusion-with-arm-cortex-m-interrupt-priorities
#define NO_PR_BITS_IMPLEMENTED			4

typedef struct {
	__vo uint32_t PRI[60];
}NVIC_PIR_RegDef_t;

typedef struct {
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< TODO,     										Address offset: 0x04      */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

typedef struct {
	  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
	  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
	  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
	  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
	  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
	  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
	  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
	  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
	  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
	  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
	  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
	  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
	  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
	  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
	  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
	  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
	  __vo uint32_t DCKCFGR2;      /*!< TODO, */
} RCC_RegDef_t;

typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];        /*!< Reserved, 0x18 - 0x1C  */
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2]; 		  /*!< Reserved, 0x24 - 0x28  */
	__vo uint32_t CFGR;
} SYSCFG_RegDef_t;

// peripheral register definition for EXTI
typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

// peripheral register definition for SPI
typedef struct {
	__vo uint32_t CR1;				/*!< Offset, 0x00  */
	__vo uint32_t CR2;				/*!< Offset, 0x04  */
	__vo uint32_t SR;				/*!< Offset, 0x08  */
	__vo uint32_t DR;				/*!< Offset, 0x0C  */
	__vo uint32_t CRCPR;			/*!< Offset, 0x10  */
	__vo uint32_t RXCRCR;			/*!< Offset, 0x14  */
	__vo uint32_t TXCRCR;			/*!< Offset, 0x18  */
	__vo uint32_t I2SCFGR;			/*!< Offset, 0x1C  */
	__vo uint32_t I2SPR;			/*!< Offset, 0x20  */
}SPI_RegDef_t;

#define AHB1PERIPH_BASEADDR 		0x40020000U
#define APB2PERIPH_BASEADDR			0x40010000U
#define APB1PERIPH_BASEADDR			0x40000000U


#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)


// @GPIOx_ADDR
#define GPIOA						((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define RCC							((RCC_RegDef_t *)  RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t *) EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

#define SPI1						((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t *) SPI4_BASEADDR)

// RCC CLOCK ENABLE
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 0u))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 1u))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 2u))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 3u))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 4u))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 5u))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 6u))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 7u))

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14u))

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12u))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14u))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15u))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13u))

// RCC RESET
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

#define SPI1_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0);
#define SPI2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0);
#define SPI3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0);
#define SPI4_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0);

// EXIT position in interrupt vector table
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

// @GPIO_IRQ_Priority
#define IRQ_Prio_NO_15			15

#define ENABLE			1
#define DISABLE			0
#define SET				1
#define RESET			0

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA        0u
#define SPI_CR1_CPOL        1u
#define SPI_CR1_MSTR        2u
#define SPI_CR1_BR          3u
#define SPI_CR1_SPE         6u
#define SPI_CR1_LSBFIRST    7u
#define SPI_CR1_SSI         8u
#define SPI_CR1_SSM         9u
#define SPI_CR1_RXONLY      10u
#define SPI_CR1_DFF         11u
#define SPI_CR1_CRCNEXT     12u
#define SPI_CR1_CRCEN       13u
#define SPI_CR1_BIDIOE      14u
#define SPI_CR1_BIDIMODE    15u

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN                 0
#define SPI_CR2_TXDMAEN                 1
#define SPI_CR2_SSOE                    2
#define SPI_CR2_FRF                     4
#define SPI_CR2_ERRIE                   5
#define SPI_CR2_RXNEIE                  6
#define SPI_CR2_TXEIE                   7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE                     0
#define SPI_SR_TXE                      1
#define SPI_SR_CHSIDE                   2
#define SPI_SR_UDR                      3
#define SPI_SR_CRCERR                   4
#define SPI_SR_MODF                     5
#define SPI_SR_OVR                      6
#define SPI_SR_BSY                      7
#define SPI_SR_FRE                      8

inline uint8_t gpio_baseAddr_to_code(GPIO_RegDef_t *Port) {
	return ( (Port == GPIOA) ? 0x00 : \
			 (Port == GPIOB) ? 0x01 : \
			 (Port == GPIOC) ? 0x02 : \
			 (Port == GPIOD) ? 0x03 : \
			 (Port == GPIOE) ? 0x04 : \
			 (Port == GPIOF) ? 0x05 : \
			 (Port == GPIOG) ? 0x06 : \
			 (Port == GPIOH) ? 0x07 : 0x00 );
}

#endif /* INC_STM3F407XX_H_ */
