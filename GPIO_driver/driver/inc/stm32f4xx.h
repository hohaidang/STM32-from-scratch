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


#define AHB1PERIPH_BASEADDR 		0x40020000U


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

#define GPIOA						((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define RCC							((RCC_RegDef_t *) RCC_BASEADDR)

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |=  (1 << 7))

// set first then reset
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


#define GPIOA_PCLK_DIS()			(RCC->AHB1RSTR |= (0 << 0))

#define ENABLE			1
#define DISABLE			0
#define SET				1
#define RESET			0


#endif /* INC_STM3F407XX_H_ */
