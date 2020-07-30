/*
 * stm32f446re_gpio_driver.cpp
 *
 *  Created on: Jul 13, 2020
 *      Author: prnsoft
 */

#include "../inc/stm32f446re_gpio_driver.h"
static inline uint8_t get_irq_pinNum(uint8_t);


/*********************************************************************
 * @class      		  - GPIO_Handler
 *
 * @brief             - Constructor, initialize GPIO, clock, IRQ, Alt Function
 **********************************************************************/
GPIO_Handler::GPIO_Handler(
		GPIO_RegDef_t *GPIOx_ADDR,
		uint8_t GPIO_PinNumber,
		uint8_t GPIO_PinMode,
		uint8_t GPIO_PinSpeed,
		uint8_t GPIO_IRQ_Priority,
		uint8_t GPIO_PinOPType,
		uint8_t GPIO_PinPuPdControl,
		uint8_t GPIO_PinAltFunMode) {
	GPIOx_.pGPIOx = ((GPIO_RegDef_t *)(GPIOx_ADDR));
	GPIOx_.GPIO_PinConfig.GPIO_PinNumber = GPIO_PinNumber;
	GPIOx_.GPIO_PinConfig.GPIO_PinMode = GPIO_PinMode;
	GPIOx_.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PinSpeed;
	GPIOx_.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PinPuPdControl;
	GPIOx_.GPIO_PinConfig.GPIO_PinOPType = GPIO_PinOPType;
	GPIOx_.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PinAltFunMode;
	GPIO_PeriClockControl();
	GPIO_Init();
	// Configure interrupt
	if(GPIO_PinMode >= GPIO_MODE_IT_FT) {
		uint8_t IRQ_number = get_irq_pinNum(GPIO_PinNumber);
		GPIO_IRQInterruptConfig(IRQ_number, ENABLE);
		GPIO_IRQPriorityConfig(IRQ_number, GPIO_IRQ_Priority);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Handler
 *
 * @brief             - DeConstructor, Reset GPIO PORT
 *
 * @param None
 *
 * @return None
 *
 * @Node: Be careful when using many Peripheral device in the same PORT
 **********************************************************************/
GPIO_Handler::~GPIO_Handler() {
	GPIO_DeInit();
}

// peripheral clock setup
void GPIO_Handler::GPIO_PeriClockControl() {
	if (GPIOx_.pGPIOx == GPIOA) {
		GPIOA_PCLK_EN();
	} else if (GPIOx_.pGPIOx == GPIOB) {
		GPIOB_PCLK_EN();
	} else if (GPIOx_.pGPIOx == GPIOC) {
		GPIOC_PCLK_EN();
	} else if (GPIOx_.pGPIOx == GPIOD) {
		GPIOD_PCLK_EN();
	} else if (GPIOx_.pGPIOx == GPIOE) {
		GPIOE_PCLK_EN();
	} else if (GPIOx_.pGPIOx == GPIOF) {
		GPIOF_PCLK_EN();
	} else if (GPIOx_.pGPIOx == GPIOG) {
		GPIOG_PCLK_EN();
	} else if (GPIOx_.pGPIOx == GPIOH) {
		GPIOH_PCLK_EN();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Initial GPIO with clock, Speed, In/out mode, ...
 *
 * @param None
 *
 * @return None
 **********************************************************************/
void GPIO_Handler::GPIO_Init() {
	uint32_t temp = 0;
	// 1. configure the mode of gpio pin
	if(GPIOx_.GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// the non interrupt mode
		temp = GPIOx_.GPIO_PinConfig.GPIO_PinMode << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
		GPIOx_.pGPIOx->MODER &= ~(0x3 << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber)); // clearing
		GPIOx_.pGPIOx->MODER |= temp;
	}
	else {
		// interrupt mode
		if(GPIO_MODE_IT_FT == GPIOx_.GPIO_PinConfig.GPIO_PinMode) {
			// 1. configure the FTSR
			EXTI->FTSR |= (1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (GPIO_MODE_IT_RT == GPIOx_.GPIO_PinConfig.GPIO_PinMode) {
			// 1. configure the RTSR
			EXTI->RTSR |= (1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (GPIO_MODE_IT_RFT == GPIOx_.GPIO_PinConfig.GPIO_PinMode) {
			// 1. configure both FTSR and RTSR
			EXTI->FTSR |= (1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = GPIOx_.GPIO_PinConfig.GPIO_PinNumber >> 2;
		uint8_t temp2 = GPIOx_.GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = gpio_baseAddr_to_code(GPIOx_.pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portCode << (4 * temp2));

		// 3. Enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;
	// 2. configure the speed
	temp = GPIOx_.GPIO_PinConfig.GPIO_PinSpeed << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
	GPIOx_.pGPIOx->OSPEEDR &= ~(0x3 << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber)); // clearing
	GPIOx_.pGPIOx->OSPEEDR |= temp;

	temp = 0;
	// 3. configure the pupd settings
	temp = GPIOx_.GPIO_PinConfig.GPIO_PinPuPdControl << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
	GPIOx_.pGPIOx->PUPDR &= ~(0x3 << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber));// clearing
	GPIOx_.pGPIOx->PUPDR |= temp;

	temp = 0;
	// 4. configure the optype
	temp = GPIOx_.GPIO_PinConfig.GPIO_PinOPType << GPIOx_.GPIO_PinConfig.GPIO_PinNumber;
	GPIOx_.pGPIOx->OTYPER &= ~(0x1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber); // clearing
	GPIOx_.pGPIOx->OTYPER |= temp;

	temp = 0;
	// 5. configure the alt functionality
	if(GPIOx_.GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// configure alt function register
		uint8_t temp1, temp2;
		temp1 = GPIOx_.GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = GPIOx_.GPIO_PinConfig.GPIO_PinNumber % 8;
		GPIOx_.pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		GPIOx_.pGPIOx->AFR[temp1] |= (GPIOx_.GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief - Reset GPIO Port
 *
 * @return uint8, b'0000_000x
 */
void GPIO_Handler::GPIO_DeInit() {
	if(GPIOx_.pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}
	else if (GPIOx_.pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}
	else if (GPIOx_.pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}
	else if (GPIOx_.pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}
	else if (GPIOx_.pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}
	else if (GPIOx_.pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	}
	else if (GPIOx_.pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	}
	else if (GPIOx_.pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief - Read single bit in Pin
 *
 * @return uint8, b'0000_000x
 */
uint8_t GPIO_Handler::GPIO_ReadFromInputPin() const {
	uint8_t value;
	value = (uint8_t)((GPIOx_.pGPIOx->IDR >> GPIOx_.GPIO_PinConfig.GPIO_PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief - Read 16 bits PORT(A, B, C,...)
 *
 * @return uint16_t: PORT value
 */
uint16_t GPIO_Handler::GPIO_ReadFromInputPort() const {
	uint16_t value;
	value = GPIOx_.pGPIOx->IDR;
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief - Write single bit to Pin
 * @Param[in] Value: write value
 * @return None
 */
void GPIO_Handler::GPIO_WriteToOutputPin(const uint8_t Value) {
	if(Value == SET) {
		GPIOx_.pGPIOx->ODR |= (0x1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
	}
	else {
		GPIOx_.pGPIOx->ODR &= ~(0x1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief - Write value to PORT
 * @Param[in] Value: write value
 * @return None
 */
void GPIO_Handler::GPIO_WriteToOutputPort(const uint16_t Value) {
	GPIOx_.pGPIOx->ODR &= 0x0000;
	GPIOx_.pGPIOx->ODR = Value;
}


void GPIO_Handler::GPIO_ToggleOutputPin() {
	GPIOx_.pGPIOx->ODR ^= (0x1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - Enable Interrupt for NVIC
 *
 * @param[in] IRQNumber     - IRQNumber IRQ Position number
 * @param[in] EnorDi		- Enable/Disable Flag
 *
 * @return None
 */
void GPIO_Handler::GPIO_IRQInterruptConfig(const uint8_t IRQNumber, const uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			//	program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else {
		if (IRQNumber <= 31) {
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ICE2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}


/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief - Configure priority for GPIO, 0->15, lower number higher priority
 *
 * @param[in] IRQNumber: IRQ position for EXIT
 * @param[in] IRPriority: 0->15
 * @return None
 */
void GPIO_Handler::GPIO_IRQPriorityConfig(const uint8_t IRQNumber, const uint8_t IRQPriority) {
	// 1. first lets find out the ipr register
	uint8_t iprx = IRQNumber >> 2;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief - Clear the interrupt flag for the next interrupt
 *
 * @param None
 *
 * @return None
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    //clear the exti pr register corresponding to the pin number
    if(EXTI->PR & ( 1 << PinNumber))
    {
        //clear
        EXTI->PR |= ( 1 << PinNumber);
    }

}


