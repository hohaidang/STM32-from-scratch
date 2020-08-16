/*
 * stm32f446re_gpio_driver.cpp
 *
 *  Created on: Jul 13, 2020
 *      Author: prnsoft
 */

#include "../inc/stm32f446re_gpio_driver.h"
constexpr static inline u8 get_irq_pinNum(u8);


/*!
 * @brief Constructor, initialize GPIO, clock, IRQ, Alt Function
 *
 * @param None
 *
 * @return None
 *
 */
GPIO_Handler::GPIO_Handler(
		GPIO_RegDef_t *GPIOx_addr,
		u8 pin_number,
		u8 pin_mode,
		u8 pin_speed,
		u8 output_type,
		u8 pupd_control,
		u8 IRQ_priority,
		u8 alt_func_mode) : GPIOx_(GPIOx_addr), config_({pin_number, pin_mode, pin_speed, pupd_control, output_type, alt_func_mode}) {
	GPIO_PeriClockControl();
	GPIO_Init();
	// Configure interrupt
	if(pin_mode >= GPIO_MODE_IT_FT) {
		u8 IRQ_number = get_irq_pinNum(pin_number);
		GPIO_IRQInterruptConfig(IRQ_number, ENABLE);
		GPIO_IRQPriorityConfig(IRQ_number, IRQ_priority);
	}
}

/*!
 * @brief DeConstructor, Reset GPIO PORT
 *
 * @param None
 *
 * @return None
 *
 * @node: Be careful when using many Peripheral device in the same PORT
 */
GPIO_Handler::~GPIO_Handler() {
	GPIO_DeInit();
}

/*!
 * @brief peripheral clock setup
 *
 * @param None
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_PeriClockControl() {
	if (GPIOx_ == GPIOA) {
		GPIOA_PCLK_EN();
	} else if (GPIOx_ == GPIOB) {
		GPIOB_PCLK_EN();
	} else if (GPIOx_== GPIOC) {
		GPIOC_PCLK_EN();
	} else if (GPIOx_ == GPIOD) {
		GPIOD_PCLK_EN();
	} else if (GPIOx_ == GPIOE) {
		GPIOE_PCLK_EN();
	} else if (GPIOx_ == GPIOF) {
		GPIOF_PCLK_EN();
	} else if (GPIOx_ == GPIOG) {
		GPIOG_PCLK_EN();
	} else if (GPIOx_ == GPIOH) {
		GPIOH_PCLK_EN();
	}
}

/*!
 * @brief Initial GPIO with clock, Speed, In/out mode, ...
 *
 * @param None
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_Init() {
	uint32_t temp = 0;
	// 1. configure the mode of gpio pin
	if(config_.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// the non interrupt mode
		temp = config_.GPIO_PinMode << (2 * config_.GPIO_PinNumber);
		GPIOx_->MODER &= ~(0x3 << (2 * config_.GPIO_PinNumber)); // clearing
		GPIOx_->MODER |= temp;
	}
	else {
		// interrupt mode
		if(GPIO_MODE_IT_FT == config_.GPIO_PinMode) {
			// 1. configure the FTSR
			EXTI->FTSR |= (1 << config_.GPIO_PinNumber);

			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << config_.GPIO_PinNumber);

		}
		else if (GPIO_MODE_IT_RT == config_.GPIO_PinMode) {
			// 1. configure the RTSR
			EXTI->RTSR |= (1 << config_.GPIO_PinNumber);

			// clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << config_.GPIO_PinNumber);
		}
		else if (GPIO_MODE_IT_RFT == config_.GPIO_PinMode) {
			// 1. configure both FTSR and RTSR
			EXTI->FTSR |= (1 << config_.GPIO_PinNumber);
			EXTI->RTSR |= (1 << config_.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		u8 temp1 = config_.GPIO_PinNumber >> 2;
		u8 temp2 = config_.GPIO_PinNumber % 4;
		u8 portCode = gpio_baseAddr_to_code(GPIOx_);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portCode << (4 * temp2));

		// 3. Enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << config_.GPIO_PinNumber;
	}

	temp = 0;
	// 2. configure the speed
	temp = config_.GPIO_PinSpeed << (2 * config_.GPIO_PinNumber);
	GPIOx_->OSPEEDR &= ~(0x3 << (2 * config_.GPIO_PinNumber)); // clearing
	GPIOx_->OSPEEDR |= temp;

	temp = 0;
	// 3. configure the pupd settings
	temp = config_.GPIO_PinPuPdControl << (2 * config_.GPIO_PinNumber);
	GPIOx_->PUPDR &= ~(0x3 << (2 * config_.GPIO_PinNumber));// clearing
	GPIOx_->PUPDR |= temp;

	temp = 0;
	// 4. configure the optype
	temp = config_.GPIO_PinOutputType << config_.GPIO_PinNumber;
	GPIOx_->OTYPER &= ~(0x1 << config_.GPIO_PinNumber); // clearing
	GPIOx_->OTYPER |= temp;

	temp = 0;
	// 5. configure the alt functionality
	if(config_.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// configure alt function register
		u8 temp1, temp2;
		temp1 = config_.GPIO_PinNumber / 8;
		temp2 = config_.GPIO_PinNumber % 8;
		GPIOx_->AFR[temp1] &= ~(0xF << (4 * temp2));
		GPIOx_->AFR[temp1] |= (config_.GPIO_PinAltFunMode << (4 * temp2));
	}
}


/*!
 * @brief Reset GPIO Port
 *
 * @param None
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_DeInit() {
	if(GPIOx_ == GPIOA) {
		GPIOA_REG_RESET();
	}
	else if (GPIOx_ == GPIOB) {
		GPIOB_REG_RESET();
	}
	else if (GPIOx_ == GPIOC) {
		GPIOC_REG_RESET();
	}
	else if (GPIOx_ == GPIOD) {
		GPIOD_REG_RESET();
	}
	else if (GPIOx_ == GPIOE) {
		GPIOE_REG_RESET();
	}
	else if (GPIOx_ == GPIOF) {
		GPIOF_REG_RESET();
	}
	else if (GPIOx_ == GPIOG) {
		GPIOG_REG_RESET();
	}
	else if (GPIOx_ == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*!
 * @brief Read single bit in Pin
 *
 * @param None
 *
 * @return uint8, b'0000_000x
 *
 */
u8 GPIO_Handler::GPIO_ReadFromInputPin() const {
	u8 value;
	value = (u8)((GPIOx_->IDR >> config_.GPIO_PinNumber) & 0x00000001);
	return value;
}


/*!
 * @brief Read 16 bits PORT(A, B, C,...)
 *
 * @param None
 *
 * @return uint16_t: PORT value
 *
 */
uint16_t GPIO_Handler::GPIO_ReadFromInputPort() const {
	uint16_t value;
	value = GPIOx_->IDR;
	return value;
}


/*!
 * @brief Write single bit to Pin
 *
 * @param None
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_WriteToOutputPin(const u8 Value) {
	if(Value == SET) {
		GPIOx_->ODR |= (0x1 << config_.GPIO_PinNumber);
	}
	else {
		GPIOx_->ODR &= ~(0x1 << config_.GPIO_PinNumber);
	}
}


/*!
 * @brief Write value to PORT
 *
 * @param[in] Value: write value
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_WriteToOutputPort(const uint16_t Value) {
	GPIOx_->ODR &= 0x0000;
	GPIOx_->ODR = Value;
}

/*!
 * @brief Toggle output pin
 *
 * @param None
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_ToggleOutputPin() {
	GPIOx_->ODR ^= (0x1 << config_.GPIO_PinNumber);
}

/*!
 * @brief Enable Interrupt for NVIC
 *
 * @param[in] IRQNumber     - IRQNumber IRQ Position number
 * @param[in] EnorDi        - Enable/Disable Flag
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_IRQInterruptConfig(const u8 IRQNumber, const u8 EnorDi) {
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


/*!
 * @brief Configure priority for GPIO, 0->15, lower number higher priority
 *
 * @param[in] IRQNumber: IRQ position for EXIT
 * @param[in] IRPriority: 0->15
 *
 * @return None
 *
 */
void GPIO_Handler::GPIO_IRQPriorityConfig(const u8 IRQNumber, const u8 IRQPriority) {
	// 1. first lets find out the ipr register
	u8 iprx = IRQNumber >> 2;
	u8 iprx_section = IRQNumber % 4;
	u8 shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*!
 * @brief Clear the interrupt flag for the next interrupt
 *
 * @param None
 *
 * @return None
 *
 */
void GPIO_IRQHandling(u8 PinNumber) {
	// clear the EXIT PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)) {
		// clear by set to 1
		EXTI->PR |= (1 << PinNumber);
	}
}

constexpr static inline u8 get_irq_pinNum(u8 PinNumber) {
	return (PinNumber < 5) 	? PinNumber + 6 : \
		   (PinNumber < 10)	? IRQ_NO_EXTI9_5 : \
		   (PinNumber < 16) ? IRQ_NO_EXTI15_10 : 0;
}
