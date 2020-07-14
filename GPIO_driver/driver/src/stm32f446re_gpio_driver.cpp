/*
 * stm32f446re_gpio_driver.cpp
 *
 *  Created on: Jul 13, 2020
 *      Author: prnsoft
 */




#include "../inc/stm32f446re_gpio_driver.h"

GPIO_Handler::GPIO_Handler(
		uint8_t GPIO_PinNumber,
		uint8_t GPIO_PinMode,
		uint8_t GPIO_PinSpeed,
		uint8_t GPIO_PinOPType,
		uint8_t GPIO_PinPuPdControl,
		uint8_t GPIO_PinAltFunMode) {
	GPIOx_.GPIO_PinConfig.GPIO_PinNumber = GPIO_PinNumber;
	GPIOx_.GPIO_PinConfig.GPIO_PinMode = GPIO_PinMode;
	GPIOx_.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PinSpeed;
	GPIOx_.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PinPuPdControl;
	GPIOx_.GPIO_PinConfig.GPIO_PinOPType = GPIO_PinOPType;
	GPIOx_.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PinAltFunMode;
	GPIO_PeriClockControl();
	GPIO_Init();
}

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

// init and de-init
void GPIO_Handler::GPIO_Init() {
	uint32_t temp;
	// 1. configure the mode of gpio pin
	if(GPIOx_.GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// the non interrupt mode
		temp = GPIOx_.GPIO_PinConfig.GPIO_PinMode << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
		GPIOx_.pGPIOx->MODER &= ~(0x3 << (2 * GPIOx_.GPIO_PinConfig.GPIO_PinNumber)); // clearing
		GPIOx_.pGPIOx->MODER |= temp;
	}
	else {
		// interrupt mode
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

// Data read and write
uint8_t GPIO_Handler::GPIO_ReadFromInputPin() {
	uint8_t value;
	value = (uint8_t)((GPIOx_.pGPIOx->IDR >> GPIOx_.GPIO_PinConfig.GPIO_PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_Handler::GPIO_ReadFromInputPort() {
	uint16_t value;
	value = GPIOx_.pGPIOx->IDR;
	return value;
}

void GPIO_Handler::GPIO_WriteToOutputPin(uint8_t Value) {
	if(Value == SET) {
		GPIOx_.pGPIOx->ODR |= (0x1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
	}
	else {
		GPIOx_.pGPIOx->ODR &= ~(0x1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
	}
}

void GPIO_Handler::GPIO_WriteToOutputPort(uint16_t Value) {
	GPIOx_.pGPIOx->ODR &= 0x0000;
	GPIOx_.pGPIOx->ODR = Value;
}

void GPIO_Handler::GPIO_ToggleOutputPin() {
	GPIOx_.pGPIOx->ODR ^= (0x1 << GPIOx_.GPIO_PinConfig.GPIO_PinNumber);
}

// IRQ Configuration and ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}

void GPIO_IRQHandling(uint8_t PinNumber) {

}
