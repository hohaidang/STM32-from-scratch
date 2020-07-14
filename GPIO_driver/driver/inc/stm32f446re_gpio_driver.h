/*
 * stm32f446re_gpio_driver.h
 *
 *  Created on: Jul 13, 2020
 *      Author: prnsoft
 */

#ifndef INC_STM32F446RE_GPIO_DRIVER_H_
#define INC_STM32F446RE_GPIO_DRIVER_H_

#include "stm32f4xx.h"

// @GPIO_PIN_NUMS
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15


//@GPIO_PIN_MODES
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4 // faling edge interrupt
#define GPIO_MODE_IT_RT			5 // rising edge interrupt
#define GPIO_MODE_IT_RFT		6 // rising edge / faling edge trigger interrupt

// @GPIO_PIN_OUTPUTTYPE
#define GPIO_OP_TYPE_PP			0 // output push-pull
#define GPIO_OP_TYPE_OP			1 // output open-drain

// @GPIO_PIN_SPEEDS
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

#define GPIO_NO_PUPD			0 // Gpio pull up, pull down
#define GPIO_PIN_PU				1 // pull-up
#define GPIO_PIN_PD				2 // pull-down

class GPIO_Handler {
	// Configuration pin
	typedef struct {
		uint8_t GPIO_PinNumber;
		uint8_t GPIO_PinMode;
		uint8_t GPIO_PinSpeed;
		uint8_t GPIO_PinPuPdControl;
		uint8_t GPIO_PinOPType;
		uint8_t GPIO_PinAltFunMode;
	}GPIO_PinConfig_t;

	typedef struct {
		GPIO_RegDef_t *pGPIOx; // this holds the base address of GPIO port
		GPIO_PinConfig_t GPIO_PinConfig; // this hold Pin configuration setting
	}GPIO_Handle_t;

protected:
	GPIO_Handle_t GPIOx_;

public:
	GPIO_Handler(
			uint8_t GPIO_PinNumber, /*!<possible values from @GPIO_PIN_NUMS>*/
			uint8_t GPIO_PinMode, 	/*!<possible values from @GPIO_PIN_MODES>*/
			uint8_t GPIO_PinSpeed,	/*!<possible values from @GPIO_PIN_SPEEDS>*/
			uint8_t GPIO_PinOPType, /*!<possible values from @GPIO_PIN_OUTPUTTYPE>*/
			uint8_t GPIO_PinPuPdControl = GPIO_NO_PUPD,
			uint8_t GPIO_PinAltFunMode = 1);

	~GPIO_Handler();

	// Data read and write
	uint8_t GPIO_ReadFromInputPin();
	uint16_t GPIO_ReadFromInputPort();
	void GPIO_WriteToOutputPin(uint8_t Value);
	void GPIO_WriteToOutputPort(uint16_t Value);
	void GPIO_ToggleOutputPin();

	// IRQ Configuration and ISR Handling
	void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
	void GPIO_IRQHandling(uint8_t PinNumber);

private:
	// peripheral clock setup
	void GPIO_PeriClockControl(); // EnorDi: enable or disable
	void GPIO_Init();
	void GPIO_DeInit();
};

#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
