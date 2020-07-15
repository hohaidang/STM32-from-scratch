/*
 * led_button.cpp
 *
 *  Created on: Jul 14, 2020
 *      Author: prnsoft
 */



#include "../driver/inc/stm32f446re_gpio_driver.h"


void delay(void) {
	for(uint32_t i = 0; i < 500000; ++i){};
}

void small_delay(void) {
	for(uint32_t i = 0; i < 250000; ++i){};
}

int main(void)
{
	GPIO_Handler LED2 = GPIO_Handler(GPIOA, GPIO_PIN_NO_5, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
	GPIO_Handler user_button = GPIO_Handler(GPIOC, GPIO_PIN_NO_13, GPIO_MODE_IN, GPIO_SPEED_LOW);
	LED2.GPIO_WriteToOutputPin(1);
	for(;;) {
		if(!user_button.GPIO_ReadFromInputPin()) {
			small_delay(); // remove debouncing for Botton
			LED2.GPIO_ToggleOutputPin();
		}
	}

}
