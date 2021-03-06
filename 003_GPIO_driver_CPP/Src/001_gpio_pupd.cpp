/*
 * gpio_pupd.cpp
 *
 *  Created on: Jul 14, 2020
 *      Author: prnsoft
 */




#include "../driver/stm32f446re_gpio_driver.h"


void delay(void) {
	for(uint32_t i = 0; i < 500000; ++i){};
}

void small_delay(void) {
	for(uint32_t i = 0; i < 250000; ++i){};
}

int main(void)
{
	gpio_handler LED2(GPIOA);
	LED2.gpio_init(GPIO_PIN_NO_5, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
	LED2.gpio_write_to_output_pin(1);
	for(;;) {
		delay();
		LED2.gpio_toggle_output_pin();
	}
}
