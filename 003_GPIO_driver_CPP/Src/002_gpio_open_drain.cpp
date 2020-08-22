/*
 * gpio_open_drain.cpp
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
	GPIO_Handler LED2 = GPIO_Handler(GPIOA, GPIO_PIN_NO_5, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_OD, GPIO_NO_PUPD);
	LED2.gpio_write_to_output_pin(1);
	for(;;) {
		delay();
		LED2.gpio_toggle_output_pin();
	}

}
