/*
 * 005_blinkLEDwithTimer.cpp
 *
 *  Created on: Aug 10, 2020
 *      Author: hohaidang
 */




#include "../driver/inc/stm32f446re_gpio_driver.h"

void delay(void)
{
    for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void)
{

    GPIO_Handler LED2 = GPIO_Handler(GPIOA,
                                    GPIO_PIN_NO_5,
                                    GPIO_MODE_OUT,
                                    GPIO_SPEED_LOW,
                                    GPIO_OP_TYPE_PP,
                                    GPIO_NO_PUPD);

    while(1)
    {
        LED2.GPIO_ToggleOutputPin();
        delay();
    }
    return 0;
}
