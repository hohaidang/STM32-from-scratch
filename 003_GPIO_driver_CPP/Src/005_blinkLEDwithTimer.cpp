/*
 * 005_blinkLEDwithTimer.cpp
 *
 *  Created on: Aug 10, 2020
 *      Author: hohaidang
 */



#include "../driver/inc/stm32f446re_gpio_driver.h"
#include "../driver/inc/sys_tick_driver.h"


int main(void)
{
    SysTick SysTick;
    GPIO_Handler LED2 = GPIO_Handler(GPIOA,
                                    GPIO_PIN_NO_5,
                                    GPIO_MODE_OUT,
                                    GPIO_SPEED_LOW,
                                    GPIO_OP_TYPE_PP,
                                    GPIO_NO_PUPD);

    while(1)
    {
        LED2.gpio_toggle_output_pin();
        SysTick.delay_ms(1000);
    }
    return 0;
}



