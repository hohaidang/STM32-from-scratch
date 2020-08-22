/*
 * 005_blinkLEDwithTimer.cpp
 *
 *  Created on: Aug 10, 2020
 *      Author: hohaidang
 */



#include "../driver/stm32f446re_gpio_driver.h"
#include "../driver/sys_tick_driver.h"

SysTick SysTick;
GPIO_Handler<GPIO_PIN_NO_5,
                 GPIO_MODE_OUT,
                 GPIO_SPEED_LOW,
                 GPIO_OP_TYPE_PP,
                 GPIO_NO_PUPD,
                 IRQ_Prio_NO_15,
                 DISABLE> LED2(GPIOA);

int main(void)
{



    while(1)
    {
        LED2.gpio_toggle_output_pin();
        SysTick.delay_ms(1000);
    }
    return 0;
}



