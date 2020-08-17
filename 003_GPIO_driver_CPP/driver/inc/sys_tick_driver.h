/*
 * sys_tick_driver.h
 *
 *  Created on: Aug 17, 2020
 *      Author: prnsoft
 */

#ifndef INC_SYS_TICK_DRIVER_H_
#define INC_SYS_TICK_DRIVER_H_

#include "stm32f4xx.h"



class SysTick {
public:
    SysTick();
    ~SysTick();
    void delay_ms(u32 period);
    u32 get_tick();
};



#endif /* INC_SYS_TICK_DRIVER_H_ */
