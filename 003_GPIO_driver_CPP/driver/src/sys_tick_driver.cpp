/*
 * sys_tick_driver.cpp
 *
 *  Created on: Aug 17, 2020
 *      Author: prnsoft
 */


#include "../inc/sys_tick_driver.h"



#define HSI_CLOCK                   (u32)(16000000UL) /* 16MHz */

static volatile int32_t utick = 0x00000000UL;

template <u32 Clk_Source>
struct compute_reload_tick {
    enum { value = static_cast<u32>((Clk_Source / 1000U) - 1U) };
};

// TODO: giai quyet van de khi nao goi delay thi moi init system tick
SysTick::SysTick() {
    SYSTICK->CSR = 0; // disable systick

    SYSTICK->RVR = compute_reload_tick<HSI_CLOCK>::value; // set reload register
    // Set priority 0 for systick
    SCB->SHPR[3] |= 0;

    // Reset Systick counter value
    SYSTICK->CVR = 0;

    // select processor clock
    /* Enable systick interrupt*/
    u32 temp = 0;
    temp = (SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENA);
    SYSTICK->CSR = temp;
}

void SysTick::delay_ms(u32 period) {
    utick = period;
    while(utick != RESET);
}

extern "C" {
    void SysTick_Handler(void) {
        // handle the interrupt
        if (utick > 0) {
            --utick;
        }
    }
}
