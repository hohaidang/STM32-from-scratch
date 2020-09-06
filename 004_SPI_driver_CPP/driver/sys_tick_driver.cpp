/*
 * sys_tick_driver.cpp
 *
 *  Created on: Aug 17, 2020
 *      Author: hohaidang
 */


#include "sys_tick_driver.h"
#include "core_cm4.h"

#define HSI_CLOCK                   (u32)(16000000UL) /* 16MHz */

static volatile int32_t utick = 0x00000000UL;

template <u32 Clk_Source>
struct compute_reload_tick {
    enum { value = static_cast<u32>((Clk_Source / 1000U) - 1U) };
};

/*!
 * @brief:  initialize system tick with 1ms
 *
 * @param: None
 *
 * @return: None
 *
 */

void sys_tick::init() {
    SYSTICK->CSR = 0; /* disable systick */

    SYSTICK->RVR = compute_reload_tick<HSI_CLOCK>::value; /* set reload register */

    SCB->SHPR[2] |= static_cast<u32>(IRQ_Prio_NO_0 << SCB_SHPR_SYSTICK_Pos); /* Set priority 0 for system tick */

    // Reset Systick counter value
    SYSTICK->CVR = 0;

    /* Select processor clock
     * Enable System Tick interrupt
     */
    SYSTICK->CSR |= (SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENA);
}

/*!
 * @brief: Destructor disable system tick
 *
 * @param: None
 *
 * @return: None
 *
 */
sys_tick::~sys_tick() {
    SYSTICK->CSR = 0; /* disable systick */
    SYSTICK->RVR = 0; /* reset reload register */
    SYSTICK->CSR &= ~(SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENA);
}

/*!
 * @brief: This API is used for delay milliseconds.
 *
 * @param[in]: period of time in millisecond.
 *
 * @return: None
 *
 */
void sys_tick::delay_ms(const u32 period) const {
    utick = period;
    while(utick != RESET);
}

/*!
 * @brief: Interrupt service routine for system tick. This API will be triggered every time counter count from 1 to 0. (1ms)
 *
 * @param: None
 *
 * @return: None
 *
 */
extern "C" {
    void SysTick_Handler(void) {
        utick = (utick > 0) ? (utick - 1u) : (utick);
    }
}
