/*
 * stm32f446re_gpio_driver.h
 *
 *  Created on: Jul 13, 2020
 *      Author: hohaidang
 */

#ifndef INC_STM32F446RE_GPIO_DRIVER_H_
#define INC_STM32F446RE_GPIO_DRIVER_H_

#include "stm32f4xx.h"
#include "core_cm4.h"

// @GPIO_PIN_NUMS
#define GPIO_PIN_NO_0           static_cast<u8>(0)
#define GPIO_PIN_NO_1           static_cast<u8>(1)
#define GPIO_PIN_NO_2           static_cast<u8>(2)
#define GPIO_PIN_NO_3           static_cast<u8>(3)
#define GPIO_PIN_NO_4           static_cast<u8>(4)
#define GPIO_PIN_NO_5           static_cast<u8>(5)
#define GPIO_PIN_NO_6           static_cast<u8>(6)
#define GPIO_PIN_NO_7           static_cast<u8>(7)
#define GPIO_PIN_NO_8           static_cast<u8>(8)
#define GPIO_PIN_NO_9           static_cast<u8>(9)
#define GPIO_PIN_NO_10          static_cast<u8>(10)
#define GPIO_PIN_NO_11          static_cast<u8>(11)
#define GPIO_PIN_NO_12          static_cast<u8>(12)
#define GPIO_PIN_NO_13          static_cast<u8>(13)
#define GPIO_PIN_NO_14          static_cast<u8>(14)
#define GPIO_PIN_NO_15          static_cast<u8>(15)

//@GPIO_PIN_MODES
#define GPIO_MODE_IN            static_cast<u8>(0)
#define GPIO_MODE_OUT           static_cast<u8>(1)
#define GPIO_MODE_ALTFN         static_cast<u8>(2)
#define GPIO_MODE_ANALOG        static_cast<u8>(3)
#define GPIO_MODE_IT_FT         static_cast<u8>(4) // failing edge interrupt
#define GPIO_MODE_IT_RT         static_cast<u8>(5) // rising edge interrupt
#define GPIO_MODE_IT_RFT        static_cast<u8>(6) // rising edge / failing edge trigger interrupt

// @GPIO_PIN_OUTPUTTYPE
#define GPIO_OP_TYPE_PP         static_cast<u8>(0) // output push-pull
#define GPIO_OP_TYPE_OD         static_cast<u8>(1) // output open-drain

// @GPIO_PIN_SPEEDS
#define GPIO_SPEED_LOW          static_cast<u8>(0)
#define GPIO_SPEED_MEDIUM       static_cast<u8>(1)
#define GPIO_SPEED_FAST         static_cast<u8>(2)
#define GPIO_SPEED_HIGH         static_cast<u8>(3)

// @GPIO_PIN_PUPD, pull-up/down register
#define GPIO_NO_PUPD            static_cast<u8>(0) // GPIO pull up, pull down
#define GPIO_PIN_PU             static_cast<u8>(1) // pull-up
#define GPIO_PIN_PD             static_cast<u8>(2) // pull-down

/* @GPIO_PIN_ALT_FUNC */
#define GPIO_ALT_1              static_cast<u8>(1)
#define GPIO_ALT_2              static_cast<u8>(2)
#define GPIO_ALT_3              static_cast<u8>(3)
#define GPIO_ALT_4              static_cast<u8>(4)
#define GPIO_ALT_5              static_cast<u8>(5)
#define GPIO_ALT_6              static_cast<u8>(6)
#define GPIO_ALT_7              static_cast<u8>(7)
#define GPIO_ALT_8              static_cast<u8>(8)
#define GPIO_ALT_9              static_cast<u8>(9)
#define GPIO_ALT_10             static_cast<u8>(10)
#define GPIO_ALT_11             static_cast<u8>(11)
#define GPIO_ALT_12             static_cast<u8>(12)
#define GPIO_ALT_13             static_cast<u8>(13)
#define GPIO_ALT_14             static_cast<u8>(14)
#define GPIO_ALT_15             static_cast<u8>(15)

void GPIO_IRQHandling(u8 PinNumber);

class gpio_handler {
protected:
    GPIO_RegDef_t *gpiox_;
    u8 pin_number_;

public:             
    void gpio_init( GPIO_RegDef_t *GPIOx_addr,                 /*!<possible values from @GPIOx_ADDR>*/
                    u8 pin_number,                             /*!<possible values from @GPIO_PIN_NUMS>*/
                    u8 pin_mode,                               /*!<possible values from @GPIO_PIN_MODES>*/
                    u8 pin_speed = GPIO_SPEED_LOW,             /*!<possible values from @GPIO_PIN_SPEEDS>*/
                    u8 output_type = GPIO_OP_TYPE_PP,          /*!<possible values from @GPIO_PIN_OUTPUTTYPE>*/
                    u8 pupd_control = GPIO_NO_PUPD,            /*!<possible values from @GPIO_PIN_PUPD>*/
                    u8 IRQ_priority = IRQ_Prio_NO_15,          /*!<possible values from @GPIO_IRQ_Priority>*/
                    u8 alt_func_mode = DISABLE);               /*!<possible values from @GPIO_PIN_ALT_FUNC>*/

    /* Data read and write */
    u8 gpio_read_from_input_pin() const;
    u16 gpio_read_from_input_port() const;
    void gpio_write_to_output_pin(const u8 value);
    void gpio_write_to_output_port(const u16 value);
    void gpio_toggle_output_pin();
    void gpio_irq_handling();
    void gpio_deinit();

private:
    /* peripheral clock setup */
    void gpio_peripheral_clk_control();

    /* IRQ Configuration and ISR Handling */
    void gpio_irq_config(const u8 irq_number, const u8 en_or_di);
    void gpio_irq_priority_config(const u8 irq_number, const u8 irq_priority);
    inline u8 get_irq_pin_num(u8 pin_number);
};

inline u8 gpio_handler::get_irq_pin_num(u8 pin_number) {
    return (pin_number < 5) ? pin_number + 6 :
           (pin_number < 10) ? EXTI9_5_IRQn :
           (pin_number < 16) ? EXTI15_10_IRQn : 0;
}
#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
