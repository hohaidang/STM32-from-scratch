/*
 * stm32f446re_gpio_driver.cpp
 *
 *  Created on: Jul 13, 2020
 *      Author: hohaidang
 */

#include "stm32f446re_gpio_driver.h"


/*!
 * @brief DeConstructor, Reset GPIO PORT
 *
 * @param None
 *
 * @return None
 *
 * @node: Be careful when using many Peripheral device in the same PORT
 */
gpio_handler::~gpio_handler() {
    gpio_deinit();
}

/*!
 * @brief peripheral clock setup
 *
 * @param None
 *
 * @return None
 *
 */
void gpio_handler::gpio_peripheral_clk_control() {
    if (gpiox_ == GPIOA) {
        GPIOA_PCLK_EN();
    } else if (gpiox_ == GPIOB) {
        GPIOB_PCLK_EN();
    } else if (gpiox_ == GPIOC) {
        GPIOC_PCLK_EN();
    } else if (gpiox_ == GPIOD) {
        GPIOD_PCLK_EN();
    } else if (gpiox_ == GPIOE) {
        GPIOE_PCLK_EN();
    } else if (gpiox_ == GPIOF) {
        GPIOF_PCLK_EN();
    } else if (gpiox_ == GPIOG) {
        GPIOG_PCLK_EN();
    } else if (gpiox_ == GPIOH) {
        GPIOH_PCLK_EN();
    }
}

/*!
 * @brief Initial GPIO with clock, Speed, In/out mode, ...
 *
 * @param None
 *
 * @return None
 *
 */
void gpio_handler::gpio_init(GPIO_RegDef_t *GPIOx_addr,
                             u8 pin_number,
                             u8 pin_mode, 
                             u8 pin_speed, 
                             u8 output_type, 
                             u8 pupd_control,
                             u8 IRQ_priority, 
                             u8 alt_func_mode) {
    gpiox_ = GPIOx_addr;
    pin_number_ = pin_number;
    gpio_peripheral_clk_control();
    
    uint32_t temp = 0;
    // 1. configure the mode of gpio pin
    if (pin_mode <= GPIO_MODE_ANALOG) {
        // the non interrupt mode
        temp = pin_mode << (2 * pin_number_);
        gpiox_->MODER &= ~(0x3 << (2 * pin_number_)); // clearing
        gpiox_->MODER |= temp;
    } else {
        // interrupt mode
        if (GPIO_MODE_IT_FT == pin_mode) {
            // 1. configure the FTSR
            EXTI->FTSR |= (1 << pin_number_);

            // clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pin_number_);

        } else if (GPIO_MODE_IT_RT == pin_mode) {
            // 1. configure the RTSR
            EXTI->RTSR |= (1 << pin_number_);

            // clear the corresponding RTSR bit
            EXTI->FTSR &= ~(1 << pin_number_);
        } else if (GPIO_MODE_IT_RFT == pin_mode) {
            // 1. configure both FTSR and RTSR
            EXTI->FTSR |= (1 << pin_number_);
            EXTI->RTSR |= (1 << pin_number_);
        }

        // 2. configure the GPIO port selection in SYSCFG_EXTICR
        u8 temp1 = pin_number_ >> 2;
        u8 temp2 = pin_number_ % 4;
        u8 portCode = gpio_baseAddr_to_code(gpiox_);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] |= (portCode << (4 * temp2));

        // 3. Enable the exti interrupt delivery using IMR
        EXTI->IMR |= 1 << pin_number_;
    }

    temp = 0;
    // 2. configure the speed
    temp = pin_speed << (2 * pin_number_);
    gpiox_->OSPEEDR &= ~(0x3 << (2 * pin_number_)); // clearing
    gpiox_->OSPEEDR |= temp;

    temp = 0;
    // 3. configure the pupd settings
    temp = pupd_control << (2 * pin_number_);
    gpiox_->PUPDR &= ~(0x3 << (2 * pin_number_)); // clearing
    gpiox_->PUPDR |= temp;

    temp = 0;
    // 4. configure the optype
    temp = output_type << pin_number_;
    gpiox_->OTYPER &= ~(0x1 << pin_number_); // clearing
    gpiox_->OTYPER |= temp;

    temp = 0;
    // 5. configure the alt functionality
    if (pin_mode == GPIO_MODE_ALTFN) {
        // configure alt function register
        u8 temp1, temp2;
        temp1 = pin_number_ / 8;
        temp2 = pin_number_ % 8;
        gpiox_->AFR[temp1] &= ~(0xF << (4 * temp2));
        gpiox_->AFR[temp1] |= (alt_func_mode << (4 * temp2));
    }

    /* Configure interrupt */
    if (pin_mode >= GPIO_MODE_IT_FT) {
        u8 IRQ_number = get_irq_pin_num(pin_number);
        gpio_irq_config(IRQ_number, ENABLE);
        gpio_irq_priority_config(IRQ_number, IRQ_priority);
    }
}

/*!
 * @brief Reset GPIO Port
 *
 * @param None
 *
 * @return None
 *
 */
void gpio_handler::gpio_deinit() {
    if (gpiox_ == GPIOA) {
        GPIOA_REG_RESET();
    } else if (gpiox_ == GPIOB) {
        GPIOB_REG_RESET();
    } else if (gpiox_ == GPIOC) {
        GPIOC_REG_RESET();
    } else if (gpiox_ == GPIOD) {
        GPIOD_REG_RESET();
    } else if (gpiox_ == GPIOE) {
        GPIOE_REG_RESET();
    } else if (gpiox_ == GPIOF) {
        GPIOF_REG_RESET();
    } else if (gpiox_ == GPIOG) {
        GPIOG_REG_RESET();
    } else if (gpiox_ == GPIOH) {
        GPIOH_REG_RESET();
    }
}

/*!
 * @brief Read single bit in Pin
 *
 * @param None
 *
 * @return uint8, b'0000_000x
 *
 */
u8 gpio_handler::gpio_read_from_input_pin() const {
    u8 value;
    value = (u8) ((gpiox_->IDR >> pin_number_) & 0x00000001);
    return value;
}

/*!
 * @brief Read 16 bits PORT(A, B, C,...)
 *
 * @param None
 *
 * @return uint16_t: PORT value
 *
 */
uint16_t gpio_handler::gpio_read_from_input_port() const {
    uint16_t value;
    value = gpiox_->IDR;
    return value;
}

/*!
 * @brief Write single bit to Pin
 *
 * @param None
 *
 * @return None
 *
 */
void gpio_handler::gpio_write_to_output_pin(const u8 value) {
    if (value == SET) {
        gpiox_->ODR |= (0x1 << pin_number_);
    } else {
        gpiox_->ODR &= ~(0x1 << pin_number_);
    }
}

/*!
 * @brief Write value to PORT
 *
 * @param[in] Value: write value
 *
 * @return None
 *
 */
void gpio_handler::gpio_write_to_output_port(const uint16_t value) {
    gpiox_->ODR &= 0x0000;
    gpiox_->ODR = value;
}

/*!
 * @brief Toggle output pin
 *
 * @param None
 *
 * @return None
 *
 */
void gpio_handler::gpio_toggle_output_pin() {
    gpiox_->ODR ^= (0x1 << pin_number_);
}

/*!
 * @brief Enable Interrupt for NVIC
 *
 * @param[in] IRQNumber     - IRQNumber IRQ Position number
 * @param[in] EnorDi        - Enable/Disable Flag
 *
 * @return None
 *
 */
void gpio_handler::gpio_irq_config(const uint8_t irq_number, const uint8_t en_or_di) {
    if (en_or_di == ENABLE) {
        if (irq_number <= 31) {
            //  program ISER0 register
            NVIC->ISER[0] |= (1 << irq_number);
        } else if (irq_number > 31 && irq_number < 64) {
            // program ISER1 register
            NVIC->ISER[1] |= (1 << (irq_number % 32));
        } else if (irq_number >= 64 && irq_number < 96) {
            // program ISER2 register
            NVIC->ISER[2] |= (1 << (irq_number % 64));
        }
    } else {
        if (irq_number <= 31) {
            // program ICER0 register
            NVIC->ICER[0] |= (1 << irq_number);
        } else if (irq_number > 31 && irq_number < 64) {
            // program ICER1 register
            NVIC->ICER[1] |= (1 << (irq_number % 32));
        } else if (irq_number >= 64 && irq_number < 96) {
            // program ICE2 register
            NVIC->ICER[2] |= (1 << (irq_number % 64));
        }
    }
}

/*!
 * @brief Configure priority for GPIO, 0->15, lower number higher priority
 *
 * @param[in] IRQNumber: IRQ position for EXIT
 * @param[in] IRPriority: 0->15
 *
 * @return None
 *
 */
void gpio_handler::gpio_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority) {
    // 1. first lets find out the ipr register
    u8 iprx = irq_number >> 2;
    u8 iprx_section = irq_number % 4;
    u8 shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    NVIC->IPR[iprx] |= (irq_priority << shift_amount);
}

/*!
 * @brief Clear the interrupt flag for the next interrupt
 *
 * @param None
 *
 * @return None
 *
 */
void gpio_handler::gpio_irq_handling() {
    // clear the EXIT PR register corresponding to the pin number
    if (EXTI->PR & (1UL << pin_number_)) {
        // clear by set to 1
        EXTI->PR |= (1UL << pin_number_);
    }
}


