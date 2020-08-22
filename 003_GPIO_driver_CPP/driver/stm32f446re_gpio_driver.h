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
#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

//@GPIO_PIN_MODES
#define GPIO_MODE_IN            0
#define GPIO_MODE_OUT           1
#define GPIO_MODE_ALTFN         2
#define GPIO_MODE_ANALOG        3
#define GPIO_MODE_IT_FT         4 // failing edge interrupt
#define GPIO_MODE_IT_RT         5 // rising edge interrupt
#define GPIO_MODE_IT_RFT        6 // rising edge / failing edge trigger interrupt

// @GPIO_PIN_OUTPUTTYPE
#define GPIO_OP_TYPE_PP         0 // output push-pull
#define GPIO_OP_TYPE_OD         1 // output open-drain

// @GPIO_PIN_SPEEDS
#define GPIO_SPEED_LOW          0
#define GPIO_SPEED_MEDIUM       1
#define GPIO_SPEED_FAST         2
#define GPIO_SPEED_HIGH         3

// @GPIO_PIN_PUPD, pull-up/down register
#define GPIO_NO_PUPD            0 // GPIO pull up, pull down
#define GPIO_PIN_PU             1 // pull-up
#define GPIO_PIN_PD             2 // pull-down

/* @GPIO_PIN_ALT_FUNC */
#define GPIO_ALT_1              (u8)(1)
#define GPIO_ALT_2              (u8)(2)
#define GPIO_ALT_3              (u8)(3)
#define GPIO_ALT_4              (u8)(4)
#define GPIO_ALT_5              (u8)(5)
#define GPIO_ALT_6              (u8)(6)
#define GPIO_ALT_7              (u8)(7)
#define GPIO_ALT_8              (u8)(8)
#define GPIO_ALT_9              (u8)(9)
#define GPIO_ALT_10             (u8)(10)
#define GPIO_ALT_11             (u8)(11)
#define GPIO_ALT_12             (u8)(12)
#define GPIO_ALT_13             (u8)(13)
#define GPIO_ALT_14             (u8)(14)
#define GPIO_ALT_15             (u8)(15)

// Configuration pin
typedef struct {
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinOutputType;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

template<u8 pin_number = GPIO_PIN_NO_0,
         u8 pin_mode = GPIO_PIN_NO_5,
         u8 pin_speed = GPIO_PIN_NO_5,
         u8 output_type = GPIO_OP_TYPE_PP,
         u8 pupd_control = GPIO_NO_PUPD,
         u8 irq_priority = IRQ_Prio_NO_15,
         u8 alt_func_mode = DISABLE>
class GPIO_Handler {
protected:
    GPIO_RegDef_t *GPIOx_ = { };
    GPIO_PinConfig_t config_ = { };

public:
    /*!
     * @brief Constructor, initialize GPIO, clock, IRQ, Alt Function
     *
     * @param None
     *
     * @return None
     *
     */
    constexpr
    GPIO_Handler(GPIO_RegDef_t *GPIOx_addr) : GPIOx_(GPIOx_addr),
                                              config_({pin_number, pin_mode, pin_speed, output_type, pupd_control, alt_func_mode}) {
        gpio_peri_clock_control();
        gpio_init();

        // Configure interrupt
        if (pin_mode >= GPIO_MODE_IT_FT) {
            gpio_irq_interrupt_config<ENABLE>();
        }
    }

    /*!
     * @brief DeConstructor, Reset GPIO PORT
     *
     * @param None
     *
     * @return None
     *
     * @node: Be careful when using many Peripheral device in the same PORT
     */
    ~GPIO_Handler() {
        gpio_de_init();
    }

    /*!
     * @brief Read single bit in Pin
     *
     * @param None
     *
     * @return uint8, b'0000_000x
     *
     */
    u8 gpio_read_from_input_pin() const {
        return static_cast<u8>((GPIOx_->IDR >> pin_number) & 0x00000001);
    }

    /*!
     * @brief Read 16 bits PORT(A, B, C,...)
     *
     * @param None
     *
     * @return uint16_t: PORT value
     *
     */
    u16 gpio_read_from_input_port() const {
        return static_cast<u16>(GPIOx_->IDR & 0x0000FFFF);
    }

    /*!
     * @brief Write single bit to Pin
     *
     * @param[in] value: write value
     *
     * @return None
     *
     */
    void gpio_write_to_output_pin(const u8 value) const {
        if (value == SET) {
            GPIOx_->ODR |= (0x1 << config_.GPIO_PinNumber);
        } else {
            GPIOx_->ODR &= ~(0x1 << config_.GPIO_PinNumber);
        }
    }

    /*!
     * @brief Write value to PORT
     *
     * @param[in] value: write value
     *
     * @return None
     *
     */
    void gpio_write_to_output_port(const u16 value) const {
        GPIOx_->ODR &= 0x0000;
        GPIOx_->ODR = value;
    }

    /*!
     * @brief Toggle output pin
     *
     * @param None
     *
     * @return None
     *
     */
    void gpio_toggle_output_pin() {
        GPIOx_->ODR ^= static_cast<u32>(1UL << pin_number);
    }

    /*!
     * @brief Clear the interrupt flag for the next interrupt
     *
     * @param None
     *
     * @return None
     *
     */
    void gpio_irq_handling() {
        // clear the EXIT PR register corresponding to the pin number
        if (EXTI->PR & (1 << pin_number)) {
            // clear by set to 1
            EXTI->PR |= (1 << pin_number);
        }
    }
private:
    /*!
     * @brief peripheral clock setup
     *
     * @param None
     *
     * @return None
     *
     */
    void gpio_peri_clock_control() {
        if (GPIOx_ == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (GPIOx_ == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (GPIOx_ == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (GPIOx_ == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (GPIOx_ == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (GPIOx_ == GPIOF) {
            GPIOF_PCLK_EN();
        } else if (GPIOx_ == GPIOG) {
            GPIOG_PCLK_EN();
        } else if (GPIOx_ == GPIOH) {
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
    constexpr void gpio_init() const {
        uint32_t temp = 0;
        // 1. configure the mode of gpio pin
        if (pin_mode <= GPIO_MODE_ANALOG) {
            // the non interrupt mode
            temp = pin_mode << (2 * pin_number);
            GPIOx_->MODER &= ~(0x3 << (2 * pin_number)); // clearing
            GPIOx_->MODER |= temp;
        } else {
            // interrupt mode
            if (GPIO_MODE_IT_FT == pin_mode) {
                // 1. configure the FTSR
                EXTI->FTSR |= (1 << pin_number);

                // clear the corresponding RTSR bit
                EXTI->RTSR &= ~(1 << pin_number);

            } else if (GPIO_MODE_IT_RT == pin_mode) {
                // 1. configure the RTSR
                EXTI->RTSR |= (1 << pin_number);

                // clear the corresponding RTSR bit
                EXTI->FTSR &= ~(1 << pin_number);
            } else if (GPIO_MODE_IT_RFT == pin_mode) {
                // 1. configure both FTSR and RTSR
                EXTI->FTSR |= (1 << pin_number);
                EXTI->RTSR |= (1 << pin_number);
            }

            // 2. configure the GPIO port selection in SYSCFG_EXTICR
            u8 temp1 = pin_number >> 2;
            u8 temp2 = pin_number % 4;
            u8 portCode = gpio_baseAddr_to_code(GPIOx_);
            SYSCFG_PCLK_EN();
            SYSCFG->EXTICR[temp1] |= (portCode << (4 * temp2));

            // 3. Enable the exti interrupt delivery using IMR
            EXTI->IMR |= 1 << pin_number;
        }

        temp = 0;
        // 2. configure the speed
        temp = pin_speed << (2 * pin_number);
        GPIOx_->OSPEEDR &= ~(0x3 << (2 * pin_number)); // clearing
        GPIOx_->OSPEEDR |= temp;

        temp = 0;
        // 3. configure the pupd settings
        temp = pupd_control << (2 * pin_number);
        GPIOx_->PUPDR &= ~(0x3 << (2 * pin_number)); // clearing
        GPIOx_->PUPDR |= temp;

        temp = 0;
        // 4. configure the optype
        temp = output_type << pin_number;
        GPIOx_->OTYPER &= ~(0x1 << pin_number); // clearing
        GPIOx_->OTYPER |= temp;

        temp = 0;
        // 5. configure the alt functionality
        if (pin_mode == GPIO_MODE_ALTFN) {
            // configure alt function register
            u8 temp1, temp2;
            temp1 = pin_number / 8;
            temp2 = pin_number % 8;
            GPIOx_->AFR[temp1] &= ~(0xF << (4 * temp2));
            GPIOx_->AFR[temp1] |= (alt_func_mode << (4 * temp2));
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
    void gpio_de_init(){
        if (GPIOx_ == GPIOA) {
            GPIOA_REG_RESET();
        } else if (GPIOx_ == GPIOB) {
            GPIOB_REG_RESET();
        } else if (GPIOx_ == GPIOC) {
            GPIOC_REG_RESET();
        } else if (GPIOx_ == GPIOD) {
            GPIOD_REG_RESET();
        } else if (GPIOx_ == GPIOE) {
            GPIOE_REG_RESET();
        } else if (GPIOx_ == GPIOF) {
            GPIOF_REG_RESET();
        } else if (GPIOx_ == GPIOG) {
            GPIOG_REG_RESET();
        } else if (GPIOx_ == GPIOH) {
            GPIOH_REG_RESET();
        }
    }

    /*!
     * @brief Enable/Disable Interrupt for NVIC and set priority
     *
     * @param[in] IRQNumber     - IRQNumber IRQ Position number
     * @param[in] EnorDi        - Enable/Disable Flag
     *
     * @return None
     *
     */
    template<u8 en_or_di>
    constexpr void gpio_irq_interrupt_config() const {
        constexpr u8 irq_number = (pin_number < 5) ? pin_number + 6 :
                                  (pin_number < 10) ? EXTI9_5_IRQn :
                                  (pin_number < 16) ? EXTI15_10_IRQn : 0;

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

            /* Configure priority*/
            u8 iprx = irq_number >> 2;
            u8 iprx_section = irq_number % 4;
            u8 shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
            NVIC->IPR[iprx] |= (irq_priority << shift_amount);

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
};


#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
