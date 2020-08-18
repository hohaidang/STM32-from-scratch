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

void GPIO_IRQHandling(uint8_t PinNumber);

// Configuration pin
typedef struct {
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOutputType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

class GPIO_Handler {
protected:
    GPIO_RegDef_t *GPIOx_;
    GPIO_PinConfig_t config_;

public:
    GPIO_Handler(GPIO_RegDef_t *GPIOx_addr,                      /*!<possible values from @GPIOx_ADDR>*/
                 uint8_t pin_number,                             /*!<possible values from @GPIO_PIN_NUMS>*/
                 uint8_t pin_mode,                               /*!<possible values from @GPIO_PIN_MODES>*/
                 uint8_t pin_speed = GPIO_SPEED_LOW,             /*!<possible values from @GPIO_PIN_SPEEDS>*/
                 uint8_t output_type = GPIO_OP_TYPE_PP,          /*!<possible values from @GPIO_PIN_OUTPUTTYPE>*/
                 uint8_t pupd_control = GPIO_NO_PUPD,            /*!<possible values from @GPIO_PIN_PUPD>*/
                 uint8_t IRQ_priority = IRQ_Prio_NO_15,          /*!<possible values from @GPIO_IRQ_Priority>*/
                 uint8_t alt_func_mode = DISABLE);               /*!<possible values from @GPIO_PIN_ALT_FUNC>*/

    ~GPIO_Handler();

    // Data read and write
    uint8_t GPIO_ReadFromInputPin() const;
    uint16_t GPIO_ReadFromInputPort() const;
    void GPIO_WriteToOutputPin(const uint8_t Value);
    void GPIO_WriteToOutputPort(const uint16_t Value);
    void GPIO_ToggleOutputPin();

private:
    // peripheral clock setup
    void GPIO_PeriClockControl();
    void GPIO_Init();
    void GPIO_DeInit();

    // IRQ Configuration and ISR Handling
    void GPIO_IRQInterruptConfig(const uint8_t IRQNumber, const uint8_t EnorDi);
    void GPIO_IRQPriorityConfig(const uint8_t IRQNumber,
            const uint8_t IRQPriority);

};

#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
