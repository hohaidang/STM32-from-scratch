#include <array>
#include "stm32f446re_gpio_driver.h"
#include "gtest/gtest.h"

#define GPIO_(n) GPIO##n
#define GPIO_PIN_NO_(n) GPIO_PIN_NO_##n

/* Test Fixtures */
struct gpio_test : public testing::Test {
    gpio_handler mc;
    void SetUp() {
        memset(&reg, 0, sizeof(reg));
    };
};

TEST_F(gpio_test, gpio_init) {
    u32 expected = 0;
    for(int i = 0; i < 8; ++i) {
        uint8_t pin_no = GPIO_PIN_NO_(0 + i);
        GPIO_RegDef_t *gpiox = GPIO_(A + i);
        mc.gpio_init(gpiox,
                     pin_no,
                     GPIO_MODE_ALTFN,
                     GPIO_SPEED_HIGH,
                     GPIO_OP_TYPE_PP,
                     GPIO_NO_PUPD,
                     IRQ_Prio_NO_14,
                     GPIO_ALT_5);

        
        expected |= (1U << i);
        EXPECT_EQ(RCC->AHB1ENR, expected);
        EXPECT_EQ((gpiox->MODER >> (2 * pin_no)) & 0x03, GPIO_MODE_ALTFN);
        EXPECT_EQ(EXTI->FTSR, 0x00); /* no interrupt configure */
        EXPECT_EQ(EXTI->RTSR, 0x00);
        EXPECT_EQ((gpiox->OSPEEDR >> (2 * pin_no)) & 0x03, GPIO_SPEED_HIGH);
        EXPECT_EQ((gpiox->OTYPER >> pin_no) & 0x01, GPIO_OP_TYPE_PP);
        EXPECT_EQ((gpiox->PUPDR >> (2 * pin_no)) & 0x03, GPIO_OP_TYPE_PP);
        EXPECT_EQ((gpiox->AFR[pin_no / 8] >> (4 * (pin_no % 8))) & 0xFF, GPIO_ALT_5); 

        for (size_t i = 0; i < sizeof(NVIC->ISER) / sizeof(u32); ++i) {
            EXPECT_EQ(NVIC->ISER[i], 0x00);
        }
        for (size_t i = 0; i < sizeof(NVIC->IPR) / sizeof(u32); ++i) {
            EXPECT_EQ(NVIC->IPR[i], 0x00);
        }
    }
}

TEST_F(gpio_test, gpio_init_irq) {
    u32 expected = 0;
    mc.gpio_init(GPIOB,
                 GPIO_PIN_NO_6,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);

    expected |= (1U << 1U);
    EXPECT_EQ(RCC->AHB1ENR, expected);
    
    EXPECT_EQ(GPIOB->MODER, 0x00);
    EXPECT_EQ(EXTI->FTSR, 0x00); /* no interrupt configure */
    EXPECT_EQ(EXTI->RTSR, 0x40);
    expected = 0x2000;
    EXPECT_EQ(GPIOB->OSPEEDR, expected);
    EXPECT_EQ(GPIOB->OTYPER, 0x40);
    EXPECT_EQ(GPIOB->PUPDR, 0x2000);
    EXPECT_EQ(GPIOB->AFR[0], 0x00);
    EXPECT_EQ(GPIOB->AFR[1], 0x00);
    EXPECT_EQ(NVIC->ISER[0], 0x800000);
    EXPECT_EQ(NVIC->ICER[0], 0x00);
    EXPECT_EQ(NVIC->IPR[5], 0xE0000000);
    for(size_t i = 0; i < sizeof(NVIC->IPR) / sizeof(u32); ++i) {
        if(i != 5) 
	{
		EXPECT_EQ(NVIC->IPR[i], 0x00);
	}            
    }
}

TEST_F(gpio_test, gpio_de_init) {
    mc.gpio_init(GPIOD,
                 GPIO_PIN_NO_6,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);
    mc.gpio_deinit();
    EXPECT_EQ(RCC->AHB1RSTR, 0x00);
}

TEST_F(gpio_test, gpio_read_from_input_pin) {
    mc.gpio_init(GPIOD,
                 GPIO_PIN_NO_6,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);
    std::array<uint8_t, 6> data_write {0x40, 0x00, 0xFF, 0xF0, 0x03, 0x77};
    for(const auto &it : data_write) {
        GPIOD->IDR = it;
        EXPECT_EQ(mc.gpio_read_from_input_pin(), (it >> GPIO_PIN_NO_6) & 0x01);
    }
}

TEST_F(gpio_test, gpio_read_from_input_port) {
    mc.gpio_init(GPIOC,
                 GPIO_PIN_NO_1,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);
    std::array<uint8_t, 6> data_write = {0x31, 0xFF, 0x55, 0xCA, 0xAE, 0x33};
    for(const auto &it : data_write) {
        GPIOC->IDR = it;
        EXPECT_EQ(mc.gpio_read_from_input_port(), it);
    }
}

TEST_F(gpio_test, gpio_write_to_output_pin) {
    mc.gpio_init(GPIOH,
                 GPIO_PIN_NO_1,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);
    std::array<uint8_t, 6> data_write = {SET, RESET, RESET, SET, SET, SET};
    for(const auto &it : data_write) {
        mc.gpio_write_to_output_pin(it);
        EXPECT_EQ((GPIOH->ODR >> GPIO_PIN_NO_1) & 0x01, it);
    }
}

TEST_F(gpio_test, gpio_write_to_output_port) {
    mc.gpio_init(GPIOE,
                 GPIO_PIN_NO_9,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);
    std::array<uint8_t, 6> data_write {0x30, 0x91, 0x78, 0xEE, 0xF0, 0xFF};
    for(const auto &it : data_write) {
        mc.gpio_write_to_output_port(it);
        EXPECT_EQ(GPIOE->ODR, it);
    }
}

TEST_F(gpio_test, gpio_toggle_output_pin) {
    mc.gpio_init(GPIOF,
                 GPIO_PIN_NO_15,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);
    GPIOF->ODR = 0x00000000;
    uint8_t expected = 0;
    for(int i = 0; i < 5; ++i) {
        mc.gpio_toggle_output_pin();
        expected = (~expected & 0x01);
        EXPECT_EQ((GPIOF->ODR >> GPIO_PIN_NO_15) & 0x01, expected);
    }
}

TEST_F(gpio_test, gpio_irq_handling) {
    mc.gpio_init(GPIOF,
                 GPIO_PIN_NO_13,
                 GPIO_MODE_IT_RT,
                 GPIO_SPEED_FAST,
                 GPIO_OP_TYPE_OD,
                 GPIO_PIN_PD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);
    EXTI->PR = 1UL << GPIO_PIN_NO_13;
    mc.gpio_irq_handling();
    EXPECT_EQ((EXTI->PR >> GPIO_PIN_NO_13) & 0x01, 0x01); 
    EXTI->PR = 0UL << GPIO_PIN_NO_13;
    mc.gpio_irq_handling();
    EXPECT_EQ((EXTI->PR >> GPIO_PIN_NO_13) & 0x01, 0x00);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
