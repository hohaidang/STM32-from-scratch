#include "driver/stm32f446re_gpio_driver.h"
#include "gtest/gtest.h"

inline u8 CHECK_BIT(uint32_t __REG__, uint32_t __POS__) {
    return static_cast<u8>((__REG__ & (1u << __POS__)) >> __POS__);
}

/* Test Fixtures */
struct gpio_test : public testing::Test {
    gpio_handler mc;
    void SetUp() {
        memset(&reg, 0, sizeof(reg));
        std::cout << "SetUp called\n";
    };
};

TEST_F(gpio_test, gpio_init) {
    u32 expected = 0;
    mc.gpio_init(GPIOA,
                 GPIO_PIN_NO_5,
                 GPIO_MODE_ALTFN,
                 GPIO_SPEED_HIGH,
                 GPIO_OP_TYPE_PP,
                 GPIO_NO_PUPD,
                 IRQ_Prio_NO_14,
                 GPIO_ALT_5);

    expected |= (1U << 0U);
    EXPECT_EQ(RCC->AHB1ENR, expected);
    
    expected = 0x800;
    EXPECT_EQ(GPIOA->MODER, expected);
    EXPECT_EQ(EXTI->FTSR, 0x00); /* no interrupt configure */
    EXPECT_EQ(EXTI->RTSR, 0x00);

    expected = 0xC00;
    EXPECT_EQ(GPIOA->OSPEEDR, expected);

    expected = 0x00;
    EXPECT_EQ(GPIOA->OTYPER, expected);

    expected = 0x00;
    EXPECT_EQ(GPIOA->PUPDR, expected);

    expected = 0x500000;
    EXPECT_EQ(GPIOA->AFR[0], expected);
    EXPECT_EQ(GPIOA->AFR[1], 0x00);

    for(int i = 0; i < sizeof(NVIC->ISER) / sizeof(u32); ++i) {
        EXPECT_EQ(NVIC->ISER[i], 0x00);
    }
    for(int i = 0; i < sizeof(NVIC->IPR) / sizeof(u32); ++i) {
        EXPECT_EQ(NVIC->IPR[i], 0x00);
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
    for(int i = 0; i < sizeof(NVIC->IPR) / sizeof(u32); ++i) {
        if(i != 5) 
            EXPECT_EQ(NVIC->IPR[i], 0x00);
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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}