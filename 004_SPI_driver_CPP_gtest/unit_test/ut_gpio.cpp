#include "driver/stm32f446re_gpio_driver.h"
#include "gtest/gtest.h"
using namespace std;

// GPIO_Handler gpio = GPIO_Handler( GPIOA,
//                                   GPIO_PIN_NO_5,
//                                   GPIO_MODE_ALTFN,
//                                   GPIO_SPEED_HIGH,
//                                   GPIO_OP_TYPE_PP,
//                                   GPIO_NO_PUPD,
//                                   IRQ_Prio_NO_15,
//                                   GPIO_ALT_5);

// TEST(test_gpio_class, get_irq_pin_num) {
//     EXPECT_EQ(0, 0);
//     /*Test valid pin number*/
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_0), 6);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_1), 7);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_2), 8);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_3), 9);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_4), 10);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_5), 23);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_6), 23);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_7), 23);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_8), 23);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_9), 23);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_10), 40);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_11), 40);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_12), 40);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_13), 40);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_14), 40);
//     // EXPECT_EQ(gpio.get_irq_pin_num(GPIO_PIN_NO_15), 40);

//     // /*Invalid Pinnumber*/
//     // EXPECT_EQ(gpio.get_irq_pin_num(-1), 0);
//     // EXPECT_EQ(gpio.get_irq_pin_num(-15), 0);
//     // EXPECT_EQ(gpio.get_irq_pin_num(16), 0);
// }

/* Test Fixtures */
struct my_class_test : public testing::Test {
    GPIO_Handler *mc;
    void SetUp() { mc = new GPIO_Handler(GPIOA,
                                        GPIO_PIN_NO_5,
                                        GPIO_MODE_ALTFN,
                                        GPIO_SPEED_HIGH,
                                        GPIO_OP_TYPE_PP,
                                        GPIO_NO_PUPD,
                                        IRQ_Prio_NO_15,
                                        GPIO_ALT_5); }
    void TearDown() { delete mc; }
};


TEST_F(my_class_test, temp1) {
    EXPECT_EQ(mc->get_irq_pin_num(GPIO_PIN_NO_0), 6);
}

TEST( TestName, subtest_2){
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}