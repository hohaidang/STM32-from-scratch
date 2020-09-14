#include "core_cm4.h"
#include "stm32f4xx.h"

/* Simulate HW register memory */
typedef struct {
    GPIO_RegDef_t gpio_a_f_addr;
    GPIO_RegDef_t gpio_b_f_addr;
    GPIO_RegDef_t gpio_c_f_addr;
    GPIO_RegDef_t gpio_d_f_addr;
    GPIO_RegDef_t gpio_e_f_addr;
    GPIO_RegDef_t gpio_f_f_addr;
    GPIO_RegDef_t gpio_g_f_addr;
    GPIO_RegDef_t gpio_h_f_addr;
    RCC_RegDef_t rcc_f_addr;
    EXTI_RegDef_t exti_f_addr;
    SYSCFG_RegDef_t syscfg_f_addr;
    SPI_RegDef_t spi1_f_addr;
    SPI_RegDef_t spi2_f_addr;
    SPI_RegDef_t spi3_f_addr;
    SPI_RegDef_t spi4_f_addr;

    SysTick_Reg_t systick_f_addr;
    NVIC_Reg_t nvic_f_addr;
    SCB_Reg_t scb_f_addr;
} fake_reg_addr;

extern fake_reg_addr reg;

#define GPIOA                       ( &reg.gpio_a_f_addr )
#define GPIOB                       ( &reg.gpio_b_f_addr )
#define GPIOC                       ( &reg.gpio_c_f_addr )
#define GPIOD                       ( &reg.gpio_d_f_addr )
#define GPIOE                       ( &reg.gpio_e_f_addr )
#define GPIOF                       ( &reg.gpio_f_f_addr )
#define GPIOG                       ( &reg.gpio_g_f_addr )
#define GPIOH                       ( &reg.gpio_h_f_addr )
									   
#define RCC                         ( &reg.rcc_f_addr )
#define EXTI                        ( &reg.exti_f_addr )
#define SYSCFG                      ( &reg.syscfg_f_addr )
									   
#define SPI1                        ( &reg.spi1_f_addr )
#define SPI2                        ( &reg.spi2_f_addr )
#define SPI3                        ( &reg.spi3_f_addr )
#define SPI4                        ( &reg.spi4_f_addr )

#define SYSTICK                     ( &reg.systick_f_addr )
#define NVIC                        ( &reg.nvic_f_addr )
#define SCB                         ( &reg.scb_f_addr )

