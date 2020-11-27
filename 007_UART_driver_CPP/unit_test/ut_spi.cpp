#include "stm32f446re_spi_driver.h"
#include "gtest/gtest.h"
#include <unistd.h>
#include <array>

#define SPI_(n) SPI##n

/* Test Fixtures */
struct spi_test : public testing::Test {
    spi_handler spi;
    void SetUp() {
        memset(&reg, 0, sizeof(reg));
    };
};

TEST_F(spi_test, spi_init_ssm_en) {
    spi.spi_init(SPI1,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    EXPECT_EQ(RCC->AHB1ENR, 0x01);
    EXPECT_EQ(GPIOA->MODER, 0x0000A800);
    EXPECT_EQ(GPIOA->OSPEEDR, 0x0000FC00);
    EXPECT_EQ(GPIOA->PUPDR, 0x00000000);
    EXPECT_EQ(GPIOA->OTYPER, 0x00000000);
    EXPECT_EQ(GPIOA->AFR[0], 0x55500000);
    EXPECT_EQ(GPIOA->AFR[1], 0x00000000);
    EXPECT_EQ(RCC->APB2ENR, 0x1000);
    EXPECT_EQ(SPI1->CR1, 0x0304);
    EXPECT_EQ(SPI1->CR2, 0x0000);
}

TEST_F(spi_test, spi_init_ssm_dis) {
    spi.spi_init(SPI1,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_DI);
    EXPECT_EQ(RCC->AHB1ENR, 0x01);
    EXPECT_EQ(GPIOA->MODER, 0x0000AA00);
    EXPECT_EQ(GPIOA->OSPEEDR, 0x0000FF00);
    EXPECT_EQ(GPIOA->PUPDR, 0x00000000);
    EXPECT_EQ(GPIOA->OTYPER, 0x00000000);
    EXPECT_EQ(GPIOA->AFR[0], 0x55550000);
    EXPECT_EQ(GPIOA->AFR[1], 0x00000000);
    EXPECT_EQ(SPI1->CR2, 0x0004);
}

TEST_F(spi_test, spi2_init) {
    spi.spi_init(SPI2,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_HD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    EXPECT_EQ(RCC->APB1ENR, 0x4000);
    EXPECT_EQ(SPI2->CR1, 0x0000C304);
}

TEST_F(spi_test, spi3_init) {
    spi.spi_init(SPI3,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_SIMPLEX_RXONLY,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    EXPECT_EQ(RCC->APB1ENR, 0x8000);
    EXPECT_EQ(SPI3->CR1, 0x0704);
}

TEST_F(spi_test, spi4_init) {
    spi.spi_init(SPI4,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    EXPECT_EQ(RCC->APB2ENR, 0x2000);
}

TEST_F(spi_test, spi1_deinit) {
    spi.spi_init(SPI1,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    RCC->APB2RSTR = 0x1000;
    spi.spi_deinit();
    EXPECT_EQ(RCC->APB2RSTR, 0x0000);
}

TEST_F(spi_test, spi2_deinit) {
    spi.spi_init(SPI2,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    RCC->APB1RSTR = 0x4000;
    spi.spi_deinit();
    EXPECT_EQ(RCC->APB1RSTR, 0x0000);
}

TEST_F(spi_test, spi3_deinit) {
    spi.spi_init(SPI3,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    RCC->APB1RSTR = 0x8000;
    spi.spi_deinit();
    EXPECT_EQ(RCC->APB1RSTR, 0x0000);
}

TEST_F(spi_test, spi4_deinit) {
    spi.spi_init(SPI4,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    RCC->APB2RSTR = 0x2000;
    spi.spi_deinit();
    EXPECT_EQ(RCC->APB2RSTR, 0x0000);
}

TEST_F(spi_test, init_nss_sw) {
    spi.spi_init(SPI1,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    spi.spi_init_nss_sw(GPIOB, GPIO_PIN_NO_6);
    EXPECT_EQ(GPIOB->MODER, 0x00001000);
    EXPECT_EQ(GPIOB->OSPEEDR, 0x00002000);
}

TEST_F(spi_test, spi_transmit_data) {
    spi.spi_init(SPI1,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    std::array<u8, 6> data_trans = {0x33, 0x55, 0x41, 0xFF, 0xFE, 0xC0};
    SPI1->SR = 0x42;
    spi.spi_transmit_data(data_trans.data(), data_trans.size());
    EXPECT_EQ((SPI1->CR1 >> 6) & 0x01, 0x01);
}

TEST_F(spi_test, spi_receive_data) {
    spi.spi_init(SPI1,
                 usleep,
                 SPI_DEVICE_MODE_MASTER,
                 SPI_BUS_CONFIG_FD,
                 SPI_SCLK_SPEED_DIV2,
                 SPI_DFF_8BITS,
                 SPI_CPOL_LOW,
                 SPI_CPHA_LOW,
                 SPI_SSM_EN);
    std::array<u8, 6> data_receive = {0xFF, 0xF3, 0xF1, 0xF7, 0x30, 0x31};
    SPI1->SR = 0x01;
    spi.spi_receive_data(data_receive.data(), 6);
    

    EXPECT_EQ((SPI1->CR1 >> 6) & 0x01, 0x01);
    for(size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(data_receive.at(i), 0x00);
    }
}