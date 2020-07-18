################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/src/stm32f407xx_gpio_driver.c \
../driver/src/stm32f407xx_spi_driver.c 

OBJS += \
./driver/src/stm32f407xx_gpio_driver.o \
./driver/src/stm32f407xx_spi_driver.o 

C_DEPS += \
./driver/src/stm32f407xx_gpio_driver.d \
./driver/src/stm32f407xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/src/stm32f407xx_gpio_driver.o: ../driver/src/stm32f407xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"driver/src/stm32f407xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/src/stm32f407xx_spi_driver.o: ../driver/src/stm32f407xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"driver/src/stm32f407xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

