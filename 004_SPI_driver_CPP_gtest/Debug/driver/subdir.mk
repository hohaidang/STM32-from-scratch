################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../driver/bme280_driver.cpp \
../driver/stm32f446re_gpio_driver.cpp \
../driver/stm32f446re_spi_driver.cpp \
../driver/sys_tick_driver.cpp 

OBJS += \
./driver/bme280_driver.o \
./driver/stm32f446re_gpio_driver.o \
./driver/stm32f446re_spi_driver.o \
./driver/sys_tick_driver.o 

CPP_DEPS += \
./driver/bme280_driver.d \
./driver/stm32f446re_gpio_driver.d \
./driver/stm32f446re_spi_driver.d \
./driver/sys_tick_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/bme280_driver.o: ../driver/bme280_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/bme280_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/stm32f446re_gpio_driver.o: ../driver/stm32f446re_gpio_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/stm32f446re_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/stm32f446re_spi_driver.o: ../driver/stm32f446re_spi_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/stm32f446re_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/sys_tick_driver.o: ../driver/sys_tick_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/sys_tick_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

