################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../driver/src/bme280_driver.cpp \
../driver/src/stm32f446re_gpio_driver.cpp \
../driver/src/stm32f446re_spi_driver.cpp \
../driver/src/sys_tick_driver.cpp \
../driver/src/timer_driver.cpp 

OBJS += \
./driver/src/bme280_driver.o \
./driver/src/stm32f446re_gpio_driver.o \
./driver/src/stm32f446re_spi_driver.o \
./driver/src/sys_tick_driver.o \
./driver/src/timer_driver.o 

CPP_DEPS += \
./driver/src/bme280_driver.d \
./driver/src/stm32f446re_gpio_driver.d \
./driver/src/stm32f446re_spi_driver.d \
./driver/src/sys_tick_driver.d \
./driver/src/timer_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/src/bme280_driver.o: ../driver/src/bme280_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/src/bme280_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/src/stm32f446re_gpio_driver.o: ../driver/src/stm32f446re_gpio_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/src/stm32f446re_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/src/stm32f446re_spi_driver.o: ../driver/src/stm32f446re_spi_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/src/stm32f446re_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/src/sys_tick_driver.o: ../driver/src/sys_tick_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/src/sys_tick_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
driver/src/timer_driver.o: ../driver/src/timer_driver.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/src/timer_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

