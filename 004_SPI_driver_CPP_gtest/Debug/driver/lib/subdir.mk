################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../driver/lib/cpplib.cpp 

OBJS += \
./driver/lib/cpplib.o 

CPP_DEPS += \
./driver/lib/cpplib.d 


# Each subdirectory must supply rules for building sources it contributes
driver/lib/cpplib.o: ../driver/lib/cpplib.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -O3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"driver/lib/cpplib.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

