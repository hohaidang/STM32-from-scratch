################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f446retx.s 

S_DEPS += \
./Startup/startup_stm32f446retx.d 

OBJS += \
./Startup/startup_stm32f446retx.o 


# Each subdirectory must supply rules for building sources it contributes
Startup/startup_stm32f446retx.o: ../Startup/startup_stm32f446retx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Startup/startup_stm32f446retx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

