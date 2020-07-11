################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
001HelloWorld_CPP.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
001HelloWorld_CPP.list \

OBJCOPY_BIN += \
001HelloWorld_CPP.bin \


# All Target
all: 001HelloWorld_CPP.elf secondary-outputs

# Tool invocations
001HelloWorld_CPP.elf: $(OBJS) $(USER_OBJS) /Users/hohaidang/Desktop/Coding/STM32-from-scratch/001HelloWorld_CPP/STM32F446RETX_FLASH.ld
	arm-none-eabi-g++ -o "001HelloWorld_CPP.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/hohaidang/Desktop/Coding/STM32-from-scratch/001HelloWorld_CPP/STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="001HelloWorld_CPP.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

001HelloWorld_CPP.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "001HelloWorld_CPP.list"
	@echo 'Finished building: $@'
	@echo ' '

001HelloWorld_CPP.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "001HelloWorld_CPP.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
