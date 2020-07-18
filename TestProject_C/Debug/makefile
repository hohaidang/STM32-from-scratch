################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include driver/src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
TestProject_C.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
TestProject_C.list \

OBJCOPY_BIN += \
TestProject_C.bin \


# All Target
all: TestProject_C.elf secondary-outputs

# Tool invocations
TestProject_C.elf: $(OBJS) $(USER_OBJS) /Users/hohaidang/STM32CubeIDE/workspace_1.3.0/TestProject_C/STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "TestProject_C.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/hohaidang/STM32CubeIDE/workspace_1.3.0/TestProject_C/STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="TestProject_C.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

TestProject_C.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "TestProject_C.list"
	@echo 'Finished building: $@'
	@echo ' '

TestProject_C.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "TestProject_C.bin"
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
