################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include MCAL/Scr/subdir.mk
-include HAL/Src/subdir.mk
-include Core/Src/subdir.mk
-include App/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := AdaptiveCruiseControl_1.3
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
AdaptiveCruiseControl_1.3.elf \

MAP_FILES += \
AdaptiveCruiseControl_1.3.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
AdaptiveCruiseControl_1.3.list \


# All Target
all: main-build

# Main-build Target
main-build: AdaptiveCruiseControl_1.3.elf secondary-outputs

# Tool invocations
AdaptiveCruiseControl_1.3.elf AdaptiveCruiseControl_1.3.map: $(OBJS) $(USER_OBJS) D:\DOCUMENTARY\EDU\ TUTORIALS\FastBit\Embedded-C\My-workspace\host\AdaptiveCruiseControl_1.3\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "AdaptiveCruiseControl_1.3.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\DOCUMENTARY\EDU TUTORIALS\FastBit\Embedded-C\My-workspace\host\AdaptiveCruiseControl_1.3\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="AdaptiveCruiseControl_1.3.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

AdaptiveCruiseControl_1.3.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "AdaptiveCruiseControl_1.3.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) AdaptiveCruiseControl_1.3.elf AdaptiveCruiseControl_1.3.list AdaptiveCruiseControl_1.3.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
