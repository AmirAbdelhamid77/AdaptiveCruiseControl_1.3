################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../main.c 

OBJS += \
./main.o 

C_DEPS += \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o %.su %.cyclo: ../%.c subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Core/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/MCAL/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/App/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/HAL/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean--2e-

clean--2e-:
	-$(RM) ./main.cyclo ./main.d ./main.o ./main.su

.PHONY: clean--2e-

