################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/acc.c 

OBJS += \
./App/Src/acc.o 

C_DEPS += \
./App/Src/acc.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Core/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/MCAL/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/App/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/HAL/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/acc.cyclo ./App/Src/acc.d ./App/Src/acc.o ./App/Src/acc.su

.PHONY: clean-App-2f-Src

