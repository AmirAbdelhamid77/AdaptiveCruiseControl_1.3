################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/Scr/RCC.c \
../MCAL/Scr/stm32_F103C6_EXTI_driver.c \
../MCAL/Scr/stm32_F103C6_gpio_driver.c \
../MCAL/Scr/stm32_F103C8T6_timer_driver.c 

OBJS += \
./MCAL/Scr/RCC.o \
./MCAL/Scr/stm32_F103C6_EXTI_driver.o \
./MCAL/Scr/stm32_F103C6_gpio_driver.o \
./MCAL/Scr/stm32_F103C8T6_timer_driver.o 

C_DEPS += \
./MCAL/Scr/RCC.d \
./MCAL/Scr/stm32_F103C6_EXTI_driver.d \
./MCAL/Scr/stm32_F103C6_gpio_driver.d \
./MCAL/Scr/stm32_F103C8T6_timer_driver.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/Scr/%.o MCAL/Scr/%.su MCAL/Scr/%.cyclo: ../MCAL/Scr/%.c MCAL/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Core/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/MCAL/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/App/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/HAL/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL-2f-Scr

clean-MCAL-2f-Scr:
	-$(RM) ./MCAL/Scr/RCC.cyclo ./MCAL/Scr/RCC.d ./MCAL/Scr/RCC.o ./MCAL/Scr/RCC.su ./MCAL/Scr/stm32_F103C6_EXTI_driver.cyclo ./MCAL/Scr/stm32_F103C6_EXTI_driver.d ./MCAL/Scr/stm32_F103C6_EXTI_driver.o ./MCAL/Scr/stm32_F103C6_EXTI_driver.su ./MCAL/Scr/stm32_F103C6_gpio_driver.cyclo ./MCAL/Scr/stm32_F103C6_gpio_driver.d ./MCAL/Scr/stm32_F103C6_gpio_driver.o ./MCAL/Scr/stm32_F103C6_gpio_driver.su ./MCAL/Scr/stm32_F103C8T6_timer_driver.cyclo ./MCAL/Scr/stm32_F103C8T6_timer_driver.d ./MCAL/Scr/stm32_F103C8T6_timer_driver.o ./MCAL/Scr/stm32_F103C8T6_timer_driver.su

.PHONY: clean-MCAL-2f-Scr

