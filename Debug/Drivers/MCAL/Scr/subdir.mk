################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCAL/Scr/RCC.c \
../Drivers/MCAL/Scr/stm32_F103C6_EXTI_driver.c \
../Drivers/MCAL/Scr/stm32_F103C6_gpio_driver.c \
../Drivers/MCAL/Scr/stm32f103x6_spi_driver.c \
../Drivers/MCAL/Scr/stm32f103x8_i2c_driver.c \
../Drivers/MCAL/Scr/uart.c 

OBJS += \
./Drivers/MCAL/Scr/RCC.o \
./Drivers/MCAL/Scr/stm32_F103C6_EXTI_driver.o \
./Drivers/MCAL/Scr/stm32_F103C6_gpio_driver.o \
./Drivers/MCAL/Scr/stm32f103x6_spi_driver.o \
./Drivers/MCAL/Scr/stm32f103x8_i2c_driver.o \
./Drivers/MCAL/Scr/uart.o 

C_DEPS += \
./Drivers/MCAL/Scr/RCC.d \
./Drivers/MCAL/Scr/stm32_F103C6_EXTI_driver.d \
./Drivers/MCAL/Scr/stm32_F103C6_gpio_driver.d \
./Drivers/MCAL/Scr/stm32f103x6_spi_driver.d \
./Drivers/MCAL/Scr/stm32f103x8_i2c_driver.d \
./Drivers/MCAL/Scr/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCAL/Scr/%.o Drivers/MCAL/Scr/%.su Drivers/MCAL/Scr/%.cyclo: ../Drivers/MCAL/Scr/%.c Drivers/MCAL/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Core/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/Devices/Motor" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/Devices/Speed" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/Devices/Ultrasonic" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/MCAL/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-MCAL-2f-Scr

clean-Drivers-2f-MCAL-2f-Scr:
	-$(RM) ./Drivers/MCAL/Scr/RCC.cyclo ./Drivers/MCAL/Scr/RCC.d ./Drivers/MCAL/Scr/RCC.o ./Drivers/MCAL/Scr/RCC.su ./Drivers/MCAL/Scr/stm32_F103C6_EXTI_driver.cyclo ./Drivers/MCAL/Scr/stm32_F103C6_EXTI_driver.d ./Drivers/MCAL/Scr/stm32_F103C6_EXTI_driver.o ./Drivers/MCAL/Scr/stm32_F103C6_EXTI_driver.su ./Drivers/MCAL/Scr/stm32_F103C6_gpio_driver.cyclo ./Drivers/MCAL/Scr/stm32_F103C6_gpio_driver.d ./Drivers/MCAL/Scr/stm32_F103C6_gpio_driver.o ./Drivers/MCAL/Scr/stm32_F103C6_gpio_driver.su ./Drivers/MCAL/Scr/stm32f103x6_spi_driver.cyclo ./Drivers/MCAL/Scr/stm32f103x6_spi_driver.d ./Drivers/MCAL/Scr/stm32f103x6_spi_driver.o ./Drivers/MCAL/Scr/stm32f103x6_spi_driver.su ./Drivers/MCAL/Scr/stm32f103x8_i2c_driver.cyclo ./Drivers/MCAL/Scr/stm32f103x8_i2c_driver.d ./Drivers/MCAL/Scr/stm32f103x8_i2c_driver.o ./Drivers/MCAL/Scr/stm32f103x8_i2c_driver.su ./Drivers/MCAL/Scr/uart.cyclo ./Drivers/MCAL/Scr/uart.d ./Drivers/MCAL/Scr/uart.o ./Drivers/MCAL/Scr/uart.su

.PHONY: clean-Drivers-2f-MCAL-2f-Scr

