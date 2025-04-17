################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/RCC.c \
../Drivers/stm32_F103C6_EXTI_driver.c \
../Drivers/stm32_F103C6_gpio_driver.c \
../Drivers/stm32f103x6_spi_driver.c \
../Drivers/stm32f103x8_i2c_driver.c \
../Drivers/uart.c 

OBJS += \
./Drivers/RCC.o \
./Drivers/stm32_F103C6_EXTI_driver.o \
./Drivers/stm32_F103C6_gpio_driver.o \
./Drivers/stm32f103x6_spi_driver.o \
./Drivers/stm32f103x8_i2c_driver.o \
./Drivers/uart.o 

C_DEPS += \
./Drivers/RCC.d \
./Drivers/stm32_F103C6_EXTI_driver.d \
./Drivers/stm32_F103C6_gpio_driver.d \
./Drivers/stm32f103x6_spi_driver.d \
./Drivers/stm32f103x8_i2c_driver.d \
./Drivers/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o Drivers/%.su Drivers/%.cyclo: ../Drivers/%.c Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/configuring/Drivers/inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/configuring/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers

clean-Drivers:
	-$(RM) ./Drivers/RCC.cyclo ./Drivers/RCC.d ./Drivers/RCC.o ./Drivers/RCC.su ./Drivers/stm32_F103C6_EXTI_driver.cyclo ./Drivers/stm32_F103C6_EXTI_driver.d ./Drivers/stm32_F103C6_EXTI_driver.o ./Drivers/stm32_F103C6_EXTI_driver.su ./Drivers/stm32_F103C6_gpio_driver.cyclo ./Drivers/stm32_F103C6_gpio_driver.d ./Drivers/stm32_F103C6_gpio_driver.o ./Drivers/stm32_F103C6_gpio_driver.su ./Drivers/stm32f103x6_spi_driver.cyclo ./Drivers/stm32f103x6_spi_driver.d ./Drivers/stm32f103x6_spi_driver.o ./Drivers/stm32f103x6_spi_driver.su ./Drivers/stm32f103x8_i2c_driver.cyclo ./Drivers/stm32f103x8_i2c_driver.d ./Drivers/stm32f103x8_i2c_driver.o ./Drivers/stm32f103x8_i2c_driver.su ./Drivers/uart.cyclo ./Drivers/uart.d ./Drivers/uart.o ./Drivers/uart.su

.PHONY: clean-Drivers

