################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Devices/Speed/speed_driver.c 

OBJS += \
./Drivers/Devices/Speed/speed_driver.o 

C_DEPS += \
./Drivers/Devices/Speed/speed_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Devices/Speed/%.o Drivers/Devices/Speed/%.su Drivers/Devices/Speed/%.cyclo: ../Drivers/Devices/Speed/%.c Drivers/Devices/Speed/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Core/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/MCAL/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/Devices/Motor" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/Devices/Speed" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Drivers/Devices/Ultrasonic" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Devices-2f-Speed

clean-Drivers-2f-Devices-2f-Speed:
	-$(RM) ./Drivers/Devices/Speed/speed_driver.cyclo ./Drivers/Devices/Speed/speed_driver.d ./Drivers/Devices/Speed/speed_driver.o ./Drivers/Devices/Speed/speed_driver.su

.PHONY: clean-Drivers-2f-Devices-2f-Speed

