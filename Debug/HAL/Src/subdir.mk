################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/Src/motor.c \
../HAL/Src/speed_sensor.c \
../HAL/Src/ultrasonic.c 

OBJS += \
./HAL/Src/motor.o \
./HAL/Src/speed_sensor.o \
./HAL/Src/ultrasonic.o 

C_DEPS += \
./HAL/Src/motor.d \
./HAL/Src/speed_sensor.d \
./HAL/Src/ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/Src/%.o HAL/Src/%.su HAL/Src/%.cyclo: ../HAL/Src/%.c HAL/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/Core/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/MCAL/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/App/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/host/AdaptiveCruiseControl_1.3/HAL/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-HAL-2f-Src

clean-HAL-2f-Src:
	-$(RM) ./HAL/Src/motor.cyclo ./HAL/Src/motor.d ./HAL/Src/motor.o ./HAL/Src/motor.su ./HAL/Src/speed_sensor.cyclo ./HAL/Src/speed_sensor.d ./HAL/Src/speed_sensor.o ./HAL/Src/speed_sensor.su ./HAL/Src/ultrasonic.cyclo ./HAL/Src/ultrasonic.d ./HAL/Src/ultrasonic.o ./HAL/Src/ultrasonic.su

.PHONY: clean-HAL-2f-Src

