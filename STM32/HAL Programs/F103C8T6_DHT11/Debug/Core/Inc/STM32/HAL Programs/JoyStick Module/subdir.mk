################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.c 

OBJS += \
./Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.o 

C_DEPS += \
./Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.o: ../Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.c Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/HAL Programs/JoyStick Module/joystick.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-STM32-2f-HAL-20-Programs-2f-JoyStick-20-Module

clean-Core-2f-Inc-2f-STM32-2f-HAL-20-Programs-2f-JoyStick-20-Module:
	-$(RM) ./Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.cyclo ./Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.d ./Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.o ./Core/Inc/STM32/HAL\ Programs/JoyStick\ Module/joystick.su

.PHONY: clean-Core-2f-Inc-2f-STM32-2f-HAL-20-Programs-2f-JoyStick-20-Module

