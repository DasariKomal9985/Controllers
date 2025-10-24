################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.c 

OBJS += \
./Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.o 

C_DEPS += \
./Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.o: ../Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.c Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/HAL Programs/Interrupt vs Polling/ivsp.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-STM32-2f-HAL-20-Programs-2f-Interrupt-20-vs-20-Polling

clean-Core-2f-Inc-2f-STM32-2f-HAL-20-Programs-2f-Interrupt-20-vs-20-Polling:
	-$(RM) ./Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.cyclo ./Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.d ./Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.o ./Core/Inc/STM32/HAL\ Programs/Interrupt\ vs\ Polling/ivsp.su

.PHONY: clean-Core-2f-Inc-2f-STM32-2f-HAL-20-Programs-2f-Interrupt-20-vs-20-Polling

