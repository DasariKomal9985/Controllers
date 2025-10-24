################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.c 

OBJS += \
./Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.o 

C_DEPS += \
./Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.o: ../Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.c Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/BareMetal Programs/STK Timer/stktimer.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-STM32-2f-BareMetal-20-Programs-2f-STK-20-Timer

clean-Core-2f-Inc-2f-STM32-2f-BareMetal-20-Programs-2f-STK-20-Timer:
	-$(RM) ./Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.cyclo ./Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.d ./Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.o ./Core/Inc/STM32/BareMetal\ Programs/STK\ Timer/stktimer.su

.PHONY: clean-Core-2f-Inc-2f-STM32-2f-BareMetal-20-Programs-2f-STK-20-Timer

