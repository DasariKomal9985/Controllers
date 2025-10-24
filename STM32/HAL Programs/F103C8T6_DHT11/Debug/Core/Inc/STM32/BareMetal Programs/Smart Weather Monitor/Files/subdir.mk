################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.c \
../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.c \
../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.c \
../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.c \
../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.c 

OBJS += \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.o \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.o \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.o \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.o \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.o 

C_DEPS += \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.d \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.d \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.d \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.d \
./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.o: ../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.c Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/BareMetal Programs/Smart Weather Monitor/Files/ADC.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.o: ../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.c Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/BareMetal Programs/Smart Weather Monitor/Files/LCD.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.o: ../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.c Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/BareMetal Programs/Smart Weather Monitor/Files/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.o: ../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.c Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/BareMetal Programs/Smart Weather Monitor/Files/stk.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.o: ../Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.c Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/STM32/BareMetal Programs/Smart Weather Monitor/Files/wifi.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-STM32-2f-BareMetal-20-Programs-2f-Smart-20-Weather-20-Monitor-2f-Files

clean-Core-2f-Inc-2f-STM32-2f-BareMetal-20-Programs-2f-Smart-20-Weather-20-Monitor-2f-Files:
	-$(RM) ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.cyclo ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.d ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.o ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/ADC.su ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.cyclo ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.d ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.o ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/LCD.su ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.cyclo ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.d ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.o ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/main.su ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.cyclo ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.d ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.o ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/stk.su ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.cyclo ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.d ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.o ./Core/Inc/STM32/BareMetal\ Programs/Smart\ Weather\ Monitor/Files/wifi.su

.PHONY: clean-Core-2f-Inc-2f-STM32-2f-BareMetal-20-Programs-2f-Smart-20-Weather-20-Monitor-2f-Files

