################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.c \
../Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.c 

OBJS += \
./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.o \
./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.o 

C_DEPS += \
./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.d \
./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.o: ../Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.c Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno Buzzer LCD.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.o: ../Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.c Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno Buzzer TFT.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-Buzzer

clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-Buzzer:
	-$(RM) ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.cyclo ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.d ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.o ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ LCD.su ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.cyclo ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.d ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.o ./Core/Inc/Ardiuno/EPIT/Sensors/Buzzer/Ardiuno\ Buzzer\ TFT.su

.PHONY: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-Buzzer

