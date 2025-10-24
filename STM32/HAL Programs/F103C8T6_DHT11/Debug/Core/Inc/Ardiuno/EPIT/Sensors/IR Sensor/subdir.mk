################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.c \
../Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.c 

OBJS += \
./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.o \
./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.o 

C_DEPS += \
./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.d \
./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.o: ../Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.c Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/Ardiuno/EPIT/Sensors/IR Sensor/Ardiuno IR Sensor LCD.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.o: ../Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.c Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/Ardiuno/EPIT/Sensors/IR Sensor/Ardiuno IR Sensor TFT.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-IR-20-Sensor

clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-IR-20-Sensor:
	-$(RM) ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.cyclo ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.d ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.o ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ LCD.su ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.cyclo ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.d ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.o ./Core/Inc/Ardiuno/EPIT/Sensors/IR\ Sensor/Ardiuno\ IR\ Sensor\ TFT.su

.PHONY: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-IR-20-Sensor

