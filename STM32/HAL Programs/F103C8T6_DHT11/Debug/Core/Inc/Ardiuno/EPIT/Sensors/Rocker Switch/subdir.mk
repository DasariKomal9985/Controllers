################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.c \
../Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.c 

OBJS += \
./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.o \
./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.o 

C_DEPS += \
./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.d \
./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.o: ../Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.c Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/Ardiuno/EPIT/Sensors/Rocker Switch/Ardiuno Rocker Switch LCD.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.o: ../Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.c Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Inc/Ardiuno/EPIT/Sensors/Rocker Switch/Ardiuno Rocker Switch TFT.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-Rocker-20-Switch

clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-Rocker-20-Switch:
	-$(RM) ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.cyclo ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.d ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.o ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ LCD.su ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.cyclo ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.d ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.o ./Core/Inc/Ardiuno/EPIT/Sensors/Rocker\ Switch/Ardiuno\ Rocker\ Switch\ TFT.su

.PHONY: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Sensors-2f-Rocker-20-Switch

