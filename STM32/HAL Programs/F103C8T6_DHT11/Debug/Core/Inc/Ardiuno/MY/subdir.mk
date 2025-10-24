################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Ardiuno/MY/firebasecode.c 

OBJS += \
./Core/Inc/Ardiuno/MY/firebasecode.o 

C_DEPS += \
./Core/Inc/Ardiuno/MY/firebasecode.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Ardiuno/MY/%.o Core/Inc/Ardiuno/MY/%.su Core/Inc/Ardiuno/MY/%.cyclo: ../Core/Inc/Ardiuno/MY/%.c Core/Inc/Ardiuno/MY/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Ardiuno-2f-MY

clean-Core-2f-Inc-2f-Ardiuno-2f-MY:
	-$(RM) ./Core/Inc/Ardiuno/MY/firebasecode.cyclo ./Core/Inc/Ardiuno/MY/firebasecode.d ./Core/Inc/Ardiuno/MY/firebasecode.o ./Core/Inc/Ardiuno/MY/firebasecode.su

.PHONY: clean-Core-2f-Inc-2f-Ardiuno-2f-MY

