################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Ardiuno/EPIT/Projects/A_R_R_2.c \
../Core/Inc/Ardiuno/EPIT/Projects/A_R_T_2.c \
../Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_reciever.c \
../Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_transmitter.c \
../Core/Inc/Ardiuno/EPIT/Projects/DC_DC_18_March_2025.c \
../Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid.c \
../Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid_P2.c \
../Core/Inc/Ardiuno/EPIT/Projects/voicecontrol.c 

OBJS += \
./Core/Inc/Ardiuno/EPIT/Projects/A_R_R_2.o \
./Core/Inc/Ardiuno/EPIT/Projects/A_R_T_2.o \
./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_reciever.o \
./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_transmitter.o \
./Core/Inc/Ardiuno/EPIT/Projects/DC_DC_18_March_2025.o \
./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid.o \
./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid_P2.o \
./Core/Inc/Ardiuno/EPIT/Projects/voicecontrol.o 

C_DEPS += \
./Core/Inc/Ardiuno/EPIT/Projects/A_R_R_2.d \
./Core/Inc/Ardiuno/EPIT/Projects/A_R_T_2.d \
./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_reciever.d \
./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_transmitter.d \
./Core/Inc/Ardiuno/EPIT/Projects/DC_DC_18_March_2025.d \
./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid.d \
./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid_P2.d \
./Core/Inc/Ardiuno/EPIT/Projects/voicecontrol.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Ardiuno/EPIT/Projects/%.o Core/Inc/Ardiuno/EPIT/Projects/%.su Core/Inc/Ardiuno/EPIT/Projects/%.cyclo: ../Core/Inc/Ardiuno/EPIT/Projects/%.c Core/Inc/Ardiuno/EPIT/Projects/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Projects

clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Projects:
	-$(RM) ./Core/Inc/Ardiuno/EPIT/Projects/A_R_R_2.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/A_R_R_2.d ./Core/Inc/Ardiuno/EPIT/Projects/A_R_R_2.o ./Core/Inc/Ardiuno/EPIT/Projects/A_R_R_2.su ./Core/Inc/Ardiuno/EPIT/Projects/A_R_T_2.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/A_R_T_2.d ./Core/Inc/Ardiuno/EPIT/Projects/A_R_T_2.o ./Core/Inc/Ardiuno/EPIT/Projects/A_R_T_2.su ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_reciever.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_reciever.d ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_reciever.o ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_reciever.su ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_transmitter.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_transmitter.d ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_transmitter.o ./Core/Inc/Ardiuno/EPIT/Projects/Agriculture_rover_transmitter.su ./Core/Inc/Ardiuno/EPIT/Projects/DC_DC_18_March_2025.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/DC_DC_18_March_2025.d ./Core/Inc/Ardiuno/EPIT/Projects/DC_DC_18_March_2025.o ./Core/Inc/Ardiuno/EPIT/Projects/DC_DC_18_March_2025.su ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid.d ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid.o ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid.su ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid_P2.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid_P2.d ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid_P2.o ./Core/Inc/Ardiuno/EPIT/Projects/Vehicle_to_grid_P2.su ./Core/Inc/Ardiuno/EPIT/Projects/voicecontrol.cyclo ./Core/Inc/Ardiuno/EPIT/Projects/voicecontrol.d ./Core/Inc/Ardiuno/EPIT/Projects/voicecontrol.o ./Core/Inc/Ardiuno/EPIT/Projects/voicecontrol.su

.PHONY: clean-Core-2f-Inc-2f-Ardiuno-2f-EPIT-2f-Projects

