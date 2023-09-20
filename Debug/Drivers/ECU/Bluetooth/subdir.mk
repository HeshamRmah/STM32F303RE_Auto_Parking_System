################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ECU/Bluetooth/bluetooth.c 

OBJS += \
./Drivers/ECU/Bluetooth/bluetooth.o 

C_DEPS += \
./Drivers/ECU/Bluetooth/bluetooth.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ECU/Bluetooth/%.o Drivers/ECU/Bluetooth/%.su Drivers/ECU/Bluetooth/%.cyclo: ../Drivers/ECU/Bluetooth/%.c Drivers/ECU/Bluetooth/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ECU-2f-Bluetooth

clean-Drivers-2f-ECU-2f-Bluetooth:
	-$(RM) ./Drivers/ECU/Bluetooth/bluetooth.cyclo ./Drivers/ECU/Bluetooth/bluetooth.d ./Drivers/ECU/Bluetooth/bluetooth.o ./Drivers/ECU/Bluetooth/bluetooth.su

.PHONY: clean-Drivers-2f-ECU-2f-Bluetooth

