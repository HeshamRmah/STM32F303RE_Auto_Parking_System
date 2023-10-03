################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ECU/IR/ecu_ir.c 

OBJS += \
./Drivers/ECU/IR/ecu_ir.o 

C_DEPS += \
./Drivers/ECU/IR/ecu_ir.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ECU/IR/%.o Drivers/ECU/IR/%.su Drivers/ECU/IR/%.cyclo: ../Drivers/ECU/IR/%.c Drivers/ECU/IR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ECU-2f-IR

clean-Drivers-2f-ECU-2f-IR:
	-$(RM) ./Drivers/ECU/IR/ecu_ir.cyclo ./Drivers/ECU/IR/ecu_ir.d ./Drivers/ECU/IR/ecu_ir.o ./Drivers/ECU/IR/ecu_ir.su

.PHONY: clean-Drivers-2f-ECU-2f-IR

