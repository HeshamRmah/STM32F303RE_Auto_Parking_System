################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCAL/EXT1/ext1.c 

OBJS += \
./Drivers/MCAL/EXT1/ext1.o 

C_DEPS += \
./Drivers/MCAL/EXT1/ext1.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCAL/EXT1/%.o Drivers/MCAL/EXT1/%.su Drivers/MCAL/EXT1/%.cyclo: ../Drivers/MCAL/EXT1/%.c Drivers/MCAL/EXT1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MCAL-2f-EXT1

clean-Drivers-2f-MCAL-2f-EXT1:
	-$(RM) ./Drivers/MCAL/EXT1/ext1.cyclo ./Drivers/MCAL/EXT1/ext1.d ./Drivers/MCAL/EXT1/ext1.o ./Drivers/MCAL/EXT1/ext1.su

.PHONY: clean-Drivers-2f-MCAL-2f-EXT1

