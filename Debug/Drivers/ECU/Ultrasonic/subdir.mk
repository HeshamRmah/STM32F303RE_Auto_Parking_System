################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ECU/Ultrasonic/ultrasonic.c 

OBJS += \
./Drivers/ECU/Ultrasonic/ultrasonic.o 

C_DEPS += \
./Drivers/ECU/Ultrasonic/ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ECU/Ultrasonic/%.o Drivers/ECU/Ultrasonic/%.su Drivers/ECU/Ultrasonic/%.cyclo: ../Drivers/ECU/Ultrasonic/%.c Drivers/ECU/Ultrasonic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ECU-2f-Ultrasonic

clean-Drivers-2f-ECU-2f-Ultrasonic:
	-$(RM) ./Drivers/ECU/Ultrasonic/ultrasonic.cyclo ./Drivers/ECU/Ultrasonic/ultrasonic.d ./Drivers/ECU/Ultrasonic/ultrasonic.o ./Drivers/ECU/Ultrasonic/ultrasonic.su

.PHONY: clean-Drivers-2f-ECU-2f-Ultrasonic

