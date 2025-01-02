################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/logging/logs.c 

OBJS += \
./Drivers/logging/logs.o 

C_DEPS += \
./Drivers/logging/logs.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/logging/%.o Drivers/logging/%.su Drivers/logging/%.cyclo: ../Drivers/logging/%.c Drivers/logging/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/rasmus/STM32CubeIDE/workspace_1.14.0/Modulin/Drivers/logging" -I"/home/rasmus/STM32CubeIDE/workspace_1.14.0/Modulin/Drivers/ssd1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-logging

clean-Drivers-2f-logging:
	-$(RM) ./Drivers/logging/logs.cyclo ./Drivers/logging/logs.d ./Drivers/logging/logs.o ./Drivers/logging/logs.su

.PHONY: clean-Drivers-2f-logging

