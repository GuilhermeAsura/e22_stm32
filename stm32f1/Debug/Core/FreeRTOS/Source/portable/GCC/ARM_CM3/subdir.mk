################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c 

OBJS += \
./Core/FreeRTOS/Source/portable/GCC/ARM_CM3/port.o 

C_DEPS += \
./Core/FreeRTOS/Source/portable/GCC/ARM_CM3/port.d 


# Each subdirectory must supply rules for building sources it contributes
Core/FreeRTOS/Source/portable/GCC/ARM_CM3/%.o Core/FreeRTOS/Source/portable/GCC/ARM_CM3/%.su Core/FreeRTOS/Source/portable/GCC/ARM_CM3/%.cyclo: ../Core/FreeRTOS/Source/portable/GCC/ARM_CM3/%.c Core/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103x6 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source/include" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source/portable/MemMang" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM3

clean-Core-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM3:
	-$(RM) ./Core/FreeRTOS/Source/portable/GCC/ARM_CM3/port.cyclo ./Core/FreeRTOS/Source/portable/GCC/ARM_CM3/port.d ./Core/FreeRTOS/Source/portable/GCC/ARM_CM3/port.o ./Core/FreeRTOS/Source/portable/GCC/ARM_CM3/port.su

.PHONY: clean-Core-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM3

