################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FreeRTOS/Source/croutine.c \
../Core/FreeRTOS/Source/event_groups.c \
../Core/FreeRTOS/Source/list.c \
../Core/FreeRTOS/Source/queue.c \
../Core/FreeRTOS/Source/stream_buffer.c \
../Core/FreeRTOS/Source/tasks.c \
../Core/FreeRTOS/Source/timers.c 

OBJS += \
./Core/FreeRTOS/Source/croutine.o \
./Core/FreeRTOS/Source/event_groups.o \
./Core/FreeRTOS/Source/list.o \
./Core/FreeRTOS/Source/queue.o \
./Core/FreeRTOS/Source/stream_buffer.o \
./Core/FreeRTOS/Source/tasks.o \
./Core/FreeRTOS/Source/timers.o 

C_DEPS += \
./Core/FreeRTOS/Source/croutine.d \
./Core/FreeRTOS/Source/event_groups.d \
./Core/FreeRTOS/Source/list.d \
./Core/FreeRTOS/Source/queue.d \
./Core/FreeRTOS/Source/stream_buffer.d \
./Core/FreeRTOS/Source/tasks.d \
./Core/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Core/FreeRTOS/Source/%.o Core/FreeRTOS/Source/%.su Core/FreeRTOS/Source/%.cyclo: ../Core/FreeRTOS/Source/%.c Core/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103x6 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source/include" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source/portable/MemMang" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS" -I"/home/guilherme-lazzarini/STM32CubeIDE/workspace_1.19.0/PAY_lora_maldito/Core/FreeRTOS/Source/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-FreeRTOS-2f-Source

clean-Core-2f-FreeRTOS-2f-Source:
	-$(RM) ./Core/FreeRTOS/Source/croutine.cyclo ./Core/FreeRTOS/Source/croutine.d ./Core/FreeRTOS/Source/croutine.o ./Core/FreeRTOS/Source/croutine.su ./Core/FreeRTOS/Source/event_groups.cyclo ./Core/FreeRTOS/Source/event_groups.d ./Core/FreeRTOS/Source/event_groups.o ./Core/FreeRTOS/Source/event_groups.su ./Core/FreeRTOS/Source/list.cyclo ./Core/FreeRTOS/Source/list.d ./Core/FreeRTOS/Source/list.o ./Core/FreeRTOS/Source/list.su ./Core/FreeRTOS/Source/queue.cyclo ./Core/FreeRTOS/Source/queue.d ./Core/FreeRTOS/Source/queue.o ./Core/FreeRTOS/Source/queue.su ./Core/FreeRTOS/Source/stream_buffer.cyclo ./Core/FreeRTOS/Source/stream_buffer.d ./Core/FreeRTOS/Source/stream_buffer.o ./Core/FreeRTOS/Source/stream_buffer.su ./Core/FreeRTOS/Source/tasks.cyclo ./Core/FreeRTOS/Source/tasks.d ./Core/FreeRTOS/Source/tasks.o ./Core/FreeRTOS/Source/tasks.su ./Core/FreeRTOS/Source/timers.cyclo ./Core/FreeRTOS/Source/timers.d ./Core/FreeRTOS/Source/timers.o ./Core/FreeRTOS/Source/timers.su

.PHONY: clean-Core-2f-FreeRTOS-2f-Source

