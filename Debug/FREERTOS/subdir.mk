################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FREERTOS/croutine.c \
../FREERTOS/event_groups.c \
../FREERTOS/list.c \
../FREERTOS/queue.c \
../FREERTOS/stream_buffer.c \
../FREERTOS/tasks.c \
../FREERTOS/timers.c 

OBJS += \
./FREERTOS/croutine.o \
./FREERTOS/event_groups.o \
./FREERTOS/list.o \
./FREERTOS/queue.o \
./FREERTOS/stream_buffer.o \
./FREERTOS/tasks.o \
./FREERTOS/timers.o 

C_DEPS += \
./FREERTOS/croutine.d \
./FREERTOS/event_groups.d \
./FREERTOS/list.d \
./FREERTOS/queue.d \
./FREERTOS/stream_buffer.d \
./FREERTOS/tasks.d \
./FREERTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
FREERTOS/%.o FREERTOS/%.su FREERTOS/%.cyclo: ../FREERTOS/%.c FREERTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411xE -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"G:/232_Embedded_System_Programming/BT2_FreeRTOS/Utils" -I"G:/232_Embedded_System_Programming/BT2_FreeRTOS/FREERTOS/portable/GCC/ARM_CM4F" -I"G:/232_Embedded_System_Programming/BT2_FreeRTOS/FREERTOS/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FREERTOS

clean-FREERTOS:
	-$(RM) ./FREERTOS/croutine.cyclo ./FREERTOS/croutine.d ./FREERTOS/croutine.o ./FREERTOS/croutine.su ./FREERTOS/event_groups.cyclo ./FREERTOS/event_groups.d ./FREERTOS/event_groups.o ./FREERTOS/event_groups.su ./FREERTOS/list.cyclo ./FREERTOS/list.d ./FREERTOS/list.o ./FREERTOS/list.su ./FREERTOS/queue.cyclo ./FREERTOS/queue.d ./FREERTOS/queue.o ./FREERTOS/queue.su ./FREERTOS/stream_buffer.cyclo ./FREERTOS/stream_buffer.d ./FREERTOS/stream_buffer.o ./FREERTOS/stream_buffer.su ./FREERTOS/tasks.cyclo ./FREERTOS/tasks.d ./FREERTOS/tasks.o ./FREERTOS/tasks.su ./FREERTOS/timers.cyclo ./FREERTOS/timers.d ./FREERTOS/timers.o ./FREERTOS/timers.su

.PHONY: clean-FREERTOS

