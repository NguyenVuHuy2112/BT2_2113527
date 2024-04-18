################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FREERTOS/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./FREERTOS/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./FREERTOS/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
FREERTOS/portable/GCC/ARM_CM4F/%.o FREERTOS/portable/GCC/ARM_CM4F/%.su FREERTOS/portable/GCC/ARM_CM4F/%.cyclo: ../FREERTOS/portable/GCC/ARM_CM4F/%.c FREERTOS/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411xE -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"G:/232_Embedded_System_Programming/BT2_FreeRTOS/Utils" -I"G:/232_Embedded_System_Programming/BT2_FreeRTOS/FREERTOS/portable/GCC/ARM_CM4F" -I"G:/232_Embedded_System_Programming/BT2_FreeRTOS/FREERTOS/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FREERTOS-2f-portable-2f-GCC-2f-ARM_CM4F

clean-FREERTOS-2f-portable-2f-GCC-2f-ARM_CM4F:
	-$(RM) ./FREERTOS/portable/GCC/ARM_CM4F/port.cyclo ./FREERTOS/portable/GCC/ARM_CM4F/port.d ./FREERTOS/portable/GCC/ARM_CM4F/port.o ./FREERTOS/portable/GCC/ARM_CM4F/port.su

.PHONY: clean-FREERTOS-2f-portable-2f-GCC-2f-ARM_CM4F

