################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/libcanard/drivers/stm32/canard_stm32.c 

C_DEPS += \
./Modules/libcanard/drivers/stm32/canard_stm32.d 

OBJS += \
./Modules/libcanard/drivers/stm32/canard_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
Modules/libcanard/drivers/stm32/%.o Modules/libcanard/drivers/stm32/%.su Modules/libcanard/drivers/stm32/%.cyclo: ../Modules/libcanard/drivers/stm32/%.c Modules/libcanard/drivers/stm32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-libcanard-2f-drivers-2f-stm32

clean-Modules-2f-libcanard-2f-drivers-2f-stm32:
	-$(RM) ./Modules/libcanard/drivers/stm32/canard_stm32.cyclo ./Modules/libcanard/drivers/stm32/canard_stm32.d ./Modules/libcanard/drivers/stm32/canard_stm32.o ./Modules/libcanard/drivers/stm32/canard_stm32.su

.PHONY: clean-Modules-2f-libcanard-2f-drivers-2f-stm32

