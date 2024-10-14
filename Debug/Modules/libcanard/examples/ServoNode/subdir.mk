################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/libcanard/examples/ServoNode/servo_node.c 

C_DEPS += \
./Modules/libcanard/examples/ServoNode/servo_node.d 

OBJS += \
./Modules/libcanard/examples/ServoNode/servo_node.o 


# Each subdirectory must supply rules for building sources it contributes
Modules/libcanard/examples/ServoNode/%.o Modules/libcanard/examples/ServoNode/%.su Modules/libcanard/examples/ServoNode/%.cyclo: ../Modules/libcanard/examples/ServoNode/%.c Modules/libcanard/examples/ServoNode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-libcanard-2f-examples-2f-ServoNode

clean-Modules-2f-libcanard-2f-examples-2f-ServoNode:
	-$(RM) ./Modules/libcanard/examples/ServoNode/servo_node.cyclo ./Modules/libcanard/examples/ServoNode/servo_node.d ./Modules/libcanard/examples/ServoNode/servo_node.o ./Modules/libcanard/examples/ServoNode/servo_node.su

.PHONY: clean-Modules-2f-libcanard-2f-examples-2f-ServoNode

