################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/mavlink/examples/c/udp_example.c 

C_DEPS += \
./Modules/mavlink/examples/c/udp_example.d 

OBJS += \
./Modules/mavlink/examples/c/udp_example.o 


# Each subdirectory must supply rules for building sources it contributes
Modules/mavlink/examples/c/%.o Modules/mavlink/examples/c/%.su Modules/mavlink/examples/c/%.cyclo: ../Modules/mavlink/examples/c/%.c Modules/mavlink/examples/c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-mavlink-2f-examples-2f-c

clean-Modules-2f-mavlink-2f-examples-2f-c:
	-$(RM) ./Modules/mavlink/examples/c/udp_example.cyclo ./Modules/mavlink/examples/c/udp_example.d ./Modules/mavlink/examples/c/udp_example.o ./Modules/mavlink/examples/c/udp_example.su

.PHONY: clean-Modules-2f-mavlink-2f-examples-2f-c

