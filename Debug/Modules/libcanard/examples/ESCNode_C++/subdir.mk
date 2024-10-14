################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Modules/libcanard/examples/ESCNode_C++/esc_node.cpp 

OBJS += \
./Modules/libcanard/examples/ESCNode_C++/esc_node.o 

CPP_DEPS += \
./Modules/libcanard/examples/ESCNode_C++/esc_node.d 


# Each subdirectory must supply rules for building sources it contributes
Modules/libcanard/examples/ESCNode_C++/%.o Modules/libcanard/examples/ESCNode_C++/%.su Modules/libcanard/examples/ESCNode_C++/%.cyclo: ../Modules/libcanard/examples/ESCNode_C++/%.cpp Modules/libcanard/examples/ESCNode_C++/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-libcanard-2f-examples-2f-ESCNode_C-2b--2b-

clean-Modules-2f-libcanard-2f-examples-2f-ESCNode_C-2b--2b-:
	-$(RM) ./Modules/libcanard/examples/ESCNode_C++/esc_node.cyclo ./Modules/libcanard/examples/ESCNode_C++/esc_node.d ./Modules/libcanard/examples/ESCNode_C++/esc_node.o ./Modules/libcanard/examples/ESCNode_C++/esc_node.su

.PHONY: clean-Modules-2f-libcanard-2f-examples-2f-ESCNode_C-2b--2b-

