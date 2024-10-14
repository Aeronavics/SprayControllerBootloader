################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Modules/libcanard/tests/test_crc.cpp \
../Modules/libcanard/tests/test_float16.cpp \
../Modules/libcanard/tests/test_init.cpp \
../Modules/libcanard/tests/test_memory_allocator.cpp \
../Modules/libcanard/tests/test_rxerr.cpp \
../Modules/libcanard/tests/test_scalar_encoding.cpp 

OBJS += \
./Modules/libcanard/tests/test_crc.o \
./Modules/libcanard/tests/test_float16.o \
./Modules/libcanard/tests/test_init.o \
./Modules/libcanard/tests/test_memory_allocator.o \
./Modules/libcanard/tests/test_rxerr.o \
./Modules/libcanard/tests/test_scalar_encoding.o 

CPP_DEPS += \
./Modules/libcanard/tests/test_crc.d \
./Modules/libcanard/tests/test_float16.d \
./Modules/libcanard/tests/test_init.d \
./Modules/libcanard/tests/test_memory_allocator.d \
./Modules/libcanard/tests/test_rxerr.d \
./Modules/libcanard/tests/test_scalar_encoding.d 


# Each subdirectory must supply rules for building sources it contributes
Modules/libcanard/tests/%.o Modules/libcanard/tests/%.su Modules/libcanard/tests/%.cyclo: ../Modules/libcanard/tests/%.cpp Modules/libcanard/tests/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-libcanard-2f-tests

clean-Modules-2f-libcanard-2f-tests:
	-$(RM) ./Modules/libcanard/tests/test_crc.cyclo ./Modules/libcanard/tests/test_crc.d ./Modules/libcanard/tests/test_crc.o ./Modules/libcanard/tests/test_crc.su ./Modules/libcanard/tests/test_float16.cyclo ./Modules/libcanard/tests/test_float16.d ./Modules/libcanard/tests/test_float16.o ./Modules/libcanard/tests/test_float16.su ./Modules/libcanard/tests/test_init.cyclo ./Modules/libcanard/tests/test_init.d ./Modules/libcanard/tests/test_init.o ./Modules/libcanard/tests/test_init.su ./Modules/libcanard/tests/test_memory_allocator.cyclo ./Modules/libcanard/tests/test_memory_allocator.d ./Modules/libcanard/tests/test_memory_allocator.o ./Modules/libcanard/tests/test_memory_allocator.su ./Modules/libcanard/tests/test_rxerr.cyclo ./Modules/libcanard/tests/test_rxerr.d ./Modules/libcanard/tests/test_rxerr.o ./Modules/libcanard/tests/test_rxerr.su ./Modules/libcanard/tests/test_scalar_encoding.cyclo ./Modules/libcanard/tests/test_scalar_encoding.d ./Modules/libcanard/tests/test_scalar_encoding.o ./Modules/libcanard/tests/test_scalar_encoding.su

.PHONY: clean-Modules-2f-libcanard-2f-tests

