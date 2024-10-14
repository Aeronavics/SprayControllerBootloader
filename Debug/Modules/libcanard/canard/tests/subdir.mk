################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Modules/libcanard/canard/tests/canard_interface.cpp \
../Modules/libcanard/canard/tests/cxx_test_interface.cpp \
../Modules/libcanard/canard/tests/test_canard_interface.cpp \
../Modules/libcanard/canard/tests/test_cxx_wrappers.cpp 

OBJS += \
./Modules/libcanard/canard/tests/canard_interface.o \
./Modules/libcanard/canard/tests/cxx_test_interface.o \
./Modules/libcanard/canard/tests/test_canard_interface.o \
./Modules/libcanard/canard/tests/test_cxx_wrappers.o 

CPP_DEPS += \
./Modules/libcanard/canard/tests/canard_interface.d \
./Modules/libcanard/canard/tests/cxx_test_interface.d \
./Modules/libcanard/canard/tests/test_canard_interface.d \
./Modules/libcanard/canard/tests/test_cxx_wrappers.d 


# Each subdirectory must supply rules for building sources it contributes
Modules/libcanard/canard/tests/%.o Modules/libcanard/canard/tests/%.su Modules/libcanard/canard/tests/%.cyclo: ../Modules/libcanard/canard/tests/%.cpp Modules/libcanard/canard/tests/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-libcanard-2f-canard-2f-tests

clean-Modules-2f-libcanard-2f-canard-2f-tests:
	-$(RM) ./Modules/libcanard/canard/tests/canard_interface.cyclo ./Modules/libcanard/canard/tests/canard_interface.d ./Modules/libcanard/canard/tests/canard_interface.o ./Modules/libcanard/canard/tests/canard_interface.su ./Modules/libcanard/canard/tests/cxx_test_interface.cyclo ./Modules/libcanard/canard/tests/cxx_test_interface.d ./Modules/libcanard/canard/tests/cxx_test_interface.o ./Modules/libcanard/canard/tests/cxx_test_interface.su ./Modules/libcanard/canard/tests/test_canard_interface.cyclo ./Modules/libcanard/canard/tests/test_canard_interface.d ./Modules/libcanard/canard/tests/test_canard_interface.o ./Modules/libcanard/canard/tests/test_canard_interface.su ./Modules/libcanard/canard/tests/test_cxx_wrappers.cyclo ./Modules/libcanard/canard/tests/test_cxx_wrappers.d ./Modules/libcanard/canard/tests/test_cxx_wrappers.o ./Modules/libcanard/canard/tests/test_cxx_wrappers.su

.PHONY: clean-Modules-2f-libcanard-2f-canard-2f-tests

