################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/ChampiCan.cpp \
../Core/Src/ChampiState.cpp \
../Core/Src/MessageRecomposer.cpp \
../Core/Src/SCServo.cpp \
../Core/Src/VL53L4CD_api.cpp \
../Core/Src/VL53L4CD_calibration.cpp \
../Core/Src/main.cpp \
../Core/Src/platform.cpp 

C_SRCS += \
../Core/Src/msgs_can.pb.c \
../Core/Src/pb_common.c \
../Core/Src/pb_decode.c \
../Core/Src/pb_encode.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

C_DEPS += \
./Core/Src/msgs_can.pb.d \
./Core/Src/pb_common.d \
./Core/Src/pb_decode.d \
./Core/Src/pb_encode.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 

OBJS += \
./Core/Src/ChampiCan.o \
./Core/Src/ChampiState.o \
./Core/Src/MessageRecomposer.o \
./Core/Src/SCServo.o \
./Core/Src/VL53L4CD_api.o \
./Core/Src/VL53L4CD_calibration.o \
./Core/Src/main.o \
./Core/Src/msgs_can.pb.o \
./Core/Src/pb_common.o \
./Core/Src/pb_decode.o \
./Core/Src/pb_encode.o \
./Core/Src/platform.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

CPP_DEPS += \
./Core/Src/ChampiCan.d \
./Core/Src/ChampiState.d \
./Core/Src/MessageRecomposer.d \
./Core/Src/SCServo.d \
./Core/Src/VL53L4CD_api.d \
./Core/Src/VL53L4CD_calibration.d \
./Core/Src/main.d \
./Core/Src/platform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ChampiCan.cyclo ./Core/Src/ChampiCan.d ./Core/Src/ChampiCan.o ./Core/Src/ChampiCan.su ./Core/Src/ChampiState.cyclo ./Core/Src/ChampiState.d ./Core/Src/ChampiState.o ./Core/Src/ChampiState.su ./Core/Src/MessageRecomposer.cyclo ./Core/Src/MessageRecomposer.d ./Core/Src/MessageRecomposer.o ./Core/Src/MessageRecomposer.su ./Core/Src/SCServo.cyclo ./Core/Src/SCServo.d ./Core/Src/SCServo.o ./Core/Src/SCServo.su ./Core/Src/VL53L4CD_api.cyclo ./Core/Src/VL53L4CD_api.d ./Core/Src/VL53L4CD_api.o ./Core/Src/VL53L4CD_api.su ./Core/Src/VL53L4CD_calibration.cyclo ./Core/Src/VL53L4CD_calibration.d ./Core/Src/VL53L4CD_calibration.o ./Core/Src/VL53L4CD_calibration.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/msgs_can.pb.cyclo ./Core/Src/msgs_can.pb.d ./Core/Src/msgs_can.pb.o ./Core/Src/msgs_can.pb.su ./Core/Src/pb_common.cyclo ./Core/Src/pb_common.d ./Core/Src/pb_common.o ./Core/Src/pb_common.su ./Core/Src/pb_decode.cyclo ./Core/Src/pb_decode.d ./Core/Src/pb_decode.o ./Core/Src/pb_decode.su ./Core/Src/pb_encode.cyclo ./Core/Src/pb_encode.d ./Core/Src/pb_encode.o ./Core/Src/pb_encode.su ./Core/Src/platform.cyclo ./Core/Src/platform.d ./Core/Src/platform.o ./Core/Src/platform.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

