################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../IMU/Src/IMU.c \
../IMU/Src/Kalman.c \
../IMU/Src/Quaternions.c 

OBJS += \
./IMU/Src/IMU.o \
./IMU/Src/Kalman.o \
./IMU/Src/Quaternions.o 

C_DEPS += \
./IMU/Src/IMU.d \
./IMU/Src/Kalman.d \
./IMU/Src/Quaternions.d 


# Each subdirectory must supply rules for building sources it contributes
IMU/Src/%.o IMU/Src/%.su: ../IMU/Src/%.c IMU/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../IMU -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-IMU-2f-Src

clean-IMU-2f-Src:
	-$(RM) ./IMU/Src/IMU.d ./IMU/Src/IMU.o ./IMU/Src/IMU.su ./IMU/Src/Kalman.d ./IMU/Src/Kalman.o ./IMU/Src/Kalman.su ./IMU/Src/Quaternions.d ./IMU/Src/Quaternions.o ./IMU/Src/Quaternions.su

.PHONY: clean-IMU-2f-Src

