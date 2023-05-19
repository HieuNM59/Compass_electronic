################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user/MPU6050.c \
../user/button.c \
../user/dwt_stm32_delay.c \
../user/flash.c \
../user/main-app.c 

OBJS += \
./user/MPU6050.o \
./user/button.o \
./user/dwt_stm32_delay.o \
./user/flash.o \
./user/main-app.o 

C_DEPS += \
./user/MPU6050.d \
./user/button.d \
./user/dwt_stm32_delay.d \
./user/flash.d \
./user/main-app.d 


# Each subdirectory must supply rules for building sources it contributes
user/%.o user/%.su user/%.cyclo: ../user/%.c user/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/HieuNM/Desktop/Compass_electronic/user" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-user

clean-user:
	-$(RM) ./user/MPU6050.cyclo ./user/MPU6050.d ./user/MPU6050.o ./user/MPU6050.su ./user/button.cyclo ./user/button.d ./user/button.o ./user/button.su ./user/dwt_stm32_delay.cyclo ./user/dwt_stm32_delay.d ./user/dwt_stm32_delay.o ./user/dwt_stm32_delay.su ./user/flash.cyclo ./user/flash.d ./user/flash.o ./user/flash.su ./user/main-app.cyclo ./user/main-app.d ./user/main-app.o ./user/main-app.su

.PHONY: clean-user

