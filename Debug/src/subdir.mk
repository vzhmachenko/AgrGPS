################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/algorithm.c \
../src/charQueue.c \
../src/gpio.c \
../src/main.c \
../src/nmea.c \
../src/periph.c \
../src/system_stm32f4xx.c \
../src/vehicle.c 

OBJS += \
./src/algorithm.o \
./src/charQueue.o \
./src/gpio.o \
./src/main.o \
./src/nmea.o \
./src/periph.o \
./src/system_stm32f4xx.o \
./src/vehicle.o 

C_DEPS += \
./src/algorithm.d \
./src/charQueue.d \
./src/gpio.d \
./src/main.d \
./src/nmea.d \
./src/periph.d \
./src/system_stm32f4xx.d \
./src/vehicle.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -I"/home/valentyn/workspace/AgrGPS/CMSIS/core" -I"/home/valentyn/workspace/AgrGPS/CMSIS/device" -I"/home/valentyn/workspace/AgrGPS/inc" -I"/home/valentyn/workspace/AgrGPS/RTOS/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


