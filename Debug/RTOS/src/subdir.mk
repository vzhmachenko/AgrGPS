################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RTOS/src/croutine.c \
../RTOS/src/event_groups.c \
../RTOS/src/heap_4.c \
../RTOS/src/list.c \
../RTOS/src/port.c \
../RTOS/src/queue.c \
../RTOS/src/tasks.c \
../RTOS/src/timers.c 

OBJS += \
./RTOS/src/croutine.o \
./RTOS/src/event_groups.o \
./RTOS/src/heap_4.o \
./RTOS/src/list.o \
./RTOS/src/port.o \
./RTOS/src/queue.o \
./RTOS/src/tasks.o \
./RTOS/src/timers.o 

C_DEPS += \
./RTOS/src/croutine.d \
./RTOS/src/event_groups.d \
./RTOS/src/heap_4.d \
./RTOS/src/list.d \
./RTOS/src/port.d \
./RTOS/src/queue.d \
./RTOS/src/tasks.d \
./RTOS/src/timers.d 


# Each subdirectory must supply rules for building sources it contributes
RTOS/src/%.o: ../RTOS/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -I"/home/valentyn/git/AgrGPS/CMSIS/core" -I"/home/valentyn/git/AgrGPS/CMSIS/device" -I"/home/valentyn/git/AgrGPS/inc" -I"/home/valentyn/git/AgrGPS/RTOS/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


