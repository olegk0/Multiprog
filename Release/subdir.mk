################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hvprog.c \
../isp.c \
../main.c \
../picprog.c \
../serial.c \
../stk500protocol.c \
../timer.c \
../utils.c \
../vreg.c 

OBJS += \
./hvprog.o \
./isp.o \
./main.o \
./picprog.o \
./serial.o \
./stk500protocol.o \
./timer.o \
./utils.o \
./vreg.o 

C_DEPS += \
./hvprog.d \
./isp.d \
./main.d \
./picprog.d \
./serial.d \
./stk500protocol.d \
./timer.d \
./utils.d \
./vreg.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I/home/iam/Programs/my/MultiProg/usbdrv -I/home/iam/Programs/my/MultiProg -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega8 -DF_CPU=12000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


