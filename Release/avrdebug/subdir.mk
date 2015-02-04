################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../avrdebug/avrdebug.c \
../avrdebug/winusb.c 

OBJS += \
./avrdebug/avrdebug.o \
./avrdebug/winusb.o 

C_DEPS += \
./avrdebug/avrdebug.d \
./avrdebug/winusb.d 


# Each subdirectory must supply rules for building sources it contributes
avrdebug/%.o: ../avrdebug/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I/home/iam/Programs/my/MultiProg/usbdrv -I/home/iam/Programs/my/MultiProg -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega8 -DF_CPU=12000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


