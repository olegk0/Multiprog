################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include usbdrv/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
MultiProg.lss \

FLASH_IMAGE += \
MultiProg.hex \

EEPROM_IMAGE += \
MultiProg.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: MultiProg.elf secondary-outputs

# Tool invocations
MultiProg.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,MultiProg.map -mmcu=atmega8 -o "MultiProg.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

MultiProg.lss: MultiProg.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S MultiProg.elf  >"MultiProg.lss"
	@echo 'Finished building: $@'
	@echo ' '

MultiProg.hex: MultiProg.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex MultiProg.elf  "MultiProg.hex"
	@echo 'Finished building: $@'
	@echo ' '

MultiProg.eep: MultiProg.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex MultiProg.elf  "MultiProg.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: MultiProg.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega8 MultiProg.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) MultiProg.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
