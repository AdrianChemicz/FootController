################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Api/platform/src/subdir.mk
-include Api/core/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: FootController.axf

# Tool invocations
FootController.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"D:\NXP_workspace\lpc_chip_11uxx_lib\Debug" -L"D:\NXP_workspace\nxp_lpcxpresso_11u14_usblib_device\Debug" -Xlinker -Map="FootController.map" -Xlinker --gc-sections -mcpu=cortex-m0 -mthumb -T "FootController_Debug.ld" -o "FootController.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) FootController.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "FootController.axf"; # arm-none-eabi-objcopy -v -O binary "FootController.axf" "FootController.bin" ; # checksum -p LPC11U24_301 -d "FootController.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
