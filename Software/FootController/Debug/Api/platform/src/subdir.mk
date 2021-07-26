################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Api/platform/src/vl53l0x_i2c.c \
../Api/platform/src/vl53l0x_platform.c 

OBJS += \
./Api/platform/src/vl53l0x_i2c.o \
./Api/platform/src/vl53l0x_platform.o 

C_DEPS += \
./Api/platform/src/vl53l0x_i2c.d \
./Api/platform/src/vl53l0x_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Api/platform/src/%.o: ../Api/platform/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DUSB_DEVICE_ONLY -DMICROCONTROLLER -DNO_BOARD_LIB -D__REDLIB__ -D__LPC11U1X__ -DDEBUG -D__CODE_RED -DDONT_ENABLE_DISABLED_RAMBANKS -DCORE_M0 -D__USE_LPCOPEN -D__LPC11UXX__ -I"D:\NXP_workspace\FootController\inc" -I"D:\NXP_workspace\nxp_lpcxpresso_11u14_usblib_device\Drivers\USB" -I"D:\NXP_workspace\lpc_chip_11uxx_lib\inc" -I"D:\NXP_workspace\FootController\Api\core\inc" -I"D:\NXP_workspace\FootController\Api\platform\inc" -O1 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


