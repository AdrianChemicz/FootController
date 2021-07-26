################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Descriptors.c \
../src/EEPROM_Driver.c \
../src/FootController.c \
../src/GPIO_Driver.c \
../src/HW_Driver.c \
../src/I2C_Driver.c \
../src/StringGenerateParseModule.c \
../src/cr_startup_lpc11uxx.c \
../src/crp.c \
../src/iap.c \
../src/sysinit.c 

S_SRCS += \
../src/aeabi_romdiv_patch.s 

OBJS += \
./src/Descriptors.o \
./src/EEPROM_Driver.o \
./src/FootController.o \
./src/GPIO_Driver.o \
./src/HW_Driver.o \
./src/I2C_Driver.o \
./src/StringGenerateParseModule.o \
./src/aeabi_romdiv_patch.o \
./src/cr_startup_lpc11uxx.o \
./src/crp.o \
./src/iap.o \
./src/sysinit.o 

C_DEPS += \
./src/Descriptors.d \
./src/EEPROM_Driver.d \
./src/FootController.d \
./src/GPIO_Driver.d \
./src/HW_Driver.d \
./src/I2C_Driver.d \
./src/StringGenerateParseModule.d \
./src/cr_startup_lpc11uxx.d \
./src/crp.d \
./src/iap.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DUSB_DEVICE_ONLY -DMICROCONTROLLER -DNO_BOARD_LIB -D__REDLIB__ -D__LPC11U1X__ -DDEBUG -D__CODE_RED -DDONT_ENABLE_DISABLED_RAMBANKS -DCORE_M0 -D__USE_LPCOPEN -D__LPC11UXX__ -I"D:\NXP_workspace\FootController\inc" -I"D:\NXP_workspace\nxp_lpcxpresso_11u14_usblib_device\Drivers\USB" -I"D:\NXP_workspace\lpc_chip_11uxx_lib\inc" -I"D:\NXP_workspace\FootController\Api\core\inc" -I"D:\NXP_workspace\FootController\Api\platform\inc" -O1 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -D__REDLIB__ -DDEBUG -D__CODE_RED -g3 -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


