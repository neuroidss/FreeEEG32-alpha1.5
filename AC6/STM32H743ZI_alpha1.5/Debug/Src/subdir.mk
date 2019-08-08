################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ad7779.c \
../Src/main.c \
../Src/platform_drivers.c \
../Src/stm32h7xx_hal_msp.c \
../Src/stm32h7xx_it.c \
../Src/syscalls.c \
../Src/system_stm32h7xx.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/ad7779.o \
./Src/main.o \
./Src/platform_drivers.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_it.o \
./Src/syscalls.o \
./Src/system_stm32h7xx.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/ad7779.d \
./Src/main.d \
./Src/platform_drivers.d \
./Src/stm32h7xx_hal_msp.d \
./Src/stm32h7xx_it.d \
./Src/syscalls.d \
./Src/system_stm32h7xx.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H743xx -I"/home/dmitryneuro/usr/eclipse-stm32-workspace/STM32H743ZI_alpha1.5/Inc" -I"/home/dmitryneuro/usr/eclipse-stm32-workspace/STM32H743ZI_alpha1.5/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/dmitryneuro/usr/eclipse-stm32-workspace/STM32H743ZI_alpha1.5/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/dmitryneuro/usr/eclipse-stm32-workspace/STM32H743ZI_alpha1.5/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/dmitryneuro/usr/eclipse-stm32-workspace/STM32H743ZI_alpha1.5/Drivers/CMSIS/Include" -I"/home/dmitryneuro/usr/eclipse-stm32-workspace/STM32H743ZI_alpha1.5/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/dmitryneuro/usr/eclipse-stm32-workspace/STM32H743ZI_alpha1.5/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


