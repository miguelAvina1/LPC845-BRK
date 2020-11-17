################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_acomp.c \
../drivers/fsl_adc.c \
../drivers/fsl_capt.c \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_gpio.c \
../drivers/fsl_i2c.c \
../drivers/fsl_power.c \
../drivers/fsl_reset.c \
../drivers/fsl_swm.c \
../drivers/fsl_usart.c 

OBJS += \
./drivers/fsl_acomp.o \
./drivers/fsl_adc.o \
./drivers/fsl_capt.o \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_gpio.o \
./drivers/fsl_i2c.o \
./drivers/fsl_power.o \
./drivers/fsl_reset.o \
./drivers/fsl_swm.o \
./drivers/fsl_usart.o 

C_DEPS += \
./drivers/fsl_acomp.d \
./drivers/fsl_adc.d \
./drivers/fsl_capt.d \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_gpio.d \
./drivers/fsl_i2c.d \
./drivers/fsl_power.d \
./drivers/fsl_reset.d \
./drivers/fsl_swm.d \
./drivers/fsl_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_LPC845M301JBD48 -DCPU_LPC845M301JBD48_cm0plus -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\drivers" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\CMSIS" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\utilities" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\component\uart" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\device" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\drivers" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\CMSIS" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\utilities" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\component\uart" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\device" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\board" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project\source" -I"C:\Users\migue\OneDrive\Documents\MCUXpressoIDE_11.2.1_4149\workspace_LPC845\LPC845_Project" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


