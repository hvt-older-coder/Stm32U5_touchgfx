################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.c 

C_DEPS += \
./Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.d 

OBJS += \
./Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32U5xx_Nucleo/%.o Drivers/BSP/STM32U5xx_Nucleo/%.su Drivers/BSP/STM32U5xx_Nucleo/%.cyclo: ../Drivers/BSP/STM32U5xx_Nucleo/%.c Drivers/BSP/STM32U5xx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U545xx -DUSE_NUCLEO_64 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/STM32U5xx_Nucleo -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -I"D:/U5/U5_VOM_V2_4.24.0/TouchGFX" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32U5xx_Nucleo

clean-Drivers-2f-BSP-2f-STM32U5xx_Nucleo:
	-$(RM) ./Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.cyclo ./Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.d ./Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.o ./Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32U5xx_Nucleo

