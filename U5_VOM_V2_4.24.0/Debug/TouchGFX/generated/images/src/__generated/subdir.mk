################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_backgrounds_light_filled.cpp \
../TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_needles_smooth.cpp 

OBJS += \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_backgrounds_light_filled.o \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_needles_smooth.o 

CPP_DEPS += \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_backgrounds_light_filled.d \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_needles_smooth.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/generated/images/src/__generated/%.o TouchGFX/generated/images/src/__generated/%.su TouchGFX/generated/images/src/__generated/%.cyclo: ../TouchGFX/generated/images/src/__generated/%.cpp TouchGFX/generated/images/src/__generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U545xx -DUSE_NUCLEO_64 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/STM32U5xx_Nucleo -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -I"C:/Users/Admin/Documents/GitHub/vom_v1/U5_VOM_V2_4.24.0/TouchGFX" -I"C:/Users/Admin/Documents/GitHub/vom_v1/U5_VOM_V2_4.24.0/Core/Src" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated

clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated:
	-$(RM) ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_backgrounds_light_filled.cyclo ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_backgrounds_light_filled.d ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_backgrounds_light_filled.o ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_backgrounds_light_filled.su ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_needles_smooth.cyclo ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_needles_smooth.d ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_needles_smooth.o ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_gauge_medium_needles_smooth.su

.PHONY: clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated

