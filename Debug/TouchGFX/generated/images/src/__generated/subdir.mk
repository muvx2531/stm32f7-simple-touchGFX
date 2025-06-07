################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_boxprogress_normal_medium.cpp \
../TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_filler_large.cpp \
../TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_round_light.cpp \
../TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_track_medium.cpp \
../TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_textprogress_backgrounds_rounded_light.cpp 

OBJS += \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_boxprogress_normal_medium.o \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_filler_large.o \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_round_light.o \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_track_medium.o \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_textprogress_backgrounds_rounded_light.o 

CPP_DEPS += \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_boxprogress_normal_medium.d \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_filler_large.d \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_round_light.d \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_track_medium.d \
./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_textprogress_backgrounds_rounded_light.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/generated/images/src/__generated/%.o TouchGFX/generated/images/src/__generated/%.su TouchGFX/generated/images/src/__generated/%.cyclo: ../TouchGFX/generated/images/src/__generated/%.cpp TouchGFX/generated/images/src/__generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated

clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated:
	-$(RM) ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_boxprogress_normal_medium.cyclo ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_boxprogress_normal_medium.d ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_boxprogress_normal_medium.o ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_boxprogress_normal_medium.su ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_filler_large.cyclo ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_filler_large.d ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_filler_large.o ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_filler_large.su ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_round_light.cyclo ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_round_light.d ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_round_light.o ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_round_light.su ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_track_medium.cyclo ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_track_medium.d ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_track_medium.o ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_slider_horizontal_thick_track_medium.su ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_textprogress_backgrounds_rounded_light.cyclo ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_textprogress_backgrounds_rounded_light.d ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_textprogress_backgrounds_rounded_light.o ./TouchGFX/generated/images/src/__generated/image_alternate_theme_images_widgets_textprogress_backgrounds_rounded_light.su

.PHONY: clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated

