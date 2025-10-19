################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ChipDrivers/Src/bme280.c \
../ChipDrivers/Src/ds1307.c \
../ChipDrivers/Src/esp8266_at.c \
../ChipDrivers/Src/lcd_6pin_io.c \
../ChipDrivers/Src/lcd_char.c \
../ChipDrivers/Src/max30100.c \
../ChipDrivers/Src/pca9685pw.c \
../ChipDrivers/Src/pcf8574.c 

C_DEPS += \
./ChipDrivers/Src/bme280.d \
./ChipDrivers/Src/ds1307.d \
./ChipDrivers/Src/esp8266_at.d \
./ChipDrivers/Src/lcd_6pin_io.d \
./ChipDrivers/Src/lcd_char.d \
./ChipDrivers/Src/max30100.d \
./ChipDrivers/Src/pca9685pw.d \
./ChipDrivers/Src/pcf8574.d 

OBJS += \
./ChipDrivers/Src/bme280.o \
./ChipDrivers/Src/ds1307.o \
./ChipDrivers/Src/esp8266_at.o \
./ChipDrivers/Src/lcd_6pin_io.o \
./ChipDrivers/Src/lcd_char.o \
./ChipDrivers/Src/max30100.o \
./ChipDrivers/Src/pca9685pw.o \
./ChipDrivers/Src/pcf8574.o 


# Each subdirectory must supply rules for building sources it contributes
ChipDrivers/Src/%.o ChipDrivers/Src/%.su ChipDrivers/Src/%.cyclo: ../ChipDrivers/Src/%.c ChipDrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../ChipDrivers/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ChipDrivers-2f-Src

clean-ChipDrivers-2f-Src:
	-$(RM) ./ChipDrivers/Src/bme280.cyclo ./ChipDrivers/Src/bme280.d ./ChipDrivers/Src/bme280.o ./ChipDrivers/Src/bme280.su ./ChipDrivers/Src/ds1307.cyclo ./ChipDrivers/Src/ds1307.d ./ChipDrivers/Src/ds1307.o ./ChipDrivers/Src/ds1307.su ./ChipDrivers/Src/esp8266_at.cyclo ./ChipDrivers/Src/esp8266_at.d ./ChipDrivers/Src/esp8266_at.o ./ChipDrivers/Src/esp8266_at.su ./ChipDrivers/Src/lcd_6pin_io.cyclo ./ChipDrivers/Src/lcd_6pin_io.d ./ChipDrivers/Src/lcd_6pin_io.o ./ChipDrivers/Src/lcd_6pin_io.su ./ChipDrivers/Src/lcd_char.cyclo ./ChipDrivers/Src/lcd_char.d ./ChipDrivers/Src/lcd_char.o ./ChipDrivers/Src/lcd_char.su ./ChipDrivers/Src/max30100.cyclo ./ChipDrivers/Src/max30100.d ./ChipDrivers/Src/max30100.o ./ChipDrivers/Src/max30100.su ./ChipDrivers/Src/pca9685pw.cyclo ./ChipDrivers/Src/pca9685pw.d ./ChipDrivers/Src/pca9685pw.o ./ChipDrivers/Src/pca9685pw.su ./ChipDrivers/Src/pcf8574.cyclo ./ChipDrivers/Src/pcf8574.d ./ChipDrivers/Src/pcf8574.o ./ChipDrivers/Src/pcf8574.su

.PHONY: clean-ChipDrivers-2f-Src

