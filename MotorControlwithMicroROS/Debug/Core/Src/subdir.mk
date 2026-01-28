################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/STLogo.c \
../Core/Src/custom_memory_manager.c \
../Core/Src/dma.c \
../Core/Src/dma_transport.c \
../Core/Src/encoder.c \
../Core/Src/encoder_speed.c \
../Core/Src/fmc.c \
../Core/Src/font12.c \
../Core/Src/font16.c \
../Core/Src/font20.c \
../Core/Src/font24.c \
../Core/Src/font8.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/ili9341.c \
../Core/Src/main.c \
../Core/Src/microros_allocators.c \
../Core/Src/microros_time.c \
../Core/Src/motion_profile.c \
../Core/Src/motor.c \
../Core/Src/odometry.c \
../Core/Src/pid.c \
../Core/Src/pid_params_flash.c \
../Core/Src/ps2xlib.c \
../Core/Src/robot_params.c \
../Core/Src/spi.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_hal_timebase_tim.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/tim.c \
../Core/Src/touch_event.c \
../Core/Src/touch_rtos.c \
../Core/Src/usart.c \
../Core/Src/xpt2046.c 

OBJS += \
./Core/Src/STLogo.o \
./Core/Src/custom_memory_manager.o \
./Core/Src/dma.o \
./Core/Src/dma_transport.o \
./Core/Src/encoder.o \
./Core/Src/encoder_speed.o \
./Core/Src/fmc.o \
./Core/Src/font12.o \
./Core/Src/font16.o \
./Core/Src/font20.o \
./Core/Src/font24.o \
./Core/Src/font8.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/ili9341.o \
./Core/Src/main.o \
./Core/Src/microros_allocators.o \
./Core/Src/microros_time.o \
./Core/Src/motion_profile.o \
./Core/Src/motor.o \
./Core/Src/odometry.o \
./Core/Src/pid.o \
./Core/Src/pid_params_flash.o \
./Core/Src/ps2xlib.o \
./Core/Src/robot_params.o \
./Core/Src/spi.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_hal_timebase_tim.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/tim.o \
./Core/Src/touch_event.o \
./Core/Src/touch_rtos.o \
./Core/Src/usart.o \
./Core/Src/xpt2046.o 

C_DEPS += \
./Core/Src/STLogo.d \
./Core/Src/custom_memory_manager.d \
./Core/Src/dma.d \
./Core/Src/dma_transport.d \
./Core/Src/encoder.d \
./Core/Src/encoder_speed.d \
./Core/Src/fmc.d \
./Core/Src/font12.d \
./Core/Src/font16.d \
./Core/Src/font20.d \
./Core/Src/font24.d \
./Core/Src/font8.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/ili9341.d \
./Core/Src/main.d \
./Core/Src/microros_allocators.d \
./Core/Src/microros_time.d \
./Core/Src/motion_profile.d \
./Core/Src/motor.d \
./Core/Src/odometry.d \
./Core/Src/pid.d \
./Core/Src/pid_params_flash.d \
./Core/Src/ps2xlib.d \
./Core/Src/robot_params.d \
./Core/Src/spi.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_hal_timebase_tim.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/tim.d \
./Core/Src/touch_event.d \
./Core/Src/touch_rtos.d \
./Core/Src/usart.d \
./Core/Src/xpt2046.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F756xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32CubeIDE/workspace_2.0.0/MotorControlwithMicroROS/Drivers/CMSIS/RTOS2/Source" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/STLogo.cyclo ./Core/Src/STLogo.d ./Core/Src/STLogo.o ./Core/Src/STLogo.su ./Core/Src/custom_memory_manager.cyclo ./Core/Src/custom_memory_manager.d ./Core/Src/custom_memory_manager.o ./Core/Src/custom_memory_manager.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/dma_transport.cyclo ./Core/Src/dma_transport.d ./Core/Src/dma_transport.o ./Core/Src/dma_transport.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/encoder_speed.cyclo ./Core/Src/encoder_speed.d ./Core/Src/encoder_speed.o ./Core/Src/encoder_speed.su ./Core/Src/fmc.cyclo ./Core/Src/fmc.d ./Core/Src/fmc.o ./Core/Src/fmc.su ./Core/Src/font12.cyclo ./Core/Src/font12.d ./Core/Src/font12.o ./Core/Src/font12.su ./Core/Src/font16.cyclo ./Core/Src/font16.d ./Core/Src/font16.o ./Core/Src/font16.su ./Core/Src/font20.cyclo ./Core/Src/font20.d ./Core/Src/font20.o ./Core/Src/font20.su ./Core/Src/font24.cyclo ./Core/Src/font24.d ./Core/Src/font24.o ./Core/Src/font24.su ./Core/Src/font8.cyclo ./Core/Src/font8.d ./Core/Src/font8.o ./Core/Src/font8.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/ili9341.cyclo ./Core/Src/ili9341.d ./Core/Src/ili9341.o ./Core/Src/ili9341.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/microros_allocators.cyclo ./Core/Src/microros_allocators.d ./Core/Src/microros_allocators.o ./Core/Src/microros_allocators.su ./Core/Src/microros_time.cyclo ./Core/Src/microros_time.d ./Core/Src/microros_time.o ./Core/Src/microros_time.su ./Core/Src/motion_profile.cyclo ./Core/Src/motion_profile.d ./Core/Src/motion_profile.o ./Core/Src/motion_profile.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/odometry.cyclo ./Core/Src/odometry.d ./Core/Src/odometry.o ./Core/Src/odometry.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pid_params_flash.cyclo ./Core/Src/pid_params_flash.d ./Core/Src/pid_params_flash.o ./Core/Src/pid_params_flash.su ./Core/Src/ps2xlib.cyclo ./Core/Src/ps2xlib.d ./Core/Src/ps2xlib.o ./Core/Src/ps2xlib.su ./Core/Src/robot_params.cyclo ./Core/Src/robot_params.d ./Core/Src/robot_params.o ./Core/Src/robot_params.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_hal_timebase_tim.cyclo ./Core/Src/stm32f7xx_hal_timebase_tim.d ./Core/Src/stm32f7xx_hal_timebase_tim.o ./Core/Src/stm32f7xx_hal_timebase_tim.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/touch_event.cyclo ./Core/Src/touch_event.d ./Core/Src/touch_event.o ./Core/Src/touch_event.su ./Core/Src/touch_rtos.cyclo ./Core/Src/touch_rtos.d ./Core/Src/touch_rtos.o ./Core/Src/touch_rtos.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/xpt2046.cyclo ./Core/Src/xpt2046.d ./Core/Src/xpt2046.o ./Core/Src/xpt2046.su

.PHONY: clean-Core-2f-Src

