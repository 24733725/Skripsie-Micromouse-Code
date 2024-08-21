################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/diskio.c \
C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/ff.c \
C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/FatFs/diskio.o \
./Middlewares/FatFs/ff.o \
./Middlewares/FatFs/ff_gen_drv.o \
./Middlewares/FatFs/syscall.o 

C_DEPS += \
./Middlewares/FatFs/diskio.d \
./Middlewares/FatFs/ff.d \
./Middlewares/FatFs/ff_gen_drv.d \
./Middlewares/FatFs/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FatFs/diskio.o: C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/diskio.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/diskio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff.o: C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/ff.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/ff.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff_gen_drv.o: C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/ff_gen_drv.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/syscall.o: C:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src/option/syscall.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -IC:/Users/jaron/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FatFs/syscall.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-FatFs

clean-Middlewares-2f-FatFs:
	-$(RM) ./Middlewares/FatFs/diskio.d ./Middlewares/FatFs/diskio.o ./Middlewares/FatFs/diskio.su ./Middlewares/FatFs/ff.d ./Middlewares/FatFs/ff.o ./Middlewares/FatFs/ff.su ./Middlewares/FatFs/ff_gen_drv.d ./Middlewares/FatFs/ff_gen_drv.o ./Middlewares/FatFs/ff_gen_drv.su ./Middlewares/FatFs/syscall.d ./Middlewares/FatFs/syscall.o ./Middlewares/FatFs/syscall.su

.PHONY: clean-Middlewares-2f-FatFs

