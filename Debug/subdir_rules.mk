################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/develop/TI/CCS/ti_cgt_arm_llvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/LSM6DSV16X" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/VL53L0X" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/WIT" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/BNO08X_UART_RVC" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/Ultrasonic_GPIO" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/Ultrasonic_Capture" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Hardware_I2C" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Hardware_SPI" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Software_I2C" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Software_SPI" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/MPU6050" -I"D:/develop/TI/mpu6050-oled-hardware-i2c" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Debug" -I"D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/source" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/MSPM0" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/IMU_AHRS" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1685131634: ../mpu6050-oled-hardware-i2c.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/develop/TI/CCS/ccs/utils/sysconfig_1.26.0/sysconfig_cli.bat" -s "D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "D:/develop/TI/mpu6050-oled-hardware-i2c/mpu6050-oled-hardware-i2c.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1685131634 ../mpu6050-oled-hardware-i2c.syscfg
device.opt: build-1685131634
device.cmd.genlibs: build-1685131634
ti_msp_dl_config.c: build-1685131634
ti_msp_dl_config.h: build-1685131634

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/develop/TI/CCS/ti_cgt_arm_llvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/LSM6DSV16X" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/VL53L0X" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/WIT" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/BNO08X_UART_RVC" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/Ultrasonic_GPIO" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/Ultrasonic_Capture" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Hardware_I2C" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Hardware_SPI" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Software_I2C" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Software_SPI" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/MPU6050" -I"D:/develop/TI/mpu6050-oled-hardware-i2c" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Debug" -I"D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/source" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/MSPM0" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/IMU_AHRS" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/develop/TI/CCS/ti_cgt_arm_llvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/LSM6DSV16X" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/VL53L0X" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/WIT" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/BNO08X_UART_RVC" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/Ultrasonic_GPIO" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/Ultrasonic_Capture" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Hardware_I2C" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Hardware_SPI" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Software_I2C" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/OLED_Software_SPI" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/MPU6050" -I"D:/develop/TI/mpu6050-oled-hardware-i2c" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Debug" -I"D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/develop/TI/CCS/mspm0_sdk_2_10_00_04/source" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/MSPM0" -I"D:/develop/TI/mpu6050-oled-hardware-i2c/Drivers/IMU_AHRS" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


