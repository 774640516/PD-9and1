################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/Device_GPIO_I2C.c \
../User/PD_Process.c \
../User/ch32x035_it.c \
../User/main.c \
../User/my_I2C_Device.c \
../User/my_PD_Device.c \
../User/my_SPI_Slave.c \
../User/system_ch32x035.c 

OBJS += \
./User/Device_GPIO_I2C.o \
./User/PD_Process.o \
./User/ch32x035_it.o \
./User/main.o \
./User/my_I2C_Device.o \
./User/my_PD_Device.o \
./User/my_SPI_Slave.o \
./User/system_ch32x035.o 

C_DEPS += \
./User/Device_GPIO_I2C.d \
./User/PD_Process.d \
./User/ch32x035_it.d \
./User/main.d \
./User/my_I2C_Device.d \
./User/my_PD_Device.d \
./User/my_SPI_Slave.d \
./User/system_ch32x035.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"C:\MRS_DATA\workspace\CH32X035G8U6_AIO\Debug" -I"C:\MRS_DATA\workspace\CH32X035G8U6_AIO\Core" -I"C:\MRS_DATA\workspace\CH32X035G8U6_AIO\User" -I"C:\MRS_DATA\workspace\CH32X035G8U6_AIO\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

