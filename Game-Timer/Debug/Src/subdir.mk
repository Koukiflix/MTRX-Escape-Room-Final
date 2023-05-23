################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/timer_buzzInterrupt.c \
../Src/timer_countInterrupt.c \
../Src/timer_display.c \
../Src/timer_led.c \
../Src/timer_main.c 

OBJS += \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/timer_buzzInterrupt.o \
./Src/timer_countInterrupt.o \
./Src/timer_display.o \
./Src/timer_led.o \
./Src/timer_main.o 

C_DEPS += \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/timer_buzzInterrupt.d \
./Src/timer_countInterrupt.d \
./Src/timer_display.d \
./Src/timer_led.d \
./Src/timer_main.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F303VCTx -DSTM32 -DSTM32F3 -DSTM32F3DISCOVERY -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/timer_buzzInterrupt.d ./Src/timer_buzzInterrupt.o ./Src/timer_buzzInterrupt.su ./Src/timer_countInterrupt.d ./Src/timer_countInterrupt.o ./Src/timer_countInterrupt.su ./Src/timer_display.d ./Src/timer_display.o ./Src/timer_display.su ./Src/timer_led.d ./Src/timer_led.o ./Src/timer_led.su ./Src/timer_main.d ./Src/timer_main.o ./Src/timer_main.su

.PHONY: clean-Src

