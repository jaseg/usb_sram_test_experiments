# put your *.o targets here, make should handle the rest!
CUBE_PATH		?= $(HOME)/resource/STM32CubeF1
CMSIS_PATH 		?= $(CUBE_PATH)/Drivers/CMSIS
CMSIS_DEV_PATH  	?= $(CMSIS_PATH)/Device/ST/STM32F1xx
HAL_PATH   		?= $(CUBE_PATH)/Drivers/STM32F1xx_HAL_Driver

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

CFLAGS  = -Wall -Wextra -g -std=gnu11 -O2
CFLAGS += -mlittle-endian -mcpu=cortex-m3 -mthumb
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections 
CFLAGS += -Wl,-Map=main.map

CFLAGS += -DSTM32F103xE -DHSE_VALUE=72000000

CFLAGS += -Tstm32_flash.ld
CFLAGS += -I$(CMSIS_DEV_PATH)/Include -I$(CMSIS_PATH)/Include -I$(HAL_PATH)/Inc -Iconfig
CFLAGS += -L$(CMSIS_PATH)/Lib/GCC -larm_cortexM3l_math

###################################################

.PHONY: program clean

all: main.elf

cmsis_exports.c: $(CMSIS_DEV_PATH)/Include/stm32f103xe.h $(CMSIS_PATH)/Include/core_cm3.h
	python3 gen_cmsis_exports.py $^ > $@

sources.tar.xz: main.c serial.h global.h serial.c Makefile
	tar -cf $@ $^

# don't ask...
sources.tar.xz.zip: sources.tar.xz
	zip $@ $^

sources.c: sources.tar.xz.zip
	xxd -i $< | head -n -1 | sed 's/=/__attribute__((section(".source_tarball"))) =/' > $@

main.elf: main.c startup_stm32f103xe.s system_stm32f1xx.c $(HAL_PATH)/Src/stm32f1xx_ll_utils.c cmsis_exports.c sources.c
	$(CC) $(CFLAGS) -o $@ $^
	$(OBJCOPY) -O ihex $@ $(@:.elf=.hex)
	$(OBJCOPY) -O binary $@ $(@:.elf=.bin)
	$(OBJDUMP) -St $@ >$(@:.elf=.lst)
	$(SIZE) $@
	
program: main.elf openocd.cfg
	openocd -f openocd.cfg -c "program $< verify reset exit"

clean:
	rm -f *.o
	rm -f main.elf main.hex main.bin main.map main.lst
	rm -f sources.tar.xz sources.tar.xz.zip sources.c
	rm -f cmsis_exports.c
	rm -rf __pycache__

