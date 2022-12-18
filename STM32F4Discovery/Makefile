# Makefile for simple ARM programming

# We'll pick up the GCC toolchain from the Arduino installation
ARDUINO=/home/john/.arduino15/packages
STLINK_TOOLS=/home/john/Arduino/hardware/Arduino_STM32/tools/linux/stlink

# We need CMSIS from the ST repo STM32CubeF1, which is installed locally
CMSISDIR=/home/john/src/arm/STM32/STM32CubeF4/Drivers/CMSIS

MCU=cortex-m4

CC=$(ARDUINO)/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc
LD=$(ARDUINO)/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc
OC=$(ARDUINO)/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-objcopy
SZ=$(ARDUINO)/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-size
STINFO=$(STLINK_TOOLS)/st-info
STFLASH=$(STLINK_TOOLS)/st-flash

STM32MCU=STM32F407xx
LDSCRIPT=/home/john/src/arm/STM32/STM32CubeF4/Projects/STM32F4-Discovery/Templates/SW4STM32/STM32F4-Discovery/STM32F407VGTx_FLASH.ld
STARTUP=$(CMSISDIR)/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
SYSTEM=$(CMSISDIR)/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c

CFLAGS=-mthumb -c -o $@ -O3 -D$(STM32MCU) -I$(CMSISDIR)/Device/ST/STM32F4xx/Include -I$(CMSISDIR)/Include
LDFLAGS=-mthumb --specs=nosys.specs -o $@ -T$(LDSCRIPT)
OCFLAGS=-R .stack -O binary
SZFLAGS=-B -d
INFOFLAGS=--descr

OBJS=blinky.o
ELFS=$(OBJS:.o=.elf)
BINS=$(OBJS:.o=.bin)

# Default target will compile and link all C sources, but not program anything
all: $(BINS)
.PHONY: all

blinky.bin: blinky.elf
	$(OC) $(OCFLAGS) blinky.elf blinky.bin

blinky.elf: blinky.o startup_stm32f407xx.o system_stm32f4xx.o
	$(LD) -mcpu=$(MCU) $(LDFLAGS) startup_stm32f407xx.o system_stm32f4xx.o blinky.o
	$(SZ) $(SZFLAGS) blinky.elf
	
blinky.o: blinky.c
	$(CC) -mcpu=$(MCU) $(CFLAGS) blinky.c

system_stm32f4xx.o: $(SYSTEM)
	$(CC) -mcpu=$(MCU) $(CFLAGS) $(SYSTEM)

startup_stm32f407xx.o: $(STARTUP)
	$(CC) -mcpu=$(MCU) $(CFLAGS) $(STARTUP)

# Target to invoke the programmer and program the flash memory of the MCU
prog: blinky.bin
	$(STFLASH) write blinky.bin 0x8000000

.PHONY: prog

# Target 'teststlink' will connect to the programmer and read the
# device ID, but not program it
teststlink:
	$(STINFO) $(INFOFLAGS)

.PHONY: teststlink

# Target 'clean' will delete all object files, ELF files, and BIN files
clean:
	-rm -f $(OBJS) $(ELFS) $(BINS) startup_stm32f407xx.o system_stm32f4xx.o

.PHONY: clean

