# sample makefile for programming AVR microcontrollers
# be sure to fill up the blank entries before you run this!

# the C source file, without extension
SOURCE = main

# device name
MCU = attiny85

# CPU speed, needed by <util/delay.h>
F_CPU = 8000000UL

# tools
CC = avr-gcc
SIZE = avr-size
OBJ = avr-objcopy
OBJD = avr-objdump
AVRDUDE = avrdude
RM = rm -f

# avr-gcc options
CC_OPT = s
CC_WARN = all
CC_LST = -Wa,-adhlns

# avrdude options
AVRDUDE_CONF = -C /usr/share/arduino/hardware/tools/avrdude.conf -v
AVRDUDE_WRITE_FLASH = -U flash:w:$(SOURCE).hex
AVRDUDE_PROGRAMER = stk500v1
AVRDUDE_PORT = /dev/ttyACM0
BAUD = 19200

# some strings for the UI
STR_BEGIN = Starting Build...
STR_CLEAN = Starting Clean...
STR_PROGRAM = Downloading Code...
STR_END = Done.

# general targets
build:
	$(CC) -mmcu=$(MCU) -W$(CC_WARN) -DF_CPU=$(F_CPU) -O$(CC_OPT) $(CC_LST)=$(SOURCE).lst $(SOURCE).c -o $(SOURCE).elf
	$(OBJD) -S $(SOURCE).elf > $(SOURCE).lss
	$(OBJ) -O ihex $(SOURCE).elf $(SOURCE).hex
size:
	$(SIZE) --mcu=$(MCU) --format=avr $(SOURCE).elf
clean_files:
	$(RM) $(SOURCE).elf
	$(RM) $(SOURCE).lst
	$(RM) $(SOURCE).lss
	$(RM) $(SOURCE).hex
download:
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMER) -p $(MCU) -P $(AVRDUDE_PORT) $(AVRDUDE_CONF) $(AVRDUDE_WRITE_FLASH) -b $(BAUD)
begin_all:
	@echo $(STR_BEGIN)
begin_clean:
	@echo $(STR_CLEAN)
begin_program:
	@echo $(STR_PROGRAM)
end:
	@echo $(STR_END)

# WINAVR targets
all: begin_all build size begin_program download end
clean: begin_clean clean_files end
program: begin_program download end

