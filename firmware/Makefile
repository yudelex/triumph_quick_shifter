# Makefile for ATmega48 firmware build
# Target MCU: ATmega48
# Compiler: avr-gcc (WinAVR or standalone AVR toolchain)
# OS: Windows 11 (compatible with cmd, PowerShell, or MSYS2)

# ------ Project Configuration ------
# Target microcontroller
MCU = atmega48

# Target output filename (without extension)
TARGET = main

# Source directory
SRCDIR = src

# Build directory (all generated files go here)
BUILDDIR = build

# Source files (C and assembly)
SRC = main.c
ASRC = 

# Add path to source files
SRC_WITH_PATH = $(addprefix $(SRCDIR)/, $(SRC))
ASRC_WITH_PATH = $(addprefix $(SRCDIR)/, $(ASRC))

# Include directories
INCDIR = ./include

# Library directories (if any)
LIBDIR = 

# Libraries to link (e.g., -lm for math)
LIBS = 

# ------ Compiler and Toolchain Settings ------
# Compiler
CC = avr-gcc

# Assembler (usually same as compiler for AVR)
AS = avr-gcc

# Linker
LD = avr-gcc

# Object copy utility (for creating hex files)
OBJCOPY = avr-objcopy

# Object dump utility (for disassembly)
OBJDUMP = avr-objdump

# Compiler flags: specify MCU, set CPU frequency, include debug symbols, disable optimization
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -ggdb -O0

# Size utility
SIZE = avr-size

# Programmer (AVRDUDE configuration)
AVRDUDE = avrdude
# Programmer type (e.g., usbasp, arduino, jtag2ice)
PROGRAMMER = usbasp
# Port (for serial programmers like Arduino, leave blank for USB)
PORT = usb

# ------ Compiler Flags ------
# Hardware optimizations and standard
CFLAGS = -mmcu=$(MCU) -Os -std=gnu99 -ffunction-sections -fdata-sections

# Warning flags
CFLAGS += -Wall -Werror -Wextra -Wshadow

# Include paths
CFLAGS += $(addprefix -I, $(INCDIR))

# Additional defines (e.g., F_CPU for delay.h)
CFLAGS += -DF_CPU=8000000UL

# ------ Assembler Flags ------
ASFLAGS = -mmcu=$(MCU) -x assembler-with-cpp

# ------ Linker Flags ------
LDFLAGS = -mmcu=$(MCU) -Wl,-Map=$(BUILDDIR)/$(TARGET).map,--cref -Wl,--gc-sections
LDFLAGS += $(addprefix -L, $(LIBDIR))
LDFLAGS += $(LIBS)

# ------ Source and Object File Management ------
# All source files (.c) and assembly files (.S)
ALLSRC = $(SRC_WITH_PATH) $(ASRC_WITH_PATH)

# Object files (replace .c/.S with .o) and add build directory prefix
OBJ = $(addprefix $(BUILDDIR)/, $(notdir $(ALLSRC:.c=.o)))
OBJ := $(OBJ:.S=.o)

# Dependency files (for automatic header tracking) - also in build directory
DEP = $(OBJ:.o=.d)

# Target files in build directory
TARGET_ELF = $(BUILDDIR)/$(TARGET).elf
TARGET_HEX = $(BUILDDIR)/$(TARGET).hex
TARGET_EEP = $(BUILDDIR)/$(TARGET).eep
TARGET_LSS = $(BUILDDIR)/$(TARGET).lss

# ------ Build Targets ------
# Default target: build everything
all: $(TARGET_HEX) $(TARGET_EEP) $(TARGET_LSS) size

# Create build directory if it doesn't exist
$(BUILDDIR):
	mkdir -p $(BUILDDIR)

# Link object files and create ELF
$(TARGET_ELF): $(OBJ) | $(BUILDDIR)
	@echo "Linking: $@"
	$(LD) $(LDFLAGS) -o $@ $^
	@echo ""

# Compile C source files
$(BUILDDIR)/%.o: $(SRCDIR)/%.c | $(BUILDDIR)
	@echo "Compiling: $<"
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
	@echo ""

# Compile assembly source files
$(BUILDDIR)/%.o: $(SRCDIR)/%.S | $(BUILDDIR)
	@echo "Assembling: $<"
	$(AS) $(ASFLAGS) -c $< -o $@
	@echo ""

# Create Intel HEX file for flash memory
$(TARGET_HEX): $(TARGET_ELF) | $(BUILDDIR)
	@echo "Creating HEX: $@"
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	@echo ""

# Create Intel HEX file for EEPROM
$(TARGET_EEP): $(TARGET_ELF) | $(BUILDDIR)
	@echo "Creating EEPROM file: $@"
	$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
		--change-section-lma .eeprom=0 -O ihex $< $@
	@echo ""

# Create listing file (disassembly)
$(TARGET_LSS): $(TARGET_ELF) | $(BUILDDIR)
	@echo "Creating listing: $@"
	$(OBJDUMP) -h -S $< > $@
	@echo ""

# Display memory usage
size: $(TARGET_ELF)
	@echo ""
	@echo "========================================="
	@echo "Memory usage summary:"
	@echo "========================================="
	avr-size $(TARGET_ELF)
	@echo "========================================="
	@echo ""

# ------ Programming with AVRDUDE ------
# Program flash memory (main firmware)
flash: $(TARGET_HEX)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -U flash:w:$(TARGET_HEX):i

# Program EEPROM
eeprom: $(TARGET_EEP)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -U eeprom:w:$(TARGET_EEP):i

# Read fuse bits (example for safety)
fuses_read:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h

# Set fuses (example values - adjust for your project!)
# Low fuse: 0x62 (8 MHz internal RC, startup time 6 ms)
# High fuse: 0xDF (BOD enabled, reset vector at boot)
# Extended fuse: 0xFF (no additional features)
fuses_write:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -U lfuse:w:0x62:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m

# ------ Utility Targets ------
# Clean all generated files
clean:
	@echo "Cleaning..."
	rm -rf $(BUILDDIR)
	@echo "Done."

# Generate dependency files automatically (optional, handled by -MMD)
-include $(DEP)

# Help target
help:
	@echo "Available targets:"
	@echo "  all          - Build firmware (hex, eep, listing)"
	@echo "  flash        - Program flash memory using AVRDUDE"
	@echo "  eeprom       - Program EEPROM memory"
	@echo "  fuses_read   - Read fuse settings"
	@echo "  fuses_write  - Write fuse settings (CAUTION: check values!)"
	@echo "  size         - Show memory usage"
	@echo "  clean        - Remove build directory and all generated files"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Configuration:"
	@echo "  MCU: $(MCU)"
	@echo "  Source directory: $(SRCDIR)"
	@echo "  Build directory: $(BUILDDIR)"
	@echo "  Programmer: $(PROGRAMMER)"
	@echo "  Port: $(PORT)"