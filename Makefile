 #####
 # Project: marquee
 #
 #
 # Makefile for marquee project
 #
 # Stephen Caudle, Copyright (C) 2010.
 #
 #
 # This program is free software; you can redistribute it and/or modify 
 # it under the terms of the GNU General Public License as published by 
 # the Free Software Foundation; either version 3 of the License, or 
 # (at your option) any later version.
 #
 # This program is distributed in the hope that it will be useful, but 
 # WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 # or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 # for more details.
 #
 # You should have received a copy of the GNU General Public License along 
 # with this program; if not, write to the Free Software Foundation, Inc., 
 # 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 #####


# Set developer code and compile options
# Set to YES for debugging
DEBUG = YES
# Set to YES when using Code Sourcery toolchain
CODE_SOURCERY=YES

# Toolchain prefix (i.e arm-elf- -> arm-elf-gcc.exe)
TCHAIN_PREFIX = arm-none-eabi-

REMOVE_CMD = rm

FLASH_TOOL = OPENOCD
#RTOS = FREERTOS

# YES enables -mthumb option to flags for source-files listed 
# in SRC and CPPSRC
USE_THUMB_MODE = YES

# MCU name, submodel and board
# - MCU used for compiler-option (-mcpu)
# - MODEL used for linker-script name (-T) and passed as define
# - BOARD just passed as define (optional)
MCU      = cortex-m3
CHIP     = stm32f100rb
MODEL    = md_vl
BOARD    = STM32Discovery
F_XTAL   = 8000000
SYSCLOCK_CL = SYSCLK_FREQ_24MHz=24000000

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.)
OUTDIR = build

# Target file name (without extension).
TARGET = template

# Paths
COMMONDIR = common
STM32DIR = stm32
STMLIBDIR = $(STM32DIR)/Libraries
STMSPDDIR = $(STMLIBDIR)/STM32F10x_StdPeriph_Driver
STMSPDSRCDIR = $(STMSPDDIR)/src
STMSPDINCDIR = $(STMSPDDIR)/inc
CMSISDIR  = $(STMLIBDIR)/CMSIS/CM3
RTOSDIR = freertos
RTOSSRCDIR = $(RTOSDIR)/FreeRTOS/Source
RTOSINCDIR = $(RTOSSRCDIR)/include
#DOXYGENDIR = doc/doxygen

# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files

## MAIN:
SRC =  main.c

## COMMON:
#SRC =  $(COMMONDIR)/tprintf.c

## CMSIS for STM32
SRC += $(CMSISDIR)/CoreSupport/core_cm3.c
SRC += $(CMSISDIR)/DeviceSupport/ST/STM32F10x/system_stm32f10x.c

## Used parts of the STM-Library
#SRC += $(STMSPDSRCDIR)/stm32f10x_adc.c
SRC += $(STMSPDSRCDIR)/stm32f10x_bkp.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_crc.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_dac.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_dma.c
SRC += $(STMSPDSRCDIR)/stm32f10x_exti.c
SRC += $(STMSPDSRCDIR)/stm32f10x_flash.c
SRC += $(STMSPDSRCDIR)/stm32f10x_gpio.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_i2c.c
SRC += $(STMSPDSRCDIR)/stm32f10x_pwr.c
SRC += $(STMSPDSRCDIR)/stm32f10x_rcc.c
SRC += $(STMSPDSRCDIR)/stm32f10x_rtc.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_spi.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_tim.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_usart.c
SRC += $(STMSPDSRCDIR)/misc.c

## RTOS
ifeq ($(RTOS),FREERTOS)
SRC += $(RTOSSRCDIR)/list.c
SRC += $(RTOSSRCDIR)/queue.c
SRC += $(RTOSSRCDIR)/tasks.c

## RTOS Portable
SRC += $(RTOSSRCDIR)/portable/GCC/ARM_CM3/port.c
SRC += $(RTOSSRCDIR)/portable/MemMang/heap_2.c
endif

# List C source files here which must be compiled in ARM-Mode (no -mthumb).
# use file-extension c for "c-only"-files
## just for testing, timer.c could be compiled in thumb-mode too
SRCARM = 

# List C++ source files here.
# use file-extension .cpp for C++-files (not .C)
CPPSRC = 

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension .cpp for C++-files (not .C)
#CPPSRCARM = $(TARGET).cpp
CPPSRCARM = 

# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = $(CMSISDIR)/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_$(MODEL).s

# List Assembler source files here which must be assembled in ARM-Mode..
ASRCARM = 

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
EXTRAINCDIRS  += $(COMMONDIR)
EXTRAINCDIRS  += $(CMSISDIR)/CoreSupport
EXTRAINCDIRS  += $(CMSISDIR)/CoreSupport
EXTRAINCDIRS  += $(CMSISDIR)/DeviceSupport/ST/STM32F10x
EXTRAINCDIRS  += $(STM32DIR)
EXTRAINCDIRS  += $(STMSPDINCDIR)
EXTRAINCDIRS  += $(RTOSDIR)
EXTRAINCDIRS  += $(RTOSINCDIR)
EXTRAINCDIRS  += $(RTOSSRCDIR)/portable/GCC/ARM_CM3

# List any extra directories to look for library files here.
# Also add directories where the linker should search for
# includes from linker-script to the list
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS = 

# Extra Libraries
#    Each library-name must be seperated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a: 
#    EXTRA_LIBS = xyz abc efsl
# for newlib-lpc (file: libnewlibc-lpc.a):
#    EXTRA_LIBS = newlib-lpc
EXTRA_LIBS =

# Path to Linker-Scripts
LINKERSCRIPTPATH = $(STM32DIR)

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)

ifeq ($(DEBUG),YES)
OPT = 0
else
OPT = s
endif

# Output format. (can be ihex or binary or both)
#  binary to create a load-image in raw-binary format i.e. for SAM-BA, 
#  ihex to create a load-image in Intel hex format
#LOADFORMAT = ihex
LOADFORMAT = binary
#LOADFORMAT = both

# Debugging format.
DEBUGF = gdb

# Place project-specific -D (define) and/or 
# -U options for C here.
CDEFS = -DSTM32F10X_$(MODEL,UC)
CDEFS += -DUSE_STDPERIPH_DRIVER
CDEFS += -DUSE_$(BOARD)
CDEFS += -DHSE_VALUE=$(F_XTAL)UL
CDEFS += -D$(SYSCLOCK_CL)


# Place project-specific -D and/or -U options for 
# Assembler with preprocessor here.
#ADEFS = -DUSE_IRQ_ASM_WRAPPER
ADEFS = -D__ASSEMBLY__

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

#-----

# Compiler flags.

#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)

ifeq ($(DEBUG),YES)
CFLAGS = -g$(DEBUGF)
endif

CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU) -mthumb
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.

CFLAGS += -mapcs-frame 
CFLAGS += -fomit-frame-pointer
ifeq ($(CODE_SOURCERY), YES)
CFLAGS += -fpromote-loop-indices
CFLAGS += -fno-strict-aliasing
endif

CFLAGS += -Wall
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
#CFLAGS += -MD -MP -MF $(OUTDIR)/dep/$(@F).d

# flags only for C
#CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
ASFLAGS  = -mcpu=$(MCU) -mthumb -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -nostartfiles -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref,--gc-sections
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += -lc
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))
LDFLAGS += $(MATH_LIB)
LDFLAGS += -lc -lgcc

# Set linker-script name depending on selected submodel name
LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP).ld

# ---------------------------------------------------------------------------
# Options for OpenOCD flash-programming
# see openocd.pdf/openocd.texi for further information
#
OOCD_LOADFILE+=$(OUTDIR)/$(TARGET).elf
# if OpenOCD is in the $PATH just set OPENOCDEXE=openocd
OOCD_EXE=/home/doceme/sandbox/openocd/bin/openocd
# debug level
OOCD_CL=-d0
# interface and board/target settings (using the OOCD target-library here)
OOCD_CL+=-f openocd/openocd.cfg
# initialize
OOCD_CL+=-c init
# show the targets
OOCD_CL+=-c targets
# commands to prepare flash-write
OOCD_CL+= -c "reset halt"
# flash erase 
OOCD_CL+=-c "stm32x mass_erase 0"
# flash-write
OOCD_CL+=-c "flash write_image $(OOCD_LOADFILE)"
# Verify
OOCD_CL+=-c "verify_image $(OOCD_LOADFILE)"
# reset target
OOCD_CL+=-c "reset run"
# terminate OOCD after programming
OOCD_CL+=-c shutdown
# ---------------------------------------------------------------------------


# Define programs and commands.
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
REMOVE  = $(REMOVE_CMD) -rf
###SHELL   = sh
###COPY    = cp


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = "-------- begin (mode: $(RUN_MODE)) --------"
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE = Creating load file:
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = "**** Linking :"
MSG_COMPILING = "**** Compiling C :"
MSG_COMPILING_ARM = "**** Compiling C (ARM-only):"
MSG_COMPILINGCPP = "Compiling C++ :"
MSG_COMPILINGCPP_ARM = "Compiling C++ (ARM-only):"
MSG_ASSEMBLING = "**** Assembling:"
MSG_ASSEMBLING_ARM = "****Assembling (ARM-only):"
MSG_CLEANING = Cleaning project:
MSG_FORMATERROR = Can not handle output-format
MSG_ASMFROMC = "Creating asm-File from C-Source:"
MSG_ASMFROMC_ARM = "Creating asm-File from C-Source (ARM-only):"

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTDIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
#DEPFILES   = $(addprefix $(OUTDIR)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

elf: $(OUTDIR)/$(TARGET).elf
lss: $(OUTDIR)/$(TARGET).lss
sym: $(OUTDIR)/$(TARGET).sym
hex: $(OUTDIR)/$(TARGET).hex
bin: $(OUTDIR)/$(TARGET).bin

# Default target.
#all: begin gccversion sizebefore build sizeafter finished end
all: begin gccversion build sizeafter finished end

ifeq ($(LOADFORMAT),ihex)
build: elf hex lss sym
else 
ifeq ($(LOADFORMAT),binary)
build: elf bin lss sym
else 
ifeq ($(LOADFORMAT),both)
build: elf hex bin lss sym
else 
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif


# Eye candy.
begin:
##	@echo
	@echo $(MSG_BEGIN)

finished:
##	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
##	@echo

# Display sizes of sections.
ELFSIZE = $(SIZE) -A  $(OUTDIR)/$(TARGET).elf
##ELFSIZE = $(SIZE) --format=Berkeley --common $(OUTDIR)/$(TARGET).elf
sizebefore:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi
	@echo $(MSG_SIZE_AFTER)
	$(ELFSIZE)
	
# Display compiler version information.
gccversion : 
	@$(CC) --version
#	@echo $(ALLOBJ)

# Program the device.
ifeq ($(FLASH_TOOL),OPENOCD)
# Program the device with Dominic Rath's OPENOCD in "batch-mode", needs cfg and "reset-script".
program: $(OUTDIR)/$(TARGET).elf
	@echo "Programming with OPENOCD"
	$(OOCD_EXE) $(OOCD_CL)
endif

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
##	@echo
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O ihex $< $@
	
# Create final output file (.bin) from ELF output file.
%.bin: %.elf
##	@echo
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump testing: option -C
%.lss: %.elf
##	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C -r $< > $@
#	$(OBJDUMP) -x -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
##	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(ALLOBJ)
%.elf:  $(ALLOBJ)
	@echo
	@echo $(MSG_LINKING) $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
#	$(CPP) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)


# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_ASSEMBLING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src)))) 

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_ASSEMBLING_ARM) $$< "->" $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src)))) 

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILING_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILINGCPP) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILINGCPP_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src)))) 


# Compile: create assembler files from C source files. ARM/Thumb
$(SRC:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC) $< to $@
	$(CC) $(THUMB) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create assembler files from C source files. ARM only
$(SRCARM:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC_ARM) $< to $@
	$(CC) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Generate Doxygen documents
#docs:
#	doxygen  $(DOXYGENDIR)/doxygen.cfg
	
# Target: clean project.
clean: begin clean_list finished end

clean_list :
##	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OUTDIR)
#	$(REMOVE) $(OUTDIR)/$(TARGET).map
#	$(REMOVE) $(OUTDIR)/$(TARGET).elf
#	$(REMOVE) $(OUTDIR)/$(TARGET).hex
#	$(REMOVE) $(OUTDIR)/$(TARGET).bin
#	$(REMOVE) $(OUTDIR)/$(TARGET).sym
#	$(REMOVE) $(OUTDIR)/$(TARGET).lss
#	$(REMOVE) $(ALLOBJ)
#	$(REMOVE) $(LSTFILES)
#	$(REMOVE) $(DEPFILES)
#	$(REMOVE) $(SRC:.c=.s)
#	$(REMOVE) $(SRCARM:.c=.s)
#	$(REMOVE) $(CPPSRC:.cpp=.s)
#	$(REMOVE) $(CPPSRCARM:.cpp=.s)


# Create output files directory
$(shell mkdir -p $(OUTDIR))

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program

