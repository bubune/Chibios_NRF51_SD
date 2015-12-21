#
############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -ggdb -fomit-frame-pointer #-O2
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files
CHIBIOS = chibios
PORTED = ported
include $(PORTED)/boards/NRF51/board.mk
include $(PORTED)/os/hal/platforms/NRF51/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(PORTED)/os/hal/hal.mk
include $(PORTED)/os/ports/GCC/ARMCMx/NRF51/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk

# Define linker script file here
LDSCRIPT= $(PORTLD)/NRF51.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(PORTEDHALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/syscalls.c \
       main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various $(DRVINC) $(PORTEDHALINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m0

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
GDB  = $(TRGT)gdb -tui

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
DDEFS = -DNRF51 -D__NEWLIB__

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

OHEX = $(BUILDDIR)/$(PROJECT).hex
OELF = $(BUILDDIR)/$(PROJECT).elf
OBIN = $(BUILDDIR)/$(PROJECT).bin

RULESPATH = $(CHIBIOS)/os/ports/GCC/ARMCMx
include $(RULESPATH)/rules.mk

##############################################################################
# Terminal command
TERMINAL ?= gnome-terminal -e

#FLASH_START_ADDRESS = $(shell $(OD) -h $(OELF) -j .text | grep .text | awk '{print $$4}')
FLASH_START_ADDRESS = 0x0000C000
#SOFTDEVICE = ../../SoftDevice/S310/s310_nrf51422_1.0.0/s310_nrf51422_1.0.0_softdevice.hex
SOFTDEVICE = ../../SoftDevice/S210/s210_nrf51422_3.0.0/s210_nrf51422_3.0.0_softdevice.hex
SOFTDEVICE_OUTPUT = $(BUILDDIR)/$(notdir $(SOFTDEVICE))

JLINK_OPTIONS = -device nrf51422 -if swd -speed 1000
JLINK = -JLinkExe $(JLINK_OPTIONS)

# GBD Server command
JLINKGDBSERVER = JLinkGDBServer $(JLINK_OPTIONS)
# GBD Port
GDB_PORT_NUMBER = 2331
##############################################################################

clean::
	rm -f *.jlink
	rm -f JLink.log
	rm -f .gdbinit
	@echo
	@echo Done

flash: all flash.jlink
	$(JLINK) flash.jlink

flash.jlink:
	printf "w4 4001e504 1\nloadbin $(OBIN) $(FLASH_START_ADDRESS)\nverifybin $(OBIN) $(FLASH_START_ADDRESS)\nr\ng\nexit\n" > flash.jlink

flash-softdevice: flash-softdevice.jlink
ifndef SOFTDEVICE
	$(error "You need to set the SOFTDEVICE command-line parameter to a path (without spaces) to the softdevice hex-file")
endif

	$(CP) -Iihex -Obinary --remove-section .sec2 $(SOFTDEVICE) $(SOFTDEVICE_OUTPUT:.hex=_mainpart.bin)
	$(CP) -Iihex -Obinary --remove-section .sec1 $(SOFTDEVICE) $(SOFTDEVICE_OUTPUT:.hex=_uicr.bin)
	$(JLINK) flash-softdevice.jlink

flash-softdevice.jlink:
	# Write to NVMC to enable write. Write mainpart, write UICR. Assumes device is erased.
	printf "w4 4001e504 1\nloadbin \"$(SOFTDEVICE_OUTPUT:.hex=_mainpart.bin)\" 0\nloadbin \"$(SOFTDEVICE_OUTPUT:.hex=_uicr.bin)\" 0x10001000\nr\ng\nexit\n" > flash-softdevice.jlink

erase-all: erase-all.jlink
	$(JLINK) erase-all.jlink

erase-all.jlink:
	# Write to NVMC to enable erase, do erase all, wait for completion. reset
	printf "w4 4001e504 2\nw4 4001e50c 1\nsleep 100\nr\nexit\n" > erase-all.jlink

debug: debug-gdbinit
	$(TERMINAL) "$(JLINKGDBSERVER) -port $(GDB_PORT_NUMBER)"
	sleep 1
	$(TERMINAL) "$(GDB) --command=.gdbinit $(OELF)"

debug-gdbinit:
	printf "target remote localhost:$(GDB_PORT_NUMBER)\nmon speed 10000\nmon endian little\nmon flash download = 1\nmon flash device = NRF51422\nbreak main\nload\nmon reset\ncontinue" > .gdbinit

