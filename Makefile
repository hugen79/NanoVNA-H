##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

#Build target
ifeq ($(TARGET),)
  TARGET = F072
endif
TARGET=F303

# Compiler options here.
ifeq ($(USE_OPT),)
 ifeq ($(TARGET),F303)
USE_OPT = -O2 -fno-inline-small-functions -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nano.specs -fstack-usage -std=c11 -DLCD_DRIVER_ST7796S -DLCD_480x320 -D__VNA_ENABLE_DAC__ -D__LCD_BRIGHTNESS__  -DPOINTS_COUNT=401  -D_USE_FONT_=1  -D_USE_BIG_MARKER_=1
#USE_OPT+=-fstack-protector-strong
 else
USE_OPT = -O2 -fno-inline-small-functions -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nano.specs -fstack-usage -std=c11 -DLCD_DRIVER_ILI9341 -DLCD_320x240 -D__DFU_SOFTWARE_MODE__  -DPOINTS_COUNT=101 -D_USE_FONT_=1 -D_USE_BIG_MARKER_=1

 endif
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
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

ifeq ($(VERSION),)
  VERSION="$(shell git describe --tags)"
endif

##############################################################################
# Architecture or project specific options
#
ifeq ($(TARGET),F303)
  USE_FPU = hard
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x200
endif
# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x100
endif
#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files and paths
#CHIBIOS = ../ChibiOS-RT
CHIBIOS = ChibiOS
PROJ = .
# Startup files.
# HAL-OSAL files (optional).
ifeq ($(TARGET),F303)
 include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f3xx.mk
 include $(CHIBIOS)/os/hal/hal.mk
 include $(CHIBIOS)/os/hal/ports/STM32/STM32F3xx/platform.mk
 include NANOVNA_STM32_F303/board.mk
else
 include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f0xx.mk
 include $(CHIBIOS)/os/hal/hal.mk
 include $(CHIBIOS)/os/hal/ports/STM32/STM32F0xx/platform.mk
 include NANOVNA_STM32_F072/board.mk
endif

include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
ifeq ($(TARGET),F303)
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
else
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v6m.mk
endif
# Other files (optional).
#include $(CHIBIOS)/test/rt/test.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
#include $(CHIBIOS)/os/various/shell/shell.mk

# Define linker script file here
ifeq ($(TARGET),F303)
 LDSCRIPT= NANOVNA_STM32_F303/STM32F303xC.ld
else
 LDSCRIPT= NANOVNA_STM32_F072/STM32F072xB.ld
endif

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(STREAMSSRC) \
       FatFs/ff.c \
       FatFs/ffunicode.c \
       usbcfg.c \
       main.c si5351.c tlv320aic3204.c dsp.c plot.c ui.c ili9341.c numfont20x22.c Font5x7.c Font7x11b.c Font10x14.c Font6x11.c flash.c adc.c rtc.c vna_math.c

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
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC)  \
         $(STREAMSINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

ifeq ($(TARGET),F303)
 MCU  = cortex-m4
else
 MCU  = cortex-m0
endif

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
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
ELF  = $(CP) -O elf

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
ifeq ($(TARGET),F303)
 UDEFS = -DARM_MATH_CM4 -DVERSION=\"$(VERSION)\" -DNANOVNA_F303
else
 UDEFS = -DARM_MATH_CM0 -DVERSION=\"$(VERSION)\" 
endif
#Enable if use RTC and need auto select source LSE or LSI
UDEFS+= -DVNA_AUTO_SELECT_RTC_SOURCE
#Enable if install external 32.768kHz clock quartz on PC14 and PC15 pins on STM32 CPU and no VNA_AUTO_SELECT_RTC_SOURCE
#UDEFS+= -DVNA_USE_LSE

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

flash: build/ch.bin
	dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin

dfu:
	-@printf "reset dfu\r" >/dev/cu.usbmodem401

TAGS: Makefile
ifeq ($(TARGET),F303)
	@etags *.[ch] NANOVNA_STM32_F303/*.[ch] $(shell find ChibiOS/os/hal/ports/STM32/STM32F3xx ChibiOS/os -name \*.\[ch\] -print) 
else
	@etags *.[ch] NANOVNA_STM32_F072/*.[ch] $(shell find ChibiOS/os/hal/ports/STM32/STM32F0xx ChibiOS/os -name \*.\[ch\] -print) 
endif
	@ls -l TAGS

