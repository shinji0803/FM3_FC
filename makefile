#################################################
# MAKEFILE For FUJITSU MB9BF618T Device 		#
# (c) 20120401 Nemui Trinomius					#
# http://nemuisan.blog.bai.ne.jp				#
#################################################

# Environment Dependent!!! This Environment assure under WINDOWS !!
# Throw path into YOUR environments
export PATH = %SYSTEMROOT%;$(TOOLDIR)/bin;$(OCDIR);$(MAKEDIR)

# Toolchain prefix (i.e arm-none-eabi -> arm-none-eabi-gcc.exe)
TCHAIN  = arm-none-eabi

# OpenOCD prefix
OCD		= openocd
# Select OpenOCD Transport
#OCDMODE = SWD
OCDMODE = JTAG
# OpenOCD For FT2232 Devices Special Setting
# If u use WinUSB as Device Driver,uncomment this!
MPSSE	= ftdi
#MPSSE	= 

# Select SWD Writer
WRITER  = VERSALOON
#WRITER  = STLINKV2

# Development Tools based on GNU Compiler Collection
#DEVTOOL = LAUNCHPAD
#DEVTOOL = BLEEDING_EDGE
#DEVTOOL = YAGARTO
#DEVTOOL = DEVKITARM
DEVTOOL = SOURCERY

# Check BuildTools
ifeq ($(DEVTOOL),LAUNCHPAD)
 TOOLDIR = C:/Devz/ARM/Launchpad
 NANOLIB = --specs=nano.specs
  NANOLIB += -u _printf_float
#  NANOLIB += -u _scanf_float
 REMOVAL = rm
else ifeq ($(DEVTOOL),BLEEDING_EDGE)
 TOOLDIR = C:/Devz/ARM/Bleeding-edge
 REMOVAL = rm
else ifeq ($(DEVTOOL),YAGARTO)
 TOOLDIR = C:/Devz/ARM/Yagarto
 REMOVAL = rm
else ifeq ($(DEVTOOL),DEVKITARM)
 TOOLDIR = C:/Devz/ARM/devkitARM
 REMOVAL = rm
else ifeq ($(DEVTOOL),SOURCERY)
 TOOLDIR = C:/ARM_dev_tool/arm_gcc
 REMOVAL = cs-rm
else
 $(error SET BUILD-TOOLS AT FIRST!!)
endif

# Set UNIX-Like tools (Coreutils)
MAKEDIR = C:/ARM_dev_tool/coreutils-5.3.0-bin/bin

# Set Flasher and Debugger
OCDIR	= C:/Devz/ARM/OCD
ifeq ($(OCDMODE),SWD)
 ifeq ($(WRITER),VERSALOON)
 OCD_CMD = -s $(OCDIR)/tcl						\
		  -f interface/vsllink_swd.cfg			\
          -f target/mb9bf618t_flash.cfg
 else ifeq ($(WRITER),STLINKV2)
 OCD_CMD = -s $(OCDIR)/tcl						\
		  -f interface/stlink-v2.cfg			\
          -f target/mb9bf618t_hla_flash.cfg
 endif
else
OCD_CMD = -s $(OCDIR)/tcl						\
		  -f interface/$(MPSSE)/jtagkey2.cfg 	\
          -f target/mb9bf618t_flash.cfg
endif


WSHELL  = cmd
MSGECHO = echo.exe
GDBDIR  = C:/Devz/ARM/insight/bin
INSIGHT = $(GDBDIR)/arm-none-eabi-insight
# Environment Dependent!!!



# OPTIMIZE definition
OPTIMIZE		= 0
#OPTIMIZE		= s

# GCC4.6.x Specific Option
ifneq ($(OPTIMIZE),0)
USE_LTO			= -flto-partition=none -fipa-sra
#USE_LTO			= -flto -fipa-sra
endif
ALIGNED_ACCESS	= -mno-unaligned-access

# Semihosting Definition
#USING_HOSTAGE   = USE_SEMIHOSTING
ifeq ($(USING_HOSTAGE),USE_SEMIHOSTING)
SEMIHOST_LIB = --specs=rdimon.specs -lrdimon
else
START_LIB    = -nostartfiles
endif


# Apprication Version
APP_VER = W.I.P

# Board definition
SUBMODEL		= MB9BF618T
EVAL_BOARD    	= USE_FRK_FM3
HSE_CLOCK 		= 4000000

# Use FreeRTOS?
OS_SUPPORT		= BARE_METAL
#OS_SUPPORT		= USE_FREERTOS

#MFS Define
SEL_CH = USE_CH03

# Synthesis makefile Defines
DEFZ = $(SUBMODEL)   $(EVAL_BOARD)   $(MPU_DENSITY)  $(PERIF_DRIVER)    $(VECTOR_START)    \
	   $(USING_HOSTAGE) $(OS_SUPPORT) $(USE_EXT_SRAM) $(USE_EXT_HEAP) $(SEL_CH)
# Defines if Display and Font Drivers
DEFZ += $(USE_DISPLAY) $(USE_FONTSIZE) $(USE_KANJI) $(USE_TOUCH_SENCE)  $(USE_XMSTN)		\
        $(USE_JPEG_LIB) $(USE_PNG_LIB) 
SYNTHESIS_DEFS	= $(addprefix -D,$(DEFZ)) 							\
				 -DPACK_STRUCT_END=__attribute\(\(packed\)\) 		\
				 -DALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
				 -DMPU_SUBMODEL=\"$(SUBMODEL)\"						\
				 -DAPP_VERSION=\"$(APP_VER)\"						\
				 -DHSE_VALUE=$(HSE_CLOCK)UL 

# TARGET definition
TARGET 		= main
TARGET_ELF  = $(TARGET).elf
TARGET_SREC = $(TARGET).s19
TARGET_HEX  = $(TARGET).hex
TARGET_BIN  = $(TARGET).bin
TARGET_LSS  = $(TARGET).lss
TARGET_SYM  = $(TARGET).sym

# define CMSIS LIBRARY PATH
FWLIB  			= ./lib/Drivers
USBLIB 			=
CMSISLIB 		= ./lib/CMSIS
CMSIS_DEVICE 	= $(CMSISLIB)/Device/SPANSION/MB9BF61xT
CMSIS_CORE		= $(CMSISLIB)/Include

# include PATH
INCPATHS	 = 	./							\
				./inc						\
				$(USBLIB)/Core/inc			\
				$(CMSIS_DEVICE)/Include		\
				$(CMSIS_CORE)				\
				$(LIBINCDIRS)
INCLUDES     = $(addprefix -I ,$(INCPATHS))


# Set library PATH
LIBPATHS     = $(FWLIB) $(USBLIB) $(CMSISLIB) $(DISPLAY_LIB)
LIBRARY_DIRS = $(addprefix -L,$(LIBPATHS))
# if you use math-library, put "-lm" 
MATH_LIB	 =	-lm

# LinkerScript PATH
LINKER_PATH =  ./lib/linker
LINKER_DIRS = $(addprefix -L,$(LINKER_PATH)) 

# Object definition
OBJS 	 = $(CFILES:%.c=%.o) 
LIBOBJS  = $(LIBCFILES:%.c=%.o) $(SFILES:%.s=%.o)

# C code PATH
SOURCE  = ./src
CFILES = \
 $(SOURCE)/$(TARGET).c			\
 $(SOURCE)/syscalls.c			\
 $(SOURCE)/scheduler.c			\
 $(SOURCE)/uart_support.c		\
 $(SOURCE)/console.c			\
 $(SOURCE)/myMath.c				\
 $(SOURCE)/mavlink_support.c	\
 $(SOURCE)/parameters.c


#/*----- MultiFunctionSerial(MFS) library PATH -----*/	
MFS_LIB	= ./lib/drivers/mfserial
include $(MFS_LIB)/mfs_cfg.mk

#/*----- My library PATH -----*/	
MYLIB = ./lib/my_lib
include $(MYLIB)/mylib_cfg.mk

#/*----- Mavlink lib PATH -----*/
MAVLINK = ./lib/Mavlink
include $(MAVLINK)/mavlink_cfg.mk

#/*----- STARTUP code PATH -----*/
STARTUP_DIR = $(CMSIS_DEVICE)/Source/Templates/startup/gcc
ifeq ($(OS_SUPPORT),USE_FREERTOS)
SFILES += \
	./src/startup_mb9bf61xt_rtos.s
else
SFILES += \
	$(STARTUP_DIR)/startup_mb9bf61xt.s
endif


#/*----- MB9BFxxxx library PATH -----*/
BTIMER = ./lib/drivers/btimer
LIBINCDIRS += $(BTIMER)
LIBCFILES = \
 $(CMSIS_DEVICE)/Source/Templates/system_mb9bf61x.c

 
#/*-----MB9BF61xx Debug library -----*/
ifeq ($(OPTIMIZE),0)
CFILES += \
 ./lib/IOView/mb9bf61x_io_view.c
else
endif

# TOOLCHAIN SETTING
CC 			= $(TCHAIN)-gcc
AS	 		= $(TCHAIN)-as
CPP 		= $(TCHAIN)-g++
OBJCOPY 	= $(TCHAIN)-objcopy
OBJDUMP 	= $(TCHAIN)-objdump
SIZE 		= $(TCHAIN)-size
AR 			= $(TCHAIN)-ar
LD 			= $(TCHAIN)-gcc
NM 			= $(TCHAIN)-nm
REMOVE		= $(REMOVAL) -f
REMOVEDIR 	= $(REMOVAL) -rf

# C and ASM FLAGS
CFLAGS  = -MD -mcpu=cortex-m3 -mtune=cortex-m3 -mfix-cortex-m3-ldrd
CFLAGS += -mthumb -mlittle-endian $(ALIGNED_ACCESS)
CFLAGS += -mapcs-frame -mno-sched-prolog -msoft-float
CFLAGS += -std=gnu99
CFLAGS += -gdwarf-2 -O$(OPTIMIZE) $(USE_LTO) $(NANOLIB) $(SEMIHOST_LIB)
CFLAGS += -fno-strict-aliasing -fsigned-char
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-schedule-insns2
CFLAGS += --param max-inline-insns-single=1000
CFLAGS += -fno-common -fno-hosted
CFLAGS += -Wall -Wno-array-bounds
#CFLAGS += -Wdouble-promotion
#CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
CFLAGS += $(SYNTHESIS_DEFS) 


# ASM FLAGS
AFLAGS  =

# Linker FLAGS
LDFLAGS  = -mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd
LDFLAGS += -u g_pfnVectors -Wl,-static -Wl,--gc-sections $(START_LIB)
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += $(LIBRARY_DIRS) $(LINKER_DIRS) $(MATH_LIB)
LDFLAGS +=-T$(LINKER_PATH)/MB9BF618T-ROM.ld

# Object Copy and dfu generation FLAGS
OBJCPFLAGS = -O
OBJDUMPFLAGS = -h -S -C
DFU	  = hex2dfu
DFLAGS = -w


# Build Object
all: gccversion clean build buildinform sizeafter
build: $(TARGET_ELF) $(TARGET_LSS) $(TARGET_SYM) $(TARGET_HEX) $(TARGET_SREC) $(TARGET_BIN)

.SUFFIXES: .o .c .s   

$(TARGET_LSS): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Disassemble: $@
	$(OBJDUMP) $(OBJDUMPFLAGS) $< > $@ 
$(TARGET_SYM): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Symbol: $@
	$(NM) -n $< > $@
$(TARGET).hex: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) ihex $^ $@    
$(TARGET).s19: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) srec $^ $@ 
$(TARGET).bin: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) binary $< $@
$(TARGET).elf: $(OBJS) $(SUBMODEL)_lib.a
	@$(MSGECHO) Link: $@
	$(LD) $(CFLAGS) $(LDFLAGS) $^ -o $@
	@$(MSGECHO)

$(SUBMODEL)_lib.a: $(LIBOBJS)
	@$(MSGECHO) Archive: $@
	$(AR) cr $@ $(LIBOBJS)    
	@$(MSGECHO)
.c.o:
	@$(MSGECHO) Compile: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)
.s.o:
	@$(MSGECHO) Assemble: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)

# Object Size Informations
sizeafter:
	@$(MSGECHO) 
	@$(MSGECHO) Built Object Informations:
	@$(MSGECHO) === Total Binary Size ===
	@$(SIZE) $(TARGET).hex
	@$(MSGECHO) === Verbose ELF Size ===
	@$(SIZE) $(TARGET).elf
	@$(SIZE) -A -x $(TARGET).elf

# Display compiler version information.
gccversion : 
	@$(CC) --version
	@$(MSGECHO) 

buildinform :
	@$(MSGECHO) 
	@$(MSGECHO) 
	@$(MSGECHO) Built Informations:
	@$(MSGECHO) USING_SYSTEM = $(OS_SUPPORT)
	@$(MSGECHO) USING_DISPLAY = $(USE_DISPLAY)
	@$(MSGECHO) USING_DEVBOARD = $(EVAL_BOARD)

# Flash and Debug Program
debug :
	$(WSHELL) /c start /B $(INSIGHT) $(TARGET).elf
	$(OCD) $(OCD_CMD) -c"reset halt" -c"arm semihosting enable"
program :
	$(OCD) $(OCD_CMD) -c "mt_flash $(TARGET).elf"
#	$(OCD) $(OCD_CMD) -c "eraser"


# Drop files into dust-shoot
.PHONY clean:
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(SUBMODEL)_lib.a
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).s19
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(TARGET).dfu
	$(REMOVE) $(wildcard *.stackdump)
	$(REMOVE) $(OBJS)
	$(REMOVE) $(AOBJ)
	$(REMOVE) $(LIBOBJS)
	$(REMOVE) $(LST)
	$(REMOVE) $(CFILES:.c=.lst)
	$(REMOVE) $(CFILES:.c=.d)
	$(REMOVE) $(LIBCFILES:.c=.lst)
	$(REMOVE) $(LIBCFILES:.c=.d)
	$(REMOVE) $(SFILES:.s=.lst)
	$(REMOVE) $(wildcard ./lib/IOView/*.d)
	$(REMOVE) $(wildcard ./lib/IOView/*.lst)
	$(REMOVE) $(wildcard ./lib/IOView/*.o)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.d)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.lst)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.o)
	$(REMOVE) $(wildcard $(FATFS)/*.d)
	$(REMOVE) $(wildcard $(FATFS)/*.lst)
	$(REMOVE) $(wildcard $(FATFS)/*.o)
	$(REMOVE) $(wildcard $(CMSIS_DEVICE)/*.d)
	$(REMOVE) $(wildcard $(CMSIS_DEVICE)/*.lst)
	$(REMOVE) $(wildcard $(CMSIS_DEVICE)/*.o)
	$(REMOVEDIR) .dep
	@$(MSGECHO)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program
