#  Project Name
PROJECT=main

# libs dir
LIBDIR=../../lib

# STM32 stdperiph lib defines
CDEFS=-DHSE_VALUE=8000000 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

#  List of the objects files to be compiled/assembled
OBJECTS=main.o md5.o tmr4.o

CMSIS_SOURCES=\
$(LIBDIR)/cmsis/startup_stm32f10x_md.o \
$(LIBDIR)/cmsis/system_stm32f10x.o

STM_SOURCES=\
$(LIBDIR)/stm32f10x/src/stm32f10x_gpio.o \
$(LIBDIR)/stm32f10x/src/stm32f10x_rcc.o \
$(LIBDIR)/stm32f10x/src/stm32f10x_exti.o \
$(LIBDIR)/stm32f10x/src/misc.o \
$(LIBDIR)/stm32f10x/src/stm32f10x_usart.o \
$(LIBDIR)/stm32f10x/src/stm32f10x_spi.o \
$(LIBDIR)/stm32f10x/src/stm32f10x_tim.o \
$(LIBDIR)/stm32f10x/src/stm32f10x_i2c.o \
$(LIBDIR)/stm32f10x/src/stm32f10x_adc.o

MAT_SOURCES=\
$(LIBDIR)/mat/circbuf8.o \
$(LIBDIR)/mat/itoa.o \
$(LIBDIR)/mat/serialq.o \
$(LIBDIR)/mat/spi.o \
$(LIBDIR)/mat/fls_25.o \
$(LIBDIR)/mat/ee_95.o \
$(LIBDIR)/mat/i2c.o \
$(LIBDIR)/mat/ee_24.o \
$(LIBDIR)/mat/adc.o

USBFS_SOURCES=\
$(LIBDIR)/stm32usbfs/src/usb_core.o \
$(LIBDIR)/stm32usbfs/src/usb_init.o \
$(LIBDIR)/stm32usbfs/src/usb_int.o \
$(LIBDIR)/stm32usbfs/src/usb_mem.o \
$(LIBDIR)/stm32usbfs/src/usb_regs.o \
$(LIBDIR)/stm32usbfs/src/usb_sil.o

USBCDC_SOURCES=\
$(LIBDIR)/mat/usb-cdc/usb_desc.o \
$(LIBDIR)/mat/usb-cdc/usb_prop.o \
$(LIBDIR)/mat/usb-cdc/usb_endp.o \
$(LIBDIR)/mat/usb-cdc/usb_hwi.o

OBJECTS+=$(CMSIS_SOURCES)
OBJECTS+=$(STM_SOURCES)
OBJECTS+=$(MAT_SOURCES)
OBJECTS+=$(USBFS_SOURCES)
OBJECTS+=$(USBCDC_SOURCES)

LSCRIPT=stm32f103x8.ld

OPTIMIZATION = s
DEBUG = dwarf-2
#LISTING = -ahls

#  Compiler Options
GCFLAGS = -g$(DEBUG)
GCFLAGS += $(CDEFS)
GCFLAGS += -O$(OPTIMIZATION)
GCFLAGS += -Wall -std=gnu99 -fno-common -mcpu=cortex-m3 -mthumb -ffunction-sections
GCFLAGS += -I$(LIBDIR)/stm32f10x/inc -I$(LIBDIR)/cmsis -I$(LIBDIR) -I$(LIBDIR)/stm32usbfs/inc -I$(LIBDIR)/mat/usb-cdc
#GCFLAGS += -Wcast-align -Wcast-qual -Wimplicit -Wpointer-arith -Wswitch
#GCFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
LDFLAGS = -mcpu=cortex-m3 -mthumb -O$(OPTIMIZATION) -Wl,-Map=$(PROJECT).map -T$(LSCRIPT) -Wl,--gc-sections
ASFLAGS = $(LISTING) -mcpu=cortex-m3

#  Compiler/Assembler/Linker Paths
GCC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
ifeq ($(OS), Windows_NT)
REMOVE = rm.py -f
else
REMOVE = rm -f
endif
SIZE = arm-none-eabi-size

#########################################################################

all: $(PROJECT).hex $(PROJECT).bin stats

$(PROJECT).bin: $(PROJECT).elf
#	$(OBJCOPY) -O binary -j .text -j .data $(PROJECT).elf $(PROJECT).bin
	$(OBJCOPY) -R .stack -O binary $(PROJECT).elf $(PROJECT).bin

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -R .stack -O ihex $(PROJECT).elf $(PROJECT).hex

$(PROJECT).elf: $(OBJECTS)
	$(GCC) $(LDFLAGS) $(OBJECTS) -o $(PROJECT).elf

stats: $(PROJECT).elf
	$(SIZE) $(PROJECT).elf

clean:
	$(REMOVE) $(OBJECTS)
	$(REMOVE) $(PROJECT).hex
	$(REMOVE) $(PROJECT).elf
	$(REMOVE) $(PROJECT).map
	$(REMOVE) $(PROJECT).bin

program:
	st-flash write main.bin 0x08000000

#########################################################################
#  Default rules to compile .c and .cpp file to .o
#  and assemble .s files to .o

.c.o :
	$(GCC) $(GCFLAGS) -c $< -o $@

.cpp.o :
	$(GCC) $(GCFLAGS) -c $< -o $@

.s.o :
	$(AS) $(ASFLAGS) -o $@ $<
#	$(AS) $(ASFLAGS) -o $(PROJECT)_crt.o $< > $(PROJECT)_crt.lst

#########################################################################
-include $(shell mkdir .dep) $(wildcard .dep/*)
