ifeq ($(strip $(DEVKITARM)),)
$(error "Please set DEVKITARM in your environment. export DEVKITARM=<path to>devkitARM)
endif
ifeq ($(strip $(DEVKITPRO)),)
$(error "Please set DEVKITPRO in your environment. export DEVKITPRO=<path to>devkitPro)
endif

PROJ  :=  $(shell basename $(CURDIR))
TARGET  :=  $(CURDIR)/$(PROJ)

OBJDIR := obj
SRCS := gbak.c 
OBJS := $(OBJDIR)/gbak.o

PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
AR      := $(PREFIX)ar

ARCH    := -mthumb-interwork -mthumb
SPECS   := -specs=gba.specs

CFLAGS  := $(ARCH) -O2 -Wall -fno-strict-aliasing
ARFLAGS := rcs

# Set paths for tools being used
PATH := $(DEVKITARM)/bin:$(PATH)
PATH := $(DEVKITPRO)/tools/bin:$(PATH)

.PHONY : all clean assets

all: $(TARGET).a

clean : 
	@-rm -rf $(OBJDIR)
	@-rm $(TARGET).a

# Link into an ELF file
$(TARGET).a : $(OBJS)
	$(AR) $(ARFLAGS) $@ $^ 

# Compile objects
$(OBJDIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) -c $< $(CFLAGS) -o $@
