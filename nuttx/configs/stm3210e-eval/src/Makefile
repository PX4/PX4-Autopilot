############################################################################
# configs/stm3210e-eval/src/Makefile
#
#   Copyright (C) 2009-2010, 2012 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

CFLAGS		+= -I$(TOPDIR)/sched

ASRCS		= 
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= up_boot.c up_leds.c up_buttons.c up_spi.c up_usbdev.c

ifeq ($(CONFIG_STM32_FSMC),y)
CSRCS		+= up_lcd.c up_extcontext.c up_extmem.c up_selectnor.c \
			  up_deselectnor.c up_selectsram.c up_deselectsram.c \
			  up_selectlcd.c up_deselectlcd.c
endif

ifeq ($(CONFIG_NSH_ARCHINIT),y)
CSRCS		+= up_nsh.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS		+= up_adc.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS		+= up_usbmsc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS		+= up_composite.c
endif

ifeq ($(CONFIG_I2C_LM75),y)
CSRCS		+= up_lm75.c
endif

ifeq ($(CONFIG_CAN),y)
CSRCS		+= up_can.c
endif

ifeq ($(CONFIG_WATCHDOG),y)
CSRCS		+= up_watchdog.c
endif

ifeq ($(CONFIG_PM_CUSTOMINIT),y)
CSRCS		+= up_pm.c
endif

ifeq ($(CONFIG_PM_BUTTONS),y)
CSRCS		+= up_pmbuttons.c 
endif

ifeq ($(CONFIG_IDLE_CUSTOM),y)
CSRCS		+= up_idle.c
endif

COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ARCH_SRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL),y)
  CFLAGS	+= -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}" \
  		   -I "${shell cygpath -w $(ARCH_SRCDIR)/common}" \
  		   -I "${shell cygpath -w $(ARCH_SRCDIR)/armv7-m}"
else
  CFLAGS	+= -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common -I$(ARCH_SRCDIR)/armv7-m
endif

all: libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
