############################################################################
# configs/teensy/src/Makefile
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

ASRCS		= 
CSRCS		= up_boot.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS		+= up_leds.c
endif
ifeq ($(CONFIG_USBMSC),y)
CSRCS		+= up_usbmsc.c
endif
ifeq ($(CONFIG_AVR_SPI),y)
CSRCS		+= up_spi.c
endif

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ARCH_SRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(CONFIG_ARCH_AVR32),y)
ARCH_SUBDIR	= avr32
endif
ifeq ($(CONFIG_ARCH_AVR),y)
ARCH_SUBDIR	= avr
endif

ifeq ($(WINTOOL),y)
  CFLAGS	+= -I "${shell cygpath -w $(TOPDIR)/sched}"
  CFLAGS	+= -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}" 
  CFLAGS	+= -I "${shell cygpath -w $(ARCH_SRCDIR)/common}" 
  CFLAGS	+= -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}"
else
  CFLAGS	+= -I "$(TOPDIR)/sched"
  CFLAGS	+= -I "$(ARCH_SRCDIR)/chip"
  CFLAGS	+= -I "$(ARCH_SRCDIR)/common" 
  CFLAGS	+= -I "$(ARCH_SRCDIR)/$(ARCH_SUBDIR)"
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
