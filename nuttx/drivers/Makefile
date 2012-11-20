############################################################################
# drivers/Makefile
#
#   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

ifeq ($(WINTOOL),y)
INCDIROPT = -w
endif

DEPPATH = --dep-path .
ASRCS =
CSRCS =
VPATH = .

# Include support for various drivers.  Each Make.defs file will add its
# files to the source file list, add its DEPPATH info, and will add
# the appropriate paths to the VPATH variable

include analog/Make.defs
include bch/Make.defs
include input/Make.defs
include lcd/Make.defs
include mmcsd/Make.defs
include mtd/Make.defs
include net/Make.defs
include pipes/Make.defs
include power/Make.defs
include sensors/Make.defs
include sercomm/Make.defs
include serial/Make.defs
include syslog/Make.defs
include usbdev/Make.defs
include usbhost/Make.defs
include wireless/Make.defs

ifneq ($(CONFIG_NFILE_DESCRIPTORS),0)
  CSRCS += dev_null.c dev_zero.c loop.c

ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)
  CSRCS += ramdisk.c rwbuffer.c
endif

ifeq ($(CONFIG_CAN),y)
  CSRCS += can.c
endif

ifeq ($(CONFIG_PWM),y)
  CSRCS += pwm.c
endif

ifeq ($(CONFIG_WATCHDOG),y)
  CSRCS += watchdog.c
endif
endif

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN			= libdrivers$(LIBEXT)

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
