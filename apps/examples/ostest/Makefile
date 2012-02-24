############################################################################
# apps/examples/ostest/Makefile
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

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# NuttX OS Test

ASRCS		=
CSRCS		= main.c dev_null.c

ifeq ($(CONFIG_ARCH_FPU),y)
CSRCS		+= fpu.c
endif

ifneq ($(CONFIG_DISABLE_PTHREAD),y)
CSRCS		+= cancel.c cond.c mutex.c sem.c barrier.c
ifneq ($(CONFIG_RR_INTERVAL),0)
CSRCS		+= roundrobin.c
endif # CONFIG_RR_INTERVAL
ifeq ($(CONFIG_MUTEX_TYPES),y)
CSRCS		+= rmutex.c
endif # CONFIG_MUTEX_TYPES
endif # CONFIG_DISABLE_PTHREAD

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
CSRCS		+= sighand.c
ifneq ($(CONFIG_DISABLE_PTHREAD),y)
ifneq ($(CONFIG_DISABLE_CLOCK),y)
CSRCS		+= timedwait.c
endif # CONFIG_DISABLE_CLOCK
endif # CONFIG_DISABLE_PTHREAD
endif # CONFIG_DISABLE_SIGNALS

ifneq ($(CONFIG_DISABLE_MQUEUE),y)
ifneq ($(CONFIG_DISABLE_PTHREAD),y)
CSRCS		+= mqueue.c 
ifneq ($(CONFIG_DISABLE_CLOCK),y)
CSRCS		+= timedmqueue.c 
endif # CONFIG_DISABLE_CLOCK
endif # CONFIG_DISABLE_PTHREAD
endif # CONFIG_DISABLE_MQUEUE

ifneq ($(CONFIG_DISABLE_POSIX_TIMERS),y)
CSRCS		+= posixtimer.c
endif

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
ifneq ($(CONFIG_DISABLE_PTHREAD),y)
ifeq ($(CONFIG_PRIORITY_INHERITANCE),y)
CSRCS		+= prioinherit.c
endif # CONFIG_PRIORITY_INHERITANCE
endif # CONFIG_DISABLE_PTHREAD
endif # CONFIG_DISABLE_SIGNALS

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ifeq ($(WINTOOL),y)
  BIN		= "${shell cygpath -w  $(APPDIR)/libapps$(LIBEXT)}"
else
  BIN		= "$(APPDIR)/libapps$(LIBEXT)"
endif

ROOTDEPPATH	= --dep-path .

# Common build

VPATH		= 

all: .built
.PHONY: clean depend distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $(BIN), $${obj}); \
	done ; )
	@touch .built

context:

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f *.o *~ .*.swp .built
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
