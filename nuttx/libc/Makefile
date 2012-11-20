############################################################################
# libc/Makefile
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
###########################################################################

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS =

DEPPATH := --dep-path .
VPATH := .

include stdio/Make.defs
include stdlib/Make.defs
include unistd/Make.defs
include sched/Make.defs
include string/Make.defs
include pthread/Make.defs
include semaphore/Make.defs
include signal/Make.defs
include mqueue/Make.defs
include math/Make.defs
include fixedmath/Make.defs
include net/Make.defs
include time/Make.defs
include libgen/Make.defs
include dirent/Make.defs
include termios/Make.defs
include queue/Make.defs
include misc/Make.defs

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

UBIN = libuc$(LIBEXT)
KBIN = libkc$(LIBEXT)
BIN  = libc$(LIBEXT)

all: $(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

ifneq ($(BIN),$(UBIN))
.userlib:
	$(Q) $(MAKE) $(UBIN) BIN=$(UBIN) TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) touch .userlib

$(UBIN): kclean .userlib
endif

ifneq ($(BIN),$(KBIN))
.kernlib:
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) touch .kernlib

$(KBIN): uclean .kernlib
endif

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

# Clean Targets:
# Clean user-mode temporary files (retaining the UBIN binary)

uclean:
ifneq ($(OBJEXT),)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist .userlib ]; then del *$(OBJEXT)
else
	$(Q) ( if [ -f .userlib ]; then rm -f *$(OBJEXT); fi )
endif
endif
	$(call DELFILE, .userlib)

# Clean kernel-mode temporary files (retaining the KBIN binary)

kclean:
ifneq ($(OBJEXT),)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist .kernlib ]; then del *$(OBJEXT)
else
	$(Q) ( if [ -f .kernlib ]; then rm -f *$(OBJEXT); fi )
endif
endif
	$(call DELFILE, .kernlib)

# Really clean everything

clean: uclean kclean
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
