############################################################################
# examples/nettest/Makefile
#
#   Copyright (C) 2007-2008, 2010-2012 Gregory Nutt. All rights reserved.
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

# Basic TCP networking test

TARG_ASRCS = 
TARG_AOBJS = $(TARG_ASRCS:.S=$(OBJEXT))

TARG_CSRCS = nettest.c
ifeq ($(CONFIG_EXAMPLES_NETTEST_SERVER),y)
TARG_CSRCS += nettest_server.c
else
TARG_CSRCS += nettest_client.c
endif

TARG_COBJS = $(TARG_CSRCS:.c=$(OBJEXT))

TARG_SRCS = $(TARG_ASRCS) $(TARG_CSRCS)
TARG_OBJS = $(TARG_AOBJS) $(TARG_COBJS)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  TARG_BIN = ..\..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  TARG_BIN = ..\\..\\libapps$(LIBEXT)
else
  TARG_BIN = ../../libapps$(LIBEXT)
endif
endif

HOSTCFLAGS += -DNETTEST_HOST=1
ifeq ($(CONFIG_EXAMPLES_NETTEST_SERVER),y)
HOSTCFLAGS += -DCONFIG_EXAMPLES_NETTEST_SERVER=1 -DCONFIG_EXAMPLES_NETTEST_CLIENTIP="$(CONFIG_EXAMPLES_NETTEST_CLIENTIP)"
endif
ifeq ($(CONFIG_EXAMPLES_NETTEST_PERFORMANCE),y)
HOSTCFLAGS += -DCONFIG_EXAMPLES_NETTEST_PERFORMANCE=1
endif

HOST_SRCS = host.c
ifeq ($(CONFIG_EXAMPLES_NETTEST_SERVER),y)
HOST_SRCS += nettest_client.c
else
HOST_SRCS += nettest_server.c
endif

HOSTOBJEXT ?= .hobj
HOST_OBJS = $(HOST_SRCS:.c=$(HOSTOBJEXT))
HOST_BIN = host

ROOTDEPPATH = --dep-path .

# NET test built-in application info

APPNAME = nettest
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = 2048

# Common build

VPATH = 

all: .built $(HOST_BIN)
.PHONY: clean depend distclean

$(TARG_AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(TARG_COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(HOST_OBJS): %$(HOSTOBJEXT): %.c
	@echo "CC:  $<"
	@$(HOSTCC) -c $(HOSTCFLAGS) $< -o $@

$(HOST_BIN): $(HOST_OBJS)
	@echo "LD:  $@"
	@$(HOSTCC) $(HOSTLDFLAGS) $(HOST_OBJS) -o $@

.built: $(TARG_OBJS)
	$(call ARCHIVE, $(TARG_BIN), $(TARG_OBJS))
	@touch .built

ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
$(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat: $(DEPCONFIG) Makefile
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)

context: $(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat
else
context:
endif

.depend: Makefile $(TARG_SRCS)
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(TARG_SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, *$(HOSTOBJEXT))
	$(call DELFILE, $(HOST_BIN))
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

