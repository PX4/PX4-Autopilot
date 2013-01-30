############################################################################
# nxflat/Makefile
#
#   Copyright (C) 2007-2009, 2012-2013 Gregory Nutt. All rights reserved.
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
DELIM ?= $(strip /)

ifeq ($(WINTOOL),y)
INCDIROPT = -w
endif
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(TOPDIR)$(DELIM)sched"}

# Basic BINFMT source files 

BINFMT_ASRCS  =
BINFMT_CSRCS  = binfmt_globals.c binfmt_register.c binfmt_unregister.c
BINFMT_CSRCS += binfmt_loadmodule.c binfmt_unloadmodule.c binfmt_execmodule.c
BINFMT_CSRCS += binfmt_exec.c binfmt_dumpmodule.c

ifeq ($(CONFIG_BINFMT_EXEPATH),y)
BINFMT_CSRCS += binfmt_exepath.c
endif

ifeq ($(CONFIG_SCHED_HAVE_PARENT),y)
BINFMT_CSRCS += binfmt_schedunload.c
endif

# Symbol table source files

BINFMT_CSRCS += symtab_findbyname.c symtab_findbyvalue.c
BINFMT_CSRCS += symtab_findorderedbyname.c symtab_findorderedbyvalue.c

# Add configured binary modules

VPATH = 
SUBDIRS =
DEPPATH = --dep-path .

include libnxflat$(DELIM)Make.defs
include libelf$(DELIM)Make.defs
include libbuiltin$(DELIM)Make.defs

BINFMT_AOBJS = $(BINFMT_ASRCS:.S=$(OBJEXT))
BINFMT_COBJS = $(BINFMT_CSRCS:.c=$(OBJEXT))

BINFMT_SRCS = $(BINFMT_ASRCS) $(BINFMT_CSRCS)
BINFMT_OBJS = $(BINFMT_AOBJS) $(BINFMT_COBJS)

BIN = libbinfmt$(LIBEXT)

all: $(BIN)

$(BINFMT_AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(BINFMT_COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(BINFMT_OBJS)
	$(call ARCHIVE, $@, $(BINFMT_OBJS))

.depend: Makefile $(BINFMT_SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(BINFMT_SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
