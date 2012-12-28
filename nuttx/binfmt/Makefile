############################################################################
# nxflat/Makefile
#
#   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
INCDIROPT	= -w
endif
CFLAGS		+= ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/sched}

ifeq ($(CONFIG_NXFLAT),y)
include libnxflat/Make.defs
LIBNXFLAT_CSRCS	+= nxflat.c 
endif

BINFMT_ASRCS	=
BINFMT_CSRCS	= binfmt_globals.c binfmt_register.c binfmt_unregister.c \
		  binfmt_loadmodule.c binfmt_unloadmodule.c binfmt_execmodule.c \
		  binfmt_exec.c binfmt_dumpmodule.c

SYMTAB_ASRCS	=
SYMTAB_CSRCS	= symtab_findbyname.c symtab_findbyvalue.c \
		  symtab_findorderedbyname.c symtab_findorderedbyvalue.c

SUBDIRS		= libnxflat

ASRCS		= $(BINFMT_ASRCS) $(SYMTAB_ASRCS) $(LIBNXFLAT_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(BINFMT_CSRCS) $(SYMTAB_CSRCS) $(LIBNXFLAT_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libbinfmt$(LIBEXT)

VPATH		= libnxflat

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

.depend: Makefile $(SRCS)
	@$(MKDEP) --dep-path . --dep-path libnxflat \
		$(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f $(BIN) *~ .*.swp
	$(call CLEAN)
	@( for dir in $(SUBDIRS); do \
		rm -f $${dir}/*~ $${dir}/.*.swp; \
	done ; )

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
