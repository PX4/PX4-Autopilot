############################################################################
# apps/interpreters/pcode/Makefile
#
#   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
include $(APPDIR)$(DELIM)Make.defs
DELIM ?= $(strip /)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  PCODEDIR := ${shell echo %CD%}
else
  PCODEDIR := ${shell pwd | sed -e 's/ /\\ /g'}
endif

# Default tools

ifeq ($(DIRLINK),)
  DIRLINK = $(TOPDIR)$(DELIM)tools$(DELIM)link.sh
  DIRUNLINK = $(TOPDIR)$(DELIM)tools$(DELIM)unlink.sh
endif

ifeq ($(WINTOOL),y)
  INCDIROPT = -w
endif

USRINCLUDES = ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(PCODEDIR)$(DELIM)include $(PCODEDIR)$(DELIM)insn$(DELIM)include}

COMPILER = ${shell basename $(CC)}
ifeq ($(COMPILER),zneocc.exe)
  INCLUDES = $(ARCHSTDINCLUDES) $(USRINCLUDES)
else
  INCLUDES = $(ARCHINCLUDES) $(USRINCLUDES)
endif

CFLAGS = $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(INCLUDES) $(ARCHDEFINES)

include insn$(DELIM)prun$(DELIM)Make.defs
include libpoff$(DELIM)Make.defs
include libpas$(DELIM)Make.defs

ASRCS = $(PRUN_ASRCS) $(POFF_ASRCS) $(PAS_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(PRUN_CSRCS) $(POFF_CSRCS) $(PAS_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN		= ..\..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN		= ..\\..\\libapps$(LIBEXT)
else
  BIN		= ../../libapps$(LIBEXT)
endif
endif

ROOTDEPPATH = --dep-path .
PRUNDEPPATH = --dep-path insn$(DELIM)prun
POFFDEPPATH = --dep-path libpoff
PASDEPPATH  = --dep-path libpas

VPATH = insn$(DELIM)prun:libpoff:libpas

all: .built
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(APPDIR)$(DELIM)include$(DELIM)pcode: include
	@$(DIRLINK) $(PCODEDIR)$(DELIM)include $(APPDIR)$(DELIM)include$(DELIM)pcode

$(APPDIR)$(DELIM)include$(DELIM)pcode$(DELIM)insn: $(APPDIR)$(DELIM)include$(DELIM)pcode insn$(DELIM)include
	@$(DIRLINK) $(PCODEDIR)$(DELIM)insn$(DELIM)include $(APPDIR)$(DELIM)include$(DELIM)pcode$(DELIM)insn

.built: $(APPDIR)$(DELIM)include$(DELIM)pcode $(APPDIR)$(DELIM)include$(DELIM)pcode$(DELIM)insn $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

context: $(APPDIR)$(DELIM)include$(DELIM)pcode $(APPDIR)$(DELIM)include$(DELIM)pcode$(DELIM)insn

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) $(PRUNDEPPATH) $(POFFDEPPATH) $(PASDEPPATH) \
		$(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	@$(DIRUNLINK) $(APPDIR)$(DELIM)include$(DELIM)pcode$(DELIM)insn
	@$(DIRUNLINK) $(APPDIR)$(DELIM)include$(DELIM)pcode

-include Make.dep
