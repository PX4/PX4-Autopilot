############################################################################
# apps/interpreters/pcode/Makefile
#
#   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

PCODEDIR := ${shell pwd | sed -e 's/ /\\ /g'}

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# Default tools

ifeq ($(DIRLINK),)
DIRLINK = $(TOPDIR)/tools/link.sh
DIRUNLINK = $(TOPDIR)/tools/unlink.sh
INCDIR = $(TOPDIR)/tools/incdir.sh
endif

ifeq ($(WINTOOL),y)
INCDIROPT = -w
endif

USRINCLUDES = ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(PCODEDIR)/include $(PCODEDIR)/insn/include}

COMPILER = ${shell basename $(CC)}
ifeq ($(COMPILER),zneocc.exe)
INCLUDES = $(ARCHSTDINCLUDES) $(USRINCLUDES)
else
INCLUDES = $(ARCHINCLUDES) $(USRINCLUDES)
endif

CFLAGS = $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(INCLUDES) $(ARCHDEFINES)

include insn/prun/Make.defs
include libpoff/Make.defs
include libpas/Make.defs

ASRCS = $(PRUN_ASRCS) $(POFF_ASRCS) $(PAS_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(PRUN_CSRCS) $(POFF_CSRCS) $(PAS_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ifeq ($(WINTOOL),y)
  BIN		= "${shell cygpath -w  $(APPDIR)/libapps$(LIBEXT)}"
else
  BIN		= "$(APPDIR)/libapps$(LIBEXT)"
endif

ROOTDEPPATH	= --dep-path .
PRUNDEPPATH	= --dep-path insn/prun
POFFDEPPATH = --dep-path libpoff
PASDEPPATH  = --dep-path libpas

VPATH = insn/prun:libpoff:libpas

all: .built
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
ifeq ($(COMPILER),zneocc.exe)
	$(call ASSEMBLE, `cygpath -w $<`, $@)
else
	$(call ASSEMBLE, $<, $@)
endif

$(COBJS): %$(OBJEXT): %.c
ifeq ($(COMPILER),zneocc.exe)
	$(call COMPILE, `cygpath -w $<`, $@)
else
	$(call COMPILE, $<, $@)
endif

$(APPDIR)/include/pcode: include
	@$(DIRLINK) $(PCODEDIR)/include $(APPDIR)/include/pcode

$(APPDIR)/include/pcode/insn: $(APPDIR)/include/pcode insn/include
	@$(DIRLINK) $(PCODEDIR)/insn/include $(APPDIR)/include/pcode/insn

.built: $(APPDIR)/include/pcode $(APPDIR)/include/pcode/insn $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $(BIN), $${obj}); \
	done ; )
	@touch .built

context: $(APPDIR)/include/pcode $(APPDIR)/include/pcode/insn

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) $(PRUNDEPPATH) $(POFFDEPPATH) $(PASDEPPATH) \
		$(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f *.o *~ .*.swp .built
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend
	@$(DIRUNLINK) $(APPDIR)/include/pcode/insn
	@$(DIRUNLINK) $(APPDIR)/include/pcode

-include Make.dep
