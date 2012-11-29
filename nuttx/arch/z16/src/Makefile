############################################################################
# arch/z16/src/Makefile
#
#   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
-include chip/Make.defs

COMPILER = ${shell basename "$(CC)"}
ARCHSRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  USRINCLUDES = -usrinc:'.;$(TOPDIR)/sched;$(ARCHSRCDIR);$(ARCHSRCDIR)/common'
else
ifeq ($(COMPILER),zneocc.exe)
  WARCHSRCDIR := ${shell cygpath -w $(ARCHSRCDIR)}
  USRINCLUDES = -usrinc:'.;$(WTOPDIR)\sched;$(WARCHSRCDIR);$(WARCHSRCDIR)\common'
else
  WARCHSRCDIR = $(ARCHSRCDIR)
  USRINCLUDES = -I$(TOPDIR)/sched -I$(ARCHSRCDIR) -I$(ARCHSRCDIR)/common
endif
endif

INCLUDES = $(ARCHSTDINCLUDES) $(USRINCLUDES)
CFLAGS = $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(INCLUDES) $(ARCHDEFINES) $(EXTRADEFINES)
CPPFLAGS += -I$(ARCHSRCDIR)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  LDFLAGS += @"$(ARCHSRCDIR)/nuttx.linkcmd"
else
ifeq ($(COMPILER),zneocc.exe)
  LDFLAGS += @"${shell cygpath -w $(ARCHSRCDIR)/nuttx.linkcmd}"
endif
endif

HEAD_ASRC = $(HEAD_SSRC:.S=$(ASMEXT))
HEAD_OBJ = $(HEAD_SSRC:.S=$(OBJEXT))

SSRCS = $(CHIP_SSRCS) $(CMN_SSRCS)
ASRCS = $(SSRCS:.S=$(ASMEXT))
AOBJS = $(SSRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

DEPSRCS = $(SSRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

VPATH = chip:common

all: $(HEAD_OBJ) libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT)

ifeq ($(COMPILER),zneocc.exe)
$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
	$(Q) $(CPP) $(CPPFLAGS) $< -o $@.tmp
	$(Q) cat $@.tmp | sed -e "s/^#/;/g" > $@
	$(Q) rm $@.tmp

$(AOBJS) $(HEAD_OBJ): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)
else
$(OBJS) $(HEAD_OBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)
endif

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

ifeq ($(COMPILER),zneocc.exe)
nuttx.linkcmd: $(LINKCMDTEMPLATE)
	$(Q) cp -f $(LINKCMDTEMPLATE) nuttx.linkcmd
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	@echo "$(TOPDIR)\nuttx"= \>>nuttx.linkcmd
	@echo   "$(ARCHSRCDIR)\$(HEAD_OBJ)", \>>nuttx.linkcmd
	$(Q) for %%G in ($(LINKLIBS)) do ( echo   "$(TOPDIR)\lib\%%G", \>>nuttx.linkcmd )
	@echo   "$(ARCHSRCDIR)\board\libboard$(LIBEXT)", \>>nuttx.linkcmd
	@echo   "$(ZDSSTDLIBDIR)\chelpld$(LIBEXT)", \>>nuttx.linkcmd
	@echo   "$(ZDSSTDLIBDIR)\fpld$(LIBEXT)">>nuttx.linkcmd
else
	@echo "\"${shell cygpath -w $(TOPDIR)/nuttx}\"= \\" >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ARCHSRCDIR)/$(HEAD_OBJ)}\", \\" >>nuttx.linkcmd
	$(Q) ( for lib in $(LINKLIBS); do \
		echo "  \"`cygpath -w $(TOPDIR)/lib/$${lib}`\", \\" >>nuttx.linkcmd; \
	done ; )
	@echo "  \"${shell cygpath -w $(ARCHSRCDIR)/board/libboard$(LIBEXT)}\", \\"  >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ZDSSTDLIBDIR)/chelpld$(LIBEXT)}\", \\"  >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ZDSSTDLIBDIR)/fpld$(LIBEXT)}\""  >>nuttx.linkcmd
endif
else
nuttx.linkcmd:
endif

nuttx$(EXEEXT): $(HEAD_OBJ) board/libboard$(LIBEXT) nuttx.linkcmd
	@echo "LD:  nuttx$(EXEEXT)"
	$(Q) $(LD) $(LDFLAGS)

.depend: Makefile chip/Make.defs $(DEPSRCS)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist board$(DELIM)Makefile ( $(MAKE) -C board TOPDIR="$(TOPDIR)" depend )
else
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" depend ; \
	fi
endif
	$(Q) $(MKDEP) --dep-path chip --dep-path common "$(CC)" -- $(CFLAGS) -- $(DEPSRCS) >Make.dep
	$(Q) touch $@

# This is part of the top-level export target

export_head: board/libboard$(LIBEXT) $(HEAD_OBJ)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist "$(EXPORT_DIR)$(DELIM)startup" ( copy $(HEAD_OBJ) "$(EXPORT_DIR)$(DELIM)startup$(DELIM)." /b /y)
else
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(HEAD_OBJ) "$(EXPORT_DIR)/startup"; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
		exit 1; \
	 fi
endif

# Dependencies

depend: .depend

clean:
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist board$(DELIM)Makefile ( $(MAKE) -C board TOPDIR="$(TOPDIR)" clean )
else
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" clean ; \
	fi
endif
ifeq ($(COMPILER),zneocc.exe)
	$(call DELFILE, nuttx.linkcmd)
	$(call DELFILE, *.asm)
	$(call DELFILE, *.tmp)
	$(call DELFILE, *.map)
endif
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean: clean
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist board$(DELIM)Makefile ( $(MAKE) -C board TOPDIR="$(TOPDIR)" distclean )
else
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" distclean ; \
	fi
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
