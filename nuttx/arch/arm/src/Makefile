############################################################################
# arch/arm/src/Makefile
#
#   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
-include chip/Make.defs

ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(CONFIG_ARCH_CORTEXM3),y)   # Cortex-M3 is ARMv7-M
ARCH_SUBDIR = armv7-m
else
ifeq ($(CONFIG_ARCH_CORTEXM4),y)   # Cortex-M4 is ARMv7E-M
ARCH_SUBDIR = armv7-m
else
ARCH_SUBDIR = arm
endif
endif

ifeq ($(WINTOOL),y)
  NUTTX = "${shell cygpath -w $(TOPDIR)/nuttx}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}" \
            -I "${shell cygpath -w $(ARCH_SRCDIR)/common}" \
            -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}" \
            -I "${shell cygpath -w $(TOPDIR)/sched}"
else
  NUTTX = $(TOPDIR)/nuttx
  CFLAGS += -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common \
            -I$(ARCH_SRCDIR)/$(ARCH_SUBDIR) -I$(TOPDIR)/sched
endif

HEAD_OBJ = $(HEAD_ASRC:.S=$(OBJEXT))

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

LDFLAGS += $(ARCHSCRIPT)

EXTRA_LIBS ?=
EXTRA_LIBPATHS ?=
LINKLIBS ?=

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BOARDMAKE = $(dir "$(TOPDIR)\arch\$(CONFIG_ARCH)\src\board\Makefile")
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)\lib"}"
ifdef BOARDMAKE
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)\arch\$(CONFIG_ARCH)\src\board"}"
endif

else
  BOARDMAKE = $(dir "$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board/Makefile")

ifeq ($(WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/lib"}"
ifdef BOARDMAKE
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"}"
endif

else
  LIBPATHS += -L"$(TOPDIR)/lib"
ifdef BOARDMAKE
  LIBPATHS += -L"$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"
endif
endif
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))

LIBGCC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}"
GCC_LIBDIR := ${shell dirname $(LIBGCC)}

VPATH = chip:common:$(ARCH_SUBDIR)

all: $(HEAD_OBJ) libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT)

$(AOBJS) $(HEAD_OBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, "$(OBJS)")

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

nuttx: $(HEAD_OBJ) board/libboard$(LIBEXT)
	$(Q) echo "LD: nuttx"
	$(Q) $(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX)$(EXEEXT) $(HEAD_OBJ) $(EXTRA_OBJS) \
		--start-group $(LDLIBS) -lboard $(EXTRA_LIBS) $(LIBGCC) --end-group
	$(Q) $(NM) $(NUTTX)$(EXEEXT) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map

# This is part of the top-level export target
# Note that there may not be a head object if layout is handled
# by the linker configuration.

export_head: board/libboard$(LIBEXT) $(HEAD_OBJ)
ifneq ($(HEAD_OBJ),)
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
	cp -f $(HEAD_OBJ) "$(EXPORT_DIR)/startup"; \
	 else \
	echo "$(EXPORT_DIR)/startup does not exist"; \
	exit 1; \
	 fi
endif

# Dependencies

.depend: Makefile chip/Make.defs $(SRCS)
ifdef BOARDMAKE
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" depend
endif
	$(Q) $(MKDEP) --dep-path chip --dep-path common --dep-path $(ARCH_SUBDIR) \
	 "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
ifdef BOARDMAKE
	$(MAKE) -C board TOPDIR="$(TOPDIR)" clean
endif
	$(Q) rm -f libarch$(LIBEXT) *~ .*.swp
	$(call CLEAN)

distclean: clean
ifdef BOARDMAKE
	$(MAKE) -C board TOPDIR="$(TOPDIR)" distclean
endif
	$(Q) rm -f Make.dep .depend

-include Make.dep
