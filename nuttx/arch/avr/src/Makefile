############################################################################
# arch/avr/src/Makefile
#
#   Copyright (C) 2010-2012 Gregory Nutt. All rights reserved.
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
ifeq ($(CONFIG_ARCH_AVR32),y)
ARCH_SUBDIR = avr32
else ifeq ($(CONFIG_ARCH_AVR),y)
ARCH_SUBDIR = avr
endif

ifeq ($(WINTOOL),y)
  NUTTX = "${shell cygpath -w $(TOPDIR)/nuttx}"
  INCLUDES += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}"
  INCLUDES += -I "${shell cygpath -w $(ARCH_SRCDIR)/common}"
  INCLUDES += -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}"
  INCLUDES += -I "${shell cygpath -w $(TOPDIR)/sched}"
else
  NUTTX = "$(TOPDIR)/nuttx"
  INCLUDES += -I "$(ARCH_SRCDIR)/chip"
  INCLUDES += -I "$(ARCH_SRCDIR)/common"
  INCLUDES += -I "$(ARCH_SRCDIR)/$(ARCH_SUBDIR)"
  INCLUDES += -I "$(TOPDIR)/sched"
endif

CPPFLAGS += $(INCLUDES)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
AFLAGS += $(INCLUDES)

HEAD_OBJ = $(HEAD_ASRC:.S=$(OBJEXT))

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

LDFLAGS += $(ARCHSCRIPT)
EXTRA_LIBS ?=
LINKLIBS ?=

ifeq ($(WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/lib"}"
  LIBPATHS += -L"${shell cygpath -w $(BOARDDIR)}"
else
  LIBPATHS += -L"(TOPDIR)/lib"
  LIBPATHS += -L"$(BOARDDIR)"
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))

BOARDDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src/board

LIBGCC = "${shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name}"

VPATH = chip:common:$(ARCH_SUBDIR)

all: $(HEAD_OBJ) libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT)

$(AOBJS) $(HEAD_OBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

board/libboard$(LIBEXT):
	@$(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

nuttx: $(HEAD_OBJ) board/libboard$(LIBEXT)
	@echo "LD: nuttx"
	@$(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) -o $(NUTTX)$(EXEEXT) $(HEAD_OBJ) $(EXTRA_OBJS) \
		--start-group $(LDLIBS) -lboard  $(EXTRA_LIBS) $(LIBGCC) --end-group
	@$(NM) $(NUTTX)$(EXEEXT) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map

# This is part of the top-level export target

export_head: board/libboard$(LIBEXT) $(HEAD_OBJ)
	@if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(HEAD_OBJ) "$(EXPORT_DIR)/startup"; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
		exit 1; \
	 fi

# Dependencies

.depend: Makefile chip/Make.defs $(SRCS)
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" depend ; \
	fi
	@$(MKDEP) --dep-path chip --dep-path common --dep-path $(ARCH_SUBDIR) \
	 $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" clean ; \
	fi
	@rm -f libarch$(LIBEXT) *~ .*.swp
	$(call CLEAN)

distclean: clean
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" distclean ; \
	fi
	@rm -f Make.dep .depend

-include Make.dep

