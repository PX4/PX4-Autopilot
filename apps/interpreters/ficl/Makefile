############################################################################
# apps/interpreters/ficl/Makefile
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

BUILDDIR := ${shell pwd | sed -e 's/ /\\ /g'}

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# Tools

INCDIR = $(TOPDIR)/tools/incdir.sh

ifeq ($(WINTOOL),y)
INCDIROPT = -w
endif

# Include paths

CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(BUILDDIR)/$(FICL_SUBDIR) $(BUILDDIR)/src}

# Source Files

ASRCS =
CXXSRCS =
CSRCS = nuttx.c

-include Make.srcs

ASRCS += $(FICL_ASRCS)
CXXSRCS += $(FICL_CXXSRCS)
CSRCS += $(FICL_CSRCS)

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ifeq ($(WINTOOL),y)
  BIN		= "${shell cygpath -w  $(APPDIR)/libapps$(LIBEXT)}"
else
  BIN		= "$(APPDIR)/libapps$(LIBEXT)"
endif

ROOT_DEPPATH = --dep-path .
SRC_DEPPATH = --dep-path src

VPATH = src:$(FICL_SUBDIR)

all: .built
.PHONY: debug context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

debug:
	@#echo "FICL:   $(FICL_SUBDIR)"
	@#echo "VPATH:  $(VPATH)"
	@#echo "CFLAGS: $(CFLAGS)"

.built: debug $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $(BIN), $${obj}); \
	done ; )
	@touch .built

context:

.depend: debug Makefile $(SRCS)
	@$(MKDEP) $(ROOT_DEPPATH) $(SRC_DEPPATH) $(FICL_DEPPATH) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f *.o *~ .*.swp .built
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
