#################################################################################
# NxWidgets/UnitTests/CLatchButtonArray/Makefile
#
#   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
# 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
#    me be used to endorse or promote products derived from this software
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
#################################################################################

TESTDIR := ${shell pwd | sed -e 's/ /\\ /g'}

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# Add the path to the NXWidget include directory to the CFLAGS

NXWIDGETS_DIR="$(TESTDIR)/../../libnxwidgets"
NXWIDGETS_INC="$(NXWIDGETS_DIR)/include"
NXWIDGETS_LIB="$(NXWIDGETS_DIR)/libnxwidgets$(LIBEXT)"

INCDIR=$(TOPDIR)/tools/incdir.sh
ifeq ($(WINTOOL),y)
  CFLAGS += ${shell $(INCDIR) -w "$(CC)" "$(NXWIDGETS_INC)"}
  CXXFLAGS += ${shell $(INCDIR) -w "$(CXX)" "$(NXWIDGETS_INC)"}
else
  CFLAGS += ${shell $(INCDIR) "$(CC)" "$(NXWIDGETS_INC)"}
  CXXFLAGS += ${shell $(INCDIR) "$(CXX)" "$(NXWIDGETS_INC)"}
endif

# Get the path to the archiver tool

TESTTOOL_DIR="$(TESTDIR)/../../tools"
ARCHIVER=$(TESTTOOL_DIR)/addobjs.sh

# Hello, World! C++ Example

ASRCS		=
CSRCS		=
CXXSRCS		= clatchbuttonarray_main.cxx clatchbuttonarraytest.cxx

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))
CXXOBJS		= $(CXXSRCS:.cxx=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS) $(CXXSRCS)
OBJS		= $(AOBJS) $(COBJS) $(CXXOBJS)

POSIX_BIN	= "$(APPDIR)/libapps$(LIBEXT)"
ifeq ($(WINTOOL),y)
  BIN		= "${shell cygpath -w  $(POSIX_BIN)}"
else
  BIN		= $(POSIX_BIN)
endif

ROOTDEPPATH	= --dep-path .

# helloxx built-in application info

APPNAME		= clatchbuttonarray
PRIORITY	= SCHED_PRIORITY_DEFAULT
STACKSIZE	= 2048

# Common build

VPATH		= 

all: .built
.PHONY:	clean depend context disclean chkcxx chklib

# Object file creation targets

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): %$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

# Verify that the NuttX configuration is setup to support C++

chkcxx:
ifneq ($(CONFIG_HAVE_CXX),y)
	@echo ""
	@echo "In order to use this example, you toolchain must support must"
	@echo ""
	@echo "  (1) Explicitly select CONFIG_HAVE_CXX to build in C++ support"
	@echo "  (2) Define CXX, CXXFLAGS, and COMPILEXX in the Make.defs file"
	@echo "      of the configuration that you are using."
	@echo ""
	@exit 1
endif

# Verify that the NXWidget library has been built

chklib:
	@( \
		if [ ! -e "$(NXWIDGETS_LIB)" ]; then \
			echo "$(NXWIDGETS_LIB) does not exist."; \
			echo "Please go to $(NXWIDGETS_DIR)"; \
			echo "and rebuild the library"; \
			exit 1; \
		fi; \
	  )

# Library creation targets

$(NXWIDGETS_LIB): # Just to keep make happy.  chklib does the work.

.built: chkcxx chklib $(OBJS) $(NXWIDGETS_LIB)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $(BIN), $${obj}); \
	   done ; )
ifeq ($(WINTOOL),y)
	@$(ARCHIVER) -w -p "$(CROSSDEV)" $(BIN) $(NXWIDGETS_DIR)
else
	@$(ARCHIVER) -p "$(CROSSDEV)" $(BIN) $(NXWIDGETS_DIR)
endif
	@touch .built

# Standard housekeeping targets

.context:
ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)
endif
	@touch $@

context: .context

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) $(CXX) -- $(CXXFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f *.o *~ .*.swp .built
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
