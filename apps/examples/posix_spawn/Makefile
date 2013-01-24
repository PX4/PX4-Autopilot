############################################################################
# apps/examples/posix_spawn/Makefile
#
#   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

# ELF Example

ASRCS =
CSRCS = spawn_main.c symtab.c

AOBJS = $(ASRCS:.S=$(OBJEXT))
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

ROOTDEPPATH	= --dep-path . --dep-path filesystem

# Build targets

VPATH = filesystem

all: build
.PHONY: build clean_filesystem clean depend distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# This is a little messy.  The build is broken into two pieces:  (1) the
# filesystem/ subdir build that auto-generates several files, and (2) the library
# build.  This is done because we need a fresh build context after auto-
# generating the source files.

build_lib: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))

build:
	@$(MAKE) -C filesystem TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)
	@$(MAKE) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" build_lib

context:

# We can't make dependencies in this directory because the required
# header files may not yet exist.

.depend:
	@touch $@

depend: .depend

clean_filesystem:
	@$(MAKE) -C filesystem TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

clean: clean_filesystem
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

# There are no dependencies in this directory.  Some of more important
# and more obvious dependencies are hard-coded here:

spawn_main.o: spawn_main.c \
	$(TOPDIR)/include/nuttx/config.h \
	$(TOPDIR)/include/nuttx/compiler.h \
	$(TOPDIR)/include/sys/mount.h \
	$(TOPDIR)/include/stdio.h \
	$(TOPDIR)/include/stdlib.h \
	$(TOPDIR)/include/unistd.h \
	$(TOPDIR)/include/string.h \
	$(TOPDIR)/include/fcntl.h \
	$(TOPDIR)/include/spawn.h \
	$(TOPDIR)/include/debug.h \
	$(TOPDIR)/include/errno.h \
	$(TOPDIR)/include/nuttx/ramdisk.h \
	$(TOPDIR)/include/nuttx/binfmt/elf.h \
	$(TOPDIR)/include/nuttx/binfmt/symtab.h \
	filesystem/romfs.h

symtab.o: filesystem/symtab.c \
	$(TOPDIR)/include/nuttx/compiler.h \
	$(TOPDIR)/include/nuttx/binfmt/symtab.h
