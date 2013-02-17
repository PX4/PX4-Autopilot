############################################################################
# apps/nshlib/Makefile
#
#   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
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

# NSH Library

ASRCS  =
CSRCS  = nsh_init.c nsh_parse.c nsh_console.c nsh_script.c nsh_session.c
CSRCS += nsh_fscmds.c nsh_ddcmd.c nsh_proccmds.c nsh_mmcmds.c nsh_envcmds.c
CSRCS += nsh_dbgcmds.c

ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
CSRCS += nsh_builtin.c
endif

ifeq ($(CONFIG_NSH_FILE_APPS),y)
CSRCS += nsh_fileapps.c
endif

ifeq ($(CONFIG_NSH_ROMFSETC),y)
CSRCS += nsh_romfsetc.c
endif

ifeq ($(CONFIG_NET),y)
CSRCS += nsh_netinit.c nsh_netcmds.c
endif

ifeq ($(CONFIG_RTC),y)
CSRCS += nsh_timcmds.c
endif

ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)
CSRCS += nsh_mntcmds.c
endif

ifeq ($(CONFIG_NSH_CONSOLE),y)
CSRCS += nsh_consolemain.c
endif

ifeq ($(CONFIG_NSH_TELNET),y)
CSRCS += nsh_telnetd.c
endif

ifneq ($(CONFIG_NSH_DISABLESCRIPT),y)
CSRCS += nsh_test.c
endif

ifeq ($(CONFIG_USBDEV),y)
CSRCS += nsh_usbdev.c
endif

ifeq ($(CONFIG_NETUTILS_CODECS),y)
CSRCS += nsh_codeccmd.c
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN		= ..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN		= ..\\libapps$(LIBEXT)
else
  BIN		= ../libapps$(LIBEXT)
endif
endif

ROOTDEPPATH = --dep-path .
VPATH = 

# Build targets

all: .built
.PHONY: context .depend depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

context:

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

