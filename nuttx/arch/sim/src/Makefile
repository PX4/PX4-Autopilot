############################################################################
# arch/sim/src/Makefile
#
#   Copyright (C) 2007, 2008, 2011-2012 Gregory Nutt. All rights reserved.
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

CFLAGS += -I$(TOPDIR)/sched

ASRCS = up_setjmp.S
AOBJS = $(ASRCS:.S=$(OBJEXT))
CSRCS = up_initialize.c up_idle.c up_interruptcontext.c \
		up_initialstate.c up_createstack.c up_usestack.c \
		up_releasestack.c  up_unblocktask.c up_blocktask.c \
		up_releasepending.c up_reprioritizertr.c \
		up_exit.c up_schedulesigaction.c up_allocateheap.c \
		up_devconsole.c
HOSTSRCS = up_stdio.c up_hostusleep.c

ifeq ($(CONFIG_NX_LCDDRIVER),y)
  CSRCS += up_lcd.c
else
  CSRCS += up_framebuffer.c
ifeq ($(CONFIG_SIM_X11FB),y)
  HOSTSRCS += up_x11framebuffer.c
ifeq ($(CONFIG_SIM_TOUCHSCREEN),y)
  CSRCS += up_touchscreen.c
  HOSTSRCS += up_x11eventloop.c
endif
endif
endif

ifeq ($(CONFIG_ELF),y)
CSRCS += up_elf.c
endif

ifeq ($(CONFIG_FS_FAT),y)
CSRCS += up_blockdevice.c up_deviceimage.c
endif

ifeq ($(CONFIG_ARCH_ROMGETC),y)
CSRCS += up_romgetc.c
endif

ifeq ($(CONFIG_NET),y)
CSRCS += up_uipdriver.c
HOSTCFLAGS += -DNETDEV_BUFSIZE=$(CONFIG_NET_BUFSIZE)
ifneq ($(HOSTOS),Cygwin)
HOSTSRCS += up_tapdev.c up_netdev.c
else
HOSTSRCS += up_wpcap.c up_netdev.c
DRVLIB = /lib/w32api/libws2_32.a /lib/w32api/libiphlpapi.a
endif
endif

COBJS = $(CSRCS:.c=$(OBJEXT))

NUTTXOBJS = $(AOBJS) $(COBJS)
HOSTOBJS = $(HOSTSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(HOSTSRCS)
OBJS = $(AOBJS) $(COBJS) $(HOSTOBJS)

# Determine which standard libraries will need to be linked in

ifeq ($(CONFIG_SIM_X11FB),y)
  STDLIBS += -lX11 -lXext
ifeq ($(CONFIG_SIM_TOUCHSCREEN),y)
  STDLIBS += -lpthread
endif
endif

EXTRA_LIBS ?=
EXTRA_LIBPATHS ?=

ifeq ($(CONFIG_FS_FAT),y)
STDLIBS += -lz
endif

STDLIBS += -lc

# Determine which objects are required in the link.  The
# up_head object normally draws in all that is needed, but
# there are a fews that must be included because they 
# are called only from the host OS-specific logic (HOSTOBJS)

LINKOBJS = up_head$(OBJEXT)
REQUIREDOBJS = $(LINKOBJS)

ifeq ($(CONFIG_SIM_X11FB),y)
ifeq ($(CONFIG_SIM_TOUCHSCREEN),y)
  REQUIREDOBJS += up_touchscreen.o
endif
endif

# Determine which NuttX libraries will need to be linked in
# Most are provided by LINKLIBS on the MAKE command line

LINKLIBS ?=
RELLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
RELPATHS += -L"$(TOPDIR)/lib"

# Add the board-specific library and directory

LIBPATHS += -L board 
RELPATHS += -L board
RELLIBS += -lboard

# Make targets begin here

all: up_head$(OBJEXT) libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT) export_head clean distclean cleanrel depend

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(HOSTOBJS): %$(OBJEXT): %.c
	$(Q) echo "CC:  $<"
	$(Q) "$(CC)" -c $(HOSTCFLAGS) $< -o $@

# The architecture-specific library

libarch$(LIBEXT): $(NUTTXOBJS)
	$(call ARCHIVE, $@, $(NUTTXOBJS))

# The "board"-specific library. Of course, there really are no boards in
# the simulation.  However, this is a good place to keep parts of the simulation
# that are not hardware-related.

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

# A partially linked object containing only NuttX code (no interface to host OS)
# Change the names of most symbols that conflict with libc symbols.

GNU:
	$(Q) mkdir ./GNU

GNU/Linux-names.dat: GNU nuttx-names.dat
	$(Q) cp nuttx-names.dat $@

Cygwin-names.dat: nuttx-names.dat
	$(Q) cat $^ | sed -e "s/^/_/g" >$@

nuttx.rel : libarch$(LIBEXT) board/libboard$(LIBEXT) $(HOSTOS)-names.dat $(LINKOBJS)
	$(Q) echo "LD:  nuttx.rel"
	$(Q) $(LD) -r $(LDLINKFLAGS) $(RELPATHS) $(EXTRA_LIBPATHS) -o $@ $(REQUIREDOBJS) --start-group $(RELLIBS) $(EXTRA_LIBS) --end-group
	$(Q) $(OBJCOPY) --redefine-syms=$(HOSTOS)-names.dat $@

# Generate the final NuttX binary by linking the host-specific objects with the NuttX
# specific objects (with munged names)

nuttx$(EXEEXT): cleanrel nuttx.rel $(HOSTOBJS)
	$(Q) echo "LD:  nuttx$(EXEEXT)"
	$(Q) "$(CC)" $(CCLINKFLAGS) $(LIBPATHS) -o $(TOPDIR)/$@ nuttx.rel $(HOSTOBJS) $(DRVLIB) $(STDLIBS)
	$(Q) $(NM) $(TOPDIR)/$@ | \
		grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
		sort > $(TOPDIR)/System.map
	$(Q) rm -f nuttx.rel

# This is part of the top-level export target

export_head: board/libboard$(LIBEXT) up_head.o $(HOSTOBJS)
	cp up_head.o $(HOSTOBJS) ${EXPORT_DIR}/startup
	cp nuttx-names.dat ${EXPORT_DIR}/libs
	echo main NXmain >> ${EXPORT_DIR}/libs/nuttx-names.dat

# Dependencies

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

cleanrel: 
	$(Q) rm -f nuttx.rel GNU/Linux-names.dat Cygwin-names.dat

clean: cleanrel
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" clean ; \
	fi
	$(call DELFILE, nuttx.rel)
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" distclean ; \
	fi
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(Q) rm -rf GNU

-include Make.dep
