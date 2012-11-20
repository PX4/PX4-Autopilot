############################################################################
# net/Makefile
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

ifeq ($(CONFIG_NET),y)

# Basic networking support

SOCK_ASRCS =
SOCK_CSRCS = bind.c connect.c getsockname.c recv.c recvfrom.c socket.c \
		  sendto.c net_sockets.c net_close.c net_dup.c net_dup2.c net_clone.c \
		  net_vfcntl.c

# TCP/IP support

ifeq ($(CONFIG_NET_TCP),y)
SOCK_CSRCS += send.c listen.c accept.c net_monitor.c
endif

# Socket options

ifeq ($(CONFIG_NET_SOCKOPTS),y)
SOCK_CSRCS += setsockopt.c getsockopt.c 
ifneq ($(CONFIG_DISABLE_CLOCK),y)
SOCK_CSRCS += net_timeo.c net_dsec2timeval.c net_timeval2dsec.c
ifneq ($(CONFIG_NET_SLIP),y)
SOCK_CSRCS += net_arptimer.c
endif
endif
endif

# Support for network access using streams

ifneq ($(CONFIG_NFILE_STREAMS),0)
SOCK_CSRCS += net_checksd.c 
endif

# Support for operations on network devices

NETDEV_ASRCS =
NETDEV_CSRCS = netdev_register.c netdev_ioctl.c net_poll.c netdev_txnotify.c \
		  netdev_findbyname.c netdev_findbyaddr.c netdev_count.c \
		  netdev_foreach.c netdev_unregister.c netdev_sem.c

include uip/Make.defs
endif

ASRCS		= $(SOCK_ASRCS) $(NETDEV_ASRCS) $(UIP_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(SOCK_CSRCS) $(NETDEV_CSRCS) $(UIP_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libnet$(LIBEXT)

VPATH		= uip

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
ifeq ($(CONFIG_NET),y)
	$(Q) $(MKDEP) --dep-path . --dep-path uip "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
