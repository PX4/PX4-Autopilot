############################################################################
# insn16/libinsn/Makefile
#
#   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
#
# Directories
#
LIBINSNDIR		= ${shell pwd}
INSNDIR			= $(LIBINSNDIR)/..
PASCAL			= $(LIBINSNDIR)/../..

include $(PASCAL)/Make.config
include $(PASCAL)/Make.defs

INCDIR			= $(PASCAL)/include
LIBDIR			= $(PASCAL)/lib

#
# Tools
#
EXTRA_INCLUDES		= -I$(INSNDIR)/include
INCLUDES		+= $(EXTRA_INCLUDES)
CFLAGS			+= $(EXTRA_INCLUDES)

#
# Objects and targets
#
LIBINSNSRCS		= paddopcode.c paddtmpopcode.c pdasm.c pgen.c \
			  pgetopcode.c preloc.c
LIBINSNOBJS		= $(LIBINSNSRCS:.c=.o)

OBJS			= $(LIBINSNOBJS)

all: libinsn.a
.PHONY: all libinsn.a clean

$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(LIBDIR)/libinsn.a: $(LIBINSNOBJS)
	$(AR) $(ARFLAGS) $@ $^

libinsn.a: $(LIBDIR)/libinsn.a

clean:
	$(RM) libinsn.a *.o core *~
