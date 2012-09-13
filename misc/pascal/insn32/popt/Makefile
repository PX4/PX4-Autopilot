############################################################################
# insn32/popt/Makefile
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
POPTDIR		= ${shell pwd}
INSNDIR		= $(POPTDIR)/..
PASCAL		= $(POPTDIR)/../..

include $(PASCAL)/Make.config
include $(PASCAL)/Make.defs

INCDIR		= $(PASCAL)/include
LIBDIR		= $(PASCAL)/lib
BINDIR		= $(PASCAL)/bin32

# ----------------------------------------------------------------------
# Tools

EXTRA_INCLUDES	= -I$(INSNDIR)/include
INCLUDES	+= $(EXTRA_INCLUDES)
CFLAGS		+= $(EXTRA_INCLUDES)

# ----------------------------------------------------------------------
# Objects and targets

POPTSRCS	= popt.c psopt.c polocal.c pcopt.c pjopt.c plopt.c pfopt.c
POPTOBJS	= $(POPTSRCS:.c=.o)

OBJS		= $(POPTOBJS)
LIBS		= libpoff.a libpas.a

all: popt
.PHONY: all check_libs popt clean

$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

check_libs:
	@if [ ! -f $(LIBDIR)/libpoff.a ] ; then \
		echo "$(LIBDIR)/libpoff.a does not exist" ; \
		exit 1 ; \
	fi
	@if [ ! -f $(LIBDIR)/libpas.a ] ; then \
		echo "$(LIBDIR)/libpas.a does not exist" ; \
		exit 1 ; \
	fi
	@if [ ! -f $(LIBDIR)/libinsn.a ] ; then \
		echo "$(LIBDIR)/libinsn.a does not exist" ; \
		exit 1 ; \
	fi

$(BINDIR)/popt: check_libs $(POPTOBJS)
	$(CC) -o $@ $(LDFLAGS) $(POPTOBJS) -linsn -lpoff -lpas

popt: $(BINDIR)/popt

clean:
	$(RM) popt *.o core *~

# ----------------------------------------------------------------------
