############################################################################
# apps/netutils/cgi-src/Makefile
#
#   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

ifeq ($(WINTOOL),y)
INCDIROPT	= -w
endif

CFLAGS		+= ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(APPDIR)/netutils/thttpd" "$(APPDIR)/netutils/thttpd/cgi-src"}
CGIBINDIR	= $(APPDIR)/netutils/thttpd/cgi-bin
CLEANFILES	= *.o redirect ssi phf

BIN1		= phf
BIN2		= redirect
BIN3		= ssi
BIN			= $(BIN1) $(BIN2) $(BIN3)

R1SRCS1		= $(BIN1).c
R1OBJS1		= $(R1SRCS1:.c=.o)
R2SRC1		= $(BIN1)-thunk.S
R2OBJ1		= $(R2SRC1:.S=.o)

R1SRCS2		= $(BIN2).c
R1OBJS2		= $(R1SRCS2:.c=.o)
R2SRC2		= $(BIN2)-thunk.S
R2OBJ2		= $(R2SRC2:.S=.o)

R1SRCS3		= $(BIN3).c
R1OBJS3		= $(R1SRCS3:.c=.o)
R2SRC3		= $(BIN3)-thunk.S
R2OBJ3		= $(R2SRC3:.S=.o)

DERIVED		= $(R2SRC1) $(R2SRC2) $(R2SRC3)

R1COBJS		= $(R1OBJS1) $(R1OBJS2) $(R1OBJS3)
R2AOBJS		= $(R2OBJ1) $(R2OBJ2) $(R2OBJ3)

all: $(BIN1) $(BIN2) $(BIN3)

$(R1COBJS): %.o: %.c
	@echo "CC: $<"
	$(CC) -c $(CPICFLAGS) $< -o $@

$(R2AOBJS): %.o: %.S
	@echo "AS: $<"
	@$(CC) -c $(CPICFLAGS) $< -o $@

# BIN1

$(BIN1).r1: $(R1OBJS1)
	@echo "LD: $<"
	@$(LD) $(NXFLATLDFLAGS1) -o $@ $^

$(R2SRC1): $(BIN1).r1
	@echo "MK: $<"
	@$(MKNXFLAT) -o $@ $^

$(BIN1).r2: $(R2OBJ1)
	@echo "LD: $<"
	@$(LD) $(NXFLATLDFLAGS2) -o $@ $(R1OBJS1) $(R2OBJ1)

$(BIN1): $(BIN1).r2
	@echo "LD: $<"
	@$(LDNXFLAT) $(LDNXFLATFLAGS) -o $@ $^

#BIN2

$(BIN2).r1: $(R1OBJS2)
	@echo "LD: $<"
	@$(LD) $(NXFLATLDFLAGS1) -o $@ $^

$(R2SRC2): $(BIN2).r1
	@echo "MK: $<"
	@$(MKNXFLAT) -o $@ $^

$(BIN2).r2: $(R2OBJ2)
	@echo "LD: $<"
	@$(LD) $(NXFLATLDFLAGS2) -o $@ $(R1OBJS2) $(R2OBJ2)

$(BIN2): $(BIN2).r2
	@echo "LD: $<"
	@$(LDNXFLAT) $(LDNXFLATFLAGS) -o $@ $^
# BIN3

$(BIN3).r1: $(R1OBJS3)
	@echo "LD: $<"
	@$(LD) $(NXFLATLDFLAGS1) -o $@ $^

$(R2SRC3): $(BIN3).r1
	@echo "MK: $<"
	@$(MKNXFLAT) -o $@ $^

$(BIN3).r2: $(R2OBJ3)
	@echo "LD: $<"
	@$(LD) $(NXFLATLDFLAGS2) -o $@ $(R1OBJS3) $(R2OBJ3)

$(BIN3): $(BIN3).r2
	@echo "LD: $<"
	@$(LDNXFLAT) $(LDNXFLATFLAGS) -o $@ $^

clean:
	@rm -f $(BIN1) $(BIN2) $(BIN3) *.r1 *.r2 *-thunk.S *~ .*.swp
	$(call CLEAN)

distclean: clean

