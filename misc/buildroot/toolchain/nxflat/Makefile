############################################################################
# toolchain/nxflat/Makefile
#
#   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

CFLAGS		+= -Wall -I. -I$(BINUTILS_DIR1)/bfd -I$(BINUTILS_DIR)/include
LDFLAGS		+= -L$(BINUTILS_DIR1)/bfd -L$(BINUTILS_DIR1)/libiberty
LIBS		= -lbfd -liberty -lz -lc 

LDNXFLAT_OBJS	= ldnxflat.o
MKNXFLAT_OBJS	= mknxflat.o
READNXFLAT_OBJS	= readnxflat.o
OBJS		= $(LDNXFLAT_OBJS) $(MKNXFLAT_OBJS) $(READNXFLAT_OBJS)

BIN		= ldnxflat mknxflat readnxflat

GXX_VERSION	= ${shell $(ARCHCXX) -dumpversion | cut -d. -f1}

all:	$(BIN)

$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

arch:
	@ln -sf $(ARCH) arch

ldnxflat: arch $(LDNXFLAT_OBJS)
	$(CC) $(LDFLAGS) $(LDNXFLAT_OBJS) -o $@ $(LIBS)

mknxflat: arch $(MKNXFLAT_OBJS)
	$(CC) $(LDFLAGS) $(MKNXFLAT_OBJS) -o $@ $(LIBS)

arch/libarch.a:
	$(MAKE) -C arch CC="$(CC)"

readnxflat: arch $(READNXFLAT_OBJS) arch/libarch.a
	$(CC) $(LDFLAGS) -L arch -o $@ $(READNXFLAT_OBJS) $(LIBS) -larch

# Housekeeping

clean:
	-$(MAKE) -C arch clean
	rm -f *.o $(BIN) arch *.exe *~ .*.swp

