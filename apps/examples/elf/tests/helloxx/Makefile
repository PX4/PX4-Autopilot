############################################################################
# examples/elf/tests/helloxx/Makefile
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

BIN1 = hello++1
BIN2 = hello++2
ifeq ($(CONFIG_BINFMT_CONSTRUCTORS),y)
BIN3 = hello++3
endif
#BIN4 = hello++4
ALL_BIN = $(BIN1) $(BIN2) $(BIN3) $(BIN4)

SRCS1 = $(BIN1).c
OBJS1 = $(SRCS1:.c=.o)

SRCS2 = $(BIN2).c
OBJS2 = $(SRCS2:.c=.o)

ifeq ($(CONFIG_BINFMT_CONSTRUCTORS),y)
SRCS3 = $(BIN3).c
OBJS3 = $(SRCS3:.c=.o)
endif

#SRCS4 = $(BIN4).c
#OBJS4 = $(SRCS4:.c=.o)

SRCS = $(SRCS1) $(SRCS2) $(SRCS3) $(SRCS4)
OBJS = $(OBJS1) $(OBJS2) $(OBJS3) $(OBJS4)

LIBSTDC_STUBS_DIR	= $(TOPDIR)/libxx
LIBSTDC_STUBS_LIB	= $(LIBSTDC_STUBS_DIR)/liblibxx.a

all: $(BIN1) $(BIN2) $(BIN3) $(BIN4)

$(OBJS): %.o: %.cpp
	@echo "CC: $<"
	$(Q) $(CXX) -c $(CXXELFFLAGS) $< -o $@

# This contains libstdc++ stubs to that you can build C++ code
# without actually having libstdc++

$(LIBSTDC_STUBS_LIB):
	$(Q) $(MAKE) -C $(LIBSTDC_STUBS_DIR) TOPDIR=$(TOPDIR)

# BIN1 and BIN2 link just like C code because they contain no
# static constructors.  BIN1 is equivalent to a C hello world;
# BIN2 contains a class that implements hello world, but it is
# not statically initialized.

$(BIN1): $(OBJS1)
	@echo "LD: $<"
	$(Q) $(LD) $(LDELFFLAGS) -o $@ $^

$(BIN2): $(OBJS2)
	@echo "LD: $<"
	$(Q) $(LD) $(LDELFFLAGS) -o $@ $^

# BIN3 is equivalent to BIN2 except that is uses static initializers

ifeq ($(CONFIG_BINFMT_CONSTRUCTORS),y)
$(BIN3): $(OBJS3)
	@echo "LD: $<"
	$(Q) $(LD) $(LDELFFLAGS) -o $@ $^
endif

# BIN4 is similar to BIN3 except that it uses the streams code from libstdc++
#
# NOTE:  libstdc++ is not available for NuttX as of this writing
#
#$(BIN4): $(OBJS4)
#	@echo "LD: $<"
#	$(Q) $(LD) $(LDELFFLAGS) -o $@ $^

clean: 
	$(call DELFILE, $(BIN1))
	$(call DELFILE, $(BIN2))
	$(call DELFILE, $(BIN3))
	$(call DELFILE, $(BIN4))
	$(call CLEAN)

install: $(ALL_BIN)
	$(Q) mkdir -p $(ROMFS_DIR)
	$(Q) install $(BIN1) $(ROMFS_DIR)/$(BIN1)
	$(Q) install $(BIN2) $(ROMFS_DIR)/$(BIN2)
ifeq ($(CONFIG_BINFMT_CONSTRUCTORS),y)
	$(Q) install $(BIN3) $(ROMFS_DIR)/$(BIN3)
endif
#	$(Q) install $(BIN4) $(ROMFS_DIR)/$(BIN4)







