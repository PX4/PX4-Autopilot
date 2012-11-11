############################################################################
# configs/z16f2800100zcog/Makefile
#
#   Copyright (C) 2008, 2012 Gregory Nutt. All rights reserved.
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

ARCHSRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
WARCHSRCDIR	= ${shell cygpath -w $(ARCHSRCDIR)}
USRINCLUDES	= -usrinc:'.;$(WTOPDIR)\sched;$(WARCHSRCDIR);$(WARCHSRCDIR)\common'
INCLUDES	= $(ARCHSTDINCLUDES) $(USRINCLUDES)
CFLAGS		= $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(INCLUDES) $(ARCHDEFINES) $(EXTRADEFINES) $(EXTRADEFINES)

ASRCS		= 
AOBJS		= $(ASRCS:.S=$(OBJEXT))
CSRCS		= z16f_lowinit.c z16f_leds.c
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)


all: libboard$(LIBEXT)

$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
	@$(CPP) $(CPPFLAGS) $< -o $@.tmp
	@cat $@.tmp | sed -e "s/^#/;/g" > $@
	@rm $@.tmp

$(AOBJS): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, "$(OBJS)")

.depend: Makefile $(SRCS)
	@$(MKDEP) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f libboard$(LIBEXT) *~ .*.swp
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep
