############################################################################
#
#   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
# 3. Neither the name PX4 nor the names of its contributors may be
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
# ARM CMSIS DSP library
#

#
# Find sources
#
DSPLIB_SRCDIR		:= $(dir $(lastword $(MAKEFILE_LIST)))
CSRCS			:= $(wildcard $(DSPLIB_SRCDIR)/DSP_Lib/Source/*/*.c)

INCLUDES		+= $(DSPLIB_SRCDIR)/Include \
			   $(DSPLIB_SRCDIR)/Device/ARM/ARMCM4/Include \
			   $(DSPLIB_SRCDIR)/Device/ARM/ARMCM3/Include

# Suppress some warnings that ARM should fix, but haven't.
EXTRADEFINES		+= -Wno-unsuffixed-float-constants \
			   -Wno-sign-compare \
			   -Wno-shadow \
			   -Wno-float-equal \
			   -Wno-unused-variable

#
# Override the default visibility for symbols, since the CMSIS DSPLib doesn't
# have anything we can use to mark exported symbols.
#
DEFAULT_VISIBILITY	 = YES

include $(APPDIR)/mk/app.mk
