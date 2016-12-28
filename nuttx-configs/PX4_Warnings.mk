############################################################################
# nuttx-configs/PX4_Warnings.mk
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

PX4_ARCHWARNINGS = -Wall \
                   -Wextra \
                   -Werror \
                   -Wdouble-promotion \
                   -Wframe-larger-than=1024 \
                   -Wlogical-op \
                   -Wpacked \
                   -Wpointer-arith \
                   -Wshadow \
                   -Wno-sign-compare \
                   -Wno-unused-parameter \
                   -Wno-nonnull-compare \
                   -Wno-misleading-indentation

#   -Wcast-qual  - generates spurious noreturn attribute warnings, try again later
#   -Wconversion - would be nice, but too many "risky-but-safe" conversions in the code
#   -Wcast-align - would help catch bad casts in some cases, but generates too many false positives

PX4_ARCHCWARNINGS = $(ARCHWARNINGS) \
                   -Wbad-function-cast \
                   -Wmissing-parameter-type \
                   -Wnested-externs \
                   -Wstrict-prototypes \
                   -Wno-bad-function-cast \
                   -Wno-cpp \
                   -Wno-implicit-function-declaration \
                   -Wno-maybe-uninitialized \
                   -Wno-missing-field-initializers \
                   -Wno-nested-externs \
                   -Wno-old-style-declaration \
                   -Wno-pointer-sign \
                   -Wno-type-limits \
                   -Wno-unused-but-set-variable \
                   -Wno-unused-function \
                   -Wno-unused-label \
                   -Wno-unused-variable

PX4_ARCHWARNINGSXX = $(ARCHWARNINGS) \
                   -Wno-cpp \
                   -Wno-psabi
