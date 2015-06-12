############################################################################
#
#   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
# NuttX / uORB adapter library
#

SRCDIR=$(dir $(MODULE_MK))

SRCS		 = 	\
			px4_qurt_impl.cpp \
			px4_qurt_tasks.cpp  \
			hrt_thread.c \
                        hrt_queue.c \
                        hrt_work_cancel.c \
			work_thread.c \
			work_queue.c \
			work_lock.c \
			work_cancel.c \
			lib_crc32.c \
			drv_hrt.c \
			queue.c \
			dq_addlast.c \
			dq_remfirst.c \
			sq_addlast.c \
			sq_remfirst.c \
			sq_addafter.c \
			dq_rem.c \
			main.cpp \
                        qurt_stubs.c
ifeq ($(CONFIG),qurt_hello)
SRCS +=			commands_hello.c
endif
ifeq ($(CONFIG),qurt_default)
SRCS +=			commands_default.c
endif
ifeq ($(CONFIG),qurt_muorb_test)
SRCS +=			commands_muorb_test.c
endif

MAXOPTIMIZATION	 = -Os
