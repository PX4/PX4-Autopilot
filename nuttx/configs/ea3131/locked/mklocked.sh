#!/bin/bash
###########################################################################
# configs/ea3131/locked/mklocked.sh
#
#   Copyright (C) 2010-2012 Gregory Nutt. All rights reserved.
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

#set -x

############################################################################
# Arguments
############################################################################

USAGE="$0 <nuttx-dir>"

TOPDIR="$1"
CONFIG="$TOPDIR/.config"
if [ -z "$TOPDIR" ]; then
	echo "Missing Argument"
	echo $USAGE
	exit 1
fi
if [ ! -d "$TOPDIR" ]; then
	echo "NuttX directory does not exist: $TOPDIR"
	echo $USAGE
	exit 1
fi
if [ ! -f "$CONFIG" ]; then
	echo "Configuration file not found: $CONFIG"
	exit 1
fi

############################################################################
# Functions
############################################################################

function checkconfig () {
	CONFIGLINE=`cat "$CONFIG" | grep "$1="`
	if [ -z "$CONFIGLINE" ]; then
		echo "n"
	fi
	if [ "X${CONFIGLINE}" = "X${1}=y" ]; then
		echo "y"
	else
		echo "n"
	fi
}

function checkzero () {
	CONFIGLINE=`cat "$CONFIG" | grep "$1="`
	if [ -z "$CONFIGLINE" ]; then
		echo "y"
	fi
	if [ "X${CONFIGLINE}" = "X${1}=0" ]; then
		echo "y"
	else
		echo "n"
	fi
}

############################################################################
# Interrupt Handlers
############################################################################
#
# All interrupt handlers must be forced to lie in the locked .text region
#
# These are the vector entry points (only one is really needed since they
# are all in the same file). These should drag in all of the vector
# dispatching logic.
#

rm -f ld-locked.inc
echo "EXTERN(up_vectorswi)" >>ld-locked.inc
echo "EXTERN(up_vectordata)" >>ld-locked.inc
echo "EXTERN(up_vectorprefetch)" >>ld-locked.inc
echo "EXTERN(up_vectorundefinsn)" >>ld-locked.inc
echo "EXTERN(up_vectorfiq)" >>ld-locked.inc
echo "EXTERN(up_vectorirq)" >>ld-locked.inc
echo "EXTERN(up_vectoraddrexcptn)" >>ld-locked.inc

#
# These are the initialization entry points of all device drivers that
# handle interrupts.  We really want to include as little as possible --
# ideally just the interrupt handler itself, but that is not usually
# possible.
#
# Of course, this list must be extended as interrupt handlers are added.

echo "EXTERN(up_timerinit)" >>ld-locked.inc

answer=$(checkconfig CONFIG_LPC31_UART)
if [ $answer = y ]; then
	echo "EXTERN(up_earlyserialinit)" >>ld-locked.inc
fi

# up_i2cinitialize -- Not conditioned on anything

answer=$(checkconfig CONFIG_USBDEV)
if [ $answer = y ]; then
	echo "EXTERN(up_usbinitialize)" >>ld-locked.inc
fi

############################################################################
# Initialization logic
############################################################################
# All initialization logic must be in memory because it must execute before
# the page fill worker thread is started.  Ideally this would be in some
# region that is mapped initially, but then unmapped after initialization
# is complete -- effectively freeing the memory used for the 1-time
# initialization code.  That optimization has not yet been made and, as
# consequence, the 1-time initialization code takes up precious memory
# in the locked memory region.
#
# up_boot is a low-level initialization function called by __start:

echo "EXTERN(up_boot)" >>ld-locked.inc

# All of the initialization functions that are called by os_start up to
# the point where the page fill worker thread is started must also be
# included in the locked text section (at least for now)

answer=$(checkzero CONFIG_TASK_NAME_SIZE)
if [ $answer = n ]; then
	echo "EXTERN(up_boot)" >>ld-locked.inc
fi

echo "EXTERN(dq_addfirst)" >>ld-locked.inc
echo "EXTERN(up_initial_state)" >>ld-locked.inc

answer=$(checkconfig CONFIG_HEAP_BASE)
if [ $answer = n ]; then
	echo "EXTERN(up_allocate_heap)" >>ld-locked.inc
fi

echo "EXTERN(mm_initialize)" >>ld-locked.inc
echo "EXTERN(irq_initialize)" >>ld-locked.inc
echo "EXTERN(wd_initialize)" >>ld-locked.inc

answer=$(checkconfig CONFIG_DISABLE_CLOCK)
if [ $answer = n ]; then
	echo "EXTERN(clock_initialize)" >>ld-locked.inc
fi

answer=$(checkconfig CONFIG_DISABLE_POSIX_TIMERS)
if [ $answer = n ]; then
	echo "EXTERN(timer_initialize)" >>ld-locked.inc
fi

answer=$(checkconfig CONFIG_DISABLE_SIGNALS)
if [ $answer = n ]; then
	echo "EXTERN(sig_initialize)" >>ld-locked.inc
fi

echo "EXTERN(sem_initialize)" >>ld-locked.inc

answer=$(checkconfig CONFIG_DISABLE_MQUEUE)
if [ $answer = n ]; then
	echo "EXTERN(mq_initialize)" >>ld-locked.inc
fi

answer=$(checkconfig CONFIG_DISABLE_PTHREAD)
if [ $answer = n ]; then
	echo "EXTERN(pthread_initialize)" >>ld-locked.inc
fi

answer=$(checkzero CONFIG_NFILE_DESCRIPTORS)
if [ $answer = n ]; then
	echo "EXTERN(fs_initialize)" >>ld-locked.inc
fi

answer=$(checkconfig CONFIG_NET)
if [ $answer = y ]; then
	echo "EXTERN(net_initialize)" >>ld-locked.inc
fi

echo "EXTERN(up_initialize)" >>ld-locked.inc
echo "EXTERN(lib_initialize)" >>ld-locked.inc
echo "EXTERN(sched_setupidlefiles)" >>ld-locked.inc
echo "EXTERN(task_create)" >>ld-locked.inc

############################################################################
# Idle Loop
############################################################################
#
# The IDLE loop must be forced to lie in the locked .text region.

echo "EXTERN(os_start)" >>ld-locked.inc
echo "EXTERN(up_idle)" >>ld-locked.inc

############################################################################
# PG Fill Worker Thread
############################################################################
#
# All of the page fill worker thread must be in the locked .text region.

echo "EXTERN(pg_worker)" >>ld-locked.inc
