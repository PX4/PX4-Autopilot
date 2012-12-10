/************************************************************************************
 * common/sdcc.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_Z180_SRC_COMMON_UP_MEM_H
#define __ARCH_Z180_SRC_COMMON_UP_MEM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Locate the IDLE thread stack at the end of RAM. */

#define CONFIG_STACK_END   CONFIG_DRAM_SIZE
#define CONFIG_STACK_BASE  (CONFIG_STACK_END - CONFIG_IDLETHREAD_STACKSIZE)

/* The heap then extends from the linker determined beginning of the heap (s__HEAP).
 * to the bottom of the IDLE thread stack.  NOTE:  The symbol s__HEAP is not
 * accessible from C because it does not begin with the _ character.  g_heapbase
 * is defined in z180_head.asm to provide that value to the C code.
 */

#define CONFIG_HEAP1_END   CONFIG_STACK_BASE
#define CONFIG_HEAP1_BASE  g_heapbase

/************************************************************************************
 * Public variables
 ************************************************************************************/

/* This is the bottom of the heap as provided by the linker symbol s__HEAP. NOTE:
 * The symbol s__HEAP is not accessible from C because it does not begin with the _
 * character.  g_heapbase is defined in z180_head.asm to provide that value to the C
 * code.
 */

extern const uint16_t g_heapbase;

#endif  /* __ARCH_Z180_SRC_COMMON_UP_MEM_H */
