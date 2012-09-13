/************************************************************************************
 * arch/z80/src/z8/up_mem.h
 * arch/z80/src/chip/up_mem.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __Z8_UP_MEM_H
#define __Z8_UP_MEM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* For the ZiLOG ZDS-II toolchain(s), the heap will be set using linker-
 * defined values:
 *
 *   far_heapbot  : set to the offset to the first unused value in EDATA
 *   far_stacktop : set to the highest address in EDATA
 *
 * The top of the heap is then determined by the amount of stack setaside
 * in the NuttX configuration file
 */

#ifndef CONFIG_HEAP1_BASE
   extern far unsigned long far_heapbot;
#  define CONFIG_HEAP1_BASE ((uint16_t)&far_heapbot)
#endif

#ifndef CONFIG_HEAP1_END
   extern far unsigned long far_stacktop;
#  define CONFIG_HEAP1_END (((uint16_t)&far_stacktop) - CONFIG_IDLETHREAD_STACKSIZE + 1)
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* __Z8_UP_MEM_H */
