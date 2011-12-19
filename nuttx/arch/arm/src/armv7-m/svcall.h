/************************************************************************************
 * arch/arm/src/armv7-m/svcall.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_COMMON_CORTEXM_SVCALL_H
#define __ARCH_ARM_SRC_COMMON_CORTEXM_SVCALL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NUTTX_KERNEL
#  include <syscall.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* This logic uses three system calls {0,1,2} for context switching.  The first three
 * syscall values must be reserved.
 */

#ifdef CONFIG_NUTTX_KERNEL
#  ifndef CONFIG_SYS_RESERVED
#    error "CONFIG_SYS_RESERVED must be defined to the value 3"
#  elif CONFIG_SYS_RESERVED != 3
#    error "CONFIG_SYS_RESERVED must have the value 3"
#  endif
#endif

/* Cortex M3 system calls ***********************************************************/

/* SYS call 0:
 *
 * int up_saveusercontext(uint32_t *saveregs);
 */

#define SYS_save_context    (0)

/* SYS call 1:
 *
 * void up_fullcontextrestore(uint32_t *restoreregs) __attribute__ ((noreturn));
 */

#define SYS_restore_context (1)

/* SYS call 2:
 *
 * void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
 */

#define SYS_switch_context  (2)

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_SRC_COMMON_CORTEXM_SVCALL_H */

