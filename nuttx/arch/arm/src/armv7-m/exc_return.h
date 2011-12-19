/************************************************************************************
 * arch/arm/src/armv7-m/exc_return.h
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

#ifndef __ARCH_ARM_SRC_COMMON_CORTEXM_EXC_RETURN_H
#define __ARCH_ARM_SRC_COMMON_CORTEXM_EXC_RETURN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* The processor saves an EXC_RETURN value to the LR on exception entry. The
 * exception mechanism relies on this value to detect when the processor has
 * completed an exception handler. Bits[31:4] of an EXC_RETURN value are
 * 0xfffffffX. When the processor loads a value matching this pattern to the
 * PC it detects that the operation is a not a normal branch operation and,
 * instead, that the exception is complete. Therefore, it starts the exception
 * return sequence. Bits[3:0] of the EXC_RETURN value indicate the required
 * return stack and processor mode:
 */

/* EXC_RETURN_HANDLER: Return to handler mode. Exception return gets state from
 * the main stack. Execution uses MSP after return.
 */

#define EXC_RETURN_HANDLER   0xfffffff1

/* EXC_RETURN_PRIVTHR: Return to privileged thread mode. Exception return gets
 * state from the main stack. Execution uses MSP after return.
 */

#define EXC_RETURN_PRIVTHR   0xfffffff9

/* EXC_RETURN_UNPRIVTHR: Return to unprivileged thread mode. Exception return gets
 * state from the process stack. Execution uses PSP after return.
 */

#define EXC_RETURN_UNPRIVTHR 0xfffffffd

/* In the kernel build is not selected, then all threads run in privileged thread
 * mode.
 */

#ifdef CONFIG_NUTTX_KERNEL
#  define EXC_RETURN         0xfffffff9
#endif

/************************Th************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_SRC_COMMON_CORTEXM_EXC_RETURN_H */

