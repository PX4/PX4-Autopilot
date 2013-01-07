/****************************************************************************
 * arch/arm/src/common/up_vfork.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_vfork.h"
#include "os_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARM requires at least a 4-byte stack alignment.  For use with EABI and
 * floating point, the stack must be aligned to 8-byte addresses.
 */

#ifndef CONFIG_STACK_ALIGNMENT

/* The symbol  __ARM_EABI__ is defined by GCC if EABI is being used.  If you
 * are not using GCC, make sure that CONFIG_STACK_ALIGNMENT is set correctly!
 */

#  ifdef __ARM_EABI__
#    define CONFIG_STACK_ALIGNMENT 8
#  else
#    define CONFIG_STACK_ALIGNMENT 4
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions. 
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork().  vfork() collects context information and
 *      transfers control up up_vfork().
 *   2) up_vfork()and calls task_vforksetup().
 *   3) task_vforksetup() allocates and configures the child task's TCB.  This
 *      consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the intput parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Allocate and initialize the stack
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls task_vforkstart()
 *   6) task_vforkstart() then executes the child thread.
 *
 * task_vforkabort() may be called if an error occurs between steps 3 and 6.
 *
 * Input Paremeters:
 *   context - Caller context information saved by vfork()
 *
 * Return:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error. 
 *
 ****************************************************************************/

pid_t up_vfork(struct vfork_s *context)
{
  _TCB *parent = (FAR _TCB *)g_readytorun.head;
  _TCB *child;
  size_t stacksize;
  uint32_t newsp;
  uint32_t stackutil;
  int ret;

  /* Allocate and initialize a TCB for the child task. */

  child = task_vforksetup((start_t)context->lr);
  if (!child)
    {
      return (pid_t)ERROR;
    }

  /* Get the size of the parent task's stack.  Due to alignment operations,
   * the adjusted stack size may be smaller than the stack size originally
   * requrested.
   */

  stacksize = parent->adj_stack_size + CONFIG_STACK_ALIGNMENT - 1;

  /* Allocate the stack for the TCB */

  ret = up_create_stack(child, stacksize);
  if (ret != OK)
    {
      task_vforkabort(child, -ret);
      return (pid_t)ERROR;
    }

  /* How much of the parent's stack was utilized? */

  DEBUGASSERT(parent->adj_stack_ptr > context->sp);
  stackutil = (uint32_t)parent->adj_stack_ptr - context->sp;

  /* Make some feeble effort to perserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointer and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of vfor() usage, even this feeble
   * effort is overkill.
   */

  newsp = (uint32_t)child->adj_stack_ptr - stackutil;
  memcpy((void *)newsp, (const void *)context->sp, stackutil);
  
 /* Update the stack pointer, frame pointer, and voltile registers.  When
  * the child TCB was initialized, all of the values were set to zero.
  * up_initial_state() altered a few values, but the return value in R0
  * should be cleared to zero, providing the indication to the newly started
  * child thread.
  */

  child->xcp.regs[REG_R4]  = context->r4;  /* Volatile register r4 */
  child->xcp.regs[REG_R5]  = context->r5;  /* Volatile register r5 */
  child->xcp.regs[REG_R6]  = context->r6;  /* Volatile register r6 */
  child->xcp.regs[REG_R7]  = context->r7;  /* Volatile register r7 */
  child->xcp.regs[REG_R8]  = context->r8;  /* Volatile register r8 */
  child->xcp.regs[REG_R9]  = context->r9;  /* Volatile register r9 */
  child->xcp.regs[REG_R10] = context->r10; /* Volatile register r10 */
  child->xcp.regs[REG_FP]  = context->fp;  /* Frame pointer */
  child->xcp.regs[REG_SP]  = context->sp;  /* Stack pointer */

  /* And, finally, start the child task.  On a failure, task_vforkstart()
   * will discard the TCB by calling task_vforkabort().
   */

  return task_vforkstart(child);
}
