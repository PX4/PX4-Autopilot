/****************************************************************************
 * arch/mips/src/mips32/up_vfork.c
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
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_vfork.h"
#include "os_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STACK_ALIGNMENT
#  define CONFIG_STACK_ALIGNMENT 4
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

pid_t up_vfork(const struct vfork_s *context)
{
  _TCB *parent = (FAR _TCB *)g_readytorun.head;
  _TCB *child;
  size_t stacksize;
  uint32_t newsp;
#if CONFIG_MIPS32_FRAMEPOINTER
  uint32_t newfp;
#endif
  uint32_t stackutil;
  int ret;

  svdbg("s0:%08x s1:%08x s2:%08x s3:%08x s4:%08x\n",
        context->s0, context->s1, context->s2, context->s3, context->s4);
#if CONFIG_MIPS32_FRAMEPOINTER
  svdbg("s5:%08x s6:%08x s7:%08x\n",
        context->s5, context->s6, context->s7);
#ifdef MIPS32_SAVE_GP
  svdbg("fp:%08x sp:%08x ra:%08x gp:%08x\n",
        context->fp, context->sp, context->ra, context->gp);
#else
  svdbg("fp:%08x sp:%08x ra:%08x\n",
        context->fp context->sp, context->ra);
#endif
#else
  svdbg("s5:%08x s6:%08x s7:%08x s8:%08x\n",
        context->s5, context->s6, context->s7, context->s8);
#ifdef MIPS32_SAVE_GP
  svdbg("sp:%08x ra:%08x gp:%08x\n",
        context->sp, context->ra, context->gp);
#else
  svdbg("sp:%08x ra:%08x\n",
        context->sp, context->ra);
#endif
#endif

  /* Allocate and initialize a TCB for the child task. */

  child = task_vforksetup((start_t)context->ra);
  if (!child)
    {
      sdbg("task_vforksetup failed\n");
      return (pid_t)ERROR;
    }

  svdbg("Parent=%p Child=%p\n", parent, child);

  /* Get the size of the parent task's stack.  Due to alignment operations,
   * the adjusted stack size may be smaller than the stack size originally
   * requrested.
   */

  stacksize = parent->adj_stack_size + CONFIG_STACK_ALIGNMENT - 1;

  /* Allocate the stack for the TCB */

  ret = up_create_stack(child, stacksize);
  if (ret != OK)
    {
      sdbg("up_create_stack failed: %d\n", ret);
      task_vforkabort(child, -ret);
      return (pid_t)ERROR;
    }

  /* How much of the parent's stack was utilized?  The MIPS uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  DEBUGASSERT((uint32_t)parent->adj_stack_ptr > context->sp);
  stackutil = (uint32_t)parent->adj_stack_ptr - context->sp;

  svdbg("stacksize:%d stackutil:%d\n", stacksize, stackutil); 

  /* Make some feeble effort to perserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of vfor() usage, even this feeble
   * effort is overkill.
   */

  newsp = (uint32_t)child->adj_stack_ptr - stackutil;
  memcpy((void *)newsp, (const void *)context->sp, stackutil);

  /* Was there a frame pointer in place before? */

#if CONFIG_MIPS32_FRAMEPOINTER
  if (context->fp <= (uint32_t)parent->adj_stack_ptr &&
      context->fp >= (uint32_t)parent->adj_stack_ptr - stacksize)
    {
      uint32_t frameutil = (uint32_t)parent->adj_stack_ptr - context->fp;
      newfp = (uint32_t)child->adj_stack_ptr - frameutil;
    }
  else
    {
      newfp = context->fp;
    }

  svdbg("Old stack base:%08x SP:%08x FP:%08x\n",
        parent->adj_stack_ptr, context->sp, context->fp);
  svdbg("New stack base:%08x SP:%08x FP:%08x\n",
        child->adj_stack_ptr, newsp, newfp);
#else
  svdbg("Old stack base:%08x SP:%08x\n",
        parent->adj_stack_ptr, context->sp);
  svdbg("New stack base:%08x SP:%08x\n",
        child->adj_stack_ptr, newsp);
#endif

 /* Update the stack pointer, frame pointer, global pointer and saved
  * registers.  When the child TCB was initialized, all of the values
  * were set to zero. up_initial_state() altered a few values, but the
  * return value in v0 should be cleared to zero, providing the
  * indication to the newly started child thread.
  */

  child->xcp.regs[REG_S0]  = context->s0;  /* Saved register s0 */
  child->xcp.regs[REG_S1]  = context->s1;  /* Saved register s1 */
  child->xcp.regs[REG_S2]  = context->s2;  /* Saved register s2 */
  child->xcp.regs[REG_S3]  = context->s3;  /* Volatile register s3 */
  child->xcp.regs[REG_S4]  = context->s4;  /* Volatile register s4 */
  child->xcp.regs[REG_S5]  = context->s5;  /* Volatile register s5 */
  child->xcp.regs[REG_S6]  = context->s6;  /* Volatile register s6 */
  child->xcp.regs[REG_S7]  = context->s7;  /* Volatile register s7 */
#if CONFIG_MIPS32_FRAMEPOINTER
  child->xcp.regs[REG_FP]  = newfp;        /* Frame pointer */
#else
  child->xcp.regs[REG_S8]  = context->s8;  /* Volatile register s8 */
#endif
  child->xcp.regs[REG_SP]  = newsp;        /* Stack pointer */
#if MIPS32_SAVE_GP
  child->xcp.regs[REG_GP]  = newsp;        /* Global pointer */
#endif

  /* And, finally, start the child task.  On a failure, task_vforkstart()
   * will discard the TCB by calling task_vforkabort().
   */

  return task_vforkstart(child);
}
