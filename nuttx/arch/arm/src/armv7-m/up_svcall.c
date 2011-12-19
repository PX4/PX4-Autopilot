/****************************************************************************
 * arch/arm/src/armv7-m/up_svcall.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/sched.h>

#ifdef CONFIG_NUTTX_KERNEL
#  include <syscall.h>
#endif

#include "svcall.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#undef SYSCALL_INTERRUPTIBLE
#if defined(CONFIG_NUTTX_KERNEL)
#  if CONFIG_ARCH_INTERRUPTSTACK > 3
#    warning "CONFIG_ARCH_INTERRUPTSTACK and CONFIG_NUTTX_KERNEL are incompatible"
#    warning "options as currently implemented.  Interrupts will have to be disabled"
#    warning "during SYScall processing to avoid un-handled nested interrupts"
#  else
#    define SYSCALL_INTERRUPTIBLE 1
#  endif
#endif

/* Debug ********************************************************************/
/* Debug output from this file may interfere with context switching!  To get
 * debug output you must enabled the following in your NuttX configuration:
 *
 * CONFIG_DEBUG and CONFIG_DEBUG_SCHED
 *
 * And you must explicitly define DEBUG_SVCALL below:
 */

#undef DEBUG_SVCALL         /* Define to debug SVCall */
#ifdef DEBUG_SVCALL
# define svcdbg(format, arg...) slldbg(format, ##arg)
#else
# define svcdbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dispatch_syscall
 *
 * Description:
 *   Dispatch a system call to the appropriate handling logic.
 *
 ****************************************************************************/

#ifdef CONFIG_NUTTX_KERNEL
static inline void dispatch_syscall(uint32_t *regs)
{
  uint32_t  cmd  = regs[REG_R0];
  FAR _TCB *rtcb = sched_self();
  uintptr_t ret  = (uintptr_t)ERROR;

  /* Verify the the SYS call number is within range */

  if (cmd < SYS_maxsyscall)
    {
      /* Report error and return ERROR */

      slldbg("ERROR: Bad SYS call: %d\n", cmd);
    }
  else
    {
      /* The index into the syscall table is offset by the number of architecture-
       * specific reserved entries at the beginning of the SYS call number space.
       */

      int index = cmd - CONFIG_SYS_RESERVED;

      /* Enable interrupts while the SYSCALL executes */

#ifdef SYSCALL_INTERRUPTIBLE
      irqenable();
#endif

      /* Call the correct stub for each SYS call, based on the number of parameters */

      svcdbg("Calling stub%d at %p\n", index, g_stubloopkup[index].stub0);

      switch (g_stubnparms[index])
        {
        /* No parameters */

        case 0:
          ret = g_stublookup[index].stub0();
          break;

        /* Number of parameters: 1 */

        case 1:
          ret = g_stublookup[index].stub1(regs[REG_R1]);
          break;

        /* Number of parameters: 2 */

        case 2:
          ret = g_stublookup[index].stub2(regs[REG_R1], regs[REG_R2]);
          break;

         /* Number of parameters: 3 */

       case 3:
          ret = g_stublookup[index].stub3(regs[REG_R1], regs[REG_R2],
                                          regs[REG_R3]);
          break;

         /* Number of parameters: 4 */

       case 4:
          ret = g_stublookup[index].stub4(regs[REG_R1], regs[REG_R2],
                                          regs[REG_R3], regs[REG_R4]);
          break;

        /* Number of parameters: 5 */

        case 5:
          ret = g_stublookup[index].stub5(regs[REG_R1], regs[REG_R2],
                                          regs[REG_R3], regs[REG_R4],
                                          regs[REG_R5]);
          break;

        /* Number of parameters: 6 */

        case 6:
          ret = g_stublookup[index].stub6(regs[REG_R1], regs[REG_R2],
                                          regs[REG_R3], regs[REG_R4],
                                          regs[REG_R5], regs[REG_R6]);
          break;

        /* Unsupported number of paramters. Report error and return ERROR */

        default:
          slldbg("ERROR: Bad SYS call %d number parameters %d\n",
                 cmd, g_stubnparms[index]);
          break;
        }

#ifdef SYSCALL_INTERRUPTIBLE
      irqdisable();
#endif
    }

  /* Set up the return value.  First, check if a context switch occurred. 
   * In this case, regs will no longer be the same as current_regs.  In
   * the case of a context switch, we will have to save the return value
   * in the TCB where it can be returned later when the task is restarted.
   */

  if (regs != current_regs)
    {
      regs = rtcb->xcp.regs;
    }

  /* Then return the result in R0 */

  svcdbg("Return value regs: %p value: %d\n", regs, ret);
  regs[REG_R0] = (uint32_t)ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

int up_svcall(int irq, FAR void *context)
{
  uint32_t *regs = (uint32_t*)context;

  DEBUGASSERT(regs && regs == current_regs);

  /* The SVCall software interrupt is called with R0 = system call command
   * and R1..R7 =  variable number of arguments depending on the system call.
   */

  svcdbg("SVCALL Entry: regs: %p cmd: %d\n", regs, regs[REG_R0]);
  svcdbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
         regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
         regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
  svcdbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
         regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
  svcdbg("  PSR=%08x\n", regs[REG_XPSR]);

  /* Handle the SVCall according to the command in R0 */

  switch (regs[REG_R0])
    {
      /* R0=SYS_save_context:  This is a save context command:
       *
       *   int up_saveusercontext(uint32_t *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_save_context
       *   R1 = saveregs
       *
       * In this case, we simply need to copy the current regsters to the
       * save regiser space references in the saved R1 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0);
          memcpy((uint32_t*)regs[REG_R1], regs, XCPTCONTEXT_SIZE);
#ifdef CONFIG_ARCH_FPU
          up_savefpu((uint32_t*)regs[REG_R1]);
#endif
        }
        break;

      /* R0=SYS_restore_context: This a restore context command:
       *
       *   void up_fullcontextrestore(uint32_t *restoreregs) __attribute__ ((noreturn));
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_restore_context
       *   R1 = restoreregs
       *
       * In this case, we simply need to set current_regs to restore register
       * area referenced in the saved R1. context == current_regs is the normal
       * exception return.  By setting current_regs = context[R1], we force
       * the return to the saved context referenced in R1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0);
          current_regs = (uint32_t*)regs[REG_R1];
        }
        break;

      /* R0=SYS_switch_context: This a switch context command:
       *
       *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = 1
       *   R1 = saveregs
       *   R2 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of R1 and then set
       * current_regs to to the save register area referenced by the saved
       * contents of R2.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0 && regs[REG_R2] != 0);
          memcpy((uint32_t*)regs[REG_R1], regs, XCPTCONTEXT_SIZE);
          current_regs = (uint32_t*)regs[REG_R2];
        }
        break;

      /* This is not an architecture-specify system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
#ifdef CONFIG_NUTTX_KERNEL
        dispatch_syscall(regs);
#else
        slldbg("ERROR: Bad SYS call: %d\n", regs[REG_R0]);
#endif
        break;
    }

  /* Report what happened.  That might difficult in the case of a context switch */

  if (regs != current_regs)
    {
      svcdbg("SVCall Return: Context switch!\n");
      svcdbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             current_regs[REG_R0],  current_regs[REG_R1],  current_regs[REG_R2],  current_regs[REG_R3],
             current_regs[REG_R4],  current_regs[REG_R5],  current_regs[REG_R6],  current_regs[REG_R7]);
      svcdbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             current_regs[REG_R8],  current_regs[REG_R9],  current_regs[REG_R10], current_regs[REG_R11],
             current_regs[REG_R12], current_regs[REG_R13], current_regs[REG_R14], current_regs[REG_R15]);
      svcdbg("  PSR=%08x\n", current_regs[REG_XPSR]);
    }
  else
    {
      svcdbg("SVCall Return: %d\n", regs[REG_R0]);
    }

  return OK;
}
