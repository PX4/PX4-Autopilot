/****************************************************************************
 * arch/mips/src/mips32/up_swint0.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <syscall.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/sched.h>

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
 * And you must explicitly define DEBUG_SWINT0 below:
 */

#undef DEBUG_SWINT0         /* Define to debug SWInt */
#ifdef DEBUG_SWINT0
# define swidbg(format, arg...) slldbg(format, ##arg)
#else
# define swidbg(x...)
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
 * Name: up_registerdump
 ****************************************************************************/

#ifdef DEBUG_SWINT0
static void up_registerdump(uint32_t *regs)
{
  swidbg("MFLO:%08x MFHI:%08x EPC:%08x STATUS:%08x\n",
         regs[REG_MFLO], regs[REG_MFHI], regs[REG_EPC], regs[REG_STATUS]);
  swidbg("AT:%08x V0:%08x V1:%08x A0:%08x A1:%08x A2:%08x A3:%08x\n",
         regs[REG_AT], regs[REG_V0], regs[REG_V1], regs[REG_A0],
         regs[REG_A1], regs[REG_A2], regs[REG_A3]);
  swidbg("T0:%08x T1:%08x T2:%08x T3:%08x T4:%08x T5:%08x T6:%08x T7:%08x\n",
         regs[REG_T0], regs[REG_T1], regs[REG_T2], regs[REG_T3],
         regs[REG_T4], regs[REG_T5], regs[REG_T6], regs[REG_T7]);
  swidbg("S0:%08x S1:%08x S2:%08x S3:%08x S4:%08x S5:%08x S6:%08x S7:%08x\n",
         regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3],
         regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
#ifdef MIPS32_SAVE_GP
  swidbg("T8:%08x T9:%08x GP:%08x SP:%08x FP:%08x RA:%08x\n",
         regs[REG_T8], regs[REG_T9], regs[REG_GP], regs[REG_SP],
         regs[REG_FP], regs[REG_RA]);
#else
  swidbg("T8:%08x T9:%08x SP:%08x FP:%08x RA:%08x\n",
         regs[REG_T8], regs[REG_T9], regs[REG_SP], regs[REG_FP],
         regs[REG_RA]);
#endif
}
#else
#  define up_registerdump(regs)
#endif

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
  uint32_t  cmd  = regs[REG_A0];
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
      /* The index into the syscall table is offset by the number of
       * architecture-specific reserved entries at the beginning of the
       * SYS call number space.
       */

      int index = cmd - CONFIG_SYS_RESERVED;

      /* Enable interrupts while the SYSCALL executes */

#ifdef SYSCALL_INTERRUPTIBLE
      irqenable();
#endif

      /* Call the correct stub for each SYS call, based on the number of
       * parameters:  $5=parm1, $6=parm2, $7=parm3, $8=parm4, $9=parm5, and
       * $10=parm6.
       */

      swidbg("Calling stub%d at %p\n", index, g_stubloopkup[index].stub0);

      switch (g_stubnparms[index])
        {
        /* No parameters */

        case 0:
          ret = g_stublookup[index].stub0();
          break;

        /* Number of parameters: 1 */

        case 1:
          ret = g_stublookup[index].stub1(regs[REG_A1]);
          break;

        /* Number of parameters: 2 */

        case 2:
          ret = g_stublookup[index].stub2(regs[REG_A1], regs[REG_A2]);
          break;

         /* Number of parameters: 3 */

       case 3:
          ret = g_stublookup[index].stub3(regs[REG_A1], regs[REG_A2],
                                          regs[REG_A3]);
          break;

         /* Number of parameters: 4 */

       case 4:
          ret = g_stublookup[index].stub4(regs[REG_A1], regs[REG_A2],
                                          regs[REG_A3], regs[REG_T0]);
          break;

        /* Number of parameters: 5 */

        case 5:
          ret = g_stublookup[index].stub5(regs[REG_A1], regs[REG_A2],
                                          regs[REG_A3], regs[REG_T0],
                                          regs[REG_T1]);
          break;

        /* Number of parameters: 6 */

        case 6:
          ret = g_stublookup[index].stub6(regs[REG_A1], regs[REG_A2],
                                          regs[REG_A3], regs[REG_T0],
                                          regs[REG_T1], regs[REG_T2]);
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

  /* Set up the return vaue.  First, check if a context switch occurred. 
   * In this case, regs will no longer be the same as current_regs.  In
   * the case of a context switch, we will have to save the return value
   * in the TCB where it can be returned later when the task is restarted.
   */

  if (regs != current_regs)
    {
      regs = rtcb->xcp.regs;
    }

  /* Then return the result in v0 */

  swidbg("Return value regs: %p value: %d\n", regs, ret);
  regs[REG_v0] = (uint32_t)ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_swint0
 *
 * Description:
 *   This is software interrupt 0 exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int up_swint0(int irq, FAR void *context)
{
  uint32_t *regs = (uint32_t*)context;

  DEBUGASSERT(regs && regs == current_regs);

  /* Software interrupt 0 is invoked with REG_A0 (REG_R4) = system call
   * command and REG_A1-3 and REG_T0-2 (REG_R5-10) = variable number of
   * arguments depending on the system call.
   */

#ifdef DEBUG_SWINT0
  swidbg("Entry: regs: %p cmd: %d\n", regs, regs[REG_R4]);
  up_registerdump(regs);
#endif

  /* Handle the SWInt according to the command in $4 */

  switch (regs[REG_R4])
    {
      /* R4=SYS_restore_context: This a restore context command:
       *
       *   void up_fullcontextrestore(uint32_t *restoreregs) __attribute__ ((noreturn));
       *
       * At this point, the following values are saved in context:
       *
       *   R4 = SYS_restore_context
       *   R5 = restoreregs
       *
       * In this case, we simply need to set current_regs to restore register
       * area referenced in the saved R1. context == current_regs is the normal
       * exception return.  By setting current_regs = context[R1], we force
       * the return to the saved context referenced in R1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0);
          current_regs = (uint32_t*)regs[REG_A1];
        }
        break;

      /* R4=SYS_switch_context: This a switch context command:
       *
       *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R4 = SYS_switch_context
       *   R5 = saveregs
       *   R6 = restoreregs
       *
       * In this case, we save the context registers to the save register
       * area reference by the saved contents of R5 and then set
       * current_regs to to the save register area referenced by the saved
       * contents of R6.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0 && regs[REG_A2] != 0);
          up_copystate((uint32_t*)regs[REG_A1], regs);
          current_regs = (uint32_t*)regs[REG_A2];
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
        slldbg("ERROR: Bad SYS call: %d\n", regs[REG_A0]);
#endif
        break;
    }

  /* Report what happened.  That might difficult in the case of a context switch */

#ifdef DEBUG_SWINT0
  if (regs != current_regs)
    {
      swidbg("SWInt Return: Context switch!\n");
      up_registerdump(current_regs);
    }
  else
    {
      swidbg("SWInt Return: %d\n", regs[REG_V0]);
    }
#endif

  /* Clear the pending software interrupt 0 */
 
  up_clrpend_irq(PIC32MX_IRQSRC_CS0);
  
  return OK;
}
