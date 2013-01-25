/*****************************************************************************
 * sched/group_signal.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <assert.h>
#include <signal.h>
#include <debug.h>

#include "sig_internal.h"

#if defined(HAVE_TASK_GROUP) && !defined(CONFIG_DISABLE_SIGNALS)

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: group_signal
 *
 * Description:
 *   Send a signal to every member of the group to which task belongs.
 *
 * Parameters:
 *   tcb - The tcb of one task in the task group that needs to be signalled.
 *
 * Return Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during task terminatino in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

int group_signal(FAR _TCB *tcb, FAR siginfo_t *info)
{
#ifdef HAVE_GROUP_MEMBERS
  FAR struct task_group_s *group;
  FAR _TCB *gtcb;
  int i;

  DEBUGASSERT(tcb && tcb->group && info);
  group = tcb->group;

  /* Make sure that pre-emption is disabled to that we signal all of teh
   * members of the group before any of them actually run.
   */

  sched_lock();

  /* Send the signal to each member of the group */

  for (i = 0; i < group->tg_nmembers; i++)
    {
      gtcb = sched_gettcb(group->tg_members[i]);
      DEBUGASSERT(gtcb);
      if (gtcb)
        {
          /* Use the sig_received interface so that it does not muck with
           * the siginfo_t.
           */

#ifdef CONFIG_DEBUG
          int ret = sig_received(gtcb, info);
          DEBUGASSERT(ret == 0);
#else
          (void)sig_received(gtcb, info);
#endif
        }
    }
  
  /* Re-enable pre-emption an return success */

  sched_unlock();
  return OK;
#else
  return sig_received(tcb, info);
#endif
}

#endif /* HAVE_TASK_GROUP && !CONFIG_DISABLE_SIGNALS */
