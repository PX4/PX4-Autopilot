/****************************************************************************
 * sched/sig_pending.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <signal.h>
#include <sched.h>

#include "os_internal.h"
#include "sig_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigpending
 *
 * Description:
 *   This function stores the returns the set of signals that are blocked
 *   for delivery and that are pending for the calling process in the space
 *   pointed to by set.
 * 
 * Parameters:
 *   set - The location to return the pending signal set.
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR)
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigpending(FAR sigset_t *set)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  int       ret  = ERROR;

  if (set)
    {
      *set = sig_pendingset(rtcb);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: sig_pendingset
 *
 * Description:
 *   Convert the list of pending signals into a signal set
 *
 ****************************************************************************/

sigset_t sig_pendingset(FAR _TCB *stcb)
{
  sigset_t        sigpendset;
  FAR sigpendq_t *sigpend;
  irqstate_t      saved_state;

  sigpendset = NULL_SIGNAL_SET;

  saved_state = irqsave();
  for (sigpend = (FAR sigpendq_t*)stcb->sigpendingq.head;
       (sigpend); sigpend = sigpend->flink)
    {
      sigaddset(&sigpendset, sigpend->info.si_signo);
    }

  irqrestore(saved_state);

  return sigpendset;
}
