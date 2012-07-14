/************************************************************************
 * sched/sig_releasependingsignal.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <wdog.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "sig_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: sig_releasependingsignal
 *
 * Description:
 *   Deallocate a pending signal list entry
 *
 ************************************************************************/

void sig_releasependingsignal(FAR sigpendq_t *sigpend)
{
  irqstate_t saved_state;

  /* If this is a generally available pre-allocated structyre,
   * then just put it back in the free list.
   */

  if (sigpend->type == SIG_ALLOC_FIXED)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      saved_state = irqsave();
      sq_addlast((FAR sq_entry_t*)sigpend, &g_sigpendingsignal);
      irqrestore(saved_state);
    }

  /* If this is a message pre-allocated for interrupts,
   * then put it back in the correct free list.
   */

  else if (sigpend->type == SIG_ALLOC_IRQ)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      saved_state = irqsave();
      sq_addlast((FAR sq_entry_t*)sigpend, &g_sigpendingirqsignal);
      irqrestore(saved_state);
    }

  /* Otherwise, deallocate it.  Note:  interrupt handlers
   * will never deallocate signals because they will not
   * receive them.
   */

  else if (sigpend->type == SIG_ALLOC_DYN)
    {
      sched_free(sigpend);
    }
}
