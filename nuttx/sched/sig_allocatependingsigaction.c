/************************************************************************
 * sched/sig_allocatependingsigaction.c
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

#include <signal.h>
#include <assert.h>
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
 * Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: sig_allocatependingsigaction
 *
 * Description:
 *   Allocate a new element for the pending signal action queue
 *
 ************************************************************************/

FAR sigq_t *sig_allocatependingsigaction(void)
{
  FAR sigq_t    *sigq;
  irqstate_t saved_state;

  /* Check if we were called from an interrupt handler. */

  if (up_interrupt_context())
    {
      /* Try to get the pending signal action structure from the free list */

      sigq = (FAR sigq_t*)sq_remfirst(&g_sigpendingaction);

      /* If so, then try the special list of structures reserved for
       * interrupt handlers
       */

      if (!sigq)
        {
          sigq = (FAR sigq_t*)sq_remfirst(&g_sigpendingirqaction);
        }
    }

  /* If we were not called from an interrupt handler, then we are
   * free to allocate pending signal action structures if necessary. */

  else
    {
      /* Try to get the pending signal action structure from the free list */

      saved_state = irqsave();
      sigq = (FAR sigq_t*)sq_remfirst(&g_sigpendingaction);
      irqrestore(saved_state);

      /* Check if we got one. */

      if (!sigq)
        {
          /* No...Try the resource pool */

          if (!sigq)
            {
              sigq = (FAR sigq_t *)kmalloc((sizeof (sigq_t)));
            }

          /* Check if we got an allocated message */

          if (sigq)
            {
              sigq->type = SIG_ALLOC_DYN;
            }
        }
    }

  return sigq;
}

