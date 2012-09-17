/************************************************************************
 * sched/sig_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <queue.h>
#include <nuttx/kmalloc.h>

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

/* The g_sigfreeaction data structure is a list of available signal
 * action structures.
 */

sq_queue_t  g_sigfreeaction;

/* The g_sigpendingaction data structure is a list of available pending
 * signal action structures.
 */

sq_queue_t  g_sigpendingaction;

/* The g_sigpendingirqaction is a list of available pending signal actions
 * that are reserved for use by interrupt handlers.
 */

sq_queue_t  g_sigpendingirqaction;

/* The g_sigpendingsignal data structure is a list of available pending
 * signal structures.
 */

sq_queue_t  g_sigpendingsignal;

/* The g_sigpendingirqsignal data structure is a list of available
 * pending signal structures that are reserved for use by interrupt
 * handlers.
 */

sq_queue_t  g_sigpendingirqsignal;

/************************************************************************
 * Private Variables
 ************************************************************************/

/* g_sigactionalloc is a pointer to the start of the allocated blocks of
 * signal actions.
 */

static sigactq_t  *g_sigactionalloc;

/* g_sigpendingactionalloc is a pointer to the start of the allocated
 * blocks of pending signal actions.
 */

static sigq_t     *g_sigpendingactionalloc;

/* g_sigpendingirqactionalloc is a pointer to the start of the allocated
 * block of pending signal actions.
 */

static sigq_t     *g_sigpendingirqactionalloc;

/* g_sigpendingsignalalloc is a pointer to the start of the allocated
 * blocks of pending signals.
 */

static sigpendq_t *g_sigpendingsignalalloc;

/* g_sigpendingirqsignalalloc is a pointer to the start of the allocated
 * blocks of pending signals.
 */

static sigpendq_t *g_sigpendingirqsignalalloc;

/************************************************************************
 * Private Function Prototypes
 ************************************************************************/

static sigq_t     *sig_allocateblock(sq_queue_t *siglist, uint16_t nsigs,
                                     uint8_t sigtype);
static sigpendq_t *sig_allocatependingsignalblock(sq_queue_t *siglist,
                                                  uint16_t nsigs, uint8_t sigtype);

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Name: sig_allocateblock
 *
 * Description:
 *   Allocate a block of pending signal actions and place them
 *   on the free list.
 *
 ************************************************************************/

static sigq_t *sig_allocateblock(sq_queue_t *siglist, uint16_t nsigs,
                                 uint8_t sigtype)
{
  sigq_t *sigqalloc;
  sigq_t *sigq;
  int     i;

  /* Allocate a block of pending signal actions */

  sigqalloc = (sigq_t*)kmalloc((sizeof(sigq_t)) * nsigs);

  sigq = sigqalloc;
  for (i = 0; i < nsigs; i++)
    {
      sigq->type = sigtype;
      sq_addlast((FAR sq_entry_t*)sigq++, siglist);
    }

  return sigqalloc;
}

/************************************************************************
 * Name: sig_allocatependingsignalblock
 *
 * Description:
 *   Allocate a block of pending signal structures  and place them on
 *   the free list.
 *
 ************************************************************************/

static sigpendq_t *sig_allocatependingsignalblock(sq_queue_t *siglist,
                                                  uint16_t nsigs, uint8_t sigtype)
{
  sigpendq_t *sigpendalloc;
  sigpendq_t *sigpend;
  int i;

  /* Allocate a block of pending signal structures  */

  sigpendalloc =
     (sigpendq_t*)kmalloc((sizeof(sigpendq_t)) * nsigs);

  sigpend = sigpendalloc;
  for (i = 0; i < nsigs; i++)
    {
      sigpend->type = sigtype;
      sq_addlast((FAR sq_entry_t*)sigpend++, siglist);
    }

  return sigpendalloc;
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: sig_initialize
 *
 * Description:
 *   Perform one-time power-up initialization
 *
 ************************************************************************/

void sig_initialize(void)
{
  /* Initialize free lists */

  sq_init(&g_sigfreeaction);
  sq_init(&g_sigpendingaction);
  sq_init(&g_sigpendingirqaction);
  sq_init(&g_sigpendingsignal);
  sq_init(&g_sigpendingirqsignal);

  /* Add a block of signal structures to each list */

  g_sigpendingactionalloc =
     sig_allocateblock(&g_sigpendingaction,
                       NUM_PENDING_ACTIONS,
                       SIG_ALLOC_FIXED);

  g_sigpendingirqactionalloc =
     sig_allocateblock(&g_sigpendingirqaction,
                       NUM_PENDING_INT_ACTIONS,
                       SIG_ALLOC_IRQ);

  sig_allocateactionblock();

  g_sigpendingsignalalloc =
     sig_allocatependingsignalblock(&g_sigpendingsignal,
                             NUM_SIGNALS_PENDING,
                             SIG_ALLOC_FIXED);

  g_sigpendingirqsignalalloc =
     sig_allocatependingsignalblock(&g_sigpendingirqsignal,
                             NUM_INT_SIGNALS_PENDING,
                             SIG_ALLOC_IRQ);
}

/************************************************************************
 * Name: sig_allocateactionblock
 *
 * Description:
 *   Allocate a block of signal actions and place them
 *   on the free list.
 *
 ************************************************************************/

void sig_allocateactionblock(void)
{
  sigactq_t *sigact;
  int i;

  /* Allocate a block of signal actions */

  g_sigactionalloc =
     (sigactq_t*)kmalloc((sizeof(sigactq_t)) * NUM_SIGNAL_ACTIONS);

  sigact = g_sigactionalloc;
  for (i = 0; i < NUM_SIGNAL_ACTIONS; i++)
    {
      sq_addlast((FAR sq_entry_t*)sigact++, &g_sigfreeaction);
    }
}
