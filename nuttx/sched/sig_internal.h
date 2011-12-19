/****************************************************************************
 * sched/sig_internal.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#ifndef __SIG_INTERNAL_H
#define __SIG_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdint.h>
#include <queue.h>
#include <sched.h>

#include <nuttx/kmalloc.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The following definition determines the number of signal structures to
 * allocate in a block
 */

#define NUM_SIGNAL_ACTIONS      16
#define NUM_PENDING_ACTIONS     16
#define NUM_PENDING_INT_ACTIONS  8
#define NUM_SIGNALS_PENDING     16
#define NUM_INT_SIGNALS_PENDING  8

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

enum sigalloc_e
{
  SIG_ALLOC_FIXED = 0,  /* pre-allocated; never freed */
  SIG_ALLOC_DYN,        /* dynamically allocated; free when unused */
  SIG_ALLOC_IRQ         /* Preallocated, reserved for interrupt handling */
};
typedef enum sigalloc_e sigalloc_t;

/* The following defines the sigaction queue entry */

struct sigactq
{
  FAR struct sigactq *flink;     /* Forward link */
  struct sigaction act;          /* Sigaction data */
  uint8_t   signo;               /* Signal associated with action */
};
typedef struct sigactq  sigactq_t;

/* The following defines the queue structure within each TCB
 * to hold pending signals received by the task.  These are signals that
 * cannot be processed because:  (1) the task is not waiting for them, or
 * (2) the task has no action associated with the signal.
 */

struct sigpendq
{
  FAR struct sigpendq *flink;    /* Forward link */
  siginfo_t info;                /* Signal information */
  uint8_t   type;                /* (Used to manage allocations) */
};
typedef struct sigpendq sigpendq_t;

/* The following defines the queue structure within each TCB
 * to hold queued signal actions that need action by the task
 */

struct sigq_s
{
  FAR struct sigq_s *flink;      /* Forward link */
  union
  {
    void (*sighandler)(int signo, siginfo_t *info, void *context);
  } action;                      /* Signal action */
  sigset_t  mask;                /* Additional signals to mask while the
				  * the signal-catching functin executes */
  siginfo_t info;                /* Signal information */
  uint8_t   type;                /* (Used to manage allocations) */
};
typedef struct sigq_s sigq_t;

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* The g_sigfreeaction data structure is a list of available signal action
 * structures.
 */

extern sq_queue_t  g_sigfreeaction;

/* The g_sigpendingaction data structure is a list of available pending
 * signal action structures.
 */

extern sq_queue_t  g_sigpendingaction;

/* The g_sigpendingirqaction is a list of available pending signal actions
 * that are reserved for use by interrupt handlers.
 */

extern sq_queue_t  g_sigpendingirqaction;

/* The g_sigpendingsignal data structure is a list of available pending
 * signal structures.
 */

extern sq_queue_t  g_sigpendingsignal;

/* The g_sigpendingirqsignal data structure is a list of available pending
 * signal structures that are reserved for use by interrupt handlers.
 */

extern sq_queue_t  g_sigpendingirqsignal;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Internal signal-related interfaces ***************************************/

/* sig_intialize.c */

extern void weak_function sig_initialize(void);
extern void               sig_allocateactionblock(void);

/* sig_action.c */

extern void               sig_releaseaction(FAR sigactq_t *sigact);

/* sig_pending.c */

extern sigset_t           sig_pendingset(FAR _TCB *stcb);

/* In files of the same name */

extern FAR sigq_t        *sig_allocatependingsigaction(void);
extern void               sig_cleanup(FAR _TCB *stcb);
extern void               sig_deliver(FAR _TCB *stcb);
extern FAR sigactq_t     *sig_findaction(FAR _TCB *stcb, int signo);
extern int                sig_lowest(sigset_t *set);
#ifdef CONFIG_CAN_PASS_STRUCTS
extern int                sig_mqnotempty(int tid, int signo, union sigval value);
#else
extern int                sig_mqnotempty(int tid, int signo,
                                         void *sival_ptr);
#endif
extern int                sig_received(FAR _TCB *stcb, siginfo_t *info);
extern void               sig_releasependingsigaction(FAR sigq_t *sigq);
extern void               sig_releasependingsignal(FAR sigpendq_t *sigpend);
extern FAR sigpendq_t    *sig_removependingsignal(FAR _TCB *stcb, int signo);
extern void               sig_unmaskpendingsignal(void);

#endif /* __SIG_INTERNAL_H */
