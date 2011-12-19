/****************************************************************************
 * sched/pg_internal.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_PG_INTERNAL_H
#define __SCHED_PG_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>
#include <queue.h>

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Supply reasonable (but probably non-optimal) default settings if
 * configuration items are omitted.
 */

#ifndef CONFIG_PAGING_DEFPRIO
#  define CONFIG_PAGING_DEFPRIO 50
#endif

#ifndef CONFIG_PAGING_WORKPERIOD
#  define CONFIG_PAGING_WORKPERIOD (500*1000) /* 1/2 second */
#endif

#ifndef CONFIG_PAGING_STACKSIZE
#  define CONFIG_PAGING_STACKSIZE  CONFIG_IDLETHREAD_STACKSIZE
#endif

#ifdef CONFIG_DISABLE_SIGNALS
#  warning "Page fill support requires signals"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY

/* This is the task IDof the page fill worker thread.  This value was set in
 * os_start when the page fill worker thread was started.
 */

extern pid_t g_pgworker;

/* The page fill worker thread maintains a static variable called g_pftcb.
 * If no page fill is in progress, g_pftcb will be NULL. Otherwise, g_pftcb
 * will point to the TCB of the task which is receiving the fill that is
 * in progess.
 *
 * NOTE: I think that this is the only state in which a TCB does not reside
 * in some list.  Here is it in limbo, outside of the normally queuing while
 * the page file is in progress.  Where here, it will be marked with
 * TSTATE_TASK_INVALID.
 */

extern FAR _TCB *g_pftcb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pg_worker
 *
 * Description:
 *   This is the entry point of the worker thread that performs the actual
 *   page file.
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

extern int pg_worker(int argc, char *argv[]);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PAGING */
#endif /* __SCHED_PG_INTERNAL_H */
