/****************************************************************************
 * mm/mm_sem.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "mm_environment.h"
#include <unistd.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include "mm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define the following to enable semaphore state monitoring */
//#define MONITOR_MM_SEMAPHORE 1

#ifdef MONITOR_MM_SEMAPHORE
#  ifdef CONFIG_DEBUG
#    include <debug.h>
#    define msemdbg dbg
#  else
#    define msemdbg printf
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define msemdbg(x...)
#  else
#    define msemdbg (void)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Mutually exclusive access to this data set is enforced with
 * the following (un-named) semaphore. */

static  sem_t g_mm_semaphore;
static  pid_t g_holder;
static  int   g_counts_held;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_seminitialize
 *
 * Description:
 *   Initialize the MM mutex
 *
 ****************************************************************************/

void mm_seminitialize(void)
{
  /* Initialize the MM semaphore to one (to support one-at-
   * a-time access to private data sets.
   */

  (void)sem_init(&g_mm_semaphore, 0, 1);

  g_holder      = -1;
  g_counts_held = 0;
}

/****************************************************************************
 * Name: mm_trysemaphore
 *
 * Description:
 *   Try to take the MM mutex.  This is called only from the
 *   OS in certain conditions when it is necessary to have
 *   exclusive access to the memory manager but it is
 *   impossible to wait on a semaphore (e.g., the idle process
 *   when it performs its background memory cleanup).
 *
 ****************************************************************************/

#ifndef MM_TEST
int mm_trysemaphore(void)
{
  pid_t my_pid = getpid();

  /* Do I already have the semaphore? */

  if (g_holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

      g_counts_held++;
      return OK;
    }
  else
    {
      /* Try to tak the semaphore (perhaps waiting) */

      if (sem_trywait(&g_mm_semaphore) != 0)
       {
         return ERROR;
       }

      /* We have it.  Claim the stak and return */

      g_holder      = my_pid;
      g_counts_held = 1;
      return OK;
    }
}
#endif

/****************************************************************************
 * Name: mm_takesemaphore
 *
 * Description:
 *   Take the MM mutex.  This is the normal action before all
 *   memory management actions.
 *
 ****************************************************************************/

void mm_takesemaphore(void)
{
  pid_t my_pid = getpid();

  /* Do I already have the semaphore? */

  if (g_holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

      g_counts_held++;
    }
  else
    {
      /* Take the semaphore (perhaps waiting) */

      msemdbg("PID=%d taking\n", my_pid);
      while (sem_wait(&g_mm_semaphore) != 0)
       {
         /* The only case that an error should occur here is if
          * the wait was awakened by a signal.
          */

         ASSERT(mm_errno == EINTR);
       }

      /* We have it.  Claim the stake and return */

      g_holder      = my_pid;
      g_counts_held = 1;
    }

  msemdbg("Holder=%d count=%d\n",
          g_holder, g_counts_held);
}

/****************************************************************************
 * Name: mm_givesemaphore
 *
 * Description:
 *   Release the MM mutex when it is not longer needed.
 *
 ****************************************************************************/

void mm_givesemaphore(void)
{
#ifdef CONFIG_DEBUG
  pid_t my_pid = getpid();
#endif

  /* I better be holding at least one reference to the semaphore */

  DEBUGASSERT(g_holder == my_pid);

  /* Do I hold multiple references to the semphore */

  if (g_counts_held > 1)
    {
      /* Yes, just release one count and return */

      g_counts_held--;
      msemdbg("Holder=%d count=%d\n",
              g_holder, g_counts_held);
    }
  else
    {
      /* Nope, this is the last reference I have */

      msemdbg("PID=%d giving\n", my_pid);
      g_holder      = -1;
      g_counts_held = 0;
      ASSERT(sem_post(&g_mm_semaphore) == 0);
    }
}

/****************************************************************************
 * Name: mm_getsemaphore
 *
 * Description:
 *   Return the current value of the MM semaphore (for test
 *   purposes only)
 *
 ****************************************************************************/

#ifdef MM_TEST
int mm_getsemaphore(void)
{
  int sval;
  sem_getvalue(&g_mm_semaphore, &sval);
  return sval;
}
#endif

