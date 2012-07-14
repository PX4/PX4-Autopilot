/****************************************************************************
 * sched/sem_wait.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "sem_internal.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_wait
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem'.  If
 *   the semaphore value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 * Parameters:
 *   sem - Semaphore descriptor.
 *
 * Return Value:
 *   0 (OK), or -1 (ERROR) is unsuccessful
 *   If this function returns -1 (ERROR), then the cause of the failure will
 *   be reported in 'errno' as:
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sem_wait(FAR sem_t *sem)
{
  FAR _TCB  *rtcb = (FAR _TCB*)g_readytorun.head;
  int        ret  = ERROR;
  irqstate_t saved_state;

  /* This API should not be called from interrupt handlers */

  DEBUGASSERT(up_interrupt_context() == false)

  /* Assume any errors reported are due to invalid arguments. */

  errno = EINVAL;

  if (sem)
    {
      /* The following operations must be performed with interrupts
       * disabled because sem_post() may be called from an interrupt
       * handler.
       */

      saved_state = irqsave();

      /* Check if the lock is available */

      if (sem->semcount > 0)
        {
          /* It is, let the task take the semaphore. */

          sem->semcount--;
          sem_addholder(sem);
          rtcb->waitsem = NULL;
          ret = OK;
        }

      /* The semaphore is NOT available, We will have to block the
       * current thread of execution.
       */

      else
        {
          /* First, verify that the task is not already waiting on a
           * semaphore
           */

          if (rtcb->waitsem != NULL)
            {
              PANIC(OSERR_BADWAITSEM);
            }

          /* Handle the POSIX semaphore (but don't set the owner yet) */

          sem->semcount--;

          /* Save the waited on semaphore in the TCB */

          rtcb->waitsem = sem;

          /* If priority inheritance is enabled, then check the priority of
           * the holder of the semaphore.
           */

#ifdef CONFIG_PRIORITY_INHERITANCE
          /* Disable context switching.  The following operations must be
           * atomic with regard to the scheduler.
           */

          sched_lock();

          /* Boost the priority of any threads holding a count on the
           * semaphore.
           */

          sem_boostpriority(sem);
#endif
          /* Add the TCB to the prioritized semaphore wait queue */

          errno = 0;
          up_block_task(rtcb, TSTATE_WAIT_SEM);

          /* When we resume at this point, either (1) the semaphore has been
           * assigned to this thread of execution, or (2) the semaphore wait
           * has been interrupted by a signal or a timeout.  We can detect these
           * latter cases be examining the errno value.
           *
           * In the event that the semaphore wait was interrupted by a signal or
           * a timeout, certain semaphore clean-up operations have already been
           * performed (see sem_waitirq.c).  Specifically:
           *
           * - sem_canceled() was called to restore the priority of all threads
           *   that hold a reference to the semaphore,
           * - The semaphore count was decremented, and
           * - tcb->waitsem was nullifed.
           *
           * It is necesaary to do these things in sem_waitirq.c because a long
           * time may elapse between the time that the signal was issued and
           * this thread is awakened and this leaves a door open to several
           * race conditions.
           */

          if (errno != EINTR && errno != ETIMEDOUT)
            {
              /* Not awakened by a signal or a timeout... We hold the semaphore */

              sem_addholder(sem);
              ret = OK;
            }

#ifdef CONFIG_PRIORITY_INHERITANCE
          sched_unlock();
#endif
        }

      /* Interrupts may now be enabled. */

      irqrestore(saved_state);
    }

  return ret;
}
