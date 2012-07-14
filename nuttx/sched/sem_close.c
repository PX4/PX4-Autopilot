/****************************************************************************
 * sched/sem_close.c
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

#include <errno.h>
#include <semaphore.h>
#include <sched.h>

#include "os_internal.h"
#include "sem_internal.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sem_close
 *
 * Description:
 *   This function is called to indicate that the calling task is finished
 *   with the specified named semaphore, 'sem'.  The sem_close() deallocates
 *   any system resources allocated by the system for this named semaphore.
 *
 *   If the semaphore has not been removed with a call to sem_unlink(), then
 *   sem_close() has no effect on the named semaphore.  However, when the
 *   named semaphore has been fully unlinked, the semaphore will vanish when
 *   the last task closes it.
 *
 * Parameters:
 *  sem - semaphore descriptor
 *
 * Return Value:
 *  0 (OK), or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *   - Care must be taken to avoid risking the deletion of a semaphore that
 *     another calling task has already locked.
 *   - sem_close must not be called for an un-named semaphore
 *
 ****************************************************************************/

int sem_close(FAR sem_t *sem)
{
  FAR nsem_t *psem;
  int ret = ERROR;

  /* Verify the inputs */

  if (sem)
    {
      sched_lock();

      /* Search the list of named semaphores */

      for (psem = (FAR nsem_t*)g_nsems.head;
           ((psem) && (sem != &psem->sem));
           psem = psem->flink);

      /* Check if we found it */

      if (psem)
        {
          /* Decrement the count of sem_open connections to this semaphore */

          if (psem->nconnect) psem->nconnect--;

          /* If the semaphore is no long connected to any processes AND the
           * semaphore was previously unlinked, then deallocate it.
           */

          if (!psem->nconnect && psem->unlinked)
            {
              dq_rem((FAR dq_entry_t*)psem, &g_nsems);
              sched_free(psem);
            }
          ret = OK;
        }

      sched_unlock();
    }

  return ret;
}
