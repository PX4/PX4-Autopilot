/****************************************************************************
 * sched/sem_unlink.c
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

#include <stdbool.h>
#include <semaphore.h>
#include <sched.h>
#include <queue.h>

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
 * Name: sem_unlink
 *
 * Description:
 *   This function removes the semaphore named by the input parameter 'name.'
 *   If the semaphore named by 'name' is currently referenced by other task,
 *   the sem_unlink() will have no effect on the state of the semaphore.  If
 *   one or more processes have the semaphore open when sem_unlink() is
 *   called, destruction of the semaphore will be postponed until all
 *   references to the semaphore have been destroyed by calls of sem_close().
 *
 * Parameters:
 *   name - Semaphore name
 *
 * Return Value:
 *  0 (OK), or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sem_unlink(FAR const char *name)
{
  FAR nsem_t *psem;
  int         ret = ERROR;

  /* Verify the input values */

  if (name)
    {
      sched_lock();

      /* Find the named semaphore */

      psem = sem_findnamed(name);

      /* Check if the semaphore was found */

      if (psem)
        {
          /* If the named semaphore was found and if there are no
           * connects to it, then deallocate it
           */

          if (!psem->nconnect)
            {
              dq_rem((FAR dq_entry_t*)psem, &g_nsems);
              sched_free(psem);
            }

          /* If one or more process still has the semaphore open,
           * then just mark it as unlinked.  The unlinked semaphore will
           * be deleted when the final process closes the semaphore.
           */

          else
            {
              psem->unlinked = true;
            }
          ret = OK;
        }

      sched_unlock();
    }

  return ret;
}
