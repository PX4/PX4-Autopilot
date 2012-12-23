/****************************************************************************
 * sched/sem_open.c
 *
 *   Copyright (C) 2007, 2008, 2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdarg.h>
#include <limits.h>
#include <fcntl.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

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
 * Name: sem_open
 *
 * Description:
 *   This function establishes a connection between named semaphores and a
 *   task.  Following a call to sem_open() with the semaphore name, the task
 *   may reference the semaphore associated with name using the address
 *   returned by this call.  The semaphore may be used in subsequent calls
 *   to sem_wait(), sem_trywait(), and sem_post().  The semaphore remains
 *   usable until the semaphore is closed by a successful call to sem_close().
 *
 *   If a task makes multiple calls to sem_open() with the same name, then
 *   the same semaphore address is returned (provided there have been no
 *   calls to sem_unlink()).
 *
 * Parameters:
 *   name  - Semaphore name
 *   oflag - Semaphore creation options.  This may either or both of the
 *     following bit settings.
 *     oflag = 0:  Connect to the semaphore only if it already exists.
 *     oflag = O_CREAT:  Connect to the semaphore if it exists, otherwise
 *        create the semaphore.
 *     oflag = O_CREAT|O_EXCL:  Create a new semaphore
 *        unless one of this name already exists.
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *     parameters are expected:
 *     1. mode_t mode (ignored), and
 *     2. unsigned int value.  This initial value of the semaphore. Valid
 *        initial values of the semaphore must be less than or equal to
 *        SEM_VALUE_MAX.
 *
 * Return Value:
 *   A pointer to sem_t or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR sem_t *sem_open (FAR const char *name, int oflag, ...)
{
  int          namelen;
  FAR nsem_t  *psem;
  FAR sem_t   *sem = (FAR sem_t*)ERROR;
  va_list      arg;          /* Points to each un-named argument */
  unsigned int value;        /* Semaphore value parameter */

  /* Make sure that a non-NULL name is supplied */

  if (name)
    {
      /* The POSIX specification requires that the "check for the
       * existence of a semaphore and the creation of the semaphore
       * if it does not exist shall be atomic with respect to other
       * processes executing sem_open()..."  A simple sched_lock()
       * should be sufficient to meet this requirement.
       */

      sched_lock();
      namelen = strlen(name);
      if (namelen > 0)
        {
          /* See if the semaphore already exists */

          psem = sem_findnamed(name);
          if (psem)
            {
              /* It does.  Check if the caller wanted to created 
               * a new semahore with this name.
               */

              if (!(oflag & O_CREAT) || !(oflag & O_EXCL))
                {
                  /* Allow a new connection to the semaphore */

                  psem->nconnect++;
                  sem = &psem->sem;
                }
            }

          /* It doesn't exist.  Should we create one? */

          else if ((oflag & O_CREAT) != 0)
            {
              /* Set up to get the optional arguments needed to create
               * a message queue.
               */

              va_start(arg, oflag);
              (void)va_arg(arg, mode_t); /* Creation mode parameter (ignored) */
              value = va_arg(arg, unsigned int);

              /* Verify that a legal initial value was selected. */

              if (value <= SEM_VALUE_MAX)
                {
                  /* Allocate memory for the new semaphore */

                  psem = (FAR nsem_t*)kmalloc((sizeof(nsem_t) + namelen + 1));
                  if (psem)
                    {
                      /* Initialize the named semaphore */

                      sem = &psem->sem;
                      sem_init(sem, 0, value);

                      psem->nconnect = 1;
                      psem->unlinked = false;
                      psem->name = (FAR char*)psem + sizeof(nsem_t);
                      strcpy(psem->name, name);

                      /* Add the new semaphore to the list of named
                       * semaphores
                       */

                      dq_addfirst((FAR dq_entry_t*)psem, &g_nsems);
                    }

                  /* Clean-up variable argument stuff */

                  va_end(arg);
                }
            }
        }

      sched_unlock();
    }

  return sem;
}

