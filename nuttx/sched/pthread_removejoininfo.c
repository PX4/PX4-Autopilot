/************************************************************************
 * sched/pthread_removejoininfo.c
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

#include <sys/types.h>
#include "pthread_internal.h"

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
 * Name: pthread_removejoininfo
 *
 * Description:
 *   Remove a join_t from the local data set.
 *
 * Parameters:
 *   pid
 *
 * Return Value:
 *   None or pointer to the found entry.
 *
 * Assumptions:
 *   The caller has provided protection from re-entrancy.
 *
 ************************************************************************/

FAR join_t *pthread_removejoininfo(pid_t pid)
{
  FAR join_t *prev;
  FAR join_t *join;

  /* Find the entry with the matching pid */

  for (prev = NULL, join = g_pthread_head;
       (join && (pid_t)join->thread != pid);
       prev = join, join = join->next);

  /* Remove it from the data set. */

  /* First check if this is the entry at the head of the list. */

  if (join)
    {
      if (!prev)
        {
          /* Check if this is the only entry in the list */

          if (!join->next)
            {
              g_pthread_head = NULL;
              g_pthread_tail = NULL;
            }

          /* Otherwise, remove it from the head of the list */

          else
            {
              g_pthread_head = join->next;
            }
        }

      /* It is not at the head of the list, check if it is at the tail. */

      else if (!join->next)
        {
          g_pthread_tail = prev;
          prev->next = NULL;
        }

      /* No, remove it from the middle of the list. */

      else
        {
          prev->next = join->next;
        }
    }

  return join;
}

