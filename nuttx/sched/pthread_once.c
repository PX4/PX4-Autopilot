/********************************************************************************
 * sched/pthread_once.c
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Type Declarations
 ********************************************************************************/

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/********************************************************************************
 * Private Variables
 ********************************************************************************/

/********************************************************************************
 * Private Function Prototypes
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: pthread_once
 *
 * Description:
 *   The  first call to pthread_once() by any thread with a given once_control,
 *   will call the init_routine with no arguments. Subsequent calls to
 *   pthread_once() with the same once_control will have no effect.  On return
 *   from pthread_once(), init_routine will have completed.
 *
 * Parameters:
 *   once_control - Determines if init_routine should be called.  once_control
 *      should be declared and intialized as follows:
 *
 *        pthread_once_t once_control = PTHREAD_ONCE_INIT;
 *
 *       PTHREAD_ONCE_INIT is defined in pthread.h
 *   init_routine - The initialization routine that will be called once.
 *
 * Return Value:
 *   0 (OK) on success or EINVAL if either once_control or init_routine are
 *   invalid
 *
 * Assumptions:
 *
 ********************************************************************************/

int pthread_once(FAR pthread_once_t *once_control,
                 CODE void (*init_routine)(void))
{
  /* Sanity checks */

  if (once_control && init_routine)
    {
      /* Prohibit pre-emption while we test and set the once_control */

      sched_lock();
      if (!*once_control)
        {
          *once_control = true;

          /* Call the init_routine with pre-emption enabled. */

          sched_unlock();
          init_routine();
          return OK;
        }

      /* The init_routine has already been called.  Restore pre-emption and return */

      sched_unlock();
      return OK;
    }

  /* One of the two arguments is NULL */

  return EINVAL;
}
