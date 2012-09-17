/********************************************************************************
 * sched/pthread_barrieinit.c
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

#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

/********************************************************************************
 * Definitions
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
 * Name: pthread_barrier_init
 *
 * Description:
 *   The pthread_barrier_init() function allocates any resources required to use
 *   the barrier referenced by 'barrier' and initialized the barrier with the
 *   attributes referenced by attr.  If attr is NULL, the default barrier
 *   attributes will be used. The results are undefined if pthread_barrier_init()
 *   is called when any thread is blocked on the barrier. The results are
 *   undefined if a barrier is used without first being initialized. The results
 *   are undefined if pthread_barrier_init() is called specifying an already
 *   initialized barrier.
 *
 * Parameters:
 *   barrier - the barrier to be initialized
 *   attr - barrier attributes to be used in the initialization.
 *   count - the count to be associated with the barrier.  The count argument
 *     specifies the number of threads that must call pthread_barrier_wait() before
 *     any of them successfully return from the call.  The value specified by
 *     count must be greater than zero.
 *
 * Return Value:
 *   0 (OK) on success or on of the following error numbers:
 *
 *   EAGAIN The system lacks the necessary resources to initialize another barrier.
 *   EINVAL The barrier reference is invalid, or the values specified by attr are
 *          invalid, or the value specified by count is equal to zero.
 *   ENOMEM Insufficient memory exists to initialize the barrier.
 *   EBUSY  The implementation has detected an attempt to reinitialize a barrier
 *          while it is in use.
 *
 * Assumptions:
 *
 ********************************************************************************/

int pthread_barrier_init(FAR pthread_barrier_t *barrier,
                         FAR const pthread_barrierattr_t *attr, unsigned int count)
{
  int ret = OK;

  if (!barrier || count == 0)
    {
      ret = EINVAL;
    }
  else
    {
      sem_init(&barrier->sem, 0, 0);
      barrier->count = count;
    }

  return ret;
}
