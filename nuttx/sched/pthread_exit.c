/************************************************************************
 * sched/pthread_exit.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

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
 * Name: pthread_exit
 *
 * Description:
 *   Terminate execution of a thread started with pthread_create.
 *
 * Parameters:
 *   exit_valie
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ************************************************************************/

void pthread_exit(FAR void *exit_value)
{
  int error_code = (int)((intptr_t)exit_value);
  int status;

  sdbg("exit_value=%p\n", exit_value);

  /* Block any signal actions that would awaken us while were
   * are performing the JOIN handshake.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  {
    sigset_t set = ALL_SIGNAL_SET;
    (void)sigprocmask(SIG_SETMASK, &set, NULL);
  }
#endif

  /* Complete pending join operations */

  status = pthread_completejoin(getpid(), exit_value);
  if (status != OK)
    {
      /* Assume that the join completion failured because this
       * not really a pthread.  Exit by calling exit() to flush
       * and close all file descriptors and calling atexit()
       * functions.
       */

      if (error_code == EXIT_SUCCESS)
        {
           error_code = EXIT_FAILURE;
        }

      exit(error_code);
    }

  /* Then just exit, retaining all file descriptors and without
   * calling atexit() functions.
   */

  _exit(error_code);
}

