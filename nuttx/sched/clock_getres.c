/************************************************************************
 * sched/clock_getres.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include "clock_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Private Function Prototypes
 ************************************************************************/

/**********************************************************************
 * Public Constant Data
 **********************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: clock_getres
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 ************************************************************************/

int clock_getres(clockid_t clock_id, struct timespec *res)
{
  uint32_t time_res;
  int      ret = OK;

  sdbg("clock_id=%d\n", clock_id);

  /* Only CLOCK_REALTIME is supported */

  if (clock_id != CLOCK_REALTIME)
    {
      sdbg("Returning ERROR\n");
      set_errno(EINVAL);
      ret = ERROR;
    }
  else
    {
      /* Get the clock resolution in nanoseconds */

      time_res = MSEC_PER_TICK * NSEC_PER_MSEC;

      /* And return this as a timespec. */

      res->tv_sec  = 0;
      res->tv_nsec = time_res;

      sdbg("Returning res=(%d,%d) time_res=%d\n",
           (int)res->tv_sec, (int)res->tv_nsec,
           (int)time_res);
    }

  return ret;
}
