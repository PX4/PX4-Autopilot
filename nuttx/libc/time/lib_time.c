/****************************************************************************
 * libc/time/lib_time.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <sys/time.h>
#include <time.h>

#ifndef CONFIG_DISABLE_CLOCK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
 
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  time
 *
 * Description:
 *   Get the current calendar time as a time_t object.  The function returns
 *   this value, and if the argument is not a null pointer, the value is also
 *   set to the object pointed by tloc.
 *
 *   Note that this function is just a thin wrapper around gettimeofday()
 *   and is provided for compatibility.  gettimeofday() is the preffered way
 *   to obtain system time.
 *
 * Parameters:
 *   Pointer to an object of type time_t, where the time value is stored.
 *   Alternativelly, this parameter can be a null pointer, in which case the
 *   parameter is not used, but a time_t object is still returned by the
 *   function.
 *
 * Return Value:
 *   The current calendar time as a time_t object.  If the argument is not
 *   a null pointer, the return value is the same as the one stored in the
 *   location pointed by the argument.
 *
 *   If the function could not retrieve the calendar time, it returns a -1
 *   value.
 *
 ****************************************************************************/

time_t time(time_t *tloc)
{
  struct timeval tp;
  int ret;

  /* Get the current time from the system */

  ret = gettimeofday(&tp, NULL);
  if (ret == OK)
    {
      /* Return the seconds since the epoch */

      if (tloc)
        {
          *tloc = tp.tv_sec;
        }

      return tp.tv_sec;
    }

  return (time_t)ERROR;
}

#endif /* !CONFIG_DISABLE_CLOCK */
