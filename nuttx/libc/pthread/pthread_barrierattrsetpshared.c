/********************************************************************************
 * libc/pthread/pthread_barrierattrsetpshared.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 * Function: pthread_barrierattr_setpshared
 *
 * Description:
 *   The process-shared attribute is set to PTHREAD_PROCESS_SHARED to permit a
 *   barrier to be operated upon by any thread that has access to the memory where
 *   the barrier is allocated. If the process-shared attribute is
 *   PTHREAD_PROCESS_PRIVATE, the barrier can only be operated upon by threads
 *   created within the same process as the thread that initialized the barrier.
 *   If threads of different processes attempt to operate on such a barrier, the
 *   behavior is undefined. The default value of the attribute is
 *   PTHREAD_PROCESS_PRIVATE.
 *
 *   Both constants PTHREAD_PROCESS_SHARED and PTHREAD_PROCESS_PRIVATE are defined
 *   in pthread.h.
 *
 * Parameters:
 *   attr - barrier attributes to be modified.
 *   pshared - the new value of the pshared attribute.
 *
 * Return Value:
 *   0 (OK) on success or EINVAL if either attr is invalid or pshared is not one
 *   of PTHREAD_PROCESS_SHARED or PTHREAD_PROCESS_PRIVATE.
 *
 * Assumptions:
 *
 ********************************************************************************/

int pthread_barrierattr_setpshared(FAR pthread_barrierattr_t *attr, int pshared)
{
  int ret = OK;

  if (!attr || (pshared != PTHREAD_PROCESS_SHARED && pshared != PTHREAD_PROCESS_PRIVATE))
    {
      ret = EINVAL;
    }
  else
    {
      attr->pshared = pshared;
    }
  return ret;
}
