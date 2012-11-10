/********************************************************************************
 * libc/pthread/pthread_barrierattrgetpshared.c
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
 * Function: pthread_barrierattr_getpshared
 *
 * Description:
 *   The pthread_barrierattr_getpshared() function will obtain the value of the
 *   process-shared attribute from the attributes object referenced by attr.
 *
 * Parameters:
 *   attr - barrier attributes to be queried.
 *   pshared - the location to stored the current value of the pshared attribute.
 *
 * Return Value:
 *   0 (OK) on success or EINVAL if either attr or pshared is invalid.
 *
 * Assumptions:
 *
 ********************************************************************************/

int pthread_barrierattr_getpshared(FAR const pthread_barrierattr_t *attr, FAR int *pshared)
{
  int ret = OK;

  if (!attr || !pshared)
    {
      ret = EINVAL;
    }
  else
    {
      *pshared = attr->pshared;
    }
  return ret;
}
