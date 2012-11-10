/****************************************************************************
 * libc/pthread/pthread_attrinit.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <pthread.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/pthread.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* Default pthread attributes (see included/nuttx/pthread.h).  When configured
 * to build separate kernel- and user-address spaces, this global is
 * duplicated in each address spaced.  This copy can only be shared within
 * the user address space. 
 */

#if defined(CONFIG_NUTTX_KERNEL) && !defined(__KERNEL__)
pthread_attr_t g_default_pthread_attr = PTHREAD_ATTR_INITIALIZER;
#endif

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
 * Function:  pthread_attr_init
 *
 * Description:
 *   Initializes a thread attributes object (attr) with
 *   default values for all of the individual attributes
 *   used by a given implementation.
 *
 * Parameters:
 *   attr
 *
 * Return Value:
 *   0 on success, otherwise an error number
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_attr_init(FAR pthread_attr_t *attr)
{
  int ret = OK;

  sdbg("attr=0x%p\n", attr);
  if (!attr)
    {
      ret = ENOMEM;
    }
  else
    {
      /* Set the child thread priority to be the default
       * priority. Set the child stack size to some arbitrary
       * default value.
       */

      memcpy(attr, &g_default_pthread_attr, sizeof(pthread_attr_t));
    }

  sdbg("Returning %d\n", ret);
  return ret;
}

