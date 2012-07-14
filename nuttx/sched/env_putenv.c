/****************************************************************************
 * sched/env_putenv.c
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

#ifndef CONFIG_DISABLE_ENVIRON

#include <stdlib.h>
#include <sched.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: putenv
 *
 * Description:
 *   The putenv() function adds or changes the value of environment variables.
 *   The argument string is of the form name=value. If name does not already
 *   exist in  the  environment, then string is added to the environment. If
 *   name does exist, then the value of name in the environment is changed to
 *   value.
 *
 * Parameters:
 *   name=value string describing the environment setting to add/modify
 *
 * Return Value:
 *   Zero on sucess
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int putenv(FAR const char *string)
{
  char *pname;
  char *pequal;
  int ret = OK;

  /* Verify that a string was passed */

  if (!string)
    {
      ret = EINVAL;
      goto errout;
    }

  /* Parse the name=value string */

  pname = strdup(string);
  if (!pname)
    {
      ret = ENOMEM;
      goto errout;
    }

  pequal = strchr( pname, '=');
  if (pequal)
    {
      /* Then let setenv do all of the work */

      *pequal = '\0';
      ret = setenv(pname, pequal+1, TRUE);
    }

  kfree(pname);
  return ret;

errout:
  errno = ret;
  return ERROR;
}

#endif /* CONFIG_DISABLE_ENVIRON */



