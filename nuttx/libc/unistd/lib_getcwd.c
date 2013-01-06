/****************************************************************************
 * libc/unistd/lib_getcwd.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "lib_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getwcd
 *
 * Description:
 *   getcwd() function places the absolute pathname of the current working
 *   directory in the array pointed to by 'buf', and returns 'buf.' The
 *   pathname copied to the array shall contain no components that are
 *   symbolic links. The 'size' argument is the size in bytes of the
 *   character array pointed to by the 'buf' argument.
 *
 * Input Parmeters:
 *   buf - a pointer to the location in which the current working directory
 *     pathaname is returned.
 *   size - The size in bytes avaiable at 'buf'
 *
 * Returned Value:
 *   Upon successful completion, getcwd() returns the 'buf' argument.
 *   Otherwise, getcwd() returns a null pointer and sets errno to indicate
 *   the error:
 *
 *   EINVAL
 *     The 'size' argument is 0 or the 'buf' argument is NULL.
 *   ERANGE
 *     The size argument is greater than 0, but is smaller than the length
 *     of the currrent working directory pathname +1.
 *   EACCES
 *     Read or search permission was denied for a component of the pathname.
 *   ENOMEM
 *  Insufficient storage space is available. 
 *
 ****************************************************************************/

FAR char *getcwd(FAR char *buf, size_t size)
{
  char *pwd;

  /* Verify input parameters */

#ifdef CONFIG_DEBUG
  if (!buf || !size)
    {
      set_errno(EINVAL);
      return NULL;
    }
#endif

  /* If no working directory is defined, then default to the home directory */

  pwd = getenv("PWD");
  if (!pwd)
    {
      pwd = CONFIG_LIB_HOMEDIR;
    }

  /* Verify that the cwd will fit into the user-provided buffer */

  if (strlen(pwd) + 1 > size)
    {
      set_errno(ERANGE);
      return NULL;
    }

  /* Copy the cwd to the user buffer */

  strcpy(buf, pwd);
  sched_unlock();
  return buf;
}
#endif /* CONFIG_NFILE_DESCRIPTORS && !CONFIG_DISABLE_ENVIRON */
