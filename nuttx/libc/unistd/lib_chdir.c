/****************************************************************************
 * libc/unistd/lib_chdir.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "lib_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _trimdir
 ****************************************************************************/

#if 0
static inline void _trimdir(char *path)
{
 /* Skip any trailing '/' characters (unless it is also the leading '/') */

 int len = strlen(path) - 1;
 while (len > 0 && path[len] == '/')
   {
      path[len] = '\0';
      len--;
   }
}
#else
#  define _trimdir(p)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chdir
 *
 * Description:
 *   The chdir() function causes the directory named by the pathname pointed
 *   to by the 'path' argument to become the current working directory; that
 *   is, the starting point for path searches for pathnames not beginning
 *   with '/'.
 *
 * Input Parmeters:
 *   path - A pointer to a directory to use as the new current working
 *     directory
 *
 * Returned Value:
 *   0(OK) on success; -1(ERROR) on failure with errno set appropriately:
 *
 *   EACCES
 *     Search permission is denied for any component of the pathname.
 *   ELOOP
 *     A loop exists in symbolic links encountered during resolution of the
 *     'path' argument OR more that SYMLOOP_MAX symbolic links in the
 *     resolution of the 'path' argument.
 *   ENAMETOOLONG
 *     The length of the path argument exceeds PATH_MAX or a pathname component
 *     is longer than NAME_MAX.
 *   ENOENT
 *     A component of 'path' does not name an existing directory or path is
 *     an empty string.
 *   ENOTDIR
 *     A component of the pathname is not a directory.
 *
 ****************************************************************************/

int chdir(FAR const char *path)
{
  struct stat buf;
  char *oldpwd;
  char *alloc;
  int err;
  int ret;

  /* Verify the input parameters */

  if (!path)
    {
      err = ENOENT;
      goto errout;
    }

  /* Verify that 'path' refers to a directory */

  ret = stat(path, &buf);
  if (ret != 0)
    {
      err = ENOENT;
      goto errout;
    }

  /* Something exists here... is it a directory? */

  if (!S_ISDIR(buf.st_mode))
    {
      err = ENOTDIR;
      goto errout;
    }

  /* Yes, it is a directory. Remove any trailing '/' characters from the path */

  _trimdir(path);

  /* Replace any preceding OLDPWD with the current PWD (this is to
   * support 'cd -' in NSH)
   */

  sched_lock();
  oldpwd = getenv("PWD");
  if (!oldpwd)
    {
      oldpwd = CONFIG_LIB_HOMEDIR;
    }

  alloc = strdup(oldpwd);  /* kludge needed because environment is realloc'ed */
  setenv("OLDPWD", alloc, TRUE);
  lib_free(alloc);

  /* Set the cwd to the input 'path' */

  setenv("PWD", path, TRUE);
  sched_unlock();
  return OK;

errout:
  set_errno(err);
  return ERROR;
}
#endif /* CONFIG_NFILE_DESCRIPTORS && !CONFIG_DISABLE_ENVIRON */
