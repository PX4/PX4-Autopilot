/****************************************************************************
 * binfmt/binfmt_exepath.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_BINFMT_EXEPATH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct exepath_s
{
  FAR char *next; /* Pointer to the next (unterminated) value in the PATH variable */
  char path[1];
};
#define SIZEOF_EXEPATH_S(n) (sizeof(struct exepath_s) + (n) - 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exepath_init
 *
 * Description:
 *   Initialize for the traversal of each value in the PATH variable.  The
 *   usage is sequence is as follows:
 *
 *   1) Call exepath_init() to initialize for the traversal.  exepath_init()
 *      will return an opaque handle that can then be provided to
 *      exepath_next() and exepath_release().
 *   2) Call exepath_next() repeatedly to examine every file that lies
 *      in the directories of the PATH variable
 *   3) Call exepath_release() to free resources set aside by exepath_init().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, exepath_init() return a non-NULL, opaque handle that may
 *   subsequently be used in calls to exepath_next() and exepath_release().
 *   On error, a NULL handle value will be returned.  The most likely cause
 *   of an error would be that the there is no value associated with the
 *   PATH variable.
 *
 ****************************************************************************/

EXEPATH_HANDLE exepath_init(void)
{
  FAR struct exepath_s *exepath;
  FAR char *path;

  /* Get the value of the PATH variable */

  path = getenv("PATH");
  if (!path)
    {
      /* getenv() will return a NULL value if the PATH variable does not
       * exist in the environment.
       */

      return (EXEPATH_HANDLE)NULL;
    }

  /* Allocate a container for the PATH variable contents */

  exepath = (FAR struct exepath_s *)kmalloc(SIZEOF_EXEPATH_S(strlen(path) + 1));
  if (!exepath)
    {
      /* Ooops.. we are out of memory */

      return (EXEPATH_HANDLE)NULL;
    }

  /* Populate the container */

  strcpy(exepath->path, path);
  exepath->next = exepath->path;

  /* And return the containing cast to an opaque handle */

  return (EXEPATH_HANDLE)exepath;
}

 /****************************************************************************
 * Name: exepath_next
 *
 * Description:
 *   Traverse all possible values in the PATH variable in attempt to find
 *   the full path to an executable file when only a relative path is
 *   provided.
 *
 * Input Parameters:
 *   handle - The handle value returned by exepath_init
 *   relpath - The relative path to the file to be found. 
 *
 * Returned Value:
 *   On success, a non-NULL pointer to a null-terminated string is provided.
 *   This is the full path to a file that exists in the file system.  This
 *   function will verify that the file exists (but will not verify that it
 *   is marked executable).
 *
 *   NOTE: The string pointer return in the success case points to allocated
 *   memory.  This memory must be freed by the called by calling kfree().
 *
 *   NULL is returned if no path is found to any file with the provided
 *   'relpath' from any absolute path in the PATH variable.  In this case,
 *   there is no point in calling exepath_next() further; exepath_release()
 *   must be called to release resources set aside by expath_init().
 *
 ****************************************************************************/

FAR char *exepath_next(EXEPATH_HANDLE handle, FAR const char *relpath)
{
  FAR struct exepath_s *exepath = (FAR struct exepath_s *)handle;
  struct stat buf;
  FAR char *endptr;
  FAR char *path;
  FAR char *fullpath;
  int pathlen;
  int ret;

  /* Verify that a value handle and relative path were provided */

  DEBUGASSERT(exepath && relpath);
  DEBUGASSERT(relpath[0] != '\0' && relpath[0] != '/');

  /* Loop until (1) we find a file with this relative path from one of the
   * absolute paths in the PATH variable, or (2) all of the absolute paths
   * in the PATH variable have been considered.
   */

  for (;;)
    {
      /* Make sure that exepath->next points to the beginning of a string */

      path = exepath->next;
      if (*path == '\0')
       {
          /* If it points to a NULL it means that either (1) the PATH varialbe
           * is empty, or (2) we have already examined all of the paths in the
           * path variable.
           */

          return (FAR char *)NULL;
       }

      /* Okay... 'path' points to the beginning of the string.  The string may
       * be termined either with (1) ':' which separates the path from the
       * next path in the list, or (2) NUL which marks the end of the list.
       */

      endptr = strchr(path, ':');
      if (!endptr)
        {
          /* If strchr returns NUL it means that ':' does not appear in the
           * string.  Therefore, this must be the final path in the PATH
           * variable content.
           */

          endptr = &path[strlen(path)];
          exepath->next = endptr;
          DEBUGASSERT(*endptr == '\0');
        }
      else
        {
          DEBUGASSERT(*endptr == ':');
          exepath->next = endptr + 1;
          *endptr = '\0';
        }

      pathlen  = strlen(path) + strlen(relpath) + 2;
      fullpath = (FAR char *)kmalloc(pathlen);
      if (!fullpath)
        {
          /* Failed to allocate memory */

          return (FAR char *)NULL;
        }

      /* Construct the full path */

      sprintf(fullpath, "%s/%s", path, relpath);

      /* Verify that a regular file exists at this path */

      ret = stat(fullpath, &buf);;
      if (ret == OK && S_ISREG(buf.st_mode))
        {
          return fullpath;
        }

      /* Failed to stat the file.  Just free the allocated memory and
       * continue to try the next path.
       */

       kfree(fullpath);
    }

  /* We will not get here */
}

/****************************************************************************
 * Name: exepath_release
 *
 * Description:
 *   Release all resources set aside by exepath_init() when the handle value
 *   was created.  The handle value is invalid on return from this function.
 *   Attempts to all exepath_next() or exepath_release() with such a 'stale'
 *   handle will result in undefined (i.e., not good) behavior.
 *
 * Input Parameters:
 *   handle - The handle value returned by exepath_init
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void exepath_release(EXEPATH_HANDLE handle)
{
  kfree(handle);
}

#endif /* !CONFIG_BINFMT_DISABLE && CONFIG_BINFMT_EXEPATH */

