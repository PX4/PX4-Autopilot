/****************************************************************************
 * binfmt/binfmt_loadmodule.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt_internal.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: load_default_priority
 *
 * Description:
 *   Set the default priority of the module to be loaded.  This may be
 *   changed (1) by the actions of the binary format's load() method if
 *   the binary format contains priority informaition, or (2) by the user
 *   between calls to load_module() and exec_module().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; Otherwise, -1 (ERROR) is returned and
 *   the errno variable is set appropriately.
 *
 ****************************************************************************/

static int load_default_priority(FAR struct binary_s *bin)
{
  struct sched_param param;
  int ret;

  /* Get the priority of this thread */

  ret = sched_getparam(0, &param);
  if (ret < 0)
    {
      bdbg("ERROR: sched_getparam failed: %d\n", errno);
      return ERROR;
    }
  
  /* Save that as the priority of child thread */

  bin->priority = param.sched_priority;
  return ret;
}

/****************************************************************************
 * Name: load_absmodule
 *
 * Description:
 *   Load a module into memory, bind it to an exported symbol take, and
 *   prep the module for execution.  bin->filename is known to be an absolute
 *   path to the file to be loaded.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int load_absmodule(FAR struct binary_s *bin)
{
  FAR struct binfmt_s *binfmt;
  int ret = -ENOENT;

  bdbg("Loading %s\n", bin->filename);

  /* Disabling pre-emption should be sufficient protection while accessing
   * the list of registered binary format handlers.
   */

  sched_lock();

  /* Traverse the list of registered binary format handlers.  Stop
   * when either (1) a handler recognized and loads the format, or
   * (2) no handler recognizes the format.
   */

  for (binfmt = g_binfmts; binfmt; binfmt = binfmt->next)
    {
      /* Use this handler to try to load the format */

      ret = binfmt->load(bin);
      if (ret == OK)
        {
          /* Successfully loaded -- break out with ret == 0 */

          bvdbg("Successfully loaded module %s\n", bin->filename);
          dump_module(bin);
          break;
        }
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: load_module
 *
 * Description:
 *   Load a module into memory, bind it to an exported symbol take, and
 *   prep the module for execution.
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   Returns 0 (OK) on success.  On failure, it returns -1 (ERROR) with
 *   errno set appropriately.
 *
 ****************************************************************************/

int load_module(FAR struct binary_s *bin)
{
  int ret = -EINVAL;

  /* Verify that we were provided something to work with */

#ifdef CONFIG_DEBUG
  if (bin && bin->filename)
#endif
    {
      /* Set the default priority of the new program. */

      ret = load_default_priority(bin);
      if (ret < 0)
        {
          /* The errno is already set in this case */

          return ERROR;
        }

      /* Were we given a relative path?  Or an absolute path to the file to
       * be loaded?  Absolute paths start with '/'.
       */

#ifdef CONFIG_BINFMT_EXEPATH
      if (bin->filename[0] != '/')
        {
          FAR const char *relpath;
          FAR char *fullpath;
          EXEPATH_HANDLE handle;

          /* Set aside the relative path */

          relpath = bin->filename;
          ret     = -ENOENT;

          /* Initialize to traverse the PATH variable */

          handle = exepath_init();
          if (handle)
            {
              /* Get the next absolute file path */

              while ((fullpath = exepath_next(handle, relpath)) != NULL)
                {
                  /* Try to load the file at this path */

                  bin->filename = fullpath;
                  ret = load_absmodule(bin);

                  /* Free the allocated fullpath */

                  kfree(fullpath);

                  /* Break out of the loop with ret == OK on success */

                  if (ret == OK)
                    {
                      break;
                    }
                }
            }

          /* Restore the relative path.  This is not needed for anything
           * but debug output after the file has been loaded.
           */

          bin->filename = relpath;
        }
      else
#endif
        {
          /* We already have the one and only absolute path to the file to
           * be loaded.
           */

          ret = load_absmodule(bin);
        }
    }

  /* This is an end-user function.  Return failures via errno */

  if (ret < 0)
    {
      bdbg("Returning errno %d\n", -ret);
      errno = -ret;
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_BINFMT_DISABLE */

