/****************************************************************************
 * apps/nshlib/nsh_envcmds.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
static const char g_pwd[]    = "PWD";
static const char g_oldpwd[] = "OLDPWD";
static const char g_home[]   = CONFIG_LIB_HOMEDIR;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_getwd
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
static inline FAR const char *nsh_getwd(const char *wd)
{
  const char *val;

  /* If no working directory is defined, then default to the home directory */

  val = getenv(wd);
  if (!val)
    {
      val = g_home;
    }
  return val;
}
#endif

/****************************************************************************
 * Name: nsh_getdirpath
 ****************************************************************************/

static inline char *nsh_getdirpath(FAR struct nsh_vtbl_s *vtbl,
                                   const char *dirpath, const char *relpath)
{
  char *alloc;
  int len;

  /* Handle the special case where the dirpath is simply "/" */

  if (strcmp(dirpath, "/") == 0)
    {
      len   = strlen(relpath) + 2;
      alloc = (char*)malloc(len);
      if (alloc)
        {
          sprintf(alloc, "/%s", relpath);
        }
    }
  else
    {
      len = strlen(dirpath) + strlen(relpath) + 2;
      alloc = (char*)malloc(len);
      if (alloc)
        {
          sprintf(alloc, "%s/%s", dirpath, relpath);
        }
    }

  if (!alloc)
    {
      nsh_output(vtbl, g_fmtcmdoutofmemory, "nsh_getdirpath");
    }
  return alloc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_getwd
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
FAR const char *nsh_getcwd(void)
{
  return nsh_getwd(g_pwd);
}
#endif

/****************************************************************************
 * Name: nsh_getfullpath
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
char *nsh_getfullpath(FAR struct nsh_vtbl_s *vtbl, const char *relpath)
{
  const char *wd;

  /* Handle some special cases */

  if (!relpath || relpath[0] == '\0')
    {
      /* No relative path provided */

      return strdup(g_home);
    }
  else if (relpath[0] == '/')
    {
      return strdup(relpath);
    }

  /* Get the path to the current working directory */
   
  wd = nsh_getcwd();

  /* Fake the '.' directory */

  if (strcmp(relpath, ".") == 0)
    {
      return strdup(wd);
    }

  /* Return the full path */

  return nsh_getdirpath(vtbl, wd, relpath);
}
#endif

/****************************************************************************
 * Name: nsh_freefullpath
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
void nsh_freefullpath(char *relpath)
{
  if (relpath)
    {
      free(relpath);
    }
}
#endif

/****************************************************************************
 * Name: cmd_cd
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
#ifndef CONFIG_NSH_DISABLE_CD
int cmd_cd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  const char *path = argv[1];
  char *alloc = NULL;
  char *fullpath = NULL;
  int ret = OK;

  /* Check for special arguments */

  if (argc < 2 || strcmp(path, "~") == 0)
    {
      path = g_home;
    }
  else if (strcmp(path, "-") == 0)
    {
      alloc = strdup(nsh_getwd(g_oldpwd));
      path  = alloc;
    }
  else if (strcmp(path, "..") == 0)
    {
      alloc = strdup(nsh_getcwd());
      path  = dirname(alloc);
    }
  else
    {
      fullpath = nsh_getfullpath(vtbl, path);
      path     = fullpath;
    }

  /* Set the new workding directory */

  ret = chdir(path);
  if (ret != 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "chdir", NSH_ERRNO);
      ret = ERROR;
    }

  /* Free any memory that was allocated */

  if (alloc)
    {
      free(alloc);
    }

  if (fullpath)
    {
      nsh_freefullpath(fullpath);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_echo
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_ECHO
int cmd_echo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int i;

  /* echo each argument, separated by a space as it must have been on the
   * command line
   */

  for (i = 1; i < argc; i++)
    {
      nsh_output(vtbl, "%s ", argv[i]);
    }
  nsh_output(vtbl, "\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: cmd_pwd
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
#ifndef CONFIG_NSH_DISABLE_PWD
int cmd_pwd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_output(vtbl, "%s\n", nsh_getcwd());
  return OK;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_set
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
#ifndef CONFIG_NSH_DISABLE_SET
int cmd_set(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int ret = setenv(argv[1], argv[2], TRUE);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "setenv", NSH_ERRNO);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_unset
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
#ifndef CONFIG_NSH_DISABLE_UNSET
int cmd_unset(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int ret = unsetenv(argv[1]);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "unsetenv", NSH_ERRNO);
    }
  return ret;
}
#endif
#endif
