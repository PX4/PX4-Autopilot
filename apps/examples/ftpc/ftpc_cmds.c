/****************************************************************************
 * examples/ftpc/ftpc_cmds.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <apps/ftpc.h>

#include "ftpc.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: cmd_rlogin
 ****************************************************************************/

int cmd_rlogin(SESSION handle, int argc, char **argv)
{
  struct ftpc_login_s login = {NULL, NULL, NULL, true};

  login.uname = argv[1];
  if (argc > 2)
    {
      login.pwd = argv[2];
    }

  return ftpc_login(handle, &login);
}

/****************************************************************************
 * Name: cmd_rquit
 ****************************************************************************/

int cmd_rquit(SESSION handle, int argc, char **argv)
{
  int ret = ftpc_quit(handle);
  if (ret < 0)
    {
      printf("quit failed: %d\n", errno);
    }
  printf("Exitting...\n");
  exit(0);
  return ERROR;
}

/****************************************************************************
 * Name: cmd_rchdir
 ****************************************************************************/

int cmd_rchdir(SESSION handle, int argc, char **argv)
{
  return ftpc_chdir(handle, argv[1]);
}

/****************************************************************************
 * Name: cmd_rpwd
 ****************************************************************************/

int cmd_rpwd(SESSION handle, int argc, char **argv)
{
  FAR char *pwd = ftpc_rpwd(handle);
  if (pwd)
    {
      printf("PWD: %s\n", pwd);
      free(pwd);
      return OK;
    }
  return ERROR;
}

/****************************************************************************
 * Name: cmd_rcdup
 ****************************************************************************/

int cmd_rcdup(SESSION handle, int argc, char **argv)
{
  return ftpc_cdup(handle);
}

/****************************************************************************
 * Name: cmd_rmkdir
 ****************************************************************************/

int cmd_rmkdir(SESSION handle, int argc, char **argv)
{
  return ftpc_mkdir(handle, argv[1]);
}

/****************************************************************************
 * Name: cmd_rrmdir
 ****************************************************************************/

int cmd_rrmdir(SESSION handle, int argc, char **argv)
{
  return ftpc_rmdir(handle, argv[1]);
}

/****************************************************************************
 * Name: cmd_runlink
 ****************************************************************************/

int cmd_runlink(SESSION handle, int argc, char **argv)
{
  return ftpc_unlink(handle, argv[1]);
}

/****************************************************************************
 * Name: cmd_rchmod
 ****************************************************************************/

int cmd_rchmod(SESSION handle, int argc, char **argv)
{
  return ftpc_chmod(handle, argv[1], argv[2]);
}

/****************************************************************************
 * Name: cmd_rrename
 ****************************************************************************/

int cmd_rrename(SESSION handle, int argc, char **argv)
{
  return ftpc_rename(handle, argv[1], argv[2]);
}

/****************************************************************************
 * Name: cmd_rsize
 ****************************************************************************/

int cmd_rsize(SESSION handle, int argc, char **argv)
{
  off_t size = ftpc_filesize(handle, argv[1]);
  printf("SIZE: %lu\n", size);
  return OK;
}

/****************************************************************************
 * Name: cmd_rtime
 ****************************************************************************/

int cmd_rtime(SESSION handle, int argc, char **argv)
{
  time_t filetime = ftpc_filetime(handle, argv[1]);
  printf("TIME: %lu\n", (long)filetime);
  return OK;
}

/****************************************************************************
 * Name: cmd_ridle
 ****************************************************************************/

int cmd_ridle(SESSION handle, int argc, char **argv)
{
  unsigned int idletime = 0;

  if (argc > 1)
    {
      idletime = atoi(argv[1]);
    }

  return ftpc_idle(handle, idletime);
}

/****************************************************************************
 * Name: cmd_rnoop
 ****************************************************************************/

int cmd_rnoop(SESSION handle, int argc, char **argv)
{
  return ftpc_noop(handle);
}

/****************************************************************************
 * Name: cmd_rhelp
 ****************************************************************************/

int cmd_rhelp(SESSION handle, int argc, char **argv)
{
  FAR const char *cmd = NULL;
  int ret;

  if (argc > 1)
    {
      cmd = argv[1];
    }

  ret = ftpc_help(handle, cmd);
  if (ret == OK)
    {
      FAR char *msg = ftpc_response(handle);
      puts(msg);
      free(msg);
    }

 return ret;
}

/****************************************************************************
 * Name: cmd_rls
 ****************************************************************************/

int cmd_rls(SESSION handle, int argc, char **argv)
{
  FAR struct ftpc_dirlist_s *dirlist;
  FAR char *dirname = NULL;
  int i;

  /* Get the directory listing */

  if (argc > 1)
    {
      dirname = argv[1];
    }

  dirlist = ftpc_listdir(handle, dirname);
  if (!dirlist)
    {
      return ERROR;
    }

  /* Print the directory listing */

  printf("%s/\n", dirname ? dirname : ".");
  for (i = 0; i < dirlist->nnames; i++)
    {
      printf("  %s\n", dirlist->name[i]);
    }
  FFLUSH();

  /* We are responsible for freeing the directory structure allocated by
   * ftpc_listdir().
   */

  ftpc_dirfree(dirlist);
  return OK;
}

/****************************************************************************
 * Name: cmd_rget
 ****************************************************************************/

int cmd_rget(SESSION handle, int argc, char **argv)
{
  FAR const char *rname;
  FAR const char *lname = NULL;
  int xfrmode = FTPC_XFRMODE_ASCII;
  int option;

  while ((option = getopt(argc, argv, "ab")) != ERROR)
    {
      if (option == 'a')
        {
          xfrmode = FTPC_XFRMODE_ASCII;
        }
      else if (option == 'b')
        {
          xfrmode = FTPC_XFRMODE_BINARY;
        }
      else
        {
          printf("%s: Unrecognized option: '%c'\n", "put", option);
          return ERROR;
        }
    }

  /* There should be one or two parameters remaining on the command line */

  if (optind >= argc)
    {
      printf("%s: Missing required arguments\n", "get");
      return ERROR;
    }

  rname = argv[optind];
  optind++;

  if (optind < argc)
    {
      lname = argv[optind];
      optind++;
    }

  if (optind != argc)
    {
      printf("%s: Too many arguments\n", "get");
      return ERROR;
    }

  /* Perform the transfer */

  return ftpc_getfile(handle, rname, lname, FTPC_GET_NORMAL, xfrmode);
}

/****************************************************************************
 * Name: cmd_rput
 ****************************************************************************/

int cmd_rput(SESSION handle, int argc, char **argv)
{
  FAR const char *lname;
  FAR const char *rname = NULL;
  int xfrmode = FTPC_XFRMODE_ASCII;
  int option;

  while ((option = getopt(argc, argv, "ab")) != ERROR)
    {
      if (option == 'a')
        {
          xfrmode = FTPC_XFRMODE_ASCII;
        }
      else if (option == 'b')
        {
          xfrmode = FTPC_XFRMODE_BINARY;
        }
      else
        {
          printf("%s: Unrecognized option: '%c'\n", "put", option);
          return ERROR;
        }
    }

  /* There should be one or two parameters remaining on the command line */

  if (optind >= argc)
    {
      printf("%s: Missing required arguments\n", "get");
      return ERROR;
    }

  lname = argv[optind];
  optind++;

  if (optind < argc)
    {
      rname = argv[optind];
      optind++;
    }

  if (optind != argc)
    {
      printf("%s: Too many arguments\n ");
      return ERROR;
    }

  /* Perform the transfer */

  return ftp_putfile(handle, lname, rname, FTPC_PUT_NORMAL, xfrmode);
}
