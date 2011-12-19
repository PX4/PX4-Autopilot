/****************************************************************************
 * examples/ftpc/ftpc_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <string.h>

#include <arpa/inet.h>
#include <apps/ftpc.h>

#include "ftpc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FTPC_MAX_ARGUMENTS 4

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cmdmap_s
{
  const char *cmd;        /* Name of the command */
  cmd_t       handler;    /* Function that handles the command */
  uint8_t     minargs;    /* Minimum number of arguments (including command) */
  uint8_t     maxargs;    /* Maximum number of arguments (including command) */
  const char *usage;      /* Usage instructions for 'help' command */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_delim[] = " \t\n";

static int cmd_lhelp(SESSION handle, int argc, char **argv);
static int cmd_lunrecognized(SESSION handle, int argc, char **argv);

static const struct cmdmap_s g_cmdmap[] =
{
  { "cd",       cmd_rchdir,  2, 2, "<directory>" },
  { "chmod",    cmd_rchmod,  3, 3, "<permissions> <path>" },
  { "get",      cmd_rget,    2, 4, "[-a|b] <rname> [<lname>]" },
  { "help",     cmd_lhelp,   1, 2, "" },
  { "idle",     cmd_ridle,   1, 2, "[<idletime>]" },
  { "login",    cmd_rlogin,  2, 3, "<uname> [<password>]" },
  { "ls",       cmd_rls,     1, 2, "[<dirpath>]" },
  { "quit",     cmd_rquit,   1, 1, "" },
  { "mkdir",    cmd_rmkdir,  2, 2, "<directory>" },
  { "noop",     cmd_rnoop,   1, 1, "" },
  { "put",      cmd_rput,    2, 4, "[-a|b] <lname> [<rname>]" },
  { "pwd",      cmd_rpwd,    1, 1, "" },
  { "rename",   cmd_rrename, 3, 3, "<oldname> <newname>" },
  { "rhelp",    cmd_rhelp,   1, 2, "[<command>]" },
  { "rm",       cmd_runlink, 2, 2, "" },
  { "rmdir",    cmd_rrmdir,  2, 2, "<directory>" },
  { "size",     cmd_rsize,   2, 2, "<filepath>" },
  { "time",     cmd_rtime,   2, 2, "<filepath>" },
  { "up",       cmd_rcdup,   1, 1, "" },
  { NULL,       NULL,        1, 1, NULL }
};

static char g_line[CONFIG_FTPC_LINELEN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_lhelp
 ****************************************************************************/

static int cmd_lhelp(SESSION handle, int argc, char **argv)
{
  const struct cmdmap_s *ptr;

  printf("Local FTPC commands:\n");
  for (ptr = g_cmdmap; ptr->cmd; ptr++)
    {
      if (ptr->usage)
        {
          printf("  %s %s\n", ptr->cmd, ptr->usage);
        }
      else
        {
          printf("  %s\n", ptr->cmd);
        }
    }
  return OK;
}

/****************************************************************************
 * Name: cmd_lunrecognized
 ****************************************************************************/

static int cmd_lunrecognized(SESSION handle, int argc, char **argv)
{
  printf("Command %s unrecognized\n", argv[0]);
  return ERROR;
}

/****************************************************************************
 * Name: ftpc_argument
 ****************************************************************************/

char *ftpc_argument(char **saveptr)
{
  char *pbegin = *saveptr;
  char *pend   = NULL;
  const char *term;

  /* Find the beginning of the next token */

  for (;
       *pbegin && strchr(g_delim, *pbegin) != NULL;
       pbegin++);

  /* If we are at the end of the string with nothing
   * but delimiters found, then return NULL.
   */

  if (!*pbegin)
    {
      return NULL;
    }

  /* Does the token begin with '#' -- comment */

  else if (*pbegin == '#')
    {
      /* Return NULL meaning that we are at the end of the line */

      *saveptr = pbegin;
      pbegin   = NULL;
    }
  else
    {
      /* Otherwise, we are going to have to parse to find the end of
       * the token.  Does the token begin with '"'?
       */

      if (*pbegin == '"')
        {
          /* Yes.. then only another '"' can terminate the string */

          pbegin++;
          term = "\"";
        }
      else
        {
          /* No, then any of the usual terminators will terminate the argument */

          term = g_delim;
        }

      /* Find the end of the string */

      for (pend = pbegin + 1;
           *pend && strchr(term, *pend) == NULL;
           pend++);

      /* pend either points to the end of the string or to
       * the first delimiter after the string.
       */

      if (*pend)
        {
          /* Turn the delimiter into a null terminator */

          *pend++ = '\0';
        }

      /* Save the pointer where we left off */

      *saveptr = pend;

    }

  /* Return the beginning of the token. */

  return pbegin;
}

/****************************************************************************
 * Name: ftpc_execute
 ****************************************************************************/

static int ftpc_execute(SESSION handle, int argc, char *argv[])
{
   const struct cmdmap_s *cmdmap;
   const char            *cmd;
   cmd_t                  handler = cmd_lunrecognized;
   int                    ret;

   /* The form of argv is:
    *
    * argv[0]:      The command name.  This is argv[0] when the arguments
    *               are, finally, received by the command handler
    * argv[1]:      The beginning of argument (up to FTPC_MAX_ARGUMENTS)
    * argv[argc]:   NULL terminating pointer
    */

   cmd = argv[0];
   
   /* See if the command is one that we understand */

   for (cmdmap = g_cmdmap; cmdmap->cmd; cmdmap++)
     {
       if (strcmp(cmdmap->cmd, cmd) == 0)
         {
           /* Check if a valid number of arguments was provided.  We
            * do this simple, imperfect checking here so that it does
            * not have to be performed in each command.
            */

           if (argc < cmdmap->minargs)
             {
               /* Fewer than the minimum number were provided */

               printf("Too few arguments for '%s'\n", cmd);
               return ERROR;
             }
           else if (argc > cmdmap->maxargs)
             {
               /* More than the maximum number were provided */

               printf("Too many arguments for '%s'\n", cmd);
               return ERROR;
             }
           else
             {
               /* A valid number of arguments were provided (this does
                * not mean they are right).
                */

               handler = cmdmap->handler;
               break;
             }
         }
     }

   ret = handler(handle, argc, argv);
   if (ret < 0)
     {
       printf("%s failed: %d\n", cmd, errno);
     }
   return ret;
}

/****************************************************************************
 * Name: ftpc_parse
 ****************************************************************************/

int ftpc_parse(SESSION handle, char *cmdline)
{
  FAR char *argv[FTPC_MAX_ARGUMENTS];
  FAR char *saveptr;
  FAR char *cmd;
  int       argc;
  int       ret;

  /* Initialize parser state */

  memset(argv, 0, FTPC_MAX_ARGUMENTS*sizeof(FAR char *));

  /* Parse out the command at the beginning of the line */

  saveptr = cmdline;
  cmd = ftpc_argument(&saveptr);

  /* Check if any command was provided -OR- if command processing is
   * currently disabled.
   */

  if (!cmd)
    {
      /* An empty line is not an error */

      return OK;
    }

  /* Parse all of the arguments following the command name. */

  argv[0] = cmd;
  for (argc = 1; argc < FTPC_MAX_ARGUMENTS; argc++)
    {
      argv[argc] = ftpc_argument(&saveptr);
      if (!argv[argc])
        {
          break;
        }
    }
  argv[argc] = NULL;

  /* Check if the maximum number of arguments was exceeded */

  if (argc > FTPC_MAX_ARGUMENTS)
    {
      printf("Too many arguments\n");
      ret = -EINVAL;
    }
  else
    {
      /* Then execute the command */

      ret = ftpc_execute(handle, argc, argv);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ftpc_main(int argc, char **argv, char **envp)
{
  struct ftpc_connect_s connect = {{0}, 0};
  SESSION handle;
  FAR char *ptr;

  if (argc != 2)
    {
      printf("Usage:\n");
      printf("   %s xx.xx.xx.xx[:pp]\n", argv[0]);
      printf("Where\n");
      printf("  xx.xx.xx.xx is the IP address of the FTP server\n");
      printf("  pp is option port to use with the FTP server\n");
      exit(1);
    }

  /* Check if the argument includes a port number */

  ptr = strchr(argv[1], ':');
  if (ptr)
    {
      *ptr = '\0';
      connect.port = atoi(ptr+1);
    }

  /* In any event, we can now extract the IP address from the comman-line */

  connect.addr.s_addr = inet_addr(argv[1]);

  /* Connect to the FTP server */

  handle = ftpc_connect(&connect);
  if (!handle)
    {
      printf("Failed to connect to the server: %d\n", errno);
      exit(1);
    }

  /* Present a greeting */

  printf("NuttX FTP Client:\n");
  FFLUSH();

  /* Setting optind to -1 is a non-standard, backdoor way to reinitialize
   * getopt().  getopt() is not thread safe and we have no idea what state
   * it is in now!
   */

  optind = -1;

  /* Then enter the command line parsing loop */

  for (;;)
    {
      /* Display the prompt string */

      fputs("nfc> ", stdout);
      FFLUSH();

      /* Get the next line of input */

      if (fgets(g_line, CONFIG_FTPC_LINELEN, stdin))
        {
          /* Parse process the command */

          (void)ftpc_parse(handle, g_line);
          FFLUSH();
        }
    }

  return 0;
}
