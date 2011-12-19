/****************************************************************************
 * netutils/telnetd/telnetd.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This is a leverage of similar logic from uIP:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include "shell.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define SHELL_PROMPT "uIP 1.0> "

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ptentry_s
{
  char *commandstr;
  void (* pfunc)(void *handle, char *str);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void parse(void *handle, register char *str, struct ptentry_s *t);
static void help(void *handle, char *str);
static void unknown(void *handle, char *str);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ptentry_s g_parsetab[] =
{
  {"stats", help},
  {"conn",  help},
  {"help",  help},
  {"exit",  shell_quit},
  {"?",     help},
  {NULL,    unknown}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void parse(void *handle, char *str, struct ptentry_s *t)
{
  struct ptentry_s *p;

  for (p = t; p->commandstr != NULL; ++p)
    {
      if (strncmp(p->commandstr, str, strlen(p->commandstr)) == 0)
        {
          break;
        }
    }

  p->pfunc(handle, str);
}

static void help(void *handle, char *str)
{
  shell_output(handle, "Available commands:");
  shell_output(handle, "stats   - show network statistics");
  shell_output(handle, "conn    - show TCP connections");
  shell_output(handle, "help, ? - show help");
  shell_output(handle, "exit    - exit shell");
}

static void unknown(void *handle, char *str)
{
  if (strlen(str) > 0)
    {
      shell_output(handle, "Unknown command: ", str);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void shell_init(void *handle)
{
}

void shell_start(void *handle)
{
  shell_output(handle, "uIP command shell");
  shell_output(handle, "Type '?' and return for help");
  shell_prompt(handle, SHELL_PROMPT);
}

void shell_input(void *handle, char *cmd)
{
  parse(handle, cmd, g_parsetab);
  shell_prompt(handle, SHELL_PROMPT);
}
