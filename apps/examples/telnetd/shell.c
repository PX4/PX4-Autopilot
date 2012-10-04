/****************************************************************************
 * examples/telnetd/shell.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <net/if.h>

#include <apps/netutils/telnetd.h>
#include <apps/netutils/uiplib.h>

#include "shell.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ptentry_s
{
  FAR const char *commandstr;
  void (*pfunc)(int argc, char **argv);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void shell_help(int argc, char **argv);
static void shell_quit(int argc, char **argv);
static void shell_unknown(int argc, char **argv);
static void shell_parse(FAR char *line, int len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ptentry_s g_parsetab[] =
{
  {"help",  shell_help},
  {"exit",  shell_quit},
  {"?",     shell_help},
  {NULL,    shell_unknown}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  shell_help
 ****************************************************************************/

static void shell_help(int argc, char **argv)
{
  printf("Available commands:\n");
  printf("  help, ? - show help\n");
  printf("  exit    - exit shell\n");
}

/****************************************************************************
 * Name:  shell_help
 ****************************************************************************/

static void shell_unknown(int argc, char **argv)
{
  if (argv[0])
    {
      printf("Unknown command: %s\n", argv[0]);
    }
}

/****************************************************************************
 * Name: shell_quit
 ****************************************************************************/

static void shell_quit(int argc, char **argv)
{
  printf("Bye!\n");
  exit(0);
}

/****************************************************************************
 * Name: shell_parse
 ****************************************************************************/

static void shell_parse(FAR char *line, int len)
{
  struct ptentry_s *entry;
  FAR char *cmd;
  FAR char *saveptr;

  /* Get the command from the beginning the line */

  cmd = strtok_r(line, " \t\n\r\f\v", &saveptr);
  if (cmd)
    {
      /* Now find the matching command in the command table */

      for (entry = g_parsetab; entry->commandstr != NULL; entry++)
        {
          if (strncmp(entry->commandstr, cmd, strlen(entry->commandstr)) == 0)
            {
             break;
            }
        }

      entry->pfunc(1, &cmd);
    }
}

/****************************************************************************
 * Name: shell_session
 ****************************************************************************/

int shell_session(int argc, char *argv[])
{
  char line[128];

  printf("uIP command shell -- NuttX style\n");
  printf("Type '?' and return for help\n");

  for(;;)
    {
      printf(SHELL_PROMPT);
      fflush(stdout);

      if (fgets(line, 128, stdin) == NULL)
        {
          break;
        }

      shell_parse(line, 128);
    }

  return 0;
}

/****************************************************************************
 * Name: shell_netinit
 ****************************************************************************/

static void shell_netinit(void)
{
  struct in_addr addr;
#ifdef CONFIG_EXAMPLES_TELNETD_NOMAC
  uint8_t mac[IFHWADDRLEN];
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLES_TELNETD_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_TELNETD_IPADDR);
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_TELNETD_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_TELNETD_NETMASK);
  uip_setnetmask("eth0", &addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int shell_main(int argc, char *argv[])
{
  struct telnetd_config_s config;
  int ret;

  /* Configure the network */

  printf("shell_main: Initializing the network\n");
  shell_netinit();

  /* Configure the telnet daemon */

  config.d_port      = HTONS(23);
  config.d_priority  = CONFIG_EXAMPLES_TELNETD_DAEMONPRIO;
  config.d_stacksize = CONFIG_EXAMPLES_TELNETD_DAEMONSTACKSIZE;
  config.t_priority  = CONFIG_EXAMPLES_TELNETD_CLIENTPRIO;
  config.t_stacksize = CONFIG_EXAMPLES_TELNETD_CLIENTSTACKSIZE;
  config.t_entry     = shell_session;

  /* Start the telnet daemon */

  printf("shell_main: Starting the Telnet daemon\n");
  ret = telnetd_start(&config);
  if (ret < 0)
    {
      printf("Failed to tart the Telnet daemon\n");
    }

  printf("shell_main: Exiting\n");
  return 0;
}
