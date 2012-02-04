/****************************************************************************
 * examples/telnetd/shell.c
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

#include "ftpd.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct fptd_account_s g_ftpdaccounts[] =
{
  { FTPD_ACCOUNTFLAG_SYSTEM, "root",      "abc123", NULL },
  { FTPD_ACCOUNTFLAG_GUEST,  "ftp",       NULL,     NULL },
  { FTPD_ACCOUNTFLAG_GUEST,  "anonymous", NULL,     NULL },
};
#define NACCOUNTS (sizeof(g_ftpdaccounts) / sizeof(struct fptd_account_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: shell_netinit
 ****************************************************************************/

static void shell_netinit(void)
{
#ifndef CONFIG_EXAMPLES_FTPD_NONETINIT
  struct in_addr addr;
#ifdef CONFIG_EXAMPLE_FTPD_NOMAC
  uint8_t mac[IFHWADDRLEN];
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLE_FTPD_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xb0;
  mac[3] = 0x0b;
  mac[4] = 0xba;
  mac[5] = 0xbe;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_FTPD_IPADDR);
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_FTPD_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_FTPD_NETMASK);
  uip_setnetmask("eth0", &addr);
#endif /* CONFIG_EXAMPLES_FTPD_NONETINIT */
}

/****************************************************************************
 * Name: ftpd_accounts
 ****************************************************************************/

static void ftpd_accounts(FTPD_SESSION handle)
{
  FAR onst struct fptd_account_s *account;
  int i;

  for (i = 0; i < NACCOUNTS; i++)
    {
      account = &g_ftpdaccounts[i];
      ftpd_add_user(handle, account->flags, account->user,
                    account->password, account->home);
    }
}

/****************************************************************************
 * Name: ftpd_daemon
 ****************************************************************************/

int ftpd_daemon(int s_argc, char **s_argv)
{
  FTPD_SESSION handle;
  int ret;

  /* Open FTPD */

  handle = ftpd_open();
  if (!handle)
    {
      ndbg("Failed to open FTPD\n");
      return EXIT_FAILURE;
    }

  /* Configure acounts */

  (void)ftpd_accounts(handle);

  /* Then drive the FTPD server */

  while (g_ftpdglobls.stop == 0)
    {
      (void)ftpd_run(handle, 1000);
    }

  /* Close the FTPD server and exit */

  ftpd_close(handle);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: user_start/ftpd_main
 ****************************************************************************/

int MAIN_NAME(int s_argc, char **s_argv)
{
  FTPD_SESSION handle;
  pid_t pid;
  int ret;

  /* Check if we have already initialized the network */

  if (!g_ftpdglobls.initialized)
    {
  
      /* Bring up the network */

      ret = ftpd_netinit();
      if (ret < 0)
        {
          ndbg("Failed to initialize the network\n");
          return EXIT_FAILURE;
        }

      g_ftpdglobls.initialized = true;
      g_ftpdglobls.stop        = false;
    }

  /* Then start the new daemon */

  g_telnetdcommon.daemon = daemon;
  pid = TASK_CREATE("Telnet daemon", CONFIG_EXAMPLES_FTPD_PRIO,
                    CONFIG_EXAMPLES_FTPD_STACKSIZE,  ftpd_daemon, NULL);
  if (pid < 0)
    {
      ndbg("Failed to start the telnet daemon: %d\n", errno);
      return EXIT_FAILURE;
    }

  printf("The FTP daemon is running, pid=%d\n", pid);
  return EXIT_SUCCESS;
}
