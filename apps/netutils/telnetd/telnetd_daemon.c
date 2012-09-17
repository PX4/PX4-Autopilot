/****************************************************************************
 * netutils/telnetd/telnetd_daemon.c
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include <sys/socket.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <semaphore.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>
#include <netinet/in.h>

#include <apps/netutils/telnetd.h>
#include <apps/netutils/uiplib.h>

#include "telnetd.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This structure is used to passed information to telnet daemon when it 
 * started.
 */

struct telnetd_common_s g_telnetdcommon;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnetd_daemon
 *
 * Description:
 *   This function is the Telnet daemon.  It does not return (unless an
 *   error occurs).
 *
 * Parameters:
 *   Standard task start up arguments.
 *
 * Return:
 *   Does not return unless an error occurs.
 *
 ****************************************************************************/

static int telnetd_daemon(int argc, char *argv[])
{
  FAR struct telnetd_s *daemon;
  struct sockaddr_in myaddr;
#ifdef CONFIG_NET_HAVE_SOLINGER
  struct linger ling;
#endif
  socklen_t addrlen;
  FAR char *devpath;
  pid_t pid;
  int listensd;
  int acceptsd;
  int drvrfd;
#ifdef CONFIG_NET_HAVE_REUSEADDR
  int optval;
#endif

  /* Get daemon startup info */

  daemon = g_telnetdcommon.daemon;
  g_telnetdcommon.daemon = NULL;
  sem_post(&g_telnetdcommon.startsem);
  DEBUGASSERT(daemon != NULL);

  /* Create a new TCP socket to use to listen for connections */

  listensd = socket(PF_INET, SOCK_STREAM, 0);
  if (listensd < 0)
    {
      int errval = errno;
      ndbg("socket failure: %d\n", errval);
      return -errval;
    }

  /* Set socket to reuse address */

#ifdef CONFIG_NET_HAVE_REUSEADDR
  optval = 1;
  if (setsockopt(listensd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int)) < 0)
    {
      ndbg("setsockopt SO_REUSEADDR failure: %d\n", errno);
      goto errout_with_socket;
    }
#endif

  /* Bind the socket to a local address */

  myaddr.sin_family      = AF_INET;
  myaddr.sin_port        = daemon->port;
  myaddr.sin_addr.s_addr = INADDR_ANY;

  if (bind(listensd, (struct sockaddr*)&myaddr, sizeof(struct sockaddr_in)) < 0)
    {
      ndbg("bind failure: %d\n", errno);
      goto errout_with_socket;
    }

  /* Listen for connections on the bound TCP socket */

  if (listen(listensd, 5) < 0)
    {
      ndbg("listen failure %d\n", errno);
      goto errout_with_socket;
    }

  /* Now go silent.  Only the lldbg family of debug functions should
   * be used after this point because these do not depend on stdout
   * being available.
   */

#ifndef CONFIG_DEBUG
  close(0);
  close(1);
  close(2);
#endif

  /* Begin accepting connections */

  for (;;)
    {
      nllvdbg("Accepting connections on port %d\n", ntohs(daemon->port));

      addrlen = sizeof(struct sockaddr_in);
      acceptsd = accept(listensd, (struct sockaddr*)&myaddr, &addrlen);
      if (acceptsd < 0)
        {
          nlldbg("accept failed: %d\n", errno);
          goto errout_with_socket;
        }

      /* Configure to "linger" until all data is sent when the socket is closed */

#ifdef CONFIG_NET_HAVE_SOLINGER
      ling.l_onoff  = 1;
      ling.l_linger = 30;     /* timeout is seconds */
      if (setsockopt(acceptsd, SOL_SOCKET, SO_LINGER, &ling, sizeof(struct linger)) < 0)
        {
          nlldbg("setsockopt failed: %d\n", errno);
          goto errout_with_acceptsd;
        }
#endif

      /* Create a character device to "wrap" the accepted socket descriptor */

      nllvdbg("Creating the telnet driver\n");
      devpath = telnetd_driver(acceptsd, daemon);
      if (devpath < 0)
        {
          nlldbg("telnetd_driver failed\n");
          goto errout_with_acceptsd;
        }

      /* Open the driver */

      nllvdbg("Opening the telnet driver\n");
      drvrfd = open(devpath, O_RDWR);
      if (drvrfd < 0)
        {
          nlldbg("Failed to open %s: %d\n", devpath, errno);
          goto errout_with_acceptsd;
        }

      /* We can now free the driver string */

      free(devpath);

      /* Use this driver as stdin, stdout, and stderror */

      (void)dup2(drvrfd, 0);
      (void)dup2(drvrfd, 1);
      (void)dup2(drvrfd, 2);

      /* And we can close our original driver fd */

      if (drvrfd > 2)
        {
          close(drvrfd);
        }

      /* Create a task to handle the connection.  The created task
       * will inherit the new stdin, stdout, and stderr.
       */

      nllvdbg("Starting the telnet session\n");
      pid = TASK_CREATE("Telnet session", daemon->priority, daemon->stacksize,
                         daemon->entry, NULL);
      if (pid < 0)
        {
          nlldbg("Failed start the telnet session: %d\n", errno);
          goto errout_with_acceptsd;
        }

      /* Forget about the connection. */

      close(0);
      close(1);
      close(2);
    }

errout_with_acceptsd:
  close(acceptsd);

errout_with_socket:
  close(listensd);
  free(daemon);
  return 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnetd_start
 *
 * Description:
 *   Start the telnet daemon.
 *
 * Parameters:
 *   config    A pointer to a configuration structure that characterizes the
 *             telnet daemon.  This configuration structure may be defined
 *             on the caller's stack because it is not retained by the
 *             daemon.
 *
 * Return:
 *   The process ID (pid) of the new telnet daemon is returned on
 *   success; A negated errno is returned if the daemon was not successfully
 *   started.
 *
 ****************************************************************************/

int telnetd_start(FAR struct telnetd_config_s *config)
{
  FAR struct telnetd_s *daemon;
  pid_t pid;
  int ret;

  /* Allocate a state structure for the new daemon */

  daemon = (FAR struct telnetd_s *)malloc(sizeof(struct telnetd_s));
  if (!daemon)
    {
      return -ENOMEM;
    }

  /* Initialize the daemon structure */

  daemon->port      = config->d_port;
  daemon->priority  = config->t_priority;
  daemon->stacksize = config->t_stacksize;
  daemon->entry     = config->t_entry;

  /* Initialize the common structure if this is the first daemon */

  if (g_telnetdcommon.ndaemons < 1)
    {
      sem_init(&g_telnetdcommon.startsem, 0, 0);
      sem_init(&g_telnetdcommon.exclsem, 0, 1);
      g_telnetdcommon.minor = 0;
    }

  /* Then start the new daemon */

  g_telnetdcommon.daemon = daemon;
  pid = TASK_CREATE("Telnet daemon", config->d_priority, config->d_stacksize,
                    telnetd_daemon, NULL);
  if (pid < 0)
    {
      int errval = errno;
      free(daemon);
      ndbg("Failed to start the telnet daemon: %d\n", errval);
      return -errval;
    }

  /* Then wait for the daemon to start and complete the handshake */

  do
    {
      ret = sem_wait(&g_telnetdcommon.startsem);

      /* The only expected error condition is for sem_wait to be awakened by
       * a receipt of a signal.
       */

      if (ret < 0)
        {
          DEBUGASSERT(errno == -EINTR);
        }
    }
  while (ret < 0);

  /* Return success */

  return pid;
}
