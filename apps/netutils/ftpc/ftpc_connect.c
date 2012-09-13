/****************************************************************************
 * apps/netutils/ftpc/ftpc_connect.c
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

#include "ftpc_config.h"

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <apps/ftpc.h>

#include "ftpc_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_connect
 *
 * Description:
 *   Create a session handle and connect to the server.
 *
 ****************************************************************************/

SESSION ftpc_connect(FAR struct ftpc_connect_s *server)
{
  FAR struct ftpc_session_s *session;
  int ret;

  /* Allocate a session structure */

  session = (struct ftpc_session_s *)zalloc(sizeof(struct ftpc_session_s));
  if (!session)
    {
      ndbg("Failed to allocate a session\n");
      set_errno(ENOMEM);
      goto errout;
    }

  /* Initialize the session structure with all non-zero and variable values */

  session->addr.s_addr = server->addr.s_addr;
  session->flags      &= ~FTPC_FLAGS_CLEAR;
  session->flags      |= FTPC_FLAGS_SET;
  session->replytimeo  = CONFIG_FTP_DEFTIMEO * CLOCKS_PER_SEC;
  session->conntimeo   = CONFIG_FTP_DEFTIMEO * CLOCKS_PER_SEC;
  session->pid         = getpid();

  /* Use the default port if the user specified port number zero */

  if (!server->port)
    {
      session->port = HTONS(CONFIG_FTP_DEFPORT);
    }
  else
    {
      session->port = htons(server->port);
    }

  /* Get the local home directory, i.e., the value of the PWD environment
   * variable at the time of the connection.  We keep a local copy so that
   * we can change the current working directory without effecting any other
   * logic that may be in same context.
   */

  session->homeldir = strdup(ftpc_lpwd());
/* session->curldir = strdup(sssion->homeldir); */

  /* Create up a timer to prevent hangs */

  session->wdog = wd_create();

  /* And (Re-)connect to the server */

  ret = ftpc_reconnect(session);
  if (ret != OK)
    {
      ndbg("ftpc_reconnect() failed: %d\n", errno);
      goto errout_with_alloc;
    }

  return (SESSION)session;

errout_with_alloc:
  free(session);
errout:
  return NULL;
}

/****************************************************************************
 * Name: ftpc_reconnect
 *
 * Description:
 *   re-connect to the server either initially, or after loss of connection.
 *
 ****************************************************************************/

int ftpc_reconnect(FAR struct ftpc_session_s *session)
{
  struct sockaddr_in addr;
#ifdef CONFIG_DEBUG
  char *tmp;
#endif
  int ret;

  /* Re-initialize the session structure */

  session->replytimeo = CONFIG_FTP_DEFTIMEO * CLOCKS_PER_SEC;
  session->conntimeo  = CONFIG_FTP_DEFTIMEO * CLOCKS_PER_SEC;
  session->xfrmode    = FTPC_XFRMODE_UNKNOWN;

  /* Set up a timer to prevent hangs */

  ret = wd_start(session->wdog, session->conntimeo, ftpc_timeout, 1, session);
  if (ret != OK)
    {
      ndbg("wd_start() failed\n");
      goto errout;
    }

  /* Initialize a socket */

  ret = ftpc_sockinit(&session->cmd);
  if (ret != OK)
    {
      ndbg("ftpc_sockinit() failed: %d\n", errno);
      goto errout;
    }

  /* Connect the socket to the server */

#ifdef CONFIG_DEBUG
  tmp = inet_ntoa(session->addr);
  ndbg("Connecting to server address %s:%d\n", tmp, ntohs(session->port));
#endif

  addr.sin_family      = AF_INET;
  addr.sin_port        = session->port;
  addr.sin_addr.s_addr = session->addr.s_addr;

  ret = ftpc_sockconnect(&session->cmd, &addr);
  if (ret != OK)
    {
      ndbg("ftpc_sockconnect() failed: %d\n", errno);
      goto errout_with_socket;
    }

  /* Read startup message from server */

  fptc_getreply(session);

  /* Check for "120 Service ready in nnn minutes" */

  if (session->code == 120)
    {
      fptc_getreply(session);
    }
  wd_cancel(session->wdog);

  if (!ftpc_sockconnected(&session->cmd))
    {
      ftpc_reset(session);
      goto errout;
    }

  /* Check for "220 Service ready for new user" */

  if (session->code == 220)
    {
      FTPC_SET_CONNECTED(session);
    }

  if (!FTPC_IS_CONNECTED(session))
    {
      goto errout_with_socket;
    }

#ifdef CONFIG_DEBUG
  ndbg("Connected\n");
  tmp = inet_ntoa(addr.sin_addr);
  ndbg("  Remote address: %s:%d\n", tmp, ntohs(addr.sin_port));
  tmp = inet_ntoa(session->cmd.laddr.sin_addr);
  ndbg("  Local address:  %s:%d\n", tmp, ntohs(session->cmd.laddr.sin_port));
#endif
  return OK;

errout_with_socket:
  ftpc_sockclose(&session->cmd);
errout:
  return ERROR;
}
