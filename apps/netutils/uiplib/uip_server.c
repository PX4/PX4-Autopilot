/****************************************************************************
 * netutils/uiplib/uip_server.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <apps/netutils/uiplib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_server
 *
 * Description:
 *   Implement basic server logic
 *
 * Parameters:
 *   portno    The port to listen on (in network byte order)
 *   handler   The entrypoint of the task to spawn when a new connection is
 *             accepted.
 *   stacksize The stack size needed by the spawned task
 *
 * Return:
 *   Does not return unless an error occurs.
 *
 ****************************************************************************/

void uip_server(uint16_t portno, pthread_startroutine_t handler, int stacksize)
{
  struct sockaddr_in myaddr;
#ifdef CONFIG_NET_HAVE_SOLINGER
  struct linger ling;
#endif
  pthread_t child;
  pthread_attr_t attr;
  socklen_t addrlen;
  int listensd;
  int acceptsd;
  int ret;

  /* Create a new TCP socket to use to listen for connections */

  listensd = uip_listenon(portno);
  if (listensd < 0)
    {
      return;
    }

  /* Begin serving connections */

  for (;;)
    {
      /* Accept the next connectin */
 
      addrlen = sizeof(struct sockaddr_in);
      acceptsd = accept(listensd, (struct sockaddr*)&myaddr, &addrlen);
      if (acceptsd < 0)
        {
          ndbg("accept failure: %d\n", errno);
          break;
        }

      nvdbg("Connection accepted -- spawning sd=%d\n", acceptsd);

      /* Configure to "linger" until all data is sent when the socket is
       * closed.
       */

#ifdef CONFIG_NET_HAVE_SOLINGER
      ling.l_onoff  = 1;
      ling.l_linger = 30;     /* timeout is seconds */

      ret = setsockopt(acceptsd, SOL_SOCKET, SO_LINGER, &ling, sizeof(struct linger));
      if (ret < 0)
        {
          close(acceptsd);
          ndbg("setsockopt SO_LINGER failure: %d\n", errno);
          break;
        }
#endif

      /* Create a thread to handle the connection.  The socket descriptor is
       * provided in as the single argument to the new thread.
       */

      (void)pthread_attr_init(&attr);
      (void)pthread_attr_setstacksize(&attr, stacksize);

      ret = pthread_create(&child, &attr, handler, (void*)acceptsd);
      if (ret != 0)
        {
          /* Close the connection */

          close(acceptsd);
          ndbg("pthread_create failed\n");

          if (ret == EAGAIN)
            {
              /* Lacked resources to create a new thread. This is a temporary
               * condition, so we close this peer, but keep serving for
               * other connections.
               */

              continue;
            }

          /* Something is very wrong... Break out and stop serving */

          break;
        }

      /* We don't care when/how the child thread exits so detach from it now
       * in order to avoid memory leaks.
       */

      (void)pthread_detach(child);
    }

  /* Close the listerner socket */

  close(listensd);
}
