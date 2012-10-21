/****************************************************************************
 * examples/poll/net_listener.c
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <apps/netutils/uiplib.h>

#include "poll_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define IOBUFFER_SIZE 80

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct net_listener_s
{
  struct sockaddr_in addr;
  fd_set          master;
  fd_set          working;
  char            buffer[IOBUFFER_SIZE];
  int             listensd;
  int             mxsd;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_closeclient
 ****************************************************************************/

static bool net_closeclient(struct net_listener_s *nls, int sd)
{
  message("net_listener: Closing host side connection sd=%d\n", sd);
  close(sd);
  FD_CLR(sd, &nls->master);

  /* If we just closed the max SD, then search downward for the next biggest SD. */

  while (FD_ISSET(nls->mxsd, &nls->master) == false)
    {
      nls->mxsd -= 1;
    }
  return true;
}

/****************************************************************************
 * Name: net_incomingdata
 ****************************************************************************/

static inline bool net_incomingdata(struct net_listener_s *nls, int sd)
{
  char *ptr;
  int nbytes;
  int ret;

  /* Read data from the socket */

#ifdef FIONBIO
  for (;;)
#endif
    {
      message("net_listener: Read data from sd=%d\n", sd);
      ret = recv(sd, nls->buffer, IOBUFFER_SIZE, 0);
      if (ret < 0)
        {
          if (errno != EINTR)
            {
              message("net_listener: recv failed sd=%d: %d\n", sd, errno);
              if (errno != EAGAIN)
                {
                  net_closeclient(nls, sd);
                  return false;
                }
            }
        }
      else if (ret == 0)
        {
          message("net_listener: Client connection lost sd=%d\n", sd);
          net_closeclient(nls, sd);
          return false;
        }
      else
        {
          nls->buffer[ret]='\0';
          message("poll_listener: Read '%s' (%d bytes)\n", nls->buffer, ret);

          /* Echo the data back to the client */

          for (nbytes = ret, ptr = nls->buffer; nbytes > 0; )
            {
              ret = send(sd, ptr, nbytes, 0);
              if (ret < 0)
                {
                  if (errno != EINTR)
                    {
                       message("net_listener: Send failed sd=%d: \n", sd, errno);
                       net_closeclient(nls, sd);
                       return false;
                    }
                }
              else
                {
                  nbytes -= ret;
                  ptr    += ret;
                }
            }
        }
    }
  return 0;
}

/****************************************************************************
 * Name: net_connection
 ****************************************************************************/

static inline bool net_connection(struct net_listener_s *nls)
{
  int sd;

  /* Loop until all connections have been processed */

#ifdef FIONBIO
  for (;;)
#endif
    {
      message("net_listener: Accepting new connection on sd=%d\n", nls->listensd);

      sd = accept(nls->listensd, NULL, NULL);
      if (sd < 0)
        {
          message("net_listener: accept failed: %d\n", errno);

          if (errno != EINTR)
            {
              return false;
            }
        }
      else
        {
          /* Add the new connection to the master set */

          message("net_listener: Connection accepted for sd=%d\n", sd);

          FD_SET(sd, &nls->master);
          if (sd > nls->mxsd)
            {
               nls->mxsd = sd;
            }
          return true;
        }
    }
  return false;
}

/****************************************************************************
 * Name: net_mksocket
 ****************************************************************************/

static inline bool net_mksocket(struct net_listener_s *nls)
{
  int value;
  int ret;

  /* Create a listening socket */

  message("net_listener: Initializing listener socket\n");
  nls->listensd = socket(AF_INET, SOCK_STREAM, 0);
  if (nls->listensd < 0)
    {
      message("net_listener: socket failed: %d\n", errno);
      return false;
    }

  /* Configure the socket */

  value = 1;
  ret = setsockopt(nls->listensd, SOL_SOCKET,  SO_REUSEADDR, (char*)&value, sizeof(int));
  if (ret < 0)
    {
      message("net_listener: setsockopt failed: %d\n", errno);
      close(nls->listensd);
      return false;
    }

  /* Set the socket to non-blocking */

#ifdef FIONBIO
  ret = ioctl(nls->listensd, FIONBIO, (char *)&value);
  if (ret < 0)
    {
      message("net_listener: ioctl failed: %d\n", errno);
      close(nls->listensd);
      return false;
    }
#endif

  /* Bind the socket */

  memset(&nls->addr, 0, sizeof(struct sockaddr_in));
  nls->addr.sin_family      = AF_INET;
  nls->addr.sin_addr.s_addr = htonl(INADDR_ANY);
  nls->addr.sin_port        = htons(LISTENER_PORT);
  ret = bind(nls->listensd, (struct sockaddr *)&nls->addr, sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      message("net_listener: bind failed: %d\n", errno);
      close(nls->listensd);
      return false;
    }

  /* Mark the socket as a listener */

  ret = listen(nls->listensd, 32);
  if (ret < 0)
    {
      message("net_listener: bind failed: %d\n", errno);
      close(nls->listensd);
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: net_configure
 ****************************************************************************/

static void net_configure(void)
{
  struct in_addr addr;
#if defined(CONFIG_EXAMPLES_POLL_NOMAC)
  uint8_t mac[IFHWADDRLEN];
#endif

  /* Configure uIP */
  /* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLES_POLL_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_POLL_IPADDR);
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_POLL_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_POLL_NETMASK);
  uip_setnetmask("eth0", &addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_listener
 ****************************************************************************/

void *net_listener(pthread_addr_t pvarg)
{
  struct net_listener_s nls;
  struct timeval timeout;
  int nsds;
  int ret;
  int i;

  /* Configure uIP */

  net_configure();

  /* Set up a listening socket */

  memset(&nls, 0, sizeof(struct net_listener_s));
  if (!net_mksocket(&nls))
    {
       return (void*)1;
    }

  /* Initialize the 'master' file descriptor set */

  FD_ZERO(&nls.master);
  nls.mxsd = nls.listensd;
  FD_SET(nls.listensd, &nls.master);

  /* Set up a 3 second timeout */

  timeout.tv_sec  = NET_LISTENER_DELAY;
  timeout.tv_usec = 0;

  /* Loop waiting for incoming connections or for incoming data
   * on any of the connect sockets.
   */

  for (;;)
    {
      /* Wait on select */

      message("net_listener: Calling select(), listener sd=%d\n", nls.listensd);
      memcpy(&nls.working, &nls.master, sizeof(fd_set));
      ret = select(nls.mxsd + 1, (FAR fd_set*)&nls.working, (FAR fd_set*)NULL, (FAR fd_set*)NULL, &timeout);
      if (ret < 0)
        {
          message("net_listener: select failed: %d\n", errno);
          break;
        }

      /* Check for timeout */

      if (ret == 0)
        {
          message("net_listener: Timeout\n");
          continue;
        }

      /* Find which descriptors caused the wakeup */

      nsds = ret;
      for (i = 0; i <= nls.mxsd && nsds > 0; i++)
        {
          /* Is this descriptor ready? */

          if (FD_ISSET(i, &nls.working))
            {
              /* Yes, is it our listener? */

              message("net_listener: Activity on sd=%d\n", i);

              nsds--;
              if (i == nls.listensd)
                {
                  (void)net_connection(&nls);
                }
              else
                {
                  net_incomingdata(&nls, i);
                }
            }
        }
    }

  /* Cleanup */

#if 0 /* Don't get here */
   for (i = 0; i <= nls.mxsd; +i++)
    {
      if (FD_ISSET(i, &nls.master))
        {
          close(i);
        }
    }
#endif
  return NULL;  /* Keeps some compilers from complaining */
}
