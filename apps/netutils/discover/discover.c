/****************************************************************************
 * netutils/discover/discover.c
 *
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *
 *   Authors: Max Holtzberg <mh@uvc.de>
 *            Gregory Nutt <gnutt@nuttx.org>
 *
 * This code is derived from the netutils/dhcpd code.
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

#include <debug.h>
#include <errno.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/socket.h>

#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <apps/netutils/discover.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DISCOVER_STACK_SIZE
#  define CONFIG_DISCOVER_STACK_SIZE 1024
#endif

#ifndef CONFIG_DISCOVER_PRIORITY
#  define CONFIG_DISCOVER_PRIORITY SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_DISCOVER_PORT
#  define CONFIG_DISCOVER_PORT 96
#endif

#ifndef CONFIG_DISCOVER_INTERFACE
#  define CONFIG_DISCOVER_INTERFACE "eth0"
#endif

#ifndef CONFIG_DISCOVER_DEVICE_CLASS
#  define CONFIG_DISCOVER_DEVICE_CLASS DISCOVER_ALL
#endif

#ifndef CONFIG_DISCOVER_DESCR
#  define CONFIG_DISCOVER_DESCR CONFIG_ARCH_BOARD
#endif

/* Internal Definitions *****************************************************/
/* Discover request packet format:
 * Byte Description
 * 0    Protocol indentifier (0x99)
 * 1    Request command 0x01
 * 2    Destination device class (For querying subsets of available devices) 
 *      0xff for all devices
 * 3    Checksum (Byte 0 - Byte 1 - Byte n) & 0xff
 */

/* Discover response packet format:
 * Byte Description
 * 0    Protocol indentifier (0x99)
 * 1    Reponse command (0x02)
 * 2-33 Device description string with 0 bytes filled    
 * 34   Checksum (Byte 0 - Byte 1 - Byte n) & 0xff
 */

#define DISCOVER_PROTO_ID 0x99
#define DISCOVER_REQUEST 0x01
#define DISCOVER_RESPONSE 0x02
#define DISCOVER_ALL 0xff
#define DISCOVER_REQUEST_SIZE 4
#define DISCOVER_RESPONSE_SIZE 35

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uint8_t request_t[DISCOVER_REQUEST_SIZE];
typedef uint8_t response_t[DISCOVER_RESPONSE_SIZE];

struct discover_state_s
{
  in_addr_t serverip;
  request_t request;
  response_t response;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct discover_state_s g_state;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int discover_daemon(int argc, char *argv[]);
static inline int discover_socket(void);
static inline int discover_openlistener(void);
static inline int discover_openresponder(void);
static inline int discover_parse(request_t packet);
static inline int discover_respond(in_addr_t *ipaddr);
static inline void discover_initresponse(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void discover_initresponse()
{
  int chk = 0;
  int i;

  g_state.response[0] = DISCOVER_PROTO_ID;
  g_state.response[1] = DISCOVER_RESPONSE;

  strncpy((char*) &g_state.response[2], CONFIG_DISCOVER_DESCR,
          DISCOVER_RESPONSE_SIZE-3);

  for (i = 0; i < DISCOVER_RESPONSE_SIZE-1; i++)
    {
      chk -= g_state.response[i];
    }

  /* Append check sum */

  g_state.response[DISCOVER_RESPONSE_SIZE-1] = chk & 0xff;
}

static int discover_daemon(int argc, char *argv[])
{
  int sockfd = -1;
  int nbytes;
  int addrlen = sizeof(struct sockaddr_in);
  struct sockaddr_in srcaddr; 

  memset(&g_state, 0, sizeof(struct discover_state_s));
  discover_initresponse();

  nvdbg("Started\n");

  for (;;)
    {
      /* Create a socket to listen for requests from DHCP clients */

      if (sockfd < 0)
        {
          sockfd = discover_openlistener();
          if (sockfd < 0)
            {
                ndbg("Failed to create socket\n");
                break;
            }
        }

      /* Read the next packet */
      
      nbytes = recvfrom(sockfd, &g_state.request, sizeof(g_state.request), 0,
                        (struct sockaddr*) &srcaddr,
                        (socklen_t *) &addrlen);
      if (nbytes < 0)
        {
          /* On errors (other EINTR), close the socket and try again */

          ndbg("recv failed: %d\n", errno);
          if (errno != EINTR)
            {
              close(sockfd);
              sockfd = -1;
            }
          continue;
        }
      
      if (discover_parse(g_state.request) != OK)
        {
          continue;
        }
      
      ndbg("Received discover from %08lx'\n", srcaddr.sin_addr.s_addr);
      
      discover_respond(&srcaddr.sin_addr.s_addr);
    }

  return OK;
}

static inline int discover_parse(request_t packet)
{
  int i;
  uint8_t chk = 0;

  if (packet[0] != DISCOVER_PROTO_ID)
    {
      ndbg("Wrong protocol id: %d\n", packet[0]);
      return ERROR;
    }

  if (packet[1] != DISCOVER_REQUEST)
    {
      ndbg("Wrong command: %d\n", packet[1]);
      return ERROR;
    }
     
  if (packet[2] == 0xff || packet[2] == CONFIG_DISCOVER_DEVICE_CLASS)
    {
      for (i = 0; i < DISCOVER_REQUEST_SIZE-1; i++)
        chk -= packet[i];

      if ((chk & 0xff) != packet[3])
        {
          ndbg("Checksum does not match: %d\n", packet[3]);
          return ERROR;
        }
      else
        {
          return OK;
        }
    }
  return ERROR;
}

static inline int discover_respond(in_addr_t *ipaddr)
{
  struct sockaddr_in addr;
  int sockfd;
  int ret;

  sockfd = discover_openresponder();
  if (sockfd >= 0)
    {
      /* Then send the reponse to the DHCP client port at that address */

      memset(&addr, 0, sizeof(struct sockaddr_in));
      addr.sin_family      = AF_INET;
      addr.sin_port        = HTONS(CONFIG_DISCOVER_PORT);
      addr.sin_addr.s_addr = *ipaddr;

      ret = sendto(sockfd, &g_state.response, sizeof(g_state.response), 0,
                   (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
      if (ret < 0)
        {
          ndbg("Could not send discovery response: %d\n", errno);
        }

      close(sockfd);
    }

  return ret;
}

static inline int discover_socket()
{
  int sockfd;
#if defined(HAVE_SO_REUSEADDR) || defined(HAVE_SO_BROADCAST)
  int optval;
  int ret;
#endif

  /* Create a socket to listen for requests from DHCP clients */

  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      ndbg("socket failed: %d\n", errno);
      return ERROR;
    }

  /* Configure the socket */

#ifdef HAVE_SO_REUSEADDR
  optval = 1;
  ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int));
  if (ret < 0)
    {
      ndbg("setsockopt SO_REUSEADDR failed: %d\n", errno);
      close(sockfd);
      return ERROR;
    }
#endif

#ifdef HAVE_SO_BROADCAST
  optval = 1;
  ret = setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (void*)&optval, sizeof(int));
  if (ret < 0)
    {
      ndbg("setsockopt SO_BROADCAST failed: %d\n", errno);
      close(sockfd);
      return ERROR;
    }
#endif

  return sockfd;
}

static inline int discover_openlistener()
{
  struct sockaddr_in addr;
  struct ifreq req;
  int sockfd;
  int ret;

  /* Create a socket to listen for requests from DHCP clients */

  sockfd = discover_socket();
  if (sockfd < 0)
    {
      ndbg("socket failed: %d\n", errno);
      return ERROR;
    }

  /* Get the IP address of the selected device */

  strncpy(req.ifr_name, CONFIG_DISCOVER_INTERFACE, IFNAMSIZ);
  ret = ioctl(sockfd, SIOCGIFADDR, (unsigned long)&req);
  if (ret < 0)
    {
      ndbg("setsockopt SIOCGIFADDR failed: %d\n", errno);
      close(sockfd);
      return ERROR;
    }
  g_state.serverip = ((struct sockaddr_in*)&req.ifr_addr)->sin_addr.s_addr;
  nvdbg("serverip: %08lx\n", ntohl(g_state.serverip));

  /* Bind the socket to a local port. We have to bind to INADDRY_ANY to
   * receive broadcast messages.
   */

  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(CONFIG_DISCOVER_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  ret = bind(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
  if (ret < 0)
   {
     ndbg("bind failed, port=%d addr=%08lx: %d\n",
           addr.sin_port, (long)addr.sin_addr.s_addr, errno);
     close(sockfd);
     return ERROR;
   }

  return sockfd;
}

static inline int discover_openresponder(void)
{
  struct sockaddr_in addr;
  int sockfd;
  int ret;

  /* Create a socket for responding to discovery message */

  sockfd = discover_socket();
  if (sockfd < 0)
    {
      ndbg("socket failed: %d\n", errno);
      return ERROR;
    }

  /* Bind the socket to a local port.*/

  addr.sin_family      = AF_INET;
  addr.sin_port        = 0;
  addr.sin_addr.s_addr = g_state.serverip;

  ret = bind(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
  if (ret < 0)
   {
     ndbg("bind failed, port=%d addr=%08lx: %d\n",
           addr.sin_port, (long)addr.sin_addr.s_addr, errno);
     close(sockfd);
     return ERROR;
   }

  return sockfd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: discover_start
 *
 * Description:
 *   Start the discover daemon.
 *
 * Return:
 *   The process ID (pid) of the new discover daemon is returned on
 *   success; A negated errno is returned if the daemon was not successfully
 *   started.
 *
 ****************************************************************************/

int discover_start()
{
  pid_t pid;

  /* Then start the new daemon */

  pid = TASK_CREATE("Discover daemon", CONFIG_DISCOVER_PRIORITY,
                    CONFIG_DISCOVER_STACK_SIZE, discover_daemon, NULL);
  if (pid < 0)
    {
      int errval = errno;
      ndbg("Failed to start the discover daemon: %d\n", errval);
      return -errval;
    }

  /* Return success */

  return pid;
}
