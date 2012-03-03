/****************************************************************************
 * netuils/tftp/tftpc_packets.c
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
 * OF USE, TFTP_DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>
#include <apps/netutils/tftp.h>

#include "tftpc_internal.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tftp_mode
 ****************************************************************************/

static inline const char *tftp_mode(bool binary)
{
  return binary ? "octet" : "netascii";
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tftp_sockinit
 *
 * Description:
 *   Common initialization logic:  Create the socket and initialize the
 *   server address structure.
 *
 ****************************************************************************/

int tftp_sockinit(struct sockaddr_in *server, in_addr_t addr)
{
  struct timeval timeo;
  int sd;
  int ret;

  /* Create the UDP socket */

  sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sd < 0)
    {
      ndbg("socket failed: %d\n", errno);
      return ERROR;
    }

  /* Set the recvfrom timeout */

  timeo.tv_sec  = CONFIG_NETUTILS_TFTP_TIMEOUT / 10;
  timeo.tv_usec = (CONFIG_NETUTILS_TFTP_TIMEOUT % 10) * 100000;
  ret = setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &timeo, sizeof(struct timeval));
  if (ret < 0)
    {
      ndbg("setsockopt failed: %d\n", errno);
    }

  /* Initialize the server address structure */

  memset(server, 0, sizeof(struct sockaddr_in));
  server->sin_family      = AF_INET;
  server->sin_addr.s_addr = addr;
  server->sin_port        = HTONS(CONFIG_NETUTILS_TFTP_PORT);
  return sd;
}

/****************************************************************************
 * Name: tftp_mkreqpacket
 *
 * Description:
 *   RRQ or WRQ message format:
 *
 *     2 bytes: Opcode (network order == big-endian)
 *     N bytes: Filename
 *     1 byte:  0
 *     N bytes: mode
 *     1 byte:  0
 *
 * Return
 *  Then number of bytes in the request packet (never fails)
 *
 ****************************************************************************/

int tftp_mkreqpacket(uint8_t *buffer, int opcode, const char *path, bool binary)
{
  buffer[0] = opcode >> 8;
  buffer[1] = opcode & 0xff;
  return sprintf((char*)&buffer[2], "%s%c%s", path, 0, tftp_mode(binary)) + 3;
}

/****************************************************************************
 * Name: tftp_mkackpacket
 *
 * Description:
 *   ACK message format:
 *
 *     2 bytes: Opcode (network order == big-endian)
 *     2 bytes: Block number (network order == big-endian)
 *
 ****************************************************************************/

int tftp_mkackpacket(uint8_t *buffer, uint16_t blockno)
{
  buffer[0] = TFTP_ACK >> 8;
  buffer[1] = TFTP_ACK & 0xff;
  buffer[2] = blockno >> 8;
  buffer[3] = blockno & 0xff;
  return 4;
}

/****************************************************************************
 * Name: tftp_mkerrpacket
 *
 * Description:
 *   ERROR message format:
 *
 *     2 bytes: Opcode (network order == big-endian)
 *     2 bytes: Error number (network order == big-endian)
 *     N bytes: Error string
 *     1 byte:  0
 *
 ****************************************************************************/

int tftp_mkerrpacket(uint8_t *buffer, uint16_t errorcode, const char *errormsg)
{
  buffer[0] = TFTP_ERR >> 8;
  buffer[1] = TFTP_ERR & 0xff;
  buffer[2] = errorcode >> 8;
  buffer[3] = errorcode & 0xff;
  strcpy((char*)&buffer[4], errormsg);
  return strlen(errormsg) + 5;
}

/****************************************************************************
 * Name: tftp_parseerrpacket
 *
 * Description:
 *   ERROR message format:
 *
 *     2 bytes: Opcode (network order == big-endian)
 *     2 bytes: Error number (network order == big-endian)
 *     N bytes: Error string
 *     1 byte:  0
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
int tftp_parseerrpacket(const uint8_t *buffer)
{
  uint16_t opcode       = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
  uint16_t errcode      = (uint16_t)buffer[2] << 8 | (uint16_t)buffer[3];
  const char *errmsg  = (const char *)&buffer[4];

  if (opcode == TFTP_ERR)
    {
      ndbg("ERR message: %s (%d)\n", errmsg, errcode);
      return OK;
    }
  return ERROR;
}
#endif

/****************************************************************************
 * Name: tftp_recvfrom
 *
 * Description:
 *   recvfrom helper
 *
 ****************************************************************************/

ssize_t tftp_recvfrom(int sd, void *buf, size_t len, struct sockaddr_in *from)
{
  int     addrlen;
  ssize_t nbytes;

  /* Loop handles the case where the recvfrom is interrupted by a signal and
   * we should unconditionally try again.
   */

  for (;;)
    {
      /* For debugging, it is helpful to start with a clean buffer */

#if defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_NET)
      memset(buf, 0, len);
#endif

      /* Receive the packet */

      addrlen = sizeof(struct sockaddr_in);
      nbytes = recvfrom(sd, buf, len, 0, (struct sockaddr*)from, (socklen_t*)&addrlen);

      /* Check for errors */

      if (nbytes < 0)
        {
          /* Check for a timeout */

          if (errno == EAGAIN)
            {
              ndbg("recvfrom timed out\n");
              return ERROR;
            }

          /* If EINTR, then loop and try again.  Other errors are fatal */

          else if (errno != EINTR)
            {
              ndbg("recvfrom failed: %d\n", errno);
              return ERROR;
            }
        }

      /* No errors?  Return the number of bytes received */

      else
        {
          return nbytes;
        }
    }
}

/****************************************************************************
 * Name: tftp_sendto
 *
 * Description:
 *   sendto helper
 *
 ****************************************************************************/

ssize_t tftp_sendto(int sd, const void *buf, size_t len, struct sockaddr_in *to)
{
  ssize_t nbytes;

  /* Loop handles the case where the sendto is interrupted by a signal and
   * we should unconditionally try again.
   */

  for (;;)
    {
      /* Send the packet */

      nbytes = sendto(sd, buf, len, 0, (struct sockaddr*)to, sizeof(struct sockaddr_in));

      /* Check for errors */

      if (nbytes < 0)
        {
          /* If EINTR, then loop and try again.  Other errors are fatal */

          if (errno != EINTR)
            {
              ndbg("sendto failed: %d\n", errno);
              return ERROR;
            }
        }

      /* No errors?  Return the number of bytes received */

      else
        {
          return nbytes;
        }
    }
}

#endif /* CONFIG_NET && CONFIG_NET_UDP && CONFIG_NFILE_DESCRIPTORS */
