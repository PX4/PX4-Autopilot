/****************************************************************************
 * netuils/tftp/tftpc_put.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <sys/stat.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>
#include <apps/netutils/tftp.h>

#include "tftpc_internal.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TFTP_RETRIES 3

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tftp_read
 ****************************************************************************/

static inline ssize_t tftp_read(int fd, uint8_t *buf, size_t buflen)
{
  ssize_t nbytesread;
  ssize_t totalread = 0;

  while (totalread < buflen)
    {
      /* Read the data... repeating the read in the event that it was
       * interrupted by a signal.
       */

      do
        {
          nbytesread = read(fd, buf, buflen - totalread);
        }
      while (nbytesread < 0 && errno == EINTR);

      /* Check for non-EINTR errors */

      if (nbytesread < 0)
        {
          ndbg("read failed: %d\n", errno);
          return ERROR;
        }

      /* Check for end of file */

      else if (nbytesread == 0)
        {
          break;
        }

      /* Handle partial reads.  Partial reads can happen normally
       * when the source is some device driver that returns data
       * in bits and pieces as received (such as a pipe)
       */

      totalread += nbytesread;
      buf       += nbytesread;
    }
  return totalread;
}

/****************************************************************************
 * Name: tftp_mkdatapacket
 *
 * Description:
 *   DATA message format:
 *
 *     2 bytes: Opcode (network order == big-endian)
 *     2 bytes: Block number (network order == big-endian)
 *     N bytes: Data (where N <= 512)
 *
 * Input Parameters:
 *   fd      - File descriptor used to read from the file
 *   offset  - File offset to read from
 *   packet  - Buffer to write the data packet into
 *   blockno - The block number of the packet
 *
 * Return Value:
 *   Number of bytes read into the packet. <TFTP_PACKETSIZE means end of file;
 *   <1 if an error occurs.
 *
 ****************************************************************************/

int tftp_mkdatapacket(int fd, off_t offset, uint8_t *packet, uint16_t blockno)
{
  off_t tmp;
  int nbytesread;

  /* Format the DATA message header */

  packet[0] = TFTP_DATA >> 8;
  packet[1] = TFTP_DATA & 0xff;
  packet[2] = blockno >> 8;
  packet[3] = blockno & 0xff;

  /* Seek to the correct offset in the file */

  tmp = lseek(fd, offset, SEEK_SET);
  if (tmp == (off_t)-1)
    {
      ndbg("lseek failed: %d\n", errno);
      return ERROR;
    }

  /* Read the file data into the packet buffer */

  nbytesread = tftp_read(fd, &packet[TFTP_DATAHEADERSIZE], TFTP_DATASIZE);
  if (nbytesread < 0)
    {
      return ERROR;
    }
  return nbytesread + TFTP_DATAHEADERSIZE;
}

/****************************************************************************
 * Name: tftp_rcvack
 *
 * Description:
 *   ACK message format:
 *
 *     2 bytes: Opcode (network order == big-endian)
 *     2 bytes: Block number (network order == big-endian)
 *
 * Input Parameters:
 *   sd      - Socket descriptor to use in in the transfer
 *   packet   - buffer to use for the tranfers
 *   server  - The address of the server
 *   port    - The port number of the server (0 if not yet known)
 *   blockno - Location to return block number in the received ACK
 *
 * Returned Value:
 *   OK:success and blockno valid, ERROR:failure.
 *
 ****************************************************************************/

static int tftp_rcvack(int sd, uint8_t *packet, struct sockaddr_in *server,
                       uint16_t *port, uint16_t *blockno)
{
  struct sockaddr_in from;     /* The address the last UDP message recv'd from */
  ssize_t nbytes;              /* The number of bytes received. */
  uint16_t opcode;             /* The received opcode */
  uint16_t rblockno;           /* The received block number */
  int packetlen;               /* Packet length */
  int retry;                   /* Retry counter */

  /* Try up to TFTP_RETRIES times */

  for (retry = 0; retry < TFTP_RETRIES; retry++)
    {
      /* Try for until a valid ACK is received or some error occurs */

      for (;;)
        {
          /* Receive the next UDP packet from the server */

          nbytes = tftp_recvfrom(sd, packet, TFTP_IOBUFSIZE, &from);
          if (nbytes < TFTP_ACKHEADERSIZE)
            {
              /* Failed to receive a good packet */

              if (nbytes == 0)
                {
                  ndbg("Connection lost: %d bytes\n", nbytes);
                }
              else if (nbytes > 0)
                {
                  ndbg("Short packet: %d bytes\n", nbytes);
                }
              else
                {
                  ndbg("Recveid failure\n");
                }

              /* Break out to bump up the retry count */

              break;
            }
          else
            {
               /* Get the port being used by the server if that has not yet been established */

               if (!*port)
                 {
                   *port            = from.sin_port;
                   server->sin_port = from.sin_port;
                 }

               /* Verify that the packet was received from the correct host and port */

               if (server->sin_addr.s_addr != from.sin_addr.s_addr)
                 {
                   nvdbg("Invalid address in DATA\n");
                   continue;
                 }

              if (*port != server->sin_port)
                {
                  nvdbg("Invalid port in DATA\n");
                  packetlen = tftp_mkerrpacket(packet, TFTP_ERR_UNKID, TFTP_ERRST_UNKID);
                  (void)tftp_sendto(sd, packet, packetlen, server);
                  continue;
                }

              /* Parse the error message */

               opcode   = (uint16_t)packet[0] << 8 | (uint16_t)packet[1];
               rblockno = (uint16_t)packet[2] << 8 | (uint16_t)packet[3];

              /* Verify that the message that we received is an ACK for the
               * expected block number.
               */

               if (opcode != TFTP_ACK)
                 {
                   nvdbg("Bad opcode\n");
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
                  if (opcode == TFTP_ERR)
                    {
                      (void)tftp_parseerrpacket(packet);
                    }
                  else
#endif
                  if (opcode > TFTP_MAXRFC1350)
                    {
                      packetlen = tftp_mkerrpacket(packet, TFTP_ERR_ILLEGALOP, TFTP_ERRST_ILLEGALOP);
                      (void)tftp_sendto(sd, packet, packetlen, server);
                    }

                  /* Break out an bump up the retry count */

                  break;
                }

              /* Success! */

              nvdbg("Received ACK for block %d\n", rblockno);
              *blockno = rblockno;
              return OK;
            }
        }
    }

  /* We have tried TFTP_RETRIES times */

  ndbg("Timeout, Waiting for ACK\n");
  return ERROR; /* Will never get here */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tftpput
 *
 * Input Parameters:
 *   local  - Path to the file system object to be sent.
 *   remote - The name of the file on the TFTP server.
 *   addr   - The IP address of the server in network order
 *   binary - TRUE:  Perform binary ('octect') transfer
 *            FALSE: Perform text ('netascii') transfer
 *
 ****************************************************************************/

int tftpput(const char *local, const char *remote, in_addr_t addr, bool binary)
{
  struct sockaddr_in server;         /* The address of the TFTP server */
  uint8_t *packet;                   /* Allocated memory to hold one packet */
  off_t offset;                      /* Offset into source file */
  uint16_t blockno;                  /* The current transfer block number */
  uint16_t rblockno;                 /* The ACK'ed block number */
  uint16_t port = 0;                 /* This is the port number for the transfer */
  int packetlen;                     /* The length of the data packet */
  int sd;                            /* Socket descriptor for socket I/O */
  int fd;                            /* File descriptor for file I/O */
  int retry;                         /* Retry counter */
  int result = ERROR;                /* Assume failure */
  int ret;                           /* Generic return status */

  /* Allocate the buffer to used for socket/disk I/O */

  packet = (uint8_t*)zalloc(TFTP_IOBUFSIZE);
  if (!packet)
    {
      ndbg("packet memory allocation failure\n");
      set_errno(ENOMEM);
      goto errout;
    }

  /* Open the file for reading */

  fd = open(local, O_RDONLY);
  if (fd < 0)
    {
      ndbg("open failed: %d\n", errno);
      goto errout_with_packet;
    }

  /* Initialize a UDP socket and setup the server addresss */

  sd = tftp_sockinit(&server, addr);
  if (sd < 0)
    {
      goto errout_with_fd;
    }

  /* Send the write request using the well known port.  This may need
   * to be done several times because (1) UDP is inherenly unreliable
   * and packets may be lost normally, and (2) uIP has a nasty habit
   * of droppying packets if there is nothing hit in the ARP table.
   */

  blockno = 1;
  retry   = 0;
  for (;;)
    {
      packetlen = tftp_mkreqpacket(packet, TFTP_WRQ, remote, binary);
      ret = tftp_sendto(sd, packet, packetlen, &server);
      if (ret != packetlen)
        {
          goto errout_with_sd;
        }

      /* Receive the ACK for the write request */

      if (tftp_rcvack(sd, packet, &server, &port, NULL) == 0)
        {
          break;
        }

      ndbg("Re-sending request\n");

      /* We are going to loop and re-send the request packet. Check the
       * retry count so that we do not loop forever.
        */

      if (++retry > TFTP_RETRIES)
        {
          ndbg("Retry count exceeded\n");
          set_errno(ETIMEDOUT);
          goto errout_with_sd;
        }
    }

  /* Then loop sending the entire file to the server in chunks */

  offset     = 0;
  retry      = 0;

  for (;;)
    {
      /* Construct the next data packet */

      packetlen = tftp_mkdatapacket(fd, offset, packet, blockno);
      if (packetlen < 0)
        {
          goto errout_with_sd;
        }

      /* Send the next data chunk */

      ret = tftp_sendto(sd, packet, packetlen, &server);
      if (ret != packetlen)
        {
          goto errout_with_sd;
        }

      /* Check for an ACK for the data chunk */

      if (tftp_rcvack(sd, packet, &server, &port, &rblockno) == OK)
        {
          /* Check if the packet that we just sent was ACK'ed.  If not,
           * we just loop to resend the same packet (same blockno, same
           * file offset).
           */

          if (rblockno == blockno)
            {
               /* Yes.. If we are at the end of the file and if all of the packets
                * have been ACKed, then we are done.
                */

              if (packetlen < TFTP_PACKETSIZE)
                {
                  break;
                }

               /* Not the last block.. set up for the next block */

               blockno++;
               offset += TFTP_DATASIZE;
               retry  = 0;

               /* Skip the retry test */

               continue;
            }
        }

      /* We are going to loop and re-send the data packet. Check the retry
       * count so that we do not loop forever.
       */

      if (++retry > TFTP_RETRIES)
        {
          ndbg("Retry count exceeded\n");
          set_errno(ETIMEDOUT);
          goto errout_with_sd;
        }
    }

  /* Return success */

  result = OK;

errout_with_sd:
  close(sd);
errout_with_fd:
  close(fd);
errout_with_packet:
  free(packet);
errout:
  return result;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP && CONFIG_NFILE_DESCRIPTORS > 0 */
