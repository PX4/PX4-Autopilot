/****************************************************************************
 * apps/netutitls/smtp/smtp.c
 * smtp SMTP E-mail sender
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Heavily leveraged from uIP 1.0 which also has a BSD-like license:
 *
 * The Simple Mail Transfer Protocol (SMTP) as defined by RFC821 is
 * the standard way of sending and transfering e-mail on the
 * Internet. This simple example implementation is intended as an
 * example of how to implement protocols in uIP, and is able to send
 * out e-mail but has not been extensively tested.
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2004, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <sys/socket.h>

#include <nuttx/net/uip/uip.h>
#include <apps/netutils/smtp.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define SMTP_INPUT_BUFFER_SIZE 512

#define ISO_nl 0x0a
#define ISO_cr 0x0d

#define ISO_period 0x2e

#define ISO_2  0x32
#define ISO_3  0x33
#define ISO_4  0x34
#define ISO_5  0x35

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of a single SMTP transaction */

struct smtp_state
{
  uint8_t      state;
  bool         connected;
  sem_t        sem;
  uip_ipaddr_t smtpserver;
  const char  *localhostname;
  const char  *to;
  const char  *cc;
  const char  *from;
  const char  *subject;
  const char  *msg;
  int          msglen;
  int          sentlen;
  int          textlen;
  int          sendptr;
  char         buffer[SMTP_INPUT_BUFFER_SIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_smtp220[]            = "220";
static const char g_smtpcrnlperiodcrnl[] = "\r\n.\r\n";
static const char g_smtpdata[]           = "DATA\r\n";
static const char g_smtpfrom[]           = "From: ";
static const char g_smtphelo[]           = "HELO ";
static const char g_smtpmailfrom[]       = "MAIL FROM: ";
static const char g_smtpquit[]           = "QUIT\r\n";
static const char g_smtprcptto[]         = "RCPT TO: ";
static const char g_smtpsubject[]        = "Subject: ";
static const char g_smtpto[]             = "To: ";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int smtp_send_message(int sockfd, struct smtp_state *psmtp)
{
  if (recv(sockfd, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, 0) < 0)
    {
      return ERROR;
    }

  if (strncmp(psmtp->buffer, g_smtp220, strlen(g_smtp220)) != 0)
    {
      return ERROR;
    }

  snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtphelo, psmtp->localhostname);
  if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
    {
      return ERROR;
    }

  if (recv(sockfd, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, 0) < 0)
    {
      return ERROR;
    }

  if (psmtp->buffer[0] != ISO_2)
    {
      return ERROR;
    }

  snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtpmailfrom, psmtp->from);
  if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
    {
      return ERROR;
    }

  if (recv(sockfd, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, 0) < 0)
    {
      return ERROR;
    }

  if (psmtp->buffer[0] != ISO_2)
    {
      return ERROR;
    }

  snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtprcptto, psmtp->to);
  if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
    {
      return ERROR;
    }

  if (recv(sockfd, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, 0) < 0)
    {
      return ERROR;
    }

  if (psmtp->buffer[0] != ISO_2)
    {
      return ERROR;
    }

  if (psmtp->cc != 0)
    {
      snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtprcptto, psmtp->cc);
      if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
        {
          return ERROR;
        }

      if (recv(sockfd, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, 0) < 0)
        {
          return ERROR;
        }

      if (psmtp->buffer[0] != ISO_2)
        {
          return ERROR;
        }
    }

  if (send(sockfd, g_smtpdata, strlen(g_smtpdata), 0) < 0)
    {
      return ERROR;
    }

  if (recv(sockfd, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, 0) < 0)
    {
      return ERROR;
    }

  if (psmtp->buffer[0] != ISO_3)
    {
      return ERROR;
    }

  snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtpto, psmtp->to);
  if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
    {
      return ERROR;
    }

  if (psmtp->cc != 0)
    {
      snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtpto, psmtp->cc);
      if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
        {
          return ERROR;
        }
    }

  snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtpfrom, psmtp->from);
  if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
    {
      return ERROR;
    }

  snprintf(psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, "%s%s\r\n", g_smtpsubject, psmtp->subject);
  if (send(sockfd, psmtp->buffer, strlen(psmtp->buffer), 0) < 0)
    {
      return ERROR;
    }

  if (send(sockfd, psmtp->msg, psmtp->msglen, 0) < 0)
    {
      return ERROR;
    }

  if (send(sockfd, g_smtpcrnlperiodcrnl, strlen(g_smtpcrnlperiodcrnl), 0) < 0)
    {
      return ERROR;
    }

  if (recv(sockfd, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE, 0) < 0)
    {
      return ERROR;
    }

  if (psmtp->buffer[0] != ISO_2)
    {
      return ERROR;
    }

  if (send(sockfd, g_smtpquit, strlen(g_smtpquit), 0) < 0)
    {
      return ERROR;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Specificy an SMTP server and hostname.
 *
 * This function is used to configure the SMTP module with an SMTP server and
 * the hostname of the host.
 *
 *   lhostname - The hostname of the local, uIP host.
 *
 *   paddr     - A pointer to the IP address of the SMTP server to be
 *               configured.
 */

void smtp_configure(void *handle, const char *lhostname,
                    const uip_ipaddr_t *paddr)
{
  struct smtp_state *psmtp = (struct smtp_state *)handle;
  psmtp->localhostname = lhostname;
  uip_ipaddr_copy(psmtp->smtpserver, paddr);
}

/* Send an e-mail.
 *
 *   to      - The e-mail address of the receiver of the e-mail.
 *   cc      - The e-mail address of the CC: receivers of the e-mail.
 *   from    - The e-mail address of the sender of the e-mail.
 *   subject - The subject of the e-mail.
 *   msg     - The actual e-mail message.
 *   msglen  - The length of the e-mail message.
 */

int smtp_send(void *handle, const char *to, const char *cc, const char *from,
              const char *subject, const char *msg, int msglen)
{
  struct smtp_state *psmtp = (struct smtp_state *)handle;
  struct sockaddr_in server;
  int sockfd;
  int ret;

  /* Setup */

  psmtp->connected = true;
  psmtp->to        = to;
  psmtp->cc        = cc;
  psmtp->from      = from;
  psmtp->subject   = subject;
  psmtp->msg       = msg;
  psmtp->msglen    = msglen;

  /* Create a socket */

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      return ERROR;
    }

  /* Connect to server.  First we have to set some fields in the
   * 'server' structure.  The system will assign me an arbitrary
   * local port that is not in use.
   */

  server.sin_family = AF_INET;
  memcpy(&server.sin_addr.s_addr, &psmtp->smtpserver, sizeof(in_addr_t));
  server.sin_port = HTONS(25);

  if (connect(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0)
    {
      close(sockfd);
      return ERROR;
    }

  /* Send the message */

  ret = smtp_send_message(sockfd, psmtp);

  close(sockfd);
  return ret;
}

void *smtp_open(void)
{
  /* Allocate the handle */

  struct smtp_state *psmtp = (struct smtp_state *)malloc(sizeof(struct smtp_state));
  if (psmtp)
    {
      /* Initialize the handle */

      memset(psmtp, 0, sizeof(struct smtp_state));
     (void)sem_init(&psmtp->sem, 0, 0);
    }
  return (void*)psmtp;
}

void smtp_close(void *handle)
{
  struct smtp_state *psmtp = (struct smtp_state *)handle;
  if (psmtp)
    {
      sem_destroy(&psmtp->sem);
      free(psmtp);
    }
}
