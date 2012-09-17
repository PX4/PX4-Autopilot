/****************************************************************************
 * apps/netutils/ftpc/ftpc_getreply.c
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
#include <string.h>
#include <debug.h>

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
 * Name: ftpc_gets
 ****************************************************************************/

static int ftpc_gets(struct ftpc_session_s *session)
{
  int ch;
  int ndx = 0;

  /* Start wth an empty response string */

  session->reply[0] = '\0';

  /* Verify that the command channel is still connected */

  if (!ftpc_sockconnected(&session->cmd))
    {
      ndbg("Cmd channel disconnected\n");
      return ERROR;
    }

  /* Loop until the full line is obtained */

  for (;;)
    {
      /* Get the next character from incoming command stream */

      ch = ftpc_sockgetc(&session->cmd);

      /* Check if the command stream was closed */

      if (ch == EOF)
        {
          ndbg("EOF: Server closed command stream\n");
          ftpc_reset(session);
          return ERROR;
        }

      /* Handle embedded Telnet stuff */

      else if (ch == TELNET_IAC)
        {
          /* Handle TELNET commands */

          switch(ch = ftpc_sockgetc(&session->cmd))
            {
            case TELNET_WILL:
            case TELNET_WONT:
              ch = ftpc_sockgetc(&session->cmd);
              ftpc_sockprintf(&session->cmd, "%c%c%c", TELNET_IAC, TELNET_DONT, ch);
              ftpc_sockflush(&session->cmd);
              break;

            case TELNET_DO:
            case TELNET_DONT:
              ch = ftpc_sockgetc(&session->cmd);
              ftpc_sockprintf(&session->cmd, "%c%c%c", TELNET_IAC, TELNET_WONT, ch);
              ftpc_sockflush(&session->cmd);
              break;

            default:
            break;
            }

          continue;
        }

      /* Deal with carriage returns */

      else if (ch == ISO_cr)
        {
          /* What follows the carriage return? */
 
          ch = ftpc_sockgetc(&session->cmd);
          if (ch == '\0')
            {
              /* If it is followed by a NUL then keep it */

              ch = ISO_cr;
            }

          /* If it is followed by a newline then break out of the loop. */

          else if (ch == ISO_nl)
            {
              /* Newline terminates the reply */

              break;
            }

          /* If we did not lose the connection, then push the character
           * following the carriage back on the "stack" and continue to
           * examine it from scratch (if could be part of the Telnet
           * protocol).
           */

          else if (ch != EOF)
            {
              ungetc(ch, session->cmd.instream);
              continue;
            }
        }

      else if (ch == ISO_nl)
        {
          /* The ISO newline character terminates the string.  Just break
           * out of the loop.
           */

          break;
        }

      /* Put the character into the response buffer.  Is there space for
       * another character in the reply buffer?
       */

      if (ndx < CONFIG_FTP_MAXREPLY)
        {
          /* Yes.. put the character in the reply buffer */
 
          session->reply[ndx++] = (char)ch;
        }
      else
        {
          ndbg("Reply truncated\n");
        }
    }

  session->reply[ndx] = '\0';
  session->code = atoi(session->reply);
  return session->code;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_getreply
 ****************************************************************************/

int fptc_getreply(struct ftpc_session_s *session)
{
  char tmp[5]="xxx ";
  int ret;

  /* Set up a timeout */
  
  if (session->replytimeo)
    {
      ret = wd_start(session->wdog, session->replytimeo, ftpc_timeout, 1, session);
    }

  /* Get the next line from the server */

  ret = ftpc_gets(session);

  /* Do we still have a connection? */

  if (!ftpc_sockconnected(&session->cmd))
    {
      /* No.. cancel the timer and return an error */

      wd_cancel(session->wdog);
      nvdbg("Lost connection\n");
      return ERROR;
    }

  /* Did an error occur? */

  if (ret < 0)
    {
      /* No.. cancel the timer and return an error */

      wd_cancel(session->wdog);
      nvdbg("ftpc_gets failed\n");
      return ERROR;
    }

  nvdbg("Reply: %s\n", session->reply);        

  if (session->reply[3] == '-')
    {
      /* Multi-line response */

      strncpy(tmp, session->reply, 3);
      do
        {
          if (ftpc_gets(session) == -1)
            {
              break;
            }

          nvdbg("Reply: %s\n", session->reply);
        }
      while (strncmp(tmp, session->reply, 4) != 0);
    }

  wd_cancel(session->wdog);
  return ret;
}
