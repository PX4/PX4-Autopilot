/****************************************************************************
 * apps/netutils/ftpc/ftpc_cmd.c
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
#include <stdarg.h>
#include <errno.h>
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_restore
 *
 * Description:
 *   Restore the connection to the server and log in again.
 *
 ****************************************************************************/

#ifdef CONFIG_FTP_AUTORECONNECT
static int ftpc_restore(struct ftpc_session_s *session)
{
  int ret;

  if (session)
    {
      /* Set the initial directory to the last valid current directory */

      free(session->initrdir);
      session->initrdir = ftpc_dequote(session->currdir);

      /* Reconnect to the server */

      ret = ftpc_reconnect(session);
      if (ret == 0)
        {
          /* Log into the server */

          ret = ftpc_relogin(session);
        }
      else
        {
          /* Failed to reconnect to the server */

          ftpc_reset(session);
        }
      return ret;
  }

  return ERROR;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_cmd
 *
 * Description:
 *   Send the specified command to the server.
 *
 ****************************************************************************/

int ftpc_cmd(struct ftpc_session_s *session, const char *cmd, ...)
{
  va_list ap;
#ifdef CONFIG_FTP_AUTORECONNECT
  bool reconnect = false;
#endif
  int ret;

  /* Verify that we are still connecte to the server */
 
  if (!ftpc_sockconnected(&session->cmd))
  {
    ndbg("Cmd channel si not connected\n");
    goto errout;
  }

  /* Loop, reconnecting as necessary until the command is sent */

#ifdef CONFIG_FTP_AUTORECONNECT
  for (;;)
#endif
    {
      /* Send the command */

      va_start(ap, cmd);
      ret = ftpc_sockvprintf(&session->cmd, cmd, ap);
      if (ret >= 0)
        {
          ret = ftpc_sockprintf(&session->cmd, "\r\n");
          if (ret >= 0)
            {
              ret = ftpc_sockflush(&session->cmd);
            }
        }
      va_end(ap);

      /* Check for an error in sending the data */

      if (ret < 0)
        {
          ndbg("Error sending cmd %s: %d\n", cmd, errno);
          goto errout;
       }

      /* Get the response to the command */
 
      ret = fptc_getreply(session);
      if (ret < 0)
        {
          ndbg("Error getting reply: %d\n", errno);
          goto errout;
       }

      /* Check for "421 Service not available, closing control connection" */

      if (session->code == 421)
        {
          /* Server is closing the control connection. */

          ndbg("Server closed control connection\n");

          /* If we were previously logged in and this is not a QUIT commnad
           * then attempt to automatically reconnect to the server.
           */

#ifdef CONFIG_FTP_AUTORECONNECT
          if (ftpc_loggedin(session) && strcasecmp(cmd, "QUIT") != 0)
            {
              /* Don't try re-connecting more than once */

              if (reconnect)
                {
                  ndbg("Reconnect failed\n");
                  goto errout;
                }
              else
                {
                  /* Try to restore the connection and, if successful,
                   * continue the loop and try to send the command again.
                   */

                  ndbg("Reconnecting...\n");
                  reconnect = true;
                  ret = ftpc_restore();
                  if (ret < 0)
                    {
                      ndbg("Failed to restore the connection");
                      goto errout;
                    }
                  continue;
                }
            }
          else
#endif
            {
              /* Don't try to connect, just return an error (retaining
               * the session response code (421)
               */

              return ERROR;
            }
        }

      /* The command was successfully sent */

      return OK;
    }

errout:
  session->code = -1;
  return ERROR;
}
