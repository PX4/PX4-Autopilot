/****************************************************************************
 * apps/netutils/ftpc/ftpc_transfer.c
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

#include <sys/stat.h>
#include <sys/time.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <poll.h>
#include <ctype.h>
#include <errno.h>
#include <assert.h>
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
 * Name: ftp_pasvmode
 *
 * Description:
 *   Enter passive mode.
 *
 *   In active mode FTP the client connects from a random port (N>1023) to the
 *   FTP server's command port, port 21. Then, the client starts listening to
 *   port N+1 and sends the FTP command PORT N+1 to the FTP server. The server
 *   will then connect back to the client's specified data port from its local
 *   data port, which is port 20. In passive mode FTP the client initiates
 *   both connections to the server, solving the problem of firewalls filtering 
 *   the incoming data port connection to the client from the server. When
 *   opening an FTP connection, the client opens two random ports locally
 *   (N>1023 and N+1). The first port contacts the server on port 21, but
 *   instead of then issuing a PORT command and allowing the server to connect
 *   back to its data port, the client will issue the PASV command. The result
 *   of this is that the server then opens a random unprivileged port (P >
 *   1023) and sends the PORT P command back to the client. The client then
 *   initiates the connection from port N+1 to port P on the server to transfer 
 *   data.
 *
 ****************************************************************************/

static int ftp_pasvmode(struct ftpc_session_s *session,
                        uint8_t addrport[6])
{
  int tmpap[6];
  char *ptr;
  int nscan;
  int ret;
  int i;

  /* Does this host support the PASV command */

  if (!FTPC_HAS_PASV(session))
  {
    ndbg("Host doesn't support passive mode\n");
    return ERROR;
  }

  /* Request passive mode.  The server normally accepts PASV with code 227.
   * Its response is a single line showing the IP address of the server and
   * the TCP port number where the server is accepting connections.
   */

  ret = ftpc_cmd(session, "PASV");
  if (ret < 0 || !ftpc_connected(session))
    {
      return ERROR;
    }

  /* Skip over any leading stuff before address begins */

  ptr = session->reply + 4;
  while (!isdigit((int)*ptr))
    {
      ptr++;
    }

  /* The response is then 6 integer values:  four representing the
   * IP address and two representing the port number.
   */
 
  nscan = sscanf(ptr, "%d,%d,%d,%d,%d,%d",
                 &tmpap[0], &tmpap[1], &tmpap[2],
                 &tmpap[3], &tmpap[4], &tmpap[5]);
  if (nscan != 6)
    {
      ndbg("Error parsing PASV reply: '%s'\n", session->reply);
      return ERROR;
    }

  /* Then copy the sscanf'ed values as bytes */

  
  for (i = 0; i < 6; i++)
    {
      addrport[i] = (uint8_t)(tmpap[i] & 0xff);
    }

  return OK;
}

/****************************************************************************
 * Name: ftpc_abspath
 *
 * Description:
 *   Get the absolute path to a file, handling tilde expansion.
 *
 ****************************************************************************/

static FAR char *ftpc_abspath(FAR struct ftpc_session_s *session,
                              FAR const char *relpath, FAR const char *homedir,
                              FAR const char *curdir)
{
  FAR char *ptr = NULL;
  int ret = OK;

  /* If no relative path was provide, then use the current working directory */

  if (!relpath)
    {
      return strdup(curdir);
    }

  /* Handle tilde expansion */

  if (relpath[0] == '~')
    {
      /* Is the relative path only '~' */
 
      if (relpath[1] == '\0')
        {
          return strdup(homedir);
        }

      /* No... then a '/' better follow the tilde */

      else if (relpath[1] == '/')
        {
          ret = asprintf(&ptr, "%s%s", homedir, &relpath[1]);
        }

      /* Hmmm... this pretty much guaranteed to fail */

      else
        {
          ptr = strdup(relpath);
        }
    }

  /* No tilde expansion.  Check for a path relative to the current
   * directory.
   */
  
  else if (strncmp(relpath, "./", 2) == 0)
    {
      ret = asprintf(&ptr, "%s%s", curdir, relpath+1);
    }

  /* Check for an absolute path */

  else if (relpath[0] == '/' && relpath[1] == ':' && relpath[2] == '\\')
    {
      ptr = strdup(relpath);
    }

  /* Assume it a relative path */

  else
    {
      ret = asprintf(&ptr, "%s/%s", curdir, relpath);
    }

  return ptr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_xfrinit
 *
 * Description:
 *   Perform common transfer setup logic.
 *
 ****************************************************************************/

int ftpc_xfrinit(FAR struct ftpc_session_s *session)
{
  struct sockaddr_in addr;
  uint8_t addrport[6];
  uint8_t *paddr;
  uint8_t *pport;
  int ret;

  /* We must be connected to initiate a transfer */

  if (!ftpc_connected(session))
    {
      ndbg("Not connected\n");
      goto errout;
    }

  /* Initialize the data channel */

  ret = ftpc_sockinit(&session->data);
  if (ret != OK)
    {
      ndbg("ftpc_sockinit() failed: %d\n", errno);
      goto errout;
    }

  /* Duplicate the address and connection information of the command channel */

  ftpc_sockcopy(&session->data, &session->cmd);

  /* Should we enter passive mode? */

  if (FTPC_IS_PASSIVE(session))
    {
      /* Yes.. going passive. */
  
      ret = ftp_pasvmode(session, addrport);
      if (ret != OK)
        {
          ndbg("ftp_pasvmode() failed: %d\n", errno);
          goto errout_with_data;
        }

      /* Configure the data socket */

      ftpc_sockgetsockname(&session->cmd, &addr);
      memcpy(&addr.sin_addr, addrport, 4);
      memcpy(&addr.sin_port, addrport+4, 2);

      /* Connect the data socket */

      ret = ftpc_sockconnect(&session->data, &addr);
      if (ret < 0)
        {
          ndbg("ftpc_sockconnect() failed: %d\n", errno);
          goto errout_with_data;
        }
    }
  else
    {
      /* Wait for the connection to be established */

      ftpc_socklisten(&session->data);

      /* Then send our local data channel address to the server */

      paddr = (uint8_t *)&session->data.laddr.sin_addr;
      pport = (uint8_t *)&session->data.laddr.sin_port;

      ret = ftpc_cmd(session, "PORT %d,%d,%d,%d,%d,%d",
                     paddr[0], paddr[1], paddr[2],
                     paddr[3], pport[0], pport[1]);
      if (ret < 0)
        {
          ndbg("ftpc_cmd() failed: %d\n", errno);
          goto errout_with_data;
        }
    }
  return OK;

 errout_with_data:
  ftpc_sockclose(&session->data);
 errout:
  return ERROR;
}

/****************************************************************************
 * Name: ftpc_xfrreset
 *
 * Description:
 *   Reset transfer variables
 *
 ****************************************************************************/

void ftpc_xfrreset(struct ftpc_session_s *session)
{
  session->size     = 0;
  session->flags   &= ~FTPC_XFER_FLAGS;
}

/****************************************************************************
 * Name: ftpc_xfrmode
 *
 * Description:
 *   Select ASCII or Binary transfer mode
 *
 ****************************************************************************/

int ftpc_xfrmode(struct ftpc_session_s *session, uint8_t xfrmode)
{
  int ret;

  /* Check if we have already selected the requested mode */

  DEBUGASSERT(xfrmode != FTPC_XFRMODE_UNKNOWN);
  if (session->xfrmode != xfrmode)
    {
      /* Send the TYPE request to control the binary flag.  Parameters for the
       * TYPE request include:
       *
       *  A: Turn the binary flag off.
       *  A N: Turn the binary flag off.
       *  I: Turn the binary flag on.
       *  L 8: Turn the binary flag on.
       *
       *  The server accepts the TYPE request with code 200.
       */
  
      ret = ftpc_cmd(session, "TYPE %c", xfrmode == FTPC_XFRMODE_ASCII ? 'A' : 'I');
      session->xfrmode = xfrmode;
    }
  return OK;
}

/****************************************************************************
 * Name: ftpc_xfrabort
 *
 * Description:
 *   Abort a transfer in progress
 *
 ****************************************************************************/

int ftpc_xfrabort(FAR struct ftpc_session_s *session, FAR FILE *stream)
{
  FAR struct pollfd fds;
  int ret;

  /* Make sure that we are still connected */

  if (!ftpc_connected(session))
    {
      return ERROR;
    }

  /* Check if there is data waiting to be read from the cmd channel */

  fds.fd     = session->cmd.sd;
  fds.events = POLLIN;
  ret        = poll(&fds, 1, 0);
  if (ret > 0)
  {
    /* Read data from command channel */
 
    nvdbg("Flush cmd channel data\n");
    while (stream && fread(session->buffer, 1, CONFIG_FTP_BUFSIZE, stream) > 0);
    return OK;
  }

  FTPC_SET_INTERRUPT(session);

  /* Send the Telnet interrupt sequence to abort the transfer:
   * <IAC IP><IAC DM>ABORT<CR><LF>
   */

  nvdbg("Telnet ABORt sequence\n");
  ftpc_sockprintf(&session->cmd, "%c%c", TELNET_IAC, TELNET_IP); /* Interrupt process */
  ftpc_sockprintf(&session->cmd, "%c%c", TELNET_IAC, TELNET_DM); /* Telnet synch signal */
  ftpc_sockprintf(&session->cmd, "ABOR\r\n");                    /* Abort */
  ftpc_sockflush(&session->cmd);

  /* Read remaining bytes from connection */

  while (stream && fread(session->buffer, 1, CONFIG_FTP_BUFSIZE, stream) > 0);
  while(stream && fread(session->buffer, 1, CONFIG_FTP_BUFSIZE, stream) > 0)

  /* Get the ABORt reply */
 
  fptc_getreply(session);

  /* Expected replys are: "226 Closing data connection" or
   * "426 Connection closed; transfer aborted"
   */

  if (session->code != 226 && session->code != 426)
    {
      nvdbg("Expected 226 or 426 reply\n");
    }
  else
    {
      /* Get the next reply */

      fptc_getreply(session);

     /* Expected replys are:  or "225 Data connection open; no transfer in progress"
      * "226 Closing data connection"
      */
 
     if (session->code != 226 && session->code != 225)
       {
         nvdbg("Expected 225 or 226 reply\n");
       }
    }

  return ERROR;
}

/****************************************************************************
 * Name: ftpc_timeout
 *
 * Description:
 *   A timeout occurred -- either on a specific command or while waiting
 *   for a reply.
 *
 * NOTE:
 *   This function executes in the context of a timer interrupt handler.
 *
 ****************************************************************************/

void ftpc_timeout(int argc, uint32_t arg1, ...)
{
  FAR struct ftpc_session_s *session = (FAR struct ftpc_session_s *)arg1;

  nlldbg("Timeout!\n");
  DEBUGASSERT(argc == 1 && session);
  kill(session->pid, CONFIG_FTP_SIGNAL);
}

/****************************************************************************
 * Name: ftpc_absrpath
 *
 * Description:
 *   Get the absolute path to a remote file, handling tilde expansion.
 *
 ****************************************************************************/

FAR char *ftpc_absrpath(FAR struct ftpc_session_s *session,
                        FAR const char *relpath)
{
  FAR char *absrpath = ftpc_abspath(session, relpath,
                                    session->homerdir, session->currdir);
  nvdbg("%s -> %s\n", relpath, absrpath);
  return absrpath;
}

/****************************************************************************
 * Name: ftpc_abslpath
 *
 * Description:
 *   Get the absolute path to a local file, handling tilde expansion.
 *
 ****************************************************************************/

FAR char *ftpc_abslpath(FAR struct ftpc_session_s *session,
                        FAR const char *relpath)
{
  FAR char *abslpath = ftpc_abspath(session, relpath,
                                    session->homeldir, session->curldir);
  nvdbg("%s -> %s\n", relpath, abslpath);
  return abslpath;
}


