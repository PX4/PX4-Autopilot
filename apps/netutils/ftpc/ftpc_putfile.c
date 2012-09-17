/****************************************************************************
 * apps/netutils/ftpc/ftpc_putfile.c
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

#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
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
 * Name: ftpc_sendbinary
 *
 * Description:
 *   Send a binary file to the remote host.
 *
 ****************************************************************************/

static int ftpc_sendbinary(FAR struct ftpc_session_s *session,
                           FAR FILE *linstream, FILE *routstream)
{
  ssize_t nread;
  ssize_t nwritten;

  /* Loop until the entire file is sent */

  for (;;)
    {
      /* Read data from the file */

      nread = fread(session->buffer, sizeof(char), CONFIG_FTP_BUFSIZE, linstream);
      if (nread <= 0)
        {
          /* nread == 0 is just EOF */

          if (nread < 0)
            {
              (void)ftpc_xfrabort(session, linstream);
              return ERROR;
            }

          /* Return success */

          return OK;
        }

      /* Send the data */

      nwritten = fwrite(session->buffer, sizeof(char), nread, routstream);
      if (nwritten != nread)
        {
          (void)ftpc_xfrabort(session, routstream);

          /* Return failue */

          return ERROR;
        }

      /* Increment the size of the file sent */

      session->size += nread;
    }
}

/****************************************************************************
 * Name: ftpc_sendtext
 *
 * Description:
 *   Send a text file to the remote host.
 *
 ****************************************************************************/

static int ftpc_sendtext(FAR struct ftpc_session_s *session,
                         FAR FILE *linstream, FAR FILE *routstream)
{
  int ch;
  int ret = OK;

  /* Write characters one at a time. */

  while ((ch = fgetc(linstream)) != EOF)
    {
      /* If it is a newline, send a carriage return too */

      if (ch == '\n')
        {
          if (fputc('\r', routstream) == EOF)
            {
              (void)ftpc_xfrabort(session, routstream);
              ret = ERROR;
              break;
            }

          /* Increment the size of the file sent */

          session->size++;
        }

      /* Send the character */

      if (fputc(ch, routstream) == EOF)
        {
          (void)ftpc_xfrabort(session, routstream);
          ret = ERROR;
          break;
        }

      /* Increment the size of the file sent */

      session->size++;
    }

  return ret;
}

/****************************************************************************
 * Name: ftpc_sendfile
 *
 * Description:
 *   Send the file to the remote host.
 *
 ****************************************************************************/

static int ftpc_sendfile(struct ftpc_session_s *session, const char *path,
                         FILE *stream, uint8_t how, uint8_t xfrmode)
{
  long offset = session->offset;
#ifdef CONFIG_DEBUG
  FAR char *rname;
  FAR char *str;
  int len;
#endif
  int ret;

  session->offset = 0;

  /* Were we asked to store a file uniquely?  Does the host support the STOU
   * command?
   */

  if (how == FTPC_PUT_UNIQUE && !FTPC_HAS_STOU(session))
    {
      /* We cannot store a file uniquely */

      return ERROR;
    }

  ftpc_xfrreset(session);
  FTPC_SET_PUT(session);

  /* Initialize for the transfer */

  ret = ftpc_xfrinit(session);
  if (ret != OK)
    {
      return ERROR;
    }

  ftpc_xfrmode(session, xfrmode);

  /* The REST command sets the start position in the file.  Some servers
   * allow REST immediately before STOR for binary files.
   */

  if (offset > 0)
    {
      ret = ftpc_cmd(session, "REST %ld", offset);
      session->size = offset;
    }

  /* Send the file using STOR, STOU, or APPE:
   *
   * - STOR request asks the server to receive the contents of a file from
   *   the data connection already established by the client.
   * - APPE is just like STOR except that, if the file already exists, the
   *   server appends the client's data to the file.
   * - STOU is just like STOR except that it asks the server to create a
   *   file under a new pathname selected by the server. If the server
   *   accepts STOU, it provides the pathname in a human-readable format in
   *   the text of its response.
   */

  switch (how)
    {
    case FTPC_PUT_UNIQUE:
      {
        ret = ftpc_cmd(session, "STOU %s", path);

        /* Check for "502 Command not implemented" */

        if (session->code == 502)
          {
            /* The host does not support the STOU command */

            FTPC_CLR_STOU(session);
            return ERROR;
          }

        /* Get the remote filename from the response */

#ifdef CONFIG_DEBUG
        str = strstr(session->reply, " for ");
        if (str)
          {
            str += 5;
            len = strlen(str);
            if (len)
              {
                if (*str == '\'')
                  {
                    rname = strndup(str+1, len-3);
                  }
                else
                  {
                    rname = strndup(str, len-1);
                    nvdbg("Unique filename is: %s\n",  rname);
                  }
                free(rname);
              }
          }
#endif
      }
      break;

    case FTPC_PUT_APPEND:
      ret = ftpc_cmd(session, "APPE %s", path);
      break;

    case FTPC_PUT_NORMAL:
    default:
      ret = ftpc_cmd(session, "STOR %s", path);
      break;
  }

  /* If the server is willing to create a new file under that name, or
   * replace an existing file under that name, it responds with a mark
   * using code 150:
   *
   * - "150 File status okay; about to open data connection"
   *
   * It then attempts to read the contents of the file from the data
   * connection, and closes the data connection. Finally it accepts the STOR
   * with:
   *
   * - "226 Closing data connection" if the entire file was successfully
   *    received and stored
   *
   * Or rejects the STOR with:
   *
   * - "425 Can't open data connection" if no TCP connection was established
   * - "426 Connection closed; transfer aborted" if the TCP connection was
   *    established but then broken by the client or by network failure
   * - "451 Requested action aborted: local error in processing",
   *   "452 - Requested action not taken", or "552 Requested file action
   *   aborted" if the server had trouble saving the file to disk.
   *
   * The server may reject the STOR request with:
   *
   * - "450 Requested file action not taken", "452 - Requested action not
   *   taken" or "553 Requested action not taken" without first responding
   *   with a mark. 
   */

  /* In active mode, we need to accept a connection on the data socket
   * (in passive mode, we have already connected the data channel to
   * the FTP server).
   */

  if (!FTPC_IS_PASSIVE(session))
    {
      ret = ftpc_sockaccept(&session->data);
      if (ret != OK)
        {
          ndbg("Data connection not accepted\n");
          return ERROR;
        }
    }

  /* Then perform the data transfer */

  if (xfrmode == FTPC_XFRMODE_ASCII)
    {
      ret = ftpc_sendtext(session, stream, session->data.outstream);
    }
  else
    {
      ret = ftpc_sendbinary(session, stream, session->data.outstream);
    }

  ftpc_sockflush(&session->data);
  ftpc_sockclose(&session->data);

  if (ret == 0)
    {
      fptc_getreply(session);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_putfile
 *
 * Description:
 *   Put a file on the remote host.
 *
 ****************************************************************************/

int ftp_putfile(SESSION handle, const char *lname, const char *rname,
                uint8_t how, uint8_t xfrmode)
{
  FAR struct ftpc_session_s *session = (FAR struct ftpc_session_s *)handle;
  FAR char *abslpath;
  struct stat statbuf;
  FILE *finstream;
  int ret;

  /* Don't call this with a NULL local file name */

  DEBUGASSERT(lname);

  /* If the remote name is not specified, then it is assumed to the same as
   * the local file name.
   */

  if (!rname)
    {
      rname = lname;
    }

  /* Get the full path to the local file */

  abslpath = ftpc_abslpath(session, lname);
  if (!abslpath)
    {
      ndbg("ftpc_abslpath(%s) failed: %d\n", errno);
      goto errout;
    }

  /* Make sure that the local file exists */

  ret = stat(abslpath, &statbuf);
  if (ret != OK)
    {
      ndbg("stat(%s) failed: %d\n", errno);
      goto errout_with_abspath;
    }

  /* Make sure that the local name does not refer to a directory */

  if (S_ISDIR(statbuf.st_mode))
    {
      ndbg("%s is a directory\n", abslpath);
      goto errout_with_abspath;
    }

  /* Open the local file for reading */

  finstream = fopen(abslpath, "r");
  if (!finstream)
    {
      ndbg("fopen() failed: %d\n", errno);
      goto errout_with_abspath;
    }

  /* Are we resuming a transfer? */

  session->offset = 0;
  if (how == FTPC_PUT_RESUME)
    {
      /* Yes... Get the size of the file.  This will only work if the
       * server supports the SIZE command.
       */

      session->offset = ftpc_filesize(session, rname);
      if (session->offset == (off_t)ERROR)
        {
          ndbg("Failed to get size of remote file: %s\n", rname);
          goto errout_with_instream;
        }
      else
        {
          /* Seek to the offset in the file corresponding to the size
           * that we have already sent.
           */

          ret = fseek(finstream, session->offset, SEEK_SET);
          if (ret != OK)
            {
              ndbg("fseek failed: %d\n", errno);
              goto errout_with_instream;
            }
        }
    }

  /* Send the file */

  ret = ftpc_sendfile(session, rname, finstream, how, xfrmode);
  if (ret == OK)
    {
      fclose(finstream);
      free(abslpath);
      return OK;
    }

  /* Various error exits */

errout_with_instream:
  fclose(finstream);
errout_with_abspath:
  free(abslpath);
errout:
  return ERROR;
}
