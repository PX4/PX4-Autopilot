/****************************************************************************
 * apps/netutils/ftpc/ftpc_listdir.c
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

typedef void (*callback_t)(FAR const char *name, FAR void *arg);

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
 * Name: ftpc_dircount
 *
 * Description:
 *   This callback simply counts the number of names in the directory.
 *
 ****************************************************************************/

static void ftpc_dircount(FAR const char *name, FAR void *arg)
{
  FAR unsigned int *dircount = (FAR unsigned int *)arg;
  (*dircount)++;
}

/****************************************************************************
 * Name: ftpc_addname
 *
 * Description:
 *   This callback adds a name to the directory listing.
 *
 ****************************************************************************/

static void ftpc_addname(FAR const char *name, FAR void *arg)
{
  FAR struct ftpc_dirlist_s *dirlist = (FAR struct ftpc_dirlist_s *)arg;
  unsigned int nnames   = dirlist->nnames;
  dirlist->name[nnames] = strdup(name);
  dirlist->nnames       = nnames + 1;
}

/****************************************************************************
 * Name: ftpc_nlstparse
 *
 * Description:
 *   Parse the NLST directory response.  The NLST response consists of a
 *   sequence of pathnames. Each pathname is terminated by \r\n.
 *
 *   If a pathname starts with a slash, it represents the pathname. If a
 *   pathname does not start with a slash, it represents the pathname obtained
 *   by concatenating the pathname of the directory and the pathname.
 *
 *   IF NLST of directory /pub produces foo\r\nbar\r\n, it refers to the
 *   pathnames /pub/foo and /pub/bar.
 *
 ****************************************************************************/

static void ftpc_nlstparse(FAR FILE *instream, callback_t callback,
                           FAR void *arg)
{
  char buffer[CONFIG_FTP_MAXPATH+1];

  /* Read every filename from the temporary file */

  for (;;)
    {
      /* Read the next line from the file */

      if (!fgets(buffer, CONFIG_FTP_MAXPATH, instream))
        {
          break;
        }

      /* Remove any trailing CR-LF from the line */

      ftpc_stripcrlf(buffer);

      /* Check for empty file names */

      if (buffer[0] == '\0')
        {
          break;
        }
      nvdbg("File: %s\n", buffer);

      /* Perform the callback operation */

      callback(buffer, arg);
    }
}

/****************************************************************************
 * Name: ftpc_recvdir
 *
 * Description:
 *   Get the directory listing.
 *
 ****************************************************************************/

static int ftpc_recvdir(FAR struct ftpc_session_s *session,
                        FAR FILE *outstream)
{
  int ret;

  /* Verify that we are still connected to the server */

  if (!ftpc_connected(session))
    {
      ndbg("Not connected to server\n");
      return ERROR;
    }

  /* Setup for the transfer */

  ftpc_xfrreset(session);
  ret = ftpc_xfrinit(session);
  if (ret != OK)
    {
      return ERROR;
    }

  /* Send the "NLST" command.  Normally the server responds with a mark
   * using code 150:
   *
   * - "150 File status okay; about to open data connection"
   *
   * It then stops accepting new connections, attempts to
   * send the contents of the directory over the data connection, and
   * closes the data connection.
   */

  ret = ftpc_cmd(session, "NLST");
  if (ret != OK)
    {
      return ERROR;
    }

  /* In active mode, we need to accept a connection on the data socket
   * (in passive mode, we have already connected the data channel to
   * the FTP server).
   */

  if (!FTPC_IS_PASSIVE(session))
    {
      ret = ftpc_sockaccept(&session->data);
      if (ret != OK)
        {
          ndbg("ftpc_sockaccept() failed: %d\n", errno);
          return ERROR;
        }
    }

  /* Receive the NLST directory list  */

  ret = ftpc_recvtext(session, session->data.instream, outstream);
  ftpc_sockclose(&session->data);
  if (ret != OK)
    {
      return ERROR;
    }

  /* Get the server reply. After closing the data connection, the should
   * accept the request with:
   *
   * - "226 Closing data connection" if the entire directory was
   *    successfully transmitted;
   *
   * Or reject it with:
   *
   * - "425 Can't open data connection" if no TCP connection was established
   * - "426 - Connection closed; transfer aborted" if the TCP connection was
   *    established but then broken by the client or by network failure
   * - "451 - Requested action aborted: local error in processing" if the
   *    server had trouble reading the directory from disk.
   *
   * The server may reject the LIST or NLST request (with code 450 or 550)
   * without first responding with a mark. In this case the server does not
   * touch the data connection.
   */

  fptc_getreply(session);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_listdir
 *
 * Description:
 *   Get a simple directory listing using NLST:
 *
 *     NLST [<SP> <pathname>] <CRLF>
 *
 *   We could do much, much more here using the LIST or MLST/MLSD commands,
 *   but the parsing is a bitch. See http://cr.yp.to/ftpparse.html
 *
 *   NOTE:  We expect to receive only well structured directory paths. Tilde
 *   expansion "~/xyz" and relative pathes (abc/def) because we do have
 *   special knowledge about the home and current directories.  But otherwise
 *   the pathes are expected to be pre-sanitized:  No . or .. in paths,
 *   no '//' in paths, etc.
 *
 ****************************************************************************/

FAR struct ftpc_dirlist_s *ftpc_listdir(SESSION handle,
                                        FAR const char *dirpath)
{
  FAR struct ftpc_session_s *session = (FAR struct ftpc_session_s *)handle;
  struct ftpc_dirlist_s *dirlist;
  FILE *filestream;
  FAR char *absrpath;
  FAR char *tmpfname;
  bool iscurrdir;
  unsigned int nnames;
  int allocsize;
  int ret;

  /* Get the absolute path to the directory */

  absrpath = ftpc_absrpath(session, dirpath);
  ftpc_stripslash(absrpath);

  /* Is the directory also the remote current working directory? */

  iscurrdir = (strcmp(absrpath, session->currdir) == 0);

  /* Create a temporary file to hold the directory listing */

  asprintf(&tmpfname, "%s/TMP%d.dat", CONFIG_FTP_TMPDIR, getpid());
  filestream = fopen(tmpfname, "w+");
  if (!filestream)
    {
      ndbg("Failed to create %s: %d\n", tmpfname, errno);
      free(absrpath);
      free(tmpfname);
      return NULL;
    }

  /* "CWD" first so that we get the directory contents, not the
   * directory itself.
   */

  if (!iscurrdir)
    {
      ret = ftpc_cmd(session, "CWD %s", absrpath);
      if (ret != OK)
        {
          ndbg("CWD to %s failed\n", absrpath);
        }
    }

  /* Send the NLST command with no arguments to get the entire contents of
   * the directory.
   */

  ret = ftpc_recvdir(session, filestream);

  /* Go back to the correct current working directory */

  if (!iscurrdir)
    {
      int tmpret = ftpc_cmd(session, "CWD %s", session->currdir);
      if (tmpret != OK)
        {
          ndbg("CWD back to to %s failed\n", session->currdir);
        }
    }

  /* Did we successfully receive the directory listing? */

  dirlist = NULL;
  if (ret == OK)
    {
      /* Count the number of names in the temporary file */

      rewind(filestream);
      nnames = 0;

      ftpc_nlstparse(filestream, ftpc_dircount, &nnames);
      if (!nnames)
        {
          ndbg("Nothing found in directory\n");
          goto errout;
        }
      nvdbg("nnames: %d\n", nnames);

      /* Allocate and initialize a directory container */

      allocsize = SIZEOF_FTPC_DIRLIST(nnames);
      dirlist = (struct ftpc_dirlist_s *)malloc(allocsize);
      if (!dirlist)
        {
          ndbg("Failed to allocate dirlist\n");
          goto errout;
        }

      /* Then copy all of the directory strings into the container */

      rewind(filestream);
      dirlist->nnames = 0;

      ftpc_nlstparse(filestream, ftpc_addname, dirlist);
      DEBUGASSERT(nnames == dirlist->nnames);
    }

errout:
  fclose(filestream);
  free(absrpath);
  unlink(tmpfname);
  free(tmpfname);
  return dirlist;
}

/****************************************************************************
 * Name: ftpc_dirfree
 *
 * Description:
 *   Release the allocated directory listing.
 *
 ****************************************************************************/

void ftpc_dirfree(FAR struct ftpc_dirlist_s *dirlist)
{
  int i;

  if (dirlist)
    {
      /* Free each directory name in the directory container */

      for (i = 0; i < dirlist->nnames; i++)
        {
          /* NULL means that the caller stole the string */

          if (dirlist->name[i])
            {
              free(dirlist->name[i]);
            }
        }

      /* Then free the container itself */

      free(dirlist);
    }
}

