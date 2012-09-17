/****************************************************************************
 * apps/netutils/ftpc/ftpc_rpwd.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_rpwd
 *
 * Descripton:
 *   Returns the current working directory on the remote server.
 *
 ****************************************************************************/

FAR char *ftpc_rpwd(SESSION handle)
{
  FAR struct ftpc_session_s *session = (FAR struct ftpc_session_s *)handle;
  FAR char *start;
  FAR char *end;
  FAR char *pwd;
  FAR char *ptr;
  int len;
  int ret;

  /* Send the PWD command */

  ret = ftpc_cmd(session, "PWD");

  /* Response is like: 257 "/home/gnutt" (from vsftpd).
   *
   * Extract the quoated path name into allocated memory.
   */

  start = strchr(session->reply, '\"');
  if (!start)
    {
      ndbg("Opening quote not found\n");
      return NULL;
    }
  start++;
 
  end = strchr(start, '\"');
  if (!end)
    {
      ndbg("Closing quote not found\n");
      return NULL;
    }

  /* Allocate memory for the path name:
   *
   *   Reply: 257 "/home/gnutt"
   *               ^start     ^end
   *
   *   len = end - start + 1 = 11 (+ NUL terminator)
   */
 
  len = end - start + 1;
  pwd = (char *)malloc(len + 1);
  if (!pwd)
    {
      ndbg("Failed to allocate string\n");
      return NULL;
    }
 
  /* Copy the string into the allocated memory */

  memcpy(pwd, start, len);
  pwd[len] = '\0';

  /* Remove any trailing slashes that the server may have added */

  ftpc_stripslash(pwd);

  /* Change DOS style directory separator ('\') to UNIX style ('/') */

  for (ptr = pwd; *ptr; ptr++)
    {
      if (*ptr == '\\')
        {
          *ptr = '/';
        }
    }
  return pwd;
}
