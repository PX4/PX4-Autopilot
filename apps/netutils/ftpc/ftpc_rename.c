/****************************************************************************
 * apps/netutils/ftpc/ftpc_rename.c
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
 * Name: ftpc_rename
 *
 * Description:
 *   Rename a file on the remote server.
 *
 ****************************************************************************/

int ftpc_rename(SESSION handle, FAR const char *oldname, FAR const char *newname)
{
  FAR struct ftpc_session_s *session = (FAR struct ftpc_session_s *)handle;
  char *oldcopy;
  char *newcopy;
  int ret;

  oldcopy = strdup(oldname);
  ftpc_stripslash(oldcopy);

  /* A RNFR request asks the server to begin renaming a file. A typical
   * server accepts RNFR with:
   *
   * - "350 Requested file action pending further information" if the file
   *   exists
   *
   * Or rejects RNFR with:
   *
   * - "450 Requested file action not taken"
   * - "550 Requested action not taken"
   */

  ret = ftpc_cmd(session, "RNFR %s", oldcopy);
  if (ret != OK)
    {
      free(oldcopy);
      return ERROR;
    }

  newcopy = strdup(newname);
  ftpc_stripslash(newcopy);
  
  /* A RNTO request asks the server to finish renaming a file. RNTO must
   * come immediately after RNFR; otherwise the server may reject RNTO with:
   *
   * - "503 Bad sequence of commands"
   *
   * A typical server accepts RNTO with:
   *
   * - "250 Requested file action okay, completed" if the file was renamed
   *   successfully
   *
   * Or rejects RMD with:
   *
   * - "550 Requested action not taken"
   * - "553 Requested action not taken"
   */

  ret = ftpc_cmd(session, "RNTO %s", newcopy);

  free(oldcopy);
  free(newcopy);
  return ret;
}
