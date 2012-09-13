/****************************************************************************
 * apps/netutils/ftpc/ftpc_filetime.c
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

#include <string.h>
#include <time.h>

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
 * Name: ftpc_filetime
 *
 * Descripton:
 *   Return the timestamp on the remote file.  Returned time is UTC
 *   (Universal Coordinated Time).
 *
 ****************************************************************************/

time_t ftpc_filetime(SESSION handle, FAR const char *filename)
{
  FAR struct ftpc_session_s *session = (FAR struct ftpc_session_s *)handle;
  struct tm timestamp;
  int ret;

  /* Make sure that the server is still connected */

  if (!ftpc_connected(session))
    {
      return ERROR;
    }

  /* Does the server support the MDTM command? */

  if (!FTPC_HAS_MDTM(session))
    {
      return ERROR;
    }

  /* Get the file time in UTC */

  memset(&timestamp, 0, sizeof(timestamp));
  ret = ftpc_cmd(session, "MDTM %s", filename);
  if (ret != OK)
    {
      return ERROR;
    }

  /* Check for "202 Command not implemented, superfluous at this site" */

  if (session->code == 202)
    {
      FTPC_CLR_MDTM(session);
      return ERROR;
    }

  /* Check for "213 File status" */

  if (session->code != 213)
    {
      return ERROR;
    }

  /* Time is Universal Coordinated Time */

  sscanf(session->reply, "%*s %04d%02d%02d%02d%02d%02d",
         &timestamp.tm_year, &timestamp.tm_mon, &timestamp.tm_mday,
         &timestamp.tm_hour, &timestamp.tm_min, &timestamp.tm_sec);
  timestamp.tm_year -= 1900;
  timestamp.tm_mon--;
  return mktime(&timestamp);
}
