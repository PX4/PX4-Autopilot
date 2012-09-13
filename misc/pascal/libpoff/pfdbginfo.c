/**********************************************************************
 * pdbginfo.c
 * Manage debug information
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 **********************************************************************/

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "keywords.h"  /* Standard types */
#include "pedefs.h"    /* error code definitions */

#include "perr.h"      /* error() */
#include "pofflib.h"   /* POFF library interface */

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

static poffLibDebugFuncInfo_t *g_pDebugInfoHead = NULL;
static poffLibDebugFuncInfo_t *g_pDebugInfoTail = NULL;

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * Public Functions
 ***********************************************************************/

void poffReadDebugFuncInfoTable(poffHandle_t handle)
{
  poffLibDebugFuncInfo_t *pDebugInfo;

  /* Read all function debug information into a link list */

  while ((pDebugInfo = poffGetDebugFuncInfo(handle)) != NULL)
    {
      if (!g_pDebugInfoHead)
        {
          g_pDebugInfoHead = pDebugInfo;
        }
      else
        {
          g_pDebugInfoTail->next = pDebugInfo;
        }
      g_pDebugInfoTail = pDebugInfo;
    }
}

/***********************************************************************/

poffLibDebugFuncInfo_t *poffFindDebugFuncInfo(uint32_t offset)
{
  poffLibDebugFuncInfo_t *pDebugInfo;

  /* Search the list for an entry with PC==offset */

  for (pDebugInfo = g_pDebugInfoHead; pDebugInfo;
       pDebugInfo = pDebugInfo->next)
    {
      if (pDebugInfo->value == offset)
        {
          return pDebugInfo;
        }
    }
  return NULL;
}

/***********************************************************************/

void poffReplaceDebugFuncInfo(poffHandle_t handle)
{
  poffLibDebugFuncInfo_t *pDebugInfo;

  /* Discard any existing debug info in the POFF object */

  poffDiscardDebugFuncInfo(handle);

  /* Then add all of the buffered debug info into the object */

  for (pDebugInfo = g_pDebugInfoHead; pDebugInfo;
       pDebugInfo = pDebugInfo->next)
    {
      (void)poffAddDebugFuncInfo(handle, pDebugInfo);

    }
}

/***********************************************************************/

void poffReleaseDebugFuncInfoTable(void)
{
  poffLibDebugFuncInfo_t *pDebugInfo;
  poffLibDebugFuncInfo_t *pNextDebugInfo;

  /* Release the bufferred debug information */

  pDebugInfo = g_pDebugInfoHead;
  while (pDebugInfo)
    {
      pNextDebugInfo = pDebugInfo->next;
      poffReleaseDebugFuncContainer(pDebugInfo);
      pDebugInfo = pNextDebugInfo;
    }

  g_pDebugInfoHead = NULL;
  g_pDebugInfoTail = NULL;
}

/***********************************************************************/
