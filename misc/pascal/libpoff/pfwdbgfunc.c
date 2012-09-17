/**********************************************************************
 * pfwdbgfunc.c
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
#include <string.h>
#include <errno.h>

#include "keywords.h"  /* Standard types */
#include "pedefs.h"    /* error code definitions */

#include "perr.h"      /* error() */
#include "pofflib.h"   /* POFF library interface */
#include "pfprivate.h" /* POFF private definitions */

/**********************************************************************
 *Pre-processor Definitions
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static void poffCheckDebugFuncInfoAllocation(poffInfo_t *poffInfo)
{
  /* Check if we have allocated a line number buffer yet */

  if (!poffInfo->debugFuncTable)
    {
      /* No, allocate it now */

      poffInfo->debugFuncTable = (uint8_t*)
        malloc(INITIAL_DEBUGFUNC_TABLE_SIZE);

      if (!poffInfo->debugFuncTable)
        {
          fatal(eNOMEMORY);
        }

      poffInfo->debugFuncSection.sh_size = 0;
      poffInfo->debugFuncTableAlloc      = INITIAL_DEBUGFUNC_TABLE_SIZE;
    }
}

/***********************************************************************/

static void poffCheckDebugFuncInfoReallocation(poffInfo_t *poffInfo, uint32_t nparms)
{
  uint32_t needed = sizeof(poffDebugFuncInfo_t) + nparms*sizeof(poffDebugArgInfo_t);
  if (poffInfo->debugFuncSection.sh_size + needed > poffInfo->debugFuncTableAlloc)
    {
      uint32_t newAlloc =
        poffInfo->debugFuncTableAlloc +
        DEBUGFUNC_TABLE_INCREMENT;

      void *tmp;

      /* Reallocate the line number buffer */

      tmp = realloc(poffInfo->debugFuncTable, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      poffInfo->debugFuncTableAlloc = newAlloc;
      poffInfo->debugFuncTable      = (uint8_t*)tmp;
    }
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************/
/* Add a debug inforamtion to the debug func table.  Returns the index
 * associated with the line number entry in the line number table.
 */

uint32_t poffAddDebugFuncInfo(poffHandle_t handle,
                            poffLibDebugFuncInfo_t *pContainer)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  poffDebugFuncInfo_t *pFuncInfo;
  poffDebugArgInfo_t *pArgInfo;
  uint32_t funcInfoIndex;
  uint32_t argInfoIndex;
  int i;

  /* Verify that the debug info table has been allocated */

  poffCheckDebugFuncInfoAllocation(poffInfo);

  /* Verify that the debug infotable is large enough to hold
   * information about function
   */

  poffCheckDebugFuncInfoReallocation(poffInfo, pContainer->nparms);

  /* Save the information in the debug func info table */

  funcInfoIndex = poffInfo->debugFuncSection.sh_size;
  pFuncInfo     = (poffDebugFuncInfo_t*)&poffInfo->debugFuncTable[funcInfoIndex];

  pFuncInfo->df_value  = pContainer->value;
  pFuncInfo->df_size   = pContainer->retsize;
  pFuncInfo->df_nparms = pContainer->nparms;

  argInfoIndex = funcInfoIndex + sizeof(poffDebugFuncInfo_t);
  for (i = 0; i < pContainer->nparms; i++)
    {
      pArgInfo = (poffDebugArgInfo_t*)&poffInfo->debugFuncTable[argInfoIndex];
      pArgInfo->da_size = pContainer->argsize[i];
      argInfoIndex += sizeof(poffDebugArgInfo_t);
    }

  /* Set the new size of the debug func info table */

  poffInfo->debugFuncSection.sh_size = argInfoIndex;
  return funcInfoIndex;
}

/***********************************************************************/
