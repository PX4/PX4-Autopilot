/**********************************************************************
 * pfwreloc.c
 * Write relocation data to a POFF file
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
 * Pre-processor Definitions
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

static void poffCheckRelocationAllocation(poffInfo_t *poffInfo)
{
  /* Check if we have allocated a line number buffer yet */

  if (!poffInfo->relocTable)
    {
      /* No, allocate it now */

      poffInfo->relocTable = (uint8_t*)malloc(INITIAL_RELOC_TABLE_SIZE);
      if (!poffInfo->relocTable)
        {
          fatal(eNOMEMORY);
        }

      poffInfo->relocSection.sh_size = 0;
      poffInfo->relocAlloc           = INITIAL_RELOC_TABLE_SIZE;
    }
}

/***********************************************************************/

static void poffCheckRelocationReallocation(poffInfo_t *poffInfo)
{
  if (poffInfo->relocSection.sh_size + sizeof(poffRelocation_t) >
      poffInfo->relocAlloc)
    {
      uint32_t newAlloc =
        poffInfo->relocAlloc +
        RELOC_TABLE_INCREMENT;

      void *tmp;

      /* Reallocate the line number buffer */

      tmp = realloc(poffInfo->relocTable, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      poffInfo->relocAlloc = newAlloc;
      poffInfo->relocTable = (uint8_t*)tmp;
    }
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************/
/* Add a relocation to the relocation table.  Returns index value
 * associated with the relocation entry in the relocation table.
 */

uint32_t poffAddRelocation(poffHandle_t handle,
                         uint8_t relocType, uint32_t symIndex,
                         uint32_t sectionDataOffset)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  poffRelocation_t *prl;
  uint32_t index;

  /* Verify that the relocation table has been allocated */

  poffCheckRelocationAllocation(poffInfo);

  /* Verify that the relocation table is large enough to hold
   * information about another relocation.
   */

  poffCheckRelocationReallocation(poffInfo);

  /* Save the relocation information in the relocation table */

  index = poffInfo->relocSection.sh_size;
  prl   = (poffRelocation_t*)&poffInfo->relocTable[index];

  prl->rl_info    = RLI_MAKE(symIndex, relocType);
  prl->rl_offset  = sectionDataOffset;

  /* Set the new size of the line number table */

  poffInfo->relocSection.sh_size += sizeof(poffRelocation_t);
  return index;
}

/***********************************************************************/
