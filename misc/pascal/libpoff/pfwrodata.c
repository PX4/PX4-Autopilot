/**********************************************************************
 * pfwrodata.c
 * Write to the RODATA section of a POFF file
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

static uint16_t poffCheckRoDataAlloc(poffInfo_t *poffInfo)
{
  if (!poffInfo->roDataSectionData)
    {
      /* No, allocate it now */

      poffInfo->roDataSectionData = (uint8_t*)malloc(INITIAL_RODATA_SECTION_SIZE);
      if (!poffInfo->roDataSectionData)
        {
          return eNOMEMORY;
        }

      poffInfo->roDataSection.sh_size = 0;
      poffInfo->roDataSectionAlloc    = INITIAL_RODATA_SECTION_SIZE;
    }
  return eNOERROR;
}

static uint16_t poffCheckRoDataRealloc(poffInfo_t *poffInfo, uint16_t len)
{
  /* Check if there is room for the new data */

  if (poffInfo->roDataSection.sh_size + len > poffInfo->roDataSectionAlloc)
    {
      uint32_t newAlloc;
      void *tmp;

      /* Make certain that this is big enough (it should be) */

      newAlloc = poffInfo->roDataSectionAlloc + RODATA_SECTION_INCREMENT;
      while (poffInfo->roDataSection.sh_size + len > newAlloc)
        {
          newAlloc += RODATA_SECTION_INCREMENT;
        }

      /* Reallocate the roDataram data section buffer */

      tmp = realloc(poffInfo->roDataSectionData, newAlloc);
      if (!tmp)
        {
          return eNOMEMORY;
        }

      /* And set the new size */

      poffInfo->roDataSectionAlloc = newAlloc;
      poffInfo->roDataSectionData  = (uint8_t*)tmp;
    }
  return eNOERROR;
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

#if 0 /* Not used */
uint32_t poffAddRoDataByte(poffHandle_t handle, uint8_t dataByte)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  uint32_t    offset;
  uint16_t    errCode;

  /* Check if we have allocated a data section buffer yet */

  errCode = poffCheckRoDataAlloc(poffInfo);
  if (errCode != eNOERROR)
    {
      fatal(errCode);
    }

  /* Check if there is room for a new byte */

  errCode = poffCheckRoDataRealloc(poffInfo, 1);

  /* Copy data section byte into the data section buffer */

  offset = poffInfo->roDataSection.sh_size;
  poffInfo->roDataSectionData[offset] = dataByte;

  /* Set the new size of the string table */

  poffInfo->roDataSection.sh_size++;
  return offset;
}
#endif

/***********************************************************************/

uint32_t poffAddRoDataString(poffHandle_t handle, const char *string)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  uint32_t    len;
  uint32_t    offset;
  uint16_t    errCode;

  /* Check if we have allocated a data section buffer yet */

  errCode = poffCheckRoDataAlloc(poffInfo);
  if (errCode != eNOERROR)
    {
      fatal(errCode);
    }

  /* Check if there is room for a new byte */

  len     = strlen(string) + 1;
  errCode = poffCheckRoDataRealloc(poffInfo, len);

  /* Copy data section byte into the data section buffer */

  offset = poffInfo->roDataSection.sh_size;
  memcpy(&poffInfo->roDataSectionData[offset], string, len);

  /* Set the new size of the string table */

  poffInfo->roDataSection.sh_size += len;
  return offset;
}

/***********************************************************************/
