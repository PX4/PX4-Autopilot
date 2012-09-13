/**********************************************************************
 * pftprog.c
 * Program data manipulations on POFF temporary object
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

#include "keywords.h"  /* Standard types */
#include "pedefs.h"    /* Pascal error codes */

#include "pfprivate.h" /* POFF private definitions */
#include "pofflib.h"   /* Public interfaces */

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

static uint16_t poffCheckProgAlloc(poffProgInfo_t *poffProgInfo)
{
  if (!poffProgInfo->progSectionData)
    {
      /* No, allocate it now */

      poffProgInfo->progSectionData = (uint8_t*)malloc(INITIAL_PROG_SECTION_SIZE);
      if (!poffProgInfo->progSectionData)
        {
          return eNOMEMORY;
        }

      poffProgInfo->progSectionSize  = 0;
      poffProgInfo->progSectionAlloc = INITIAL_PROG_SECTION_SIZE;
    }
  return eNOERROR;
}

static uint16_t poffCheckProgRealloc(poffProgInfo_t *poffProgInfo, uint16_t len)
{
  /* Check if there is room for the new data */

  if (poffProgInfo->progSectionSize + len > poffProgInfo->progSectionAlloc)
    {
      uint32_t newAlloc =
        poffProgInfo->progSectionAlloc + PROG_SECTION_INCREMENT;
      void *tmp;

      /* Make certain that this is big enough (it should be) */

      while (poffProgInfo->progSectionSize + len > newAlloc)
        {
          newAlloc += PROG_SECTION_INCREMENT;
        }

      /* Reallocate the program data section buffer */

      tmp = realloc(poffProgInfo->progSectionData, newAlloc);
      if (!tmp)
        {
          return eNOMEMORY;
        }

      /* And set the new size */

      poffProgInfo->progSectionAlloc = newAlloc;
      poffProgInfo->progSectionData  = (uint8_t*)tmp;
    }
  return eNOERROR;
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

uint16_t poffAddTmpProgByte(poffProgHandle_t handle, uint8_t progByte)
{
  poffProgInfo_t *poffProgInfo = (poffProgInfo_t*)handle;
  uint16_t errCode;

  /* Check if we have allocated a program section buffer yet */

  errCode = poffCheckProgAlloc(poffProgInfo);
  if (errCode != eNOERROR)
    {
      return errCode;
    }

  /* Check if there is room for the new byte */

  errCode = poffCheckProgRealloc(poffProgInfo, 1);
  if (errCode != eNOERROR)
    {
      return errCode;
    }

  /* Copy program data byte into the program data buffer */

  poffProgInfo->progSectionData[poffProgInfo->progSectionSize] = progByte;

  /* Set the new size of the string table */

  poffProgInfo->progSectionSize++;
  return eNOERROR;
}

/***********************************************************************/

uint16_t poffWriteTmpProgBytes(uint8_t *buffer, uint32_t nbytes,
                             poffProgHandle_t handle)

{
  poffProgInfo_t *poffProgInfo = (poffProgInfo_t*)handle;
  uint16_t errCode;

  /* Check if we have allocated a program section buffer yet */

  errCode = poffCheckProgAlloc(poffProgInfo);
  if (errCode != eNOERROR)
    {
      return errCode;
    }

  /* Check if there is room for the new data */

  errCode = poffCheckProgRealloc(poffProgInfo, nbytes);
  if (errCode != eNOERROR)
    {
      return errCode;
    }

  /* Copy program data byte into the program data buffer */

  memcpy(&poffProgInfo->progSectionData[poffProgInfo->progSectionSize],
         buffer, nbytes);

  /* Set the new size of the string table */

  poffProgInfo->progSectionSize += nbytes;
  return eNOERROR;
}

/***********************************************************************/

void poffReplaceProgData(poffHandle_t handle, poffProgHandle_t progHandle)
{
  poffInfo_t     *poffInfo     = (poffInfo_t*)handle;
  poffProgInfo_t *poffProgInfo = (poffProgInfo_t*)progHandle;

  /* Discard any existing program section data */

  if (poffInfo->progSectionData)
    {
      free(poffInfo->progSectionData);
    }

  /* Replace the program section data with the tmp data */

  poffInfo->progSectionData     = poffProgInfo->progSectionData;
  poffInfo->progSection.sh_size = poffProgInfo->progSectionSize;
  poffInfo->progSectionAlloc    = poffProgInfo->progSectionAlloc;

  /* Reset the read index */

  poffInfo->progSectionIndex    = 0;

  /* Then nullify the tmp data */

  poffProgInfo->progSectionData  = NULL;
  poffProgInfo->progSectionSize  = 0;
  poffProgInfo->progSectionAlloc = 0;
}

/***********************************************************************/
