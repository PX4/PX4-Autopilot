/**********************************************************************
 * plreloc.c
 * Relocation management for the P-Code Linker
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

#include "keywords.h"
#include "pdefs.h"
#include "podefs.h"
#include "pedefs.h"

#include "pofflib.h"
#include "paslib.h"
#include "pinsn.h"
#include "perr.h"
#include "plsym.h"
#include "plreloc.h"

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

#define INITIAL_RELOC_LIST_SIZE (1024*sizeof(poffRelocation_t*))
#define RELOC_LIST_INCREMENT    (256*sizeof(poffRelocation_t*))

/**********************************************************************
 * Private Types
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

static poffRelocation_t *relocList      = NULL;
static uint32_t          relocListAlloc = 0;
static uint32_t          nRelocs        = 0;

/**********************************************************************
 * Private Function Prototypes

 **********************************************************************/

static void offsetRelocation(poffRelocation_t *reloc,
                             uint32_t pcOffset, uint32_t symOffset);
static void addRelocToList(poffRelocation_t *reloc);

/**********************************************************************
 * Public Functions
 **********************************************************************/

void mergeRelocations(poffHandle_t inHandle,
                      uint32_t pcOffset, uint32_t symOffset)
{
  poffRelocation_t reloc;
  int32_t index;

  do
    {
      /* Read each relocation record from the input File */

      index = poffGetRawRelocation(inHandle, &reloc);
      if (index >= 0)
        {
          /* If the rellocation carries a "payload" that is a program
           * section offset, then apply the pcOffset value to
           * that "payload"
           */

          offsetRelocation(&reloc, pcOffset, symOffset);

          /* Add the relocation to the in-memory relocation list */

          addRelocToList(&reloc);
        }
    }
  while (index >= 0);
}

/***********************************************************************/

void applyRelocations(poffHandle_t outHandle)
{
  uint8_t *progData;
  uint32_t progSize;
  int    i;

  /* Take ownership of the program data image for a little while */

  progSize = poffExtractProgramData(outHandle, &progData);

  /* Process each text data section reloation */

  for (i = 0; i < nRelocs; i++)
    {
      poffRelocation_t *reloc = &relocList[i];
      uint32_t          symIndex = RLI_SYM(reloc->rl_info);
      uint32_t          relType  = RLI_TYPE(reloc->rl_info);
      poffLibSymbol_t  *sym;
      uint32_t          progIndex;

      switch (relType)
        {
        case RLT_PCAL:
          /* Get the symbol referenced by the relocation.  At this
           * point, we assume that the system has already verified
           * that there are no undefined symbols.
           */

          sym = getSymbolByIndex(symIndex);

          /* Get the index to the oPCAL instruction */

          progIndex = reloc->rl_offset;

          /* Sanity checking */

          if (((sym->flags & STF_UNDEFINED) != 0) ||
              (progIndex > progSize-4))
            fatal(ePOFFCONFUSION);

          /* Perform the relocation */

          insn_FixupProcedureCall(&progData[progIndex], sym->value);
          break;

        default:
          break;
        }
    }


  /* Return ownership of the program data to the container */

  poffInsertProgramData(outHandle, progData, progSize);
}

/***********************************************************************/

void releaseRelocations(void)
{
  if (relocList) free(relocList);
  relocList = NULL;
}

/**********************************************************************
 * Private Functions
 **********************************************************************/

static void offsetRelocation(poffRelocation_t *reloc,
                             uint32_t pcOffset, uint32_t symOffset)
{
  uint32_t symIndex = RLI_SYM(reloc->rl_info);
  uint32_t relType  = RLI_TYPE(reloc->rl_info);

  switch (relType)
    {
    case RLT_PCAL:
      symIndex         += symOffset;
      reloc->rl_info    = RLI_MAKE(symIndex, relType);
      reloc->rl_offset += pcOffset;
      break;

    default:
      break;
    }
}

/***********************************************************************/
/* Add to the linear relocation list. */

static void addRelocToList(poffRelocation_t *reloc)
{
  /* Check if we have allocated a relocation buffer yet */

  if (!relocList)
    {
      /* No, allocate it now */

      relocList = (poffRelocation_t*)malloc(INITIAL_RELOC_LIST_SIZE);
      if (!relocList)
        {
          fatal(eNOMEMORY);
        }
      relocListAlloc = INITIAL_RELOC_LIST_SIZE;
    }

  /* Check if there is room for a new symbol */

  if ((nRelocs + 1) * sizeof(poffRelocation_t) > relocListAlloc)
    {
      uint32_t newAlloc = relocListAlloc + RELOC_LIST_INCREMENT;
      poffRelocation_t *tmp;

      /* Reallocate the file name buffer */

      tmp = (poffRelocation_t*)realloc(relocList, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      relocListAlloc = newAlloc;
      relocList      = tmp;
    }

  /* Save the new symbol information in the relocation data */

  relocList[nRelocs] = *reloc;
  nRelocs++;
}

/***********************************************************************/

