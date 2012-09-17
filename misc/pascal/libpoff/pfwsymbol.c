/**********************************************************************
 * pfwsymbol.c
 * Write symbol information to a POFF file
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

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************/
/* Add a symbol to the symbol table section data.  Returns index value
 * associated with the symbol entry in the symbol table section data.
 */

uint32_t poffAddSymbol(poffHandle_t handle, poffLibSymbol_t *symbol)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  poffSymbol_t *psym;
  uint32_t st_name;
  uint32_t index;

  /* Add the name to the string table */

  st_name = poffAddString(poffInfo, symbol->name);

  /* Check if we have allocated a symbol table buffer yet */

  if (!poffInfo->symbolTable)
    {
      /* No, allocate it now */

      poffInfo->symbolTable = (uint8_t*)malloc(INITIAL_SYMBOL_TABLE_SIZE);
      if (!poffInfo->symbolTable)
        {
          fatal(eNOMEMORY);
        }

      poffInfo->symbolTableSection.sh_size = 0;
      poffInfo->symbolTableAlloc           = INITIAL_SYMBOL_TABLE_SIZE;
    }

  /* Check if there is room for a new symbol */

  if (poffInfo->symbolTableSection.sh_size +
      poffInfo->symbolTableSection.sh_entsize >
      poffInfo->symbolTableAlloc)
    {
      uint32_t newAlloc = poffInfo->symbolTableAlloc + SYMBOL_TABLE_INCREMENT;
      uint8_t *tmp;

      /* Reallocate the file name buffer */

      tmp = (uint8_t*)realloc(poffInfo->symbolTable, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      poffInfo->symbolTableAlloc = newAlloc;
      poffInfo->symbolTable      = tmp;
    }

  /* Save the new symbol information in the symbol table data */

  index          = poffInfo->symbolTableSection.sh_size;
  psym           = (poffSymbol_t*)&poffInfo->symbolTable[index];

  psym->st_type  = symbol->type;
  psym->st_align = symbol->align;
  psym->st_flags = symbol->flags;
  psym->st_pad   = 0;
  psym->st_name  = st_name;
  psym->st_value = symbol->value;
  psym->st_size  = symbol->size;

  /* Set the new size of the file name table */

  poffInfo->symbolTableSection.sh_size += poffInfo->symbolTableSection.sh_entsize;
  return index / poffInfo->symbolTableSection.sh_entsize;
}

/***********************************************************************/
