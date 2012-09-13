/**********************************************************************
 * pfhandle.c
 * POFF library global variables
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "keywords.h"  /* Standard types */
#include "pfprivate.h" /* POFF private definitions */
#include "pofflib.h"   /* Public interfaces */

/**********************************************************************
 * Definitions
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
/* Set all global data structures to a known state */

poffHandle_t poffCreateHandle(void)
{
  poffInfo_t *poffInfo;

  /* Create a new POFF handle */

  poffInfo = (poffInfo_t*)malloc(sizeof(poffInfo_t));
  if (poffInfo != NULL)
    {
      /* Set everthing to zero */

      memset(poffInfo, 0, sizeof(poffInfo_t));

      /* Initialize POFF file header */

      poffInfo->fileHeader.fh_ident[FHI_MAG0] = FHI_POFF_MAG0;
      poffInfo->fileHeader.fh_ident[FHI_MAG1] = FHI_POFF_MAG1;
      poffInfo->fileHeader.fh_ident[FHI_MAG2] = FHI_POFF_MAG2;
      poffInfo->fileHeader.fh_ident[FHI_MAG3] = FHI_POFF_MAG3;
      poffInfo->fileHeader.fh_version         = FHV_CURRENT;
      poffInfo->fileHeader.fh_shsize          = sizeof(poffSectionHeader_t);
      poffInfo->fileHeader.fh_shoff           = sizeof(poffFileHeader_t);

      /* Initialize the program section header */

      poffInfo->progSection.sh_type           = SHT_PROGDATA;
      poffInfo->progSection.sh_flags          = SHF_ALLOC | SHF_EXEC;

      /* Initialize the initialized data section header */

      poffInfo->roDataSection.sh_type         = SHT_PROGDATA;
      poffInfo->roDataSection.sh_flags        = SHF_WRITE | SHF_ALLOC;

      /* Initialize the symbol table section header */

      poffInfo->symbolTableSection.sh_type    = SHT_SYMTAB;
      poffInfo->symbolTableSection.sh_entsize = sizeof(poffSymbol_t);

      /* Initialize the string table section header */

      poffInfo->stringTableSection.sh_type    = SHT_STRTAB;

      /* Initialize the relocation table section header */

      poffInfo->relocSection.sh_type          = SHT_REL;
      poffInfo->relocSection.sh_entsize       = sizeof(poffRelocation_t);

      /* Initialize the file table section header */

      poffInfo->fileNameTableSection.sh_type    = SHT_FILETAB;
      poffInfo->fileNameTableSection.sh_entsize = sizeof(poffFileTab_t);

      /* Initialize the line number section header */

      poffInfo->lineNumberSection.sh_type       = SHT_LINENO;
      poffInfo->lineNumberSection.sh_entsize    = sizeof(poffLineNumber_t);

      /* Initialize the debug function info section header */

      poffInfo->debugFuncSection.sh_type        = SHT_DEBUG;
      poffInfo->debugFuncSection.sh_entsize     = sizeof(poffDebugFuncInfo_t);
    }
  return poffInfo;
}

/***********************************************************************/

void poffDestroyHandle(poffHandle_t handle)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;

  /* Free all of the allocated, in-memory data */

  if (poffInfo->progSectionData)
    free(poffInfo->progSectionData);

  if (poffInfo->roDataSectionData)
    free(poffInfo->roDataSectionData);

  if (poffInfo->symbolTable)
    free(poffInfo->symbolTable);

  if (poffInfo->stringTable)
    free(poffInfo->stringTable);

  if (poffInfo->relocTable)
    free(poffInfo->relocTable);

  if (poffInfo->fileNameTable)
    free(poffInfo->fileNameTable);

  if (poffInfo->lineNumberTable)
    free(poffInfo->lineNumberTable);

  if (poffInfo->debugFuncTable)
    free(poffInfo->debugFuncTable);

  /* Free the container */

  free(handle);
}

/***********************************************************************/

void poffResetAccess(poffHandle_t handle)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;

  /* Reset read/write indices */

  poffInfo->symbolIndex      = 0;
  poffInfo->progSectionIndex = 0;
  poffInfo->relocIndex       = 0;
  poffInfo->fileNameIndex    = 0;
  poffInfo->lineNumberIndex  = 0;
  poffInfo->debugFuncIndex   = 0;
}

/***********************************************************************/

