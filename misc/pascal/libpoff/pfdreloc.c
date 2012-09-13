/**********************************************************************
 * pfdreloc.c
 * Dump contents of a POFF file reloc table
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

/**********************************************************************
 * Private Constant Data
 **********************************************************************/

static const char *poffRelocationTypes[RLT_NTYPES] =
{
  "NULL",    /* Shouldn't happen */
  "PCAL",    /* Procedure/Function call */
  "LDST",    /* Load from stack base */
};

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * Public Functions
 ***********************************************************************/

void poffDumpRelocTable(poffHandle_t handle, FILE *outFile)
{
  poffInfo_t       *poffInfo = (poffInfo_t*)handle;
  poffRelocation_t *prel;
  uint32_t          index;

  fprintf(outFile, "\nPOFF Relocation Table:\n");
  fprintf(outFile, "RELO   SYMBOL     SECTION\n");
  fprintf(outFile, "TYPE   TBL INDEX  DATA OFFSET\n");

  for (index = 0;
       index < poffInfo->relocSection.sh_size;
       index += poffInfo->relocSection.sh_entsize)
    {
      prel = (poffRelocation_t*)&poffInfo->relocTable[index];

      fprintf(outFile, "%-6s 0x%08lx 0x%08lx\n",
              poffRelocationTypes[RLI_TYPE(prel->rl_info)],
              RLI_SYM(prel->rl_info),
              prel->rl_offset);
    }
}

/***********************************************************************/
