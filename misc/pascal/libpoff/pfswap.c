/**********************************************************************
 * libpoff/pfswap.c
 * Handle POFF Endian-ness
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

#include "keywords.h"  /* Standard types */
#include "paslib.h"    /* Common library */
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

#ifdef CONFIG_POFF_SWAPNEEDED
static inline void poffSwapRelocationEntry(poffRelocation_t *prel)
{
  prel->rl_info   = poff32(prel->rl_info);
  prel->rl_offset = poff32(prel->rl_offset);
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
static inline void poffSwapSymbolTableEntry(poffSymbol_t *psym)
{
  psym->st_name  = poff32(psym->st_name);
  psym->st_value = poff32(psym->st_value);
  psym->st_size  = poff32(psym->st_size);
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
static inline void poffSwapFileTabEntry(poffFileTab_t *pfile)
{
  *pfile = poff32((uint32_t)*pfile);
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
static inline void poffSwaplineno(poffLineNumber_t *plineno)
{
  plineno->ln_lineno  = poff16(plineno->ln_lineno);
  plineno->ln_fileno  = poff16(plineno->ln_fileno);
  plineno->ln_poffset = poff32(plineno->ln_poffset);
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
static inline void poffDebugFuncEntry(poffDebugFuncInfo_t *pdbg)
{
  pdbg->df_value   = poff32(pdbg->df_value);
  pdbg->df_size    = poff32(pdbg->df_size);
  pdbg->df_nparms  = poff32(pdbg->df_nparms);
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
static inline void poffDebugArgEntry(poffDebugArgInfo_t *parg)
{
  parg->da_size = poff32(parg->da_size);
}
#endif

/***********************************************************************
 * Public Functions
 ***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
void poffSwapFileHeader(poffFileHeader_t *pFileHeader)
{
  pFileHeader->fh_shsize = poff16(pFileHeader->fh_shsize);
  pFileHeader->fh_shnum  = poff16(pFileHeader->fh_shnum);
  pFileHeader->fh_name   = poff32(pFileHeader->fh_name);
  pFileHeader->fh_entry  = poff32(pFileHeader->fh_entry);
  pFileHeader->fh_shoff  = poff32(pFileHeader->fh_shoff);
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
void poffSwapSectionHeader(poffSectionHeader_t *pSectionHeader)
{
  pSectionHeader->sh_entsize = poff16(pSectionHeader->sh_entsize);
  pSectionHeader->sh_name    = poff32(pSectionHeader->sh_name);
  pSectionHeader->sh_addr    = poff32(pSectionHeader->sh_addr);
  pSectionHeader->sh_offset  = poff32(pSectionHeader->sh_offset);
  pSectionHeader->sh_size    = poff32(pSectionHeader->sh_size);
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
void poffSwapSymbolTableData(poffInfo_t *poffInfo)
{
  poffSymbol_t *psym;
  uint32_t      index;

  for (index = 0;
       index < poffInfo->symbolTableSection.sh_size;
       index += poffInfo->symbolTableSection.sh_entsize)
    {
      psym = (poffSymbol_t*)&poffInfo->symbolTable[index];
      poffSwapSymbolTableEntry(psym);
    }
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
void poffSwapRelocationData(poffInfo_t *poffInfo)
{
  poffRelocation_t *prel;
  uint32_t          index;

  for (index = 0;
       index < poffInfo->relocSection.sh_size;
       index += poffInfo->relocSection.sh_entsize)
    {
      prel = (poffRelocation_t*)&poffInfo->relocTable[index];
      poffSwapRelocationEntry(prel);
    }
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
void poffSwapFileTableData(poffInfo_t *poffInfo)
{
  poffFileTab_t *pfile;
  uint32_t       index;

  for (index = 0;
       index < poffInfo->relocSection.sh_size;
       index += poffInfo->relocSection.sh_entsize)
    {
      pfile = (poffFileTab_t*)&poffInfo->relocTable[index];
      poffSwapFileTabEntry(pfile);
    }
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
void poffSwapLineNumberData(poffInfo_t *poffInfo)
{
  poffLineNumber_t *plineno;
  uint32_t          index;

  for (index = 0;
       index < poffInfo->relocSection.sh_size;
       index += poffInfo->relocSection.sh_entsize)
    {
      plineno = (poffLineNumber_t*)&poffInfo->relocTable[index];
      poffSwaplineno(plineno);
    }
}
#endif

/***********************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
void poffSwapDebugData(poffInfo_t *poffInfo)
{
  poffDebugFuncInfo_t    *pdbg;
  poffDebugArgInfo_t     *parg;
  uint32_t                i;
  int                     j;

  for (i = 0; i + sizeof(poffDebugFuncInfo_t) < poffInfo->relocSection.sh_size;)
    {
      pdbg = (poffDebugFuncInfo_t*)&poffInfo->debugFuncTable[i];
      poffDebugFuncEntry(pdbg);

      i += sizeof(poffDebugFuncInfo_t);
      for (j = 0; j < pdbg->df_nparms; j++)
        {
          parg = (poffDebugArgInfo_t*)&poffInfo->debugFuncTable[i];
          poffDebugArgEntry(parg);
          i += sizeof(poffDebugArgInfo_t);
        }
    }
}
#endif

/***********************************************************************/
