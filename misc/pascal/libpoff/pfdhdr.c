/**********************************************************************
 * pfdhdr.c
 * Dump the contents of POFF file and section headers
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

/**********************************************************************
 * Private Constant Data
 **********************************************************************/

static const char *poffFhTypes[FHT_NTYPES] =
{
  "NONE    ",  /* Shouldn't happen */
  "EXEC    ",  /* Pascal program executable */
  "SHLIB   ",  /* Pascal shared library */
  "PROGRAM ",  /* Pascal program object */
  "UNIT    "   /* Pascal unit object */
};

static const char *poffShTypes[SHT_NTYPES] =
{
  "NULL    ",  /* Shouldn't happen */
  "PROGDATA",  /* Program data */
  "SYMTAB  ",  /* Symbol table */
  "STRTAB  ",  /* String table */
  "REL     ",  /* Relocation data */
  "FILETAB ",  /* File table */
  "LINENO  ",  /* Line number data */
  "DEBUG   "   /* Func/Proc debug data */
};

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static void poffDumpSectionHeader(poffHandle_t handle,
				  poffSectionHeader_t *shdr,
				  FILE *outFile)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;

  fprintf(outFile, "%-10s %8s 0x%02x 0x%04x 0x%08lx 0x%08lx %ld\n",
	  poffGetString(poffInfo, shdr->sh_name),
	  poffShTypes[shdr->sh_type], shdr->sh_flags,
	  shdr->sh_entsize, shdr->sh_addr, shdr->sh_offset,
	  shdr->sh_size);
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************/

void poffDumpFileHeader(poffHandle_t handle, FILE *outFile)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;

  fprintf(outFile, "\nPOFF File Header:\n");
  fprintf(outFile, "  fh_ident:    %c%c%c%c\n",
	  poffInfo->fileHeader.fh_ident[0], poffInfo->fileHeader.fh_ident[1],
	  poffInfo->fileHeader.fh_ident[2], poffInfo->fileHeader.fh_ident[3]);
  fprintf(outFile, "  fh_version:  %d\n", poffInfo->fileHeader.fh_version);
  fprintf(outFile, "  fh_type:     %s\n",
	  poffFhTypes[poffInfo->fileHeader.fh_type]);
  fprintf(outFile, "  fh_shsize:   %d\n", poffInfo->fileHeader.fh_shsize);
  fprintf(outFile, "  fh_shnum:    %d\n", poffInfo->fileHeader.fh_shnum);
  fprintf(outFile, "  fh_name:     %s\n",
	  poffGetString(poffInfo, poffInfo->fileHeader.fh_name));
  fprintf(outFile, "  fh_entry:    0x%08lx\n", poffInfo->fileHeader.fh_entry);
  fprintf(outFile, "  fh_shoff:    0x%08lx\n", poffInfo->fileHeader.fh_shoff);
}

/***********************************************************************/

void poffDumpSectionHeaders(poffHandle_t handle, FILE *outFile)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;

  fprintf(outFile, "\nPOFF Section Headers:\n");
  fprintf(outFile, "NAME       TYPE     FLAG ENTSZE ADDRESS    OFFSET     SIZE\n");

  if (poffInfo->progSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->progSection, outFile);
    }

  if (poffInfo->roDataSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->roDataSection, outFile);
    }

  if (poffInfo->symbolTableSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->symbolTableSection, outFile);
    }

  if (poffInfo->stringTableSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->stringTableSection, outFile);
    }

  if (poffInfo->relocSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->relocSection, outFile);
    }

  if (poffInfo->fileNameTableSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->fileNameTableSection, outFile);
    }

  if (poffInfo->lineNumberSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->lineNumberSection, outFile);
    }

  if (poffInfo->debugFuncSection.sh_size > 0)
    {
      poffDumpSectionHeader(handle, &poffInfo->debugFuncSection, outFile);
    }
}

/***********************************************************************/
