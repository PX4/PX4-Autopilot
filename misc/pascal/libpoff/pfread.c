/**********************************************************************
 * pfread.c
 * POFF library global variables
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

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "keywords.h"  /* Standard types */
#include "pedefs.h"    /* Pascal error definitions */

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

static uint16_t poffReadFileHeader(poffHandle_t handle, FILE *poffFile);
static uint16_t poffReadSectionHeaders(poffHandle_t handle, FILE *poffFile);
static uint16_t poffReadSectionData(poffSectionHeader_t *shdr,
                                    uint8_t **sdata, FILE *poffFile);
static uint16_t poffReadAllSectionData(poffHandle_t handle, FILE *poffFile);

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************/
/* Read and verify the POFF file header */

static uint16_t poffReadFileHeader(poffHandle_t handle, FILE *poffFile)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  size_t entriesRead;

  /* Seek to the beginning of the file */

  if (fseek(poffFile, 0, SEEK_SET) != 0)
    {
      return ePOFFREADERROR;
    }

  /* Read the POFF file header */

  entriesRead = fread(&poffInfo->fileHeader, sizeof(poffFileHeader_t),
                      1, poffFile);
  if (entriesRead != 1)
    {
      return ePOFFREADERROR;
    }

  /* The POFF file is retained in big-endian order.  Fixup fields as necessary */

  poffSwapFileHeader(&poffInfo->fileHeader);

  /* Verify that this is a valid POFF header */

  if ((poffInfo->fileHeader.fh_ident[FHI_MAG0] != FHI_POFF_MAG0) ||
      (poffInfo->fileHeader.fh_ident[FHI_MAG1] != FHI_POFF_MAG1) ||
      (poffInfo->fileHeader.fh_ident[FHI_MAG2] != FHI_POFF_MAG2) ||
      (poffInfo->fileHeader.fh_ident[FHI_MAG3] != FHI_POFF_MAG3) ||
      (poffInfo->fileHeader.fh_version         != FHV_CURRENT))
    {
      return ePOFFBADFORMAT;
    }
  return eNOERROR;
}

/***********************************************************************/
/* Read and verify all of the POFF section headers */

static uint16_t poffReadSectionHeaders(poffHandle_t handle, FILE *poffFile)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  poffSectionHeader_t sectionHeader;
  poffSectionHeader_t *dest;
  long offset;
  size_t entriesRead;
  int i;

  offset = poffInfo->fileHeader.fh_shoff;

  for (i = 0; i < poffInfo->fileHeader.fh_shnum; i++)
    {
      /* Seek to the beginning of the next section header */

      if (fseek(poffFile, offset, SEEK_SET) != 0)
        {
          return ePOFFREADERROR;
        }

      /* Read the section header */

      entriesRead = fread(&sectionHeader, sizeof(poffSectionHeader_t),
                          1, poffFile);
      if (entriesRead != 1)
        {
          return ePOFFREADERROR;
        }

      /* The POFF file is retained in big-endian order.  Fixup fields as necessary */

      poffSwapSectionHeader(&sectionHeader);

      /* Copy the section header to the correct location */

      switch (sectionHeader.sh_type)
        {
        case SHT_PROGDATA : /* Program data */
          if ((sectionHeader.sh_flags & SHF_EXEC) != 0)
            dest = &poffInfo->progSection;
          else
            dest = &poffInfo->roDataSection;
          break;

        case SHT_SYMTAB : /* Symbol table */
          dest = &poffInfo->symbolTableSection;
          break;

        case SHT_STRTAB :  /* String table */
          dest = &poffInfo->stringTableSection;
          break;

        case SHT_REL : /* Relocation data */
          dest = &poffInfo->relocSection;
          break;

        case SHT_FILETAB : /* File table */
          dest = &poffInfo->fileNameTableSection;
          break;

        case SHT_LINENO :  /* Line number data */
          dest = &poffInfo->lineNumberSection;
          break;

        case SHT_DEBUG :  /* Debug function info data */
          dest = &poffInfo->debugFuncSection;
          break;

        default:
          return ePOFFREADERROR;
        }
      memcpy(dest, &sectionHeader, sizeof(poffSectionHeader_t));

      /* Get the offset to the next section */

      offset += poffInfo->fileHeader.fh_shsize;
    }
  return eNOERROR;
}

/***********************************************************************/
/* Read and buffer all of the POFF section data */

static uint16_t poffReadSectionData(poffSectionHeader_t *shdr,
                                    uint8_t **sdata, FILE *poffFile)
{
  size_t entriesRead;

  /* Seek to the beginning of the section data */

  if (fseek(poffFile, shdr->sh_offset, SEEK_SET) != 0)
    {
      return ePOFFREADERROR;
    }

  /* Allocate memory to hold the section data */

  *sdata = (uint8_t*)malloc(shdr->sh_size);
  if (*sdata == NULL)
    {
      return eNOMEMORY;
    }

  /* Read the section data */

  entriesRead = fread(*sdata, 1, shdr->sh_size, poffFile);
  if (entriesRead != shdr->sh_size)
    {
      return ePOFFREADERROR;
    }
  return eNOERROR;
}

/***********************************************************************/
/* Read and buffer all of the POFF section data */

static uint16_t poffReadAllSectionData(poffHandle_t handle, FILE *poffFile)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  uint16_t retval = eNOERROR;

  if (HAVE_PROGRAM_SECTION)
    {
      retval = poffReadSectionData(&poffInfo->progSection,
                                   (uint8_t**)&poffInfo->progSectionData,
                                   poffFile);
    }

  if ((retval == eNOERROR) && (HAVE_RODATA_SECTION))
    {
      retval = poffReadSectionData(&poffInfo->roDataSection,
                                   (uint8_t**)&poffInfo->roDataSectionData,
                                   poffFile);
    }

  if ((retval == eNOERROR) && (HAVE_SYMBOL_TABLE))
    {
      retval = poffReadSectionData(&poffInfo->symbolTableSection,
                                   (uint8_t**)&poffInfo->symbolTable,
                                   poffFile);
#ifdef CONFIG_POFF_SWAPNEEDED
      if (retval == eNOERROR)
        {
          poffSwapSymbolTableData(poffInfo);
        }
#endif
    }

  if ((retval == eNOERROR) && (HAVE_STRING_TABLE))
    {
      retval = poffReadSectionData(&poffInfo->stringTableSection,
                                   (uint8_t**)&poffInfo->stringTable,
                                   poffFile);
    }

  if ((retval == eNOERROR) && (HAVE_RELOC_SECTION))
    {
      retval = poffReadSectionData(&poffInfo->relocSection,
                                   (uint8_t**)&poffInfo->relocTable,
                                   poffFile);
#ifdef CONFIG_POFF_SWAPNEEDED
      if (retval == eNOERROR)
        {
           poffSwapRelocationData(poffInfo);
        }
#endif
    }

  if ((retval == eNOERROR) && (HAVE_FILE_TABLE))
    {
      retval = poffReadSectionData(&poffInfo->fileNameTableSection,
                                   (uint8_t**)&poffInfo->fileNameTable,
                                   poffFile);
#ifdef CONFIG_POFF_SWAPNEEDED
      if (retval == eNOERROR)
        {
          poffSwapFileTableData(poffInfo);
        }
#endif
    }

  if ((retval == eNOERROR) && (HAVE_LINE_NUMBER))
    {
      retval = poffReadSectionData(&poffInfo->lineNumberSection,
                                   (uint8_t**)&poffInfo->lineNumberTable,
                                   poffFile);
#ifdef CONFIG_POFF_SWAPNEEDED
      if (retval == eNOERROR)
        {
          poffSwapLineNumberData(poffInfo);
        }
#endif
    }

  if ((retval == eNOERROR) && (HAVE_DEBUG_SECTION))
    {
      retval = poffReadSectionData(&poffInfo->debugFuncSection,
                                   (uint8_t**)&poffInfo->debugFuncTable,
                                   poffFile);
#ifdef CONFIG_POFF_SWAPNEEDED
      if (retval == eNOERROR)
        {
          poffSwapDebugData(poffInfo);
        }
#endif
    }

  return retval;
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************/
/* Set all global data structures to a known state */

uint16_t poffReadFile(poffHandle_t handle, FILE *poffFile)
{
  uint16_t retVal;

  /* Read the POFF header file */

  retVal = poffReadFileHeader(handle, poffFile);
  if (retVal == eNOERROR)
    {
      retVal = poffReadSectionHeaders(handle, poffFile);
      if (retVal == eNOERROR)
        {
          retVal = poffReadAllSectionData(handle, poffFile);
        }
    }
  return retVal;
}

/***********************************************************************/
