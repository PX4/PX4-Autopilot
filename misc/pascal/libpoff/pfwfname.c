/**********************************************************************
 * pfwfname.c
 * Write filename data to a POFF file
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
/* Add a file name to the file table.  Returns file number index value
 * and NOT the byte offset associated with the file name entry in the
 * file name section.
 */

uint32_t poffAddFileName(poffHandle_t handle, const char *name)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  poffFileTab_t ft;
  uint32_t index;

  /* Add the name to the string table */

  ft = (poffFileTab_t)poffAddString(poffInfo, name);

  /* Add string table offset to the file table
   *
   * Check if we have allocated a filename table buffer yet
   */

  if (!poffInfo->fileNameTable)
    {
      /* No, allocate it now */

      poffInfo->fileNameTable = (poffFileTab_t*)malloc(INITIAL_FILENAME_TABLE_SIZE);
      if (!poffInfo->fileNameTable)
        {
          fatal(eNOMEMORY);
        }

      poffInfo->fileNameTableSection.sh_size = 0;
      poffInfo->fileNameTableAlloc           = INITIAL_FILENAME_TABLE_SIZE;
    }

  /* Check if there is room for the new file name index */

  if (poffInfo->fileNameTableSection.sh_size + sizeof(poffFileTab_t)>
      poffInfo->fileNameTableAlloc)
    {
      uint32_t newAlloc = poffInfo->fileNameTableAlloc + FILENAME_TABLE_INCREMENT;
      void *tmp;

      /* Reallocate the file name buffer */

      tmp = realloc(poffInfo->fileNameTable, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      poffInfo->fileNameTableAlloc = newAlloc;
      poffInfo->fileNameTable      = (poffFileTab_t*)tmp;
    }

  /* Save the index in the file name table */

  index = poffInfo->fileNameTableSection.sh_size / sizeof(poffFileTab_t);
  poffInfo->fileNameTable[index] = ft;

  /* Set the new size of the file name table */

  poffInfo->fileNameTableSection.sh_size += sizeof(poffFileTab_t);
  return index;
}

/***********************************************************************/
