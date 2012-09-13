/**********************************************************************
 * pfrfname.c
 * Read file name data from a POFF file
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

int32_t poffGetFileName(poffHandle_t handle, const char **fname)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  uint32_t    stringTableIndex;
  uint32_t    fileNameIndex;

  /* First, check if there is another file name in the table to be had.
   * This check is a little sloppy in that it assumes the the size in
   * in the header is an even multiple of file name table entries
   */

  fileNameIndex = poffInfo->fileNameIndex;
  if (fileNameIndex * sizeof(poffFileTab_t) >=
      poffInfo->fileNameTableSection.sh_size)
    {
      /* Return -1 to signal the end of the list */

      *fname = NULL;
      return -1;
    }
  else
    {
      /* Get the string table index from the file name table */

      stringTableIndex = (uint32_t)poffInfo->fileNameTable[fileNameIndex];

      /* Return the file name */

      *fname = poffGetString(handle, stringTableIndex);

      /* Set up for the next read */

      poffInfo->fileNameIndex++;
      return fileNameIndex;
    }
}

/***********************************************************************/
