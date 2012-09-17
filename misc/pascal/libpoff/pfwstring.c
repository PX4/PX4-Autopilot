/**********************************************************************
 * pfwstring.c
 * Write string table data a POFF file
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
/* Search the string table for an occurrence of the indicates string.
 * If found, return the offset in the string table to the string; if
 * not found, return -1.
 */

int32_t poffFindString(poffHandle_t handle, const char *string)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  char       *sptr     = poffInfo->stringTable;
  uint32_t    offset;

  /* Has the string table been allocated yet? */

  if (!poffInfo->stringTable) return -1;

  /* Handle the NULL string case.  Offset zero is reserved for the NULL
   * string and for the zero-size string.
   */

  if (!string) return 0;

  /* Okay, we will have to search the table */

  for (offset = 0;
       offset < poffInfo->stringTableSection.sh_size;
       offset += (strlen(sptr) + 1))
    {
      /* Get a pointer at this offset into the string table */

      sptr = &poffInfo->stringTable[offset];

      /* Check if the strings match.  If so, return the offset */

      if (strcmp(sptr, string) == 0) return offset;
    }

  /* The string does not exist in the string table */

  return -1;
}

/***********************************************************************/
/***********************************************************************/
/* Add a string to the string table and return the offset to the
 * string storage location in the string table section data.
 */

uint32_t poffAddString(poffHandle_t handle, const char *string)
{
  poffInfo_t *poffInfo = (poffInfo_t*)handle;
  int32_t index;
  int len;

  /* Check if we have allocated a string table buffer yet */

  if (!poffInfo->stringTable)
    {
      /* No, allocate it now */

      poffInfo->stringTable = (char*)malloc(INITIAL_STRING_TABLE_SIZE);
      if (!poffInfo->stringTable)
        {
          fatal(eNOMEMORY);
        }

      /* Index 0 is reserved for the NULL string */

      poffInfo->stringTable[0]             = '\0';
      poffInfo->stringTableSection.sh_size = 1;
      poffInfo->stringTableAlloc           = INITIAL_STRING_TABLE_SIZE;
    }

  /* Check if the string is already defined in the string table.
   * This is very time consuming, but guaratees that we do not keep
   * duplicate strings in the string table.
   */

  index = poffFindString(handle, string);
  if (index < 0)
    {
      /* The string was not found in the string table.  Check for the
       * NULL string.  In this case, return index == 0.  NOTE:  This
       * check is pointless.  If this is any kind of NULL string, then
       * poffFindString will have returned index == 0.
       */

      if ((string == NULL) || ((len = strlen(string)) <= 0))
        {
          index = 0;
        }
      else
        {
          /* Increment the length to include the null terminator */

          len++;

          /* Check if there is room for the new string */

          if (poffInfo->stringTableSection.sh_size + len >
              poffInfo->stringTableAlloc)
            {
              uint32_t newAlloc =
                poffInfo->stringTableAlloc + STRING_TABLE_INCREMENT;
              void *tmp;

              /* Make sure that the new string will fit in the new allocation
               * size (shouldn't happen)
               */

              while (poffInfo->stringTableSection.sh_size + len > newAlloc)
                newAlloc += STRING_TABLE_INCREMENT;

              /* Reallocate the string buffer */

              tmp = realloc(poffInfo->stringTable, newAlloc);
              if (!tmp)
                {
                  fatal(eNOMEMORY);
                }

              /* And set the new size */

              poffInfo->stringTableAlloc = newAlloc;
              poffInfo->stringTable      = (char*)tmp;
            }

          /* Copy the string into the string table */

          index = poffInfo->stringTableSection.sh_size;
          memcpy(&poffInfo->stringTable[index], string, len);

          /* Set the new size of the string table */

          poffInfo->stringTableSection.sh_size += len;
        }
    }
  return index;
}

/***********************************************************************/
