/**********************************************************************
 * pflineno.c
 * Manage line number information
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

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

#define INITIAL_LINENUMBER_TABLE_SIZE 2048*sizeof(poffLibLineNumber_t)
#define LINENUMBER_TABLE_INCREMENT    512*sizeof(poffLibLineNumber_t)

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

static poffLibLineNumber_t *lineNumberTable;
static uint32_t             nLineNumbers;
static uint32_t             lineNumberTableAlloc;
static uint32_t             prevLineNumberIndex;

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static inline void poffCheckLineNumberAllocation(void)
{
  /* Check if we have allocated a line number buffer yet */

  if (!lineNumberTable)
    {
      /* No, allocate it now */

      lineNumberTable = (poffLibLineNumber_t*)
        malloc(INITIAL_LINENUMBER_TABLE_SIZE);

      if (!lineNumberTable)
        {
          fatal(eNOMEMORY);
        }

      lineNumberTableAlloc = INITIAL_LINENUMBER_TABLE_SIZE;
      nLineNumbers         = 0;
    }
}

/***********************************************************************/

static inline void poffCheckLineNumberReallocation(void)
{
  if ((nLineNumbers +1) * sizeof(poffLibLineNumber_t) > lineNumberTableAlloc)
    {
      uint32_t newAlloc = lineNumberTableAlloc + LINENUMBER_TABLE_INCREMENT;
      void *tmp;

      /* Reallocate the line number buffer */

      tmp = realloc(lineNumberTable, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      lineNumberTableAlloc = newAlloc;
      lineNumberTable      = (poffLibLineNumber_t*)tmp;
    }
}

/***********************************************************************/
/* Add a line number to the line number table. */

static void poffAddLineNumberToTable(poffLibLineNumber_t *lineno)
{
  /* Verify that the line number table has been allocated */

  poffCheckLineNumberAllocation();

  /* Verify that the line number table is large enough to hold
   * information about another line.
   */

  poffCheckLineNumberReallocation();

  /* Save the line number information in the line number table */

  memcpy(&lineNumberTable[nLineNumbers], lineno, sizeof(poffLibLineNumber_t));
  nLineNumbers++;
}

/***********************************************************************/
/* Discard any unused memory */

static void poffDiscardUnusedAllocation(void)
{
  uint32_t newAlloc = nLineNumbers * sizeof(poffLibLineNumber_t);
  void *tmp;

  /* Was a line number table allocated? And, if so, are there unused
   * entries?
   */

  if ((lineNumberTable) && (nLineNumbers < lineNumberTableAlloc))
    {
      /* Reallocate the line number buffer */

      tmp = realloc(lineNumberTable, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      lineNumberTableAlloc = newAlloc;
      lineNumberTable      = (poffLibLineNumber_t*)tmp;
    }
}

/***********************************************************************/
/* "The comparison function must return an integer less than, equal to,
 * or greater than zero if the first argument is considered to be
 * respectively less than, equal to, or greater than the second. If two
 * members compare as equal, their order in the sorted array is undefined."
 */

static int poffCompareLineNumbers(const void *pv1, const void *pv2)
{
  register poffLibLineNumber_t *ln1 = (poffLibLineNumber_t*)pv1;
  register poffLibLineNumber_t *ln2 = (poffLibLineNumber_t*)pv2;

  if (ln1->offset < ln2->offset) return -1;
  else if (ln1->offset > ln2->offset) return 1;
  else return 0;
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

void poffReadLineNumberTable(poffHandle_t handle)
{
  poffLibLineNumber_t lineno;
  int32_t offset;

  /* Initialize global variables */

  prevLineNumberIndex = 0;

  /* Create a table of line number information */

  do
    {
      offset = poffGetLineNumber(handle, &lineno);
      if (offset >= 0)
        {
          poffAddLineNumberToTable(&lineno);
        }
    }
  while (offset >= 0);

  /* Discard any memory that is not being used */

  poffDiscardUnusedAllocation();

  /* Sort the table by offset */

  qsort(lineNumberTable, nLineNumbers,
        sizeof(poffLibLineNumber_t), poffCompareLineNumbers);
}

/***********************************************************************/

poffLibLineNumber_t *poffFindLineNumber(uint32_t offset)
{
  uint32_t firstLineNumberIndex;
  uint32_t lastLineNumberIndex;
  uint32_t lineNumberIndex;

  /* Was a line number table allocated? */

  if (!lineNumberTable) return NULL;

  /* We used the last returned line number entry as a hint to speed
   * up the next search.  We don't know how the line numbers will
   * be searched but most likely, they will be searched ina a sequence
   * of ascending offsets.  In that case, a dumb linear search
   * would be better than what we are trying to do here. we are
   * trying to support fast random access.
   */

  if (lineNumberTable[prevLineNumberIndex].offset <= offset)
    {
      firstLineNumberIndex = prevLineNumberIndex;
      lastLineNumberIndex  = nLineNumbers - 1;
    }
  else
    {
      firstLineNumberIndex = 0;
      lastLineNumberIndex  = prevLineNumberIndex;
    }

  /* Search until firstLineNumberIndex and firstLineNumberIndex+1
   * contain the searched for offset.  Exact matches may or may
   * not occur.
   */

  lineNumberIndex = firstLineNumberIndex;
  while (firstLineNumberIndex != lastLineNumberIndex)
    {
      /* Look at the midpoint index.  This will be biased toward
       * the lower index due to truncation.  This means that
       * can always be assured that as long as firstLineNumberIndex !=
       * lastLineNumberIndex, then lineNumberIndex+1 is valid.  We
       * exploit this fact below.
       */

      lineNumberIndex = (firstLineNumberIndex + lastLineNumberIndex) >> 1;

      /* If the offset at the midpoint is greater than the sought
       * for offset, then we can safely set the upper search index
       * to the midpoint.
       */
      
      if (lineNumberTable[lineNumberIndex].offset > offset)
        lastLineNumberIndex = lineNumberIndex;

      /* If we have an exact match, we break out of the loop now */

      else if (lineNumberTable[lineNumberIndex].offset == offset)
        break;

      /* If the next entry is an offset greater then the one we
       * are searching for, then we can break out of the loop now.
       * We know that lineNumberIndex+1 is a valid index (see above).
       */

      else if (lineNumberTable[lineNumberIndex + 1].offset > offset)
        break;

      /* Otherwise, we safely do the following */

      else
        firstLineNumberIndex = lineNumberIndex + 1;
    }

  /* Check that we terminated the loop with a valid line number
   * match.  This should only fail if all of the line numbers in the
   * table have offsets greater than the one in the table.  If we
   * could not find a match, return NULL.
   */

  if (lineNumberTable[lineNumberIndex].offset > offset)
    {
      prevLineNumberIndex = 0;
      return NULL;
    }
  else
    {
      prevLineNumberIndex = lineNumberIndex;
      return &lineNumberTable[lineNumberIndex];
    }
}

/***********************************************************************/

void poffReleaseLineNumberTable(void)
{
  if (lineNumberTable)
    free(lineNumberTable);

  lineNumberTable      = NULL;
  nLineNumbers         = 0;
  lineNumberTableAlloc = 0;
  prevLineNumberIndex  = 0;
}

/***********************************************************************/
