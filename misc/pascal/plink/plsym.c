/**********************************************************************
 * plsym.c
 * Symbol management for the P-Code Linker
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

#include "keywords.h"
#include "pdefs.h"
#include "podefs.h"
#include "pedefs.h"

#include "pofflib.h"
#include "paslib.h"
#include "perr.h"
#include "plsym.h"

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

#define INITIAL_SYMBOL_LIST_SIZE (1024*sizeof(symContainer_t*))
#define SYMBOL_LIST_INCREMENT    (256*sizeof(symContainer_t*))

/**********************************************************************
 * Private Types
 **********************************************************************/

/* This structure just contains a POFF library symbol */

struct symContainer_s
{
  struct symContainer_s *next;
  struct symContainer_s *prev;
  poffLibSymbol_t        s;
};
typedef struct symContainer_s symContainer_t;

/**********************************************************************
 * Private Variables
 **********************************************************************/

static symContainer_t  *symHead          = NULL;
static symContainer_t  *symTail          = NULL;
static symContainer_t **symList          = NULL;
static uint32_t         symListAlloc     = 0;

static int              nUndefined       = 0;
static int              nMultiplyDefined = 0;

/**********************************************************************
 * Private Function Prototypes

 **********************************************************************/

static void            offsetSymbolValue(poffLibSymbol_t *sym,
                                         uint32_t pcOffset);
static symContainer_t *insertSymbol(poffLibSymbol_t *sym);
static void            addSymbolToList(symContainer_t *symbol,
                                       uint32_t index);

/**********************************************************************
 * Public Functions
 **********************************************************************/

uint32_t mergeSymbols(poffHandle_t inHandle, uint32_t pcOffset, uint32_t symOffset)
{
  poffLibSymbol_t symbol;
  symContainer_t *container;
  int32_t inIndex;
  uint32_t outIndex;

  do
    {
      /* Read each symbol from the input File */

      inIndex = poffGetSymbol(inHandle, &symbol);
      if (inIndex >= 0)
        {
          /* If the symbol carries a "payload" that is a program
           * section offset, then apply the pcOffset value to
           * that "payload"
           */

          offsetSymbolValue(&symbol, pcOffset);

          /* Create a container for the symbol information */

          container = insertSymbol(&symbol);

          /* Add the symbol to the linearly indexed list */

          outIndex = inIndex + symOffset;
          addSymbolToList(container, outIndex);
        }
    }
  while (inIndex >= 0);

  /* Return the offset to the last symbol inserted */

  return outIndex;
}

/***********************************************************************/

void verifySymbols(void)
{
  symContainer_t *sym;

  /* At the conclusion the link, there should be no undefined symbols.
   * This function simply asserts that condition.  It traverses the
   * symbol container list and if any undefined symbol is found, it
   * errors out.
   */

  for (sym = symHead; (sym); sym = sym->next)
    {
      if ((sym->s.flags & STF_UNDEFINED) != 0)
        {
          fprintf(stderr, "ERROR: Undefined symbol '%s'\n",
                  sym->s.name);
          nUndefined++;
        }
    }

  if (nUndefined) fatal(eUNDEFINEDSYMBOL);
  if (nMultiplyDefined) fatal(eMULTIDEFSYMBOL);
}

/***********************************************************************/

void writeSymbols(poffHandle_t outHandle)
{
  symContainer_t *sym;

  /* Transfer all buffered symbol information to the output file */

  for (sym = symHead; (sym); sym = sym->next)
    {
      (void)poffAddSymbol(outHandle, &sym->s);
    }
}

/***********************************************************************/

poffLibSymbol_t *getSymbolByIndex(uint32_t symIndex)
{
  if (symIndex * sizeof(symContainer_t*) >= symListAlloc)
    fatal(ePOFFCONFUSION);
  return &symList[symIndex]->s;
}

/***********************************************************************/

void releaseSymbols(void)
{
  static symContainer_t *curr;
  static symContainer_t *next;

  for (curr = symHead; (curr); curr = next)
    {
      /* Get the next pointer from the container -- we are going
       * to deallocate the container!
       */

      next = curr->next;

      /* Deallocate the name string copy */

      if (curr->s.name) free((void*)curr->s.name);

      /* Free the container */

      free(curr);
    }

  /* Free the index-able symbol list */

  if (symList) free((void*)symList);

  symHead = NULL;
  symTail = NULL;
  symList = NULL;
}

/**********************************************************************/

static void offsetSymbolValue(poffLibSymbol_t *sym, uint32_t pcOffset)
{
  /* Don't do anything with undefined symbols.  By definition, these
   * cannot cannot any meaning values.
   */

  if ((sym->flags & STF_UNDEFINED) == 0)
    {
      switch (sym->type)
        {
        case STT_PROC:
        case STT_FUNC:
          sym->value += pcOffset;
          break;

        default:
          break;
        }
    }
}

/**********************************************************************/

static inline symContainer_t *makeSymContainer(poffLibSymbol_t *psym)
{
  symContainer_t *sym;
  sym = (symContainer_t*)malloc(sizeof(symContainer_t));
  if (sym == NULL)
    {
      fatal(eNOMEMORY);
    }

  /* The next container is not linked to anything yet */

  sym->next   = NULL;
  sym->prev   = NULL;

  /* Copy the whole symbol record */

  sym->s      = *psym;

  /* Duplicate the symbol name -- the reference in the symbol entry
   * belongs to the input file and will be lost if/when the input file
   * is released.
   */

  if (psym->name)
    {
      sym->s.name = strdup(psym->name);
    }
  return sym;
}

static symContainer_t *insertSymbol(poffLibSymbol_t *sym)
{
  symContainer_t *prev;
  symContainer_t *curr;
  symContainer_t *newsym;
  int compare;

  /* Find where to insert the symbol */

  for (prev = NULL, curr = symHead; (curr); prev = curr, curr = curr->next)
    {
      /* Compare the names of the symbols */

      compare = strcmp(curr->s.name, sym->name);

      /* Break out of the loop if curr->name > sym_name or
       * if curr->name == sym_name AND the types of the
       * symbols are the same.
       */

      if (compare > 0)
        {
          /* Break out... curr refers to a symbol AFTER the position
           * where we want to put the new symbol.
           */

          break;
        }
      else if (compare == 0)
        {
          /* The symbols are the same.  break out only if the types
           * are the same or this is where we need to insert the new
           * symbol (same name different type)
           */

          if (curr->s.type > sym->type)
            {
              compare = 1;
              break;
            }
          else if (curr->s.type == sym->type)
            {
              break;
            }
        }
    }

  /* We get here if:
   * a.  curr == NULL meaning that the symbol goes at the end of the 
   *     list.  (special case, prev == NULL as well. This happens when 
   *     the list is empty).
   * b.  curr != NULL mean that the symbol goes between prev and curr
   *     (special cases:  (i) compare == 0 meaning that the symbol is
   *     already in the list, or (ii) compare > 0 with prev == NULL
   *     meaning that the new entry goes at the beginning of the list).
   */

  if (curr == NULL)
    {
      /* The symbol goes at the end of the list */

      newsym       = makeSymContainer(sym);
      newsym->next = NULL;
      newsym->prev = prev;
      symTail      = newsym;

      if (prev)
        prev->next = newsym;
      else
        symHead    = newsym;
    }
  else if (compare == 0)
    {
      /* curr is non-NULL and refers to the same symbol (of the same
       * type).  If both are undefined, then just discard the new
       * symbol.
       */

      if ((curr->s.flags & STF_UNDEFINED) != 0)
        {
          /* The symbol in the table is undefined */

          if ((sym->flags & STF_UNDEFINED) != 0)
            {
              /* Both symbols are undefined. Just ignore the new one */
            }
          else
            {
              /* The symbol in the table is undefined, but the new
               * one is defined. Replace the one in the table (retaining
               * the allocated symbol name).
               */
              const char *save   = curr->s.name;
              curr->s            = *sym;
              curr->s.name       = save;
            }
        }
      else
        {
          /* The symbol in the table is defined */

          if ((sym->flags & STF_UNDEFINED) != 0)
            {
              /* But the new symbol is undefined.  Just ignore the
               * new symbol
               */
            }
          else
            {
              /* OOPS! both symbols are defined */

              fprintf(stderr,
                      "ERROR: Multiply defined symbol: '%s'\n",
                      sym->name);
              nMultiplyDefined++;
            }

          /* In any case, return the pointer to the old container */

          newsym = curr;
        }
    }
  else
    {
      /* curr is non-NULL and the symbol goes between prev and curr */

      newsym       = makeSymContainer(sym);
      newsym->next = curr;
      newsym->prev = prev;
      
      if (prev)
        prev->next = newsym;
      else
        symHead    = newsym;
    }

  return newsym;
}

/***********************************************************************/
/* Add a symbol to the linear symbol table list.  This list is necessary
 * to quickly mapped a symbol index value (as might be found in the
 * relocation data) to the unique representation of the symbol as
 * deterimed by insertSymbol().
 */

static void addSymbolToList(symContainer_t *symbol, uint32_t index)
{
  /* Check if we have allocated a symbol table buffer yet */

  if (!symList)
    {
      /* No, allocate it now */

      symList = (symContainer_t**)malloc(INITIAL_SYMBOL_LIST_SIZE);
      if (!symList)
        {
          fatal(eNOMEMORY);
        }
      symListAlloc = INITIAL_SYMBOL_LIST_SIZE;
    }

  /* Check if there is room for a new symbol */

  if ((index + 1) * sizeof(symContainer_t*) > symListAlloc)
    {
      uint32_t newAlloc = symListAlloc + SYMBOL_LIST_INCREMENT;
      symContainer_t **tmp;

      /* Reallocate the file name buffer */

      tmp = (symContainer_t**)realloc(symList, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      symListAlloc = newAlloc;
      symList      = tmp;
    }

  /* Save the new symbol information in the symbol table data */

  symList[index] = symbol;
}

/***********************************************************************/

