/**********************************************************************
 * pflabel.c
 * Label resolution logic
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

#include "keywords.h"
#include "pdefs.h"
#include "pedefs.h"
#include "perr.h"
#include "poff.h"

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

#define INITIAL_DEFINED_ALLOCATION   (1024*sizeof(optDefinedLabelRef_t))
#define DEFINED_INCREMENT            (256*sizeof(optDefinedLabelRef_t))

#define INITIAL_UNDEFINED_ALLOCATION (1024*sizeof(optUndefinedLabelRef_t))
#define UNDEFINED_INCREMENT          (256*sizeof(optUndefinedLabelRef_t))

/**********************************************************************
 * Private Types
 **********************************************************************/

struct optDefinedLabelRef_s
{
  uint32_t label;
  uint32_t pc;
};
typedef struct optDefinedLabelRef_s optDefinedLabelRef_t;

struct optUndefinedLabelRef_s
{
  uint32_t label;
  uint32_t symIndex;
};
typedef struct optUndefinedLabelRef_s optUndefinedLabelRef_t;

/**********************************************************************
 * Private Data
 **********************************************************************/

static optDefinedLabelRef_t   *definedLabelRefs       = NULL;
static uint32_t                definedLabelRefAlloc   = 0;
static uint32_t                nDefinedLabelRefs      = 0;

static optUndefinedLabelRef_t *undefinedLabelRefs     = NULL;
static uint32_t                undefinedLabelRefAlloc = 0;
static uint32_t                nUndefinedLabelRefs    = 0;

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

/**********************************************************************
 * Private Inline Functions
 **********************************************************************/

/**********************************************************************
 * Private Functions
 **********************************************************************/

/**********************************************************************/

static void poffCheckDefinedLabelAlloc(void)
{
  /* Has the label reference table been allocated */

  if (!definedLabelRefs)
    {
      /* No, allocate it now */

      definedLabelRefs = (optDefinedLabelRef_t*)
        malloc(INITIAL_DEFINED_ALLOCATION);

      if (!definedLabelRefs)
        {
          fatal(eNOMEMORY);
        }
      definedLabelRefAlloc = INITIAL_DEFINED_ALLOCATION;
    }
}

/**********************************************************************/

static void poffCheckDefinedLabelRealloc(void)
{
  /* Check if there is room for the new data */

  if (((nDefinedLabelRefs + 1)*sizeof(optDefinedLabelRef_t)) > 
      definedLabelRefAlloc)
    {
      uint32_t newAlloc = definedLabelRefAlloc + DEFINED_INCREMENT;
      void *tmp;

      /* Reallocate the label reference tabel */

      tmp = realloc(definedLabelRefs, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      definedLabelRefAlloc = newAlloc;
      definedLabelRefs     = (optDefinedLabelRef_t*)tmp;
    }
}

/**********************************************************************/

static void poffCheckUndefinedLabelAlloc(void)
{
  /* Has the label reference table been allocated */

  if (!undefinedLabelRefs)
    {
      /* No, allocate it now */

      undefinedLabelRefs = (optUndefinedLabelRef_t*)
        malloc(INITIAL_UNDEFINED_ALLOCATION);

      if (!undefinedLabelRefs)
        {
          fatal(eNOMEMORY);
        }
      undefinedLabelRefAlloc = INITIAL_UNDEFINED_ALLOCATION;
    }
}

/**********************************************************************/

static void poffCheckUndefinedLabelRealloc(void)
{
  /* Check if there is room for the new data */

  if (((nUndefinedLabelRefs + 1)*sizeof(optUndefinedLabelRef_t)) >
      undefinedLabelRefAlloc)
    {
      uint32_t newAlloc = undefinedLabelRefAlloc + UNDEFINED_INCREMENT;
      void *tmp;

      /* Reallocate the label reference tabel */

      tmp = realloc(undefinedLabelRefs, newAlloc);
      if (!tmp)
        {
          fatal(eNOMEMORY);
        }

      /* And set the new size */

      undefinedLabelRefAlloc = newAlloc;
      undefinedLabelRefs     = (optUndefinedLabelRef_t*)tmp;
    }
}

/**********************************************************************/

/**********************************************************************
 * Global Functions
 **********************************************************************/

/**********************************************************************/

void poffAddToDefinedLabelTable(uint32_t label, uint32_t pc)
{
  /* Make sure we have memory to do this.  If not, we will crash */

  poffCheckDefinedLabelAlloc();
  poffCheckDefinedLabelRealloc();

  /* Add the label to the table */

  definedLabelRefs[nDefinedLabelRefs].label = label;
  definedLabelRefs[nDefinedLabelRefs].pc    = pc;
  nDefinedLabelRefs++;
}

/**********************************************************************/

void poffAddToUndefinedLabelTable(uint32_t label, uint32_t symIndex)
{
  /* Make sure we have memory to do this.  If not, we will crash */

  poffCheckUndefinedLabelAlloc();
  poffCheckUndefinedLabelRealloc();

  /* Add the label to the table */

  undefinedLabelRefs[nUndefinedLabelRefs].label    = label;
  undefinedLabelRefs[nUndefinedLabelRefs].symIndex = symIndex;
  nUndefinedLabelRefs++;
}

/**********************************************************************/

int poffGetSymIndexForUndefinedLabel(uint32_t label)
{
  int i;

  for (i = 0; i < nUndefinedLabelRefs; i++)
    {
      if (undefinedLabelRefs[i].label == label)
        {
          return undefinedLabelRefs[i].symIndex;
        }
    }
  return -1;
}

/**********************************************************************/

int poffGetPcForDefinedLabel(uint32_t label)
{
  int i;

  for (i = 0; i < nDefinedLabelRefs; i++)
    {
      if (definedLabelRefs[i].label == label)
        {
          return definedLabelRefs[i].pc;
        }
    }
  return -1;
}

/**********************************************************************/

void poffReleaseLabelReferences(void)
{
  if (definedLabelRefs)
    {
      free(definedLabelRefs);
    }
  if (undefinedLabelRefs)
    {
      free(undefinedLabelRefs);
    }
}

/**********************************************************************/

