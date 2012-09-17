/**********************************************************************
 * regm_tree.c
 * Tree managmenet
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
#include "pdefs.h"     /* Types needed for unused protos in pinsn.h */
#include "podefs.h"    /* Types needed for unused protos in pinsh.h */
#include "pedefs.h"
#include "pofflib.h"
#include "paslib.h"
#include "pinsn.h"     /* Folr insn_GetOpCode */
#include "perr.h"

#include "pinsn32.h"
#include "regm.h"
#include "regm_tree.h"

/**********************************************************************
 * Definitions
 **********************************************************************/

#define INITIAL_PCODE_ALLOC 250
#define PCODE_RELLALLOC     100

/**********************************************************************
 * Private Types
 **********************************************************************/

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

static struct procdata_s *g_pProgramHead = NULL;

/**********************************************************************
 * Private Functions
 **********************************************************************/

/***********************************************************************/

static inline void regm_DumpIndent(uint32_t dwIndent)
{
  while (dwIndent--) putchar(' ');
}

/***********************************************************************/

void regm_DumpNode(struct procdata_s *pNode, unsigned long dwIndent)
{
  if (pNode)
    {
      if (pNode->section[0].dwOffset != pNode->section[1].dwOffset)
        {
          regm_DumpIndent(dwIndent);
          printf("%08lx:%08lx\n",
                 pNode->section[0].dwOffset,
                 pNode->section[0].dwOffset + pNode->section[0].dwSize - 1);
        }
      regm_DumpIndent(dwIndent);
      printf("%08lx:%08lx\n",
             pNode->section[1].dwOffset,
             pNode->section[1].dwOffset + pNode->section[1].dwSize - 1);

      regm_DumpNode(pNode->child, dwIndent + 3);
      regm_DumpNode(pNode->peer, dwIndent);
    }
}

/***********************************************************************/

/**********************************************************************
 * Global Functions
 **********************************************************************/

/***********************************************************************/

void regm_InitTree(void)
{
  g_pProgramHead = NULL;
}

/***********************************************************************/

struct procdata_s *regm_CreateProgSection(void)
{
  struct procdata_s *pNode;
  pNode = (struct procdata_s *)malloc(sizeof(struct procdata_s));
  if (!pNode)
    {
      fatal(eNOMEMORY);
    }
  memset(pNode, 0, sizeof(struct procdata_s));
  return pNode;
}

/***********************************************************************/

void regm_SetProgRoot(struct procdata_s *pNode)
{
  g_pProgramHead = pNode;
  pNode->child   = NULL;
  pNode->peer    = NULL;
}

/***********************************************************************/

void regm_AddProgChild(struct procdata_s *pParent, struct procdata_s *pNode)
{
  struct procdata_s *pPrev = pParent;

  /* Go to the end of the list */

  while (pPrev->child != NULL) pPrev = pPrev->child;

  /* Deposit the new node at the end of the list */

  pPrev->child = pNode;
  pNode->child = NULL;
  pNode->peer  = NULL;
}

/***********************************************************************/

void regm_AddProgPeer(struct procdata_s *pPeer, struct procdata_s *pNode)
{
  struct procdata_s *pPrev = pPeer;

  /* Go to the end of the list */

  while (pPrev->peer != NULL) pPrev = pPrev->peer;

  /* Deposit the new node at the end of the list */

  pPrev->peer  = pNode;
  pNode->child = NULL;
  pNode->peer  = NULL;
}

/***********************************************************************/

struct procdata_s *regm_GetRootNode(void)
{
  return g_pProgramHead;
}

/***********************************************************************/

int regm_ForEachPeer(struct procdata_s *pPeer,
                     int (*pfPeerFunc)(struct procdata_s*, void*),
                     void *arg)
{
  struct procdata_s *pCurr = pPeer;
  struct procdata_s *pNext;
  int retval;

  while (pCurr->peer)
    {
      pNext = pCurr->peer;
      retval = pfPeerFunc(pCurr, arg);
      if (retval)
        {
          return retval;
        }
      pCurr = pNext;
    }
  return 0;
}

/***********************************************************************/

int regm_ForEachChild(struct procdata_s *pParent,
                      int (*pfChildFunc)(struct procdata_s*, void*),
                      void *arg)
{
  struct procdata_s *pCurr = pParent;
  struct procdata_s *pNext;
  int retval;

  while (pCurr->child)
    {
      pNext = pCurr->child;
      retval = pfChildFunc(pCurr, arg);
      if (retval)
        {
          return retval;
        }
      pCurr = pNext;
    }
  return 0;
}

/***********************************************************************/

uint32_t regm_ReadNodePCodes(struct procdata_s *pNode, poffHandle_t hPoff,
                           uint32_t dwStartOffset, uint32_t dwEndOffset,
                           uint8_t cTerminalOpcode)
{
  uint32_t dwOffset = dwStartOffset;
  long nAlloc = INITIAL_PCODE_ALLOC;
  long nPCodes;
  uint8_t bTerminatorFound;

  dbg("Reading Node: %08lx %08lx %02x\n",
      dwStartOffset, dwEndOffset, cTerminalOpcode);

  /* Allocate an inital buffer to hold the instructions */

  pNode->pPCode = (OPTYPE*)
    malloc(INITIAL_PCODE_ALLOC*sizeof(struct procinsn_s*));
  if (!pNode->pPCode)
    {
      fatal(eNOMEMORY);
    }

  /* Seek to the beginning of the data */

  regm_ProgSeek(hPoff, dwStartOffset);

  /* Read all of the instructions in the main program section */

  nPCodes = 0;
  bTerminatorFound = 0;
  do
    {
      /* Make sure that there is space for another pcode */

      if (nPCodes > nAlloc)
        {
          /* If not, then reallocate the array */

          nAlloc += PCODE_RELLALLOC;
          pNode->pPCode = (OPTYPE*)
            realloc(pNode->pPCode, nAlloc*sizeof(OPTYPE));
        }

      /* Ready the pcode ito the array */

      dwOffset += insn_GetOpCode(hPoff, &pNode->pPCode[nPCodes]);

      /* Check for a terminating pcode */

      if ((GETOP(&pNode->pPCode[nPCodes]) == cTerminalOpcode) ||
          (GETOP(&pNode->pPCode[nPCodes]) == oEND))
        {
          bTerminatorFound++;
        }

      /* Increment the count of pcodes read */

      nPCodes++;
    }
  while (!bTerminatorFound && (dwOffset < dwEndOffset));

  dbg("              %08lx %08lx %02x\n",
      dwStartOffset, dwOffset, GETOP(&pNode->pPCode[nPCodes-1]));

  /* Save the number of pcodes that we found */

  pNode->nPCodes = nPCodes;

  /* Check for the correct terminator */

  if (GETOP(&pNode->pPCode[nPCodes-1]) != cTerminalOpcode)
    {
      fatal(ePOFFCONFUSION);
    }

  /* Return any unused space in the allocation */

  pNode->pPCode = (OPTYPE*)
    realloc(pNode->pPCode, nPCodes*sizeof(OPTYPE));

  /* Return the actual end offset */

  return dwOffset;
}

/***********************************************************************/

void regm_DumpTree(void)
{
  if (vRegmDebug)
    {
      regm_DumpNode(g_pProgramHead, 0);
    }
}

/***********************************************************************/

