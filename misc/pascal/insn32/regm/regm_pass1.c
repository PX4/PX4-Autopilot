/**********************************************************************
 * regm_pass1.c
 * Break the pcode data into sections
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
#include <unistd.h>

#include "keywords.h"
#include "podefs.h"
#include "pedefs.h"
#include "paslib.h"
#include "pofflib.h"
#include "pinsn.h"
#include "pinsn32.h"
#include "perr.h"

#include "regm.h"
#include "regm_tree.h"
#include "regm_pass1.h"

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void     regm_Pass1Child(poffHandle_t hPoff,
                                struct procdata_s *pParent,
                                uint32_t dwStartOffset,
                                uint32_t dwEndOffset);
static void     regm_Pass1Peer(poffHandle_t hPoff,
                               struct procdata_s *pPeer,
                               uint32_t dwStartOffset,
                               uint32_t dwEndOffset);
static struct procdata_s *regm_Pass1Node(poffHandle_t hPoff,
                                         uint32_t dwStartOffset,
                                         uint32_t pdwEndOffset,
                                         uint8_t chTerminator);
static uint32_t regm_CheckSection1(poffHandle_t hPoff, uint32_t dwOffset);
static void     regm_Pass1Family(poffHandle_t hPoff,
                                 struct procdata_s *pNode,
                                 uint32_t dwEndOffset);

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/**********************************************************************
 * Private Functions
 **********************************************************************/

/***********************************************************************/

static void regm_Pass1Child(poffHandle_t hPoff, struct procdata_s *pParent,
                           uint32_t dwStartOffset, uint32_t dwEndOffset)

{
  struct procdata_s *pNode;

  TRACE(stderr, "[regm_Pass1Child]");

  /* Read the proc/func body */

  pNode = regm_Pass1Node(hPoff, dwStartOffset, dwEndOffset, oRET);

  /* Put the func/proc body section into the tree */
  
  regm_AddProgChild(pParent, pNode);

  /* Handle nested and child proc/func blocks */

  regm_Pass1Family(hPoff, pNode, dwEndOffset);
}

/***********************************************************************/

static void regm_Pass1Peer(poffHandle_t hPoff, struct procdata_s *pPeer,
                           uint32_t dwStartOffset, uint32_t dwEndOffset)

{
  struct procdata_s *pNode;

  TRACE(stderr, "[regm_Pass1Peer]");

  /* Read the proc/func body */

  pNode = regm_Pass1Node(hPoff, dwStartOffset, dwEndOffset, oRET);

  /* Put the func/proc body section into the tree */
  
  regm_AddProgPeer(pPeer, pNode);

  /* Handle nested and child proc/func blocks */

  regm_Pass1Family(hPoff, pNode, dwEndOffset);
}

/***********************************************************************/

static struct procdata_s *regm_Pass1Node(poffHandle_t hPoff,
                                         uint32_t dwStartOffset,
                                         uint32_t pdwEndOffset,
                                         uint8_t chTerminator)

{
  struct procdata_s *pNode;
  uint32_t dwActualEndOffset;

  TRACE(stderr, "[regm_Pass1Node]");

  /* Create a container for the proc/func body, and read the data */

  pNode = regm_CreateProgSection();

  /* Check if there is a jump at the beginning of the segment */

  pNode->section[0].dwOffset = dwStartOffset;
  pNode->section[1].dwOffset = regm_CheckSection1(hPoff, dwStartOffset);

  /* Read all of the p-codes associated with the node */

  dwActualEndOffset = regm_ReadNodePCodes(pNode, hPoff,
                                          pNode->section[1].dwOffset,
                                          pdwEndOffset, chTerminator);

  /* Now calculate the size of each part of the program section */

  pNode->section[1].dwSize = dwActualEndOffset - pNode->section[1].dwOffset;

  if (pNode->section[0].dwOffset == pNode->section[1].dwOffset)
    pNode->section[0].dwSize = 0;
  else
    pNode->section[0].dwSize = 5;

  /* Associate debug info with the program section. */

  pNode->pFuncInfo = poffFindDebugFuncInfo(pNode->section[0].dwOffset);
  if (!pNode->pFuncInfo)
    {
      /* This debug information should always be present at this
       * point.  We will need it.
       */

      fatal(ePOFFCONFUSION);
    }

  return pNode;
}

/***********************************************************************/

static uint32_t regm_CheckSection1 (poffHandle_t hPoff, uint32_t dwOffset)
{
  OPTYPE op;

  /* Seek to the beginning of the section. */

  regm_ProgSeek(hPoff, dwOffset);

  /* Read the opcode at that position. */

  (void)insn_GetOpCode(hPoff, &op);

  /* Is it a oJMP instruction? This happens when there are nested
   * functions.  The entry point into the parent function is a jump
   * around the nested functions.
   */

  if (GETOP(&op) == oJMP)
    {
      /* Yes, then the block really begins at the target of the jump */

      return GETARG(&op);
    }
  else
    {
      /* No, then the block really begins right here */

      return dwOffset;
    }
}

/***********************************************************************/

static void regm_Pass1Family(poffHandle_t hPoff, struct procdata_s *pNode,
                             uint32_t dwEndOffset)
{
  uint32_t dwSectionEnd;

  /* Process any nested functions */

  if (pNode->section[0].dwOffset != pNode->section[1].dwOffset)
    {
      dwSectionEnd = pNode->section[0].dwOffset + pNode->section[0].dwSize;
      regm_Pass1Child(hPoff, pNode, dwSectionEnd, pNode->section[1].dwOffset);
    }

  /* Process any peer functions */

  dwSectionEnd = pNode->section[1].dwOffset + pNode->section[1].dwSize;
  if (dwSectionEnd < dwEndOffset)
    {
      regm_Pass1Peer(hPoff, pNode, dwSectionEnd, dwEndOffset);
    }
}

/**********************************************************************
 * Global Functions
 **********************************************************************/

/***********************************************************************/

void regm_Pass1(poffHandle_t hPoff)
{
  struct procdata_s *pNode;
  uint32_t dwEntryPoint;

  TRACE(stderr, "[regm_Pass1]");

  /* Get the program entry point from the poff header */

  dwEntryPoint = poffGetEntryPoint(hPoff);

  /* Read the main program body */

  pNode = regm_Pass1Node(hPoff, dwEntryPoint, 0xffffffff, oEND);

  /* Put the main program section into the tree (at the root) */
  
  regm_SetProgRoot(pNode);

  /* Then process any nested functions */

  if (dwEntryPoint != 0)
    {
      regm_Pass1Child(hPoff, pNode, 0, dwEntryPoint);
    }
}

/***********************************************************************/

