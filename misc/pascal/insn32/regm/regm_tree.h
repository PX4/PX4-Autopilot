/***************************************************************************
 * regm_tree.h
 * External Declarations associated with regm_tree.c
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
 ***************************************************************************/

#ifndef __REGM_TREE_H
#define __REGM_TREE_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <stdint.h>
#include "keywords.h"
#include "pofflib.h"
#include "rinsn32.h"

/***************************************************************************
 * Definitions
 ***************************************************************************/

/***************************************************************************
 * Global Types
 ***************************************************************************/

/* This structure retains information about a specific function */

struct procinsn_s
{
  RINSN32  sRegOp;
  uint32_t dwRegModified;
  uint32_t dwRegsUsed[2];
};

/* Each program section is described by an entry point offset, a
 * file offset, a size in bytes, and a list of instructions (but
 * we ignore the instructions at the trivial entry point which
 * will be one or zero pcodes).
 */

struct procsection_s
{
  uint32_t dwOffset; /* File offset to section */
  uint32_t dwSize;   /* Size of section in bytes */
};

/* But each pascal procedure may contain two program sections:
 * one before any nested functions/procedures. and one for the
 * main body.
 */

struct procdata_s
{
  struct procdata_s      *peer;       /* Next proc/func at this level */
  struct procdata_s      *child;      /* First nested proc/func */
  struct procsection_s    section[2];
  poffLibDebugFuncInfo_t *pFuncInfo;
  int                     nPCodes;
  OPTYPE                 *pPCode;
  struct procinsn_s      *pRegOps;
};

/***************************************************************************
 * Global Variables
 ***************************************************************************/

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

extern void regm_InitTree(void);
extern struct procdata_s *regm_CreateProgSection(void);
extern void regm_SetProgRoot(struct procdata_s *pNode);
extern void regm_AddProgChild(struct procdata_s *pParent,
                              struct procdata_s *pNode);
extern void regm_AddProgPeer(struct procdata_s *pPeer,
                             struct procdata_s *pNode);
extern struct procdata_s *regm_GetRootNode(void);
extern int  regm_ForEachPeer(struct procdata_s *pPeer,
                             int (*pfPeerFunc)(struct procdata_s*, void*),
                             void *arg);
extern int  regm_ForEachChild(struct procdata_s *pParent,
                              int (*pfChildFunc)(struct procdata_s*, void*),
                              void *arg);
extern uint32_t regm_ReadNodePCodes(struct procdata_s *pNode,
                                    poffHandle_t hPoff,
                                    uint32_t dwStartOffset, uint32_t dwEndOffset,
                                    uint8_t cTerminalOpcode);
extern void regm_DumpTree(void);

#endif /* __REGM_TREE_H */
