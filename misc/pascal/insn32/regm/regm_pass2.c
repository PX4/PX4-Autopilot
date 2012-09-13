/**********************************************************************
 * regm_pass2.c
 * Convert the buffered pcode to the basic register model with an
 * indefinite number of registers (arguments, general, and special
 * registers) and with 32-bit immediate size.
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
 *********************************************************************/

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "keywords.h"
#include "pdefs.h"
#include "pxdefs.h"
#include "pfdefs.h"
#include "pedefs.h"
#include "pofflib.h"
#include "perr.h"

#include "pinsn32.h"
#include "builtins.h"

#include "regm.h"
#include "regm_tree.h"
#include "regm_registers2.h"
#include "regm_pass2.h"

/************************************a*********************************
 * Definitions
 **********************************************************************/

/**********************************************************************
 * Private Types
 **********************************************************************/

struct regm_opmap_s;

typedef void (*regm_mapper_t)(const struct regm_opmap_s *pEntry,
                              OPTYPE *pOpCode, struct procdata_s *pNode);

struct regm_opmap_s
{
  uint8_t       chOpCode;
  int8_t        chImmediate;
  int8_         chSpecial;
  regm_mapper_t pMapper;
};

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void regm_NoOperation(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode,
                             struct procdata_s *pNode);
static void regm_UnaryOperation(const struct regm_opmap_s *pEntry,
                                OPTYPE *pOpCode,
                                struct procdata_s *pNode);
static void regm_BinaryOperation(const struct regm_opmap_s *pEntry,
                                 OPTYPE *pOpCode,
                                 struct procdata_s *pNode);
static void regm_CompareVsZero(const struct regm_opmap_s *pEntry,
                               OPTYPE *pOpCode,
                               struct procdata_s *pNode);
static void regm_BinaryComparison(const struct regm_opmap_s *pEntry,
                                  OPTYPE *pOpCode,
                                  struct procdata_s *pNode);
static void regm_LoadImmediate(const struct regm_opmap_s *pEntry,
                               OPTYPE *pOpCode,
                               struct procdata_s *pNode);
static void regm_LoadMultiple(uint32_t dwRDest, uint32_t dwRSrc);
static void regm_LoadMultipleImmediate(const struct regm_opmap_s *pEntry,
                                       OPTYPE *pOpCode,
                                       struct procdata_s *pNode);
static void regm_StoreMultiple(uint32_t dwRDest, uint32_t dwRSrc);
static void regm_StoreImmediate(const struct regm_opmap_s *pEntry,
                                OPTYPE *pOpCode,
                                struct procdata_s *pNode);
static void regm_StoreMultipleImmediate(const struct regm_opmap_s *pEntry,
                                        OPTYPE *pOpCode,
                                        struct procdata_s *pNode);
static void regm_Duplicate(const struct regm_opmap_s *pEntry,
                           OPTYPE *pOpCode,
                           struct procdata_s *pNode);
static void regm_PushImmediate(const struct regm_opmap_s *pEntry,
                               OPTYPE *pOpCode,
                               struct procdata_s *pNode);
static void regm_PopSpecial(const struct regm_opmap_s *pEntry,
                            OPTYPE *pOpCode,
                            struct procdata_s *pNode);
static void regm_SetDataCount(const struct regm_opmap_s *pEntry,
                              OPTYPE *pOpCode,
                              struct procdata_s *pNode);
static void regm_PushSpecial(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode,
                             struct procdata_s *pNode);
static void regm_Return(const struct regm_opmap_s *pEntry,
                        OPTYPE *pOpCode,
                        struct procdata_s *pNode);
static void regm_LoadOffset(const struct regm_opmap_s *pEntry,
                            OPTYPE *pOpCode,
                            struct procdata_s *pNode);
static void regm_LoadMultipleOffset(const struct regm_opmap_s *pEntry,
                                    OPTYPE *pOpCode,
                                    struct procdata_s *pNode);
static void regm_StoreOffset(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode,
                             struct procdata_s *pNode);
static void regm_StoreMultipleOffset(const struct regm_opmap_s *pEntry,
                                     OPTYPE *pOpCode,
                                     struct procdata_s *pNode);
static void regm_LoadIndexed(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode, struct procdata_s *pNode);
static void regm_LoadMultipleIndexed(const struct regm_opmap_s *pEntry,
                                     OPTYPE *pOpCode,
                                     struct procdata_s *pNode);
static void regm_StoreIndexed(const struct regm_opmap_s *pEntry,
                              OPTYPE *pOpCode, struct procdata_s *pNode);
static void regm_StoreMultipleIndexed(const struct regm_opmap_s *pEntry,
                                      OPTYPE *pOpCode,
                                      struct procdata_s *pNode);
static void regm_ConditionalBranchVsZero(const struct regm_opmap_s *pEntry,
                                         OPTYPE *pOpCode,
                                         struct procdata_s *pNode);
static void regm_ConditionalBranchBinary(const struct regm_opmap_s *pEntry,
                                         OPTYPE *pOpCode,
                                         struct procdata_s *pNode);
static void regm_UnconditionalBranch(const struct regm_opmap_s *pEntry,
                                     OPTYPE *pOpCode,
                                     struct procdata_s *pNode);
static void regm_IncrementSpecial(const struct regm_opmap_s *pEntry,
                                  OPTYPE *pOpCode,
                                  struct procdata_s *pNode);
static void regm_LoadAddress(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode, struct procdata_s *pNode);
static void regm_LoadAddressIndexed(const struct regm_opmap_s *pEntry,
                                    OPTYPE *pOpCode,
                                    struct procdata_s *pNode);
static void regm_SetupOutArgs(uint32_t nParms, const uint32_t *pwArgSize);
static void regm_MapInRet(uint32_t wRetSize);
static void regm_PCal(const struct regm_opmap_s *pEntry,
                      OPTYPE *pOpCode,
                      struct procdata_s *pNode);
static void regm_SysIo(const struct regm_opmap_s *pEntry,
                       OPTYPE *pOpCode,
                       struct procdata_s *pNode);
static void regm_LibCall(const struct regm_opmap_s *pEntry,
                         OPTYPE *pOpCode,
                         struct procdata_s *pNode);
static void regm_Float(const struct regm_opmap_s *pEntry,
                       OPTYPE *pOpCode,
                       struct procdata_s *pNode);
static void regm_IllegalPCode(const struct regm_opmap_s *pEntry,
                              OPTYPE *pOpCode,
                              struct procdata_s *pNode);

static void regm_GenerateRegm(struct procdata_s *pNode, void *pvArg);
static int  regm_Pass2Node(struct procdata_s *pNode, void *pvArg);

/**********************************************************************
 * Public Variables
 **********************************************************************/

/* On the initialize passes, the register number will simply be the offset
 * from the top of the stack.  The following variable keeps trck of the
 * stack offset.
 */

uint32_t g_dwStackOffset;
uint32_t g_dwRegisterCount     = 0;
int      g_bRegisterCountValid = 0;

/**********************************************************************
 * Private Variables
 **********************************************************************/

static const struct regm_opmap_s vrgOpMap1[64] = 
  {
    /* 0x00: oNOP */    {0,       0,   0,   regm_NoOperation},
    /* 0x01: oNEG */    {rRSBI,   0,   0,   regm_UnaryOperation},
    /* 0x02: oABS */    {rRSB,    0,   0,   regm_UnaryOperation},
    /* 0x03: oINC */    {rADDI,   1,   0,   regm_UnaryOperation},
    /* 0x04: oDEC */    {rSUBI,   1,   0,   regm_UnaryOperation},
    /* 0x05: oNOT */    {rMVN,    0,   0,   regm_UnaryOperation},
    /* 0x06: oADD */    {rADD,    0,   0,   regm_BinaryOperation},
    /* 0x07: oSUB */    {rSUB,    0,   0,   regm_BinaryOperation},
    /* 0x08: oMUL */    {rMUL,    0,   0,   regm_BinaryOperation},
    /* 0x09: oDIV */    {rDIV,    0,   0,   regm_BinaryOperation},
    /* 0x0a: oMOD */    {rMOD,    0,   0,   regm_BinaryOperation},
    /* 0x0b: oSLL */    {rSLL,    0,   0,   regm_BinaryOperation},
    /* 0x0c: oSRL */    {rSRL,    0,   0,   regm_BinaryOperation},
    /* 0x0d: oSRA */    {rSRA,    0,   0,   regm_BinaryOperation},
    /* 0x0e: oOR */     {rOR,     0,   0,   regm_BinaryOperation},
    /* 0x0f: oAND */    {rAND,    0,   0,   regm_BinaryOperation},

    /* 0x10: oEQUZ */   {rBEQ,    0,   0,   regm_CompareVsZero},
    /* 0x11: oNEQZ */   {rBNE,    0,   0,   regm_CompareVsZero},
    /* 0x12: oLTZ */    {rBLT,    0,   0,   regm_CompareVsZero},
    /* 0x13: oGTEZ */   {rBGTE,   0,   0,   regm_CompareVsZero},
    /* 0x14: oGTZ */    {rBGT,    0,   0,   regm_CompareVsZero},
    /* 0x15: oLTEZ */   {rBLTE,   0,   0,   regm_CompareVsZero},
    /* 0x16: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x17: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x18: oEQU */    {rBEQ,    0,   0,   regm_BinaryComparison},
    /* 0x19: oNEQ */    {rBNE,    0,   0,   regm_BinaryComparison},
    /* 0x1a: oLT */     {rBLT,    0,   0,   regm_BinaryComparison},
    /* 0x1b: oGTE */    {rBGTE,   0,   0,   regm_BinaryComparison},
    /* 0x1c: oGT */     {rBGT,    0,   0,   regm_BinaryComparison},
    /* 0x1d: oLTE */    {rBLTE,   0,   0,   regm_BinaryComparison},
    /* 0x1e: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x1f: oBIT */    {rBEQ,    0,   0,   regm_BinaryComparison},

    /* 0x20: oLDI */    {rLD,     2,   SPB, regm_LoadImmediate},
    /* 0x21: oLDIH */   {rLDH,    1,   SPB, regm_LoadImmediate},
    /* 0x22: oLDIB */   {rLDB,    0,   SPB, regm_LoadImmediate},
    /* 0x23: oLDIM */   {0,       0,   SPB, regm_LoadMultipleImmediate},
    /* 0x24: oSTI */    {rST,     2,   SPB, regm_StoreImmediate},
    /* 0x25: oSTIH */   {rSTH,    1,   SPB, regm_StoreImmediate},
    /* 0x26: oSTIB */   {rSTB,    0,   SPB, regm_StoreImmediate},
    /* 0x27: oSTIM */   {0,       0,   SPB, regm_StoreMultipleImmediate},
    /* 0x28: oDUP */    {0,       0,   0,   regm_Duplicate},
    /* 0x17: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x2a: oPUSHS */  {0,       0,   CSP, regm_PushSpecial},
    /* 0x2b: oPOPS */   {0,       0,   CSP, regm_PopSpecial},
    /* 0x2c: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x2d: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x2e: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x2f: oRET */    {0,       0,   0,   regm_Return},

    /* 0x30: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x31: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x32: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x33: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x34: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x35: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x36: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x37: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x38: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x39: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x3a: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x3b: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x3c: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x3d: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x3e: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x3f: oEND */    {0,       0,   0,   regm_Return}
  };

static const struct regm_opmap_s vrgOpMap2[64] = 
  {
    /* 0x80: oLD */     {rLD,     2,   SPB, regm_LoadOffset},
    /* 0x81: oLDH */    {rLDH,    1,   SPB, regm_LoadOffset},
    /* 0x82: oLDB */    {rLDB,    0,   SPB, regm_LoadOffset},
    /* 0x83: oLDM */    {0,       0,   SPB, regm_LoadMultipleOffset},
    /* 0x84: oST */     {rST,     2,   SPB, regm_StoreOffset},
    /* 0x85: oSTH */    {rSTH,    1,   SPB, regm_StoreOffset},
    /* 0x86: oSTB */    {rSTB,    0,   SPB, regm_StoreOffset},
    /* 0x87: oSTM */    {0,       0,   SPB, regm_StoreMultipleOffset},
    /* 0x88: oLDX */    {rLD,     2,   SPB, regm_LoadIndexed},
    /* 0x89: oLDXH */   {rLDH,    1,   SPB, regm_LoadIndexed},
    /* 0x8a: oLDXB */   {rLDB,    0,   SPB, regm_LoadIndexed},
    /* 0x8b: oLDXM */   {0,       0,   SPB, regm_LoadMultipleIndexed},
    /* 0x8c: oSTX */    {rST,     2,   SPB, regm_StoreIndexed},
    /* 0x8d: oSTXH */   {rSTH,    1,   SPB, regm_StoreIndexed},
    /* 0x8e: oSTXB */   {rSTB,    0,   SPB, regm_StoreIndexed},
    /* 0x8f: oSTXM */   {0,       0,   SPB, regm_StoreMultipleIndexed},

    /* 0x90: oJEQUZ */  {rBEQ,    0,   0,   regm_ConditionalBranchVsZero},
    /* 0x91: oJNEQZ */  {rBNE,    0,   0,   regm_ConditionalBranchVsZero},
    /* 0x92: oJLTZ */   {rBLT,    0,   0,   regm_ConditionalBranchVsZero},
    /* 0x93: oJGTEZ */  {rBGTE,   0,   0,   regm_ConditionalBranchVsZero},
    /* 0x94: oJGTZ */   {rBGT,    0,   0,   regm_ConditionalBranchVsZero},
    /* 0x95: oJLTEZ */  {rBLTE,   0,   0,   regm_ConditionalBranchVsZero},
    /* 0x96: oJMP */    {rB,      0,   0,   regm_UnconditionalBranch},
    /* 0x97: oPUSH */   {0,       0,   0,   regm_PushImmediate},
    /* 0x98: oJEQU */   {rBEQ,    0,   0,   regm_ConditionalBranchBinary},
    /* 0x99: oJNEQ */   {rBNE,    0,   0,   regm_ConditionalBranchBinary},
    /* 0x9a: oJLT */    {rBLT,    0,   0,   regm_ConditionalBranchBinary},
    /* 0x9b: oJGTE */   {rBGTE,   0,   0,   regm_ConditionalBranchBinary},
    /* 0x9c: oJGT */    {rBGT,    0,   0,   regm_ConditionalBranchBinary},
    /* 0x9d: oJLTE */   {rBLTE,   0,   0,   regm_ConditionalBranchBinary},
    /* 0x9e: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0x9f: oINDS */   {0,       0,   SP,  regm_IncrementSpecial},

    /* 0xa0: oLDS */    {rLD,     2,   LSP, regm_LoadOffset},
    /* 0xa1: oLDSH */   {rLDH,    1,   LSP, regm_LoadOffset},
    /* 0xa2: oLDSB */   {rLDB,    0,   LSP, regm_LoadOffset},
    /* 0xa3: oLDSM */   {0,       0,   LSP, regm_LoadMultipleOffset},
    /* 0xa4: oSTS */    {rST,     2,   LSP, regm_StoreOffset},
    /* 0xa5: oSTSH */   {rSTH,    1,   LSP, regm_StoreOffset},
    /* 0xa6: oSTSB */   {rSTB,    0,   LSP, regm_StoreOffset},
    /* 0xa7: oSTSM */   {0,       0,   LSP, regm_StoreMultipleOffset},
    /* 0xa8: oLDSX */   {rLD,     2,   LSP, regm_LoadIndexed},
    /* 0xa9: oLDSXH */  {rLDH,    1,   LSP, regm_LoadIndexed},
    /* 0xaa: oLDSXB */  {rLDB,    0,   LSP, regm_LoadIndexed},
    /* 0xab: oLDSXM */  {0,       0,   LSP, regm_LoadMultipleIndexed},
    /* 0xac: oSTSX */   {rST,     2,   LSP, regm_StoreIndexed},
    /* 0xad: oSTSXH */  {rSTH,    1,   LSP, regm_StoreIndexed},
    /* 0xae: oSTSXB */  {rSTB,    0,   LSP, regm_StoreIndexed},
    /* 0xaf: oSTSXM */  {0,       0,   LSP, regm_StoreMultipleIndexed},

    /* 0xb0: oLA */     {0,       0,   SPB, regm_LoadAddress},
    /* 0xb1: oLAS */    {0,       0,   LSP, regm_LoadAddress},
    /* 0xb2: oLAC */    {0,       0,   CSB, regm_LoadAddress},
    /* 0xb3: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0xb4: oLAX */    {0,       0,   SPB, regm_LoadAddressIndexed},
    /* 0xb5: oLASX */   {0,       0,   LSP, regm_LoadAddressIndexed},
    /* 0xb6: oSLSP */   {0,       0,   LSP, regm_PopSpecial},
    /* 0xb7: oSDC */    {0,       0,   DC,  regm_SetDataCount},
    /* 0xb8: */         {0,       0,   0,   regm_IllegalPCode},
    /* 0xb9: oPCAL */   {0,       0,   0,   regm_PCal},
    /* 0xba: oSYSIO */  {0,       0,   0,   regm_SysIo},
    /* 0xbb: oLIB */    {0,       0,   0,   regm_LibCall},
    /* 0xbc: oFLOAT */  {0,       0,   0,   regm_Float},
    /* 0xbd: oLABEL */  {0,       0,   0,   regm_NoOperation},
    /* 0xbe: oINCLUDE*/ {0,       0,   0,   regm_NoOperation},
    /* 0xbf: oLINE */   {0,       0,   0,   regm_NoOperation}
  };

static const struct regm_builtin_s g_rgSysIoBuiltIns[MAX_XOP] =
{
  /* 0x00 */ ILLEGAL_BUILTIN_INIT,      xEOF_INIT, 
  /* 0x02 */ xEOLN_INIT,                xRESET_INIT, 
  /* 0x04 */ xREWRITE_INIT,             ILLEGAL_BUILTIN_INIT,
  /* 0x06 */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x08 */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x0a */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x0c */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x0e */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x10 */ xREADLN_INIT,              xREAD_PAGE_INIT, 
  /* 0x12 */ xREAD_BINARY_INIT,         xREAD_INT_INIT, 
  /* 0x14 */ xREAD_CHAR_INIT,           xREAD_STRING_INIT, 
  /* 0x16 */ xREAD_REAL_INIT,           ILLEGAL_BUILTIN_INIT,
  /* 0x18 */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x1a */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x1c */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x1e */ ILLEGAL_BUILTIN_INIT,      ILLEGAL_BUILTIN_INIT,
  /* 0x20 */ xWRITELN_INIT,             xWRITE_PAGE_INIT, 
  /* 0x22 */ xWRITE_BINARY_INIT,        xWRITE_INT_INIT, 
  /* 0x24 */ xWRITE_CHAR_INIT,          xWRITE_STRING_INIT, 
  /* 0x25 */ xWRITE_REAL_INIT
};

static const struct regm_builtin_s g_rgLibCallBuiltIns[MAX_LBOP] =
{
  /* 0x00 */ lbGETENV_INIT,             lbSTR2STR_INIT,
  /* 0x02 */ lbCSTR2STR_INIT,           lbSTR2RSTR_INIT,
  /* 0x04 */ lbCSTR2RSTR_INIT,          lbVAL_INIT,
  /* 0x06 */ lbMKSTK_INIT,              lbMKSTKSTR_INIT,
  /* 0x08 */ lbMKSTKC_INIT,             lbSTRCAT_INIT,
  /* 0x0a */ lbSTRCATC_INIT,            lbSTRCMP_INIT
};

static const struct regm_builtin_s g_rgRrFopBuiltIns[MAX_FOP] =
{
  /* 0x00 */ ILLEGAL_BUILTIN_INIT, fpFLOAT_INIT,
  /* 0x02 */ fpTRUNC_INIT,         fpROUND_INIT, 
  /* 0x04 */ fpADD_RR_INIT,        fpSUB_RR_INIT, 
  /* 0x06 */ fpMUL_RR_INIT,        fpDIV_RR_INIT, 
  /* 0x00 */ fpMOD_RR_INIT,        ILLEGAL_BUILTIN_INIT,
  /* 0x0a */ fpEQU_RR_INIT,        fpNEQ_RR_INIT, 
  /* 0x0c */ fpLT_RR_INIT,         fpGTE_RR_INIT, 
  /* 0x0e */ fpGT_RR_INIT,         fpLTE_RR_INIT, 
  /* 0x10 */ fpNEG_R_INIT,         fpABS_R_INIT, 
  /* 0x12 */ fpSQR_R_INIT,         fpSQRT_R_INIT, 
  /* 0x14 */ fpSIN_R_INIT,         fpCOS_R_INIT, 
  /* 0x16 */ fpATAN_R_INIT,        fpLN_R_INIT, 
  /* 0x18 */ fpEXP_R_INIT
};

static const struct regm_builtin_s g_rgRiFopBuiltIns[MAX_FOP] =
{
  /* 0x00 */ ILLEGAL_BUILTIN_INIT, fpFLOAT_INIT,
  /* 0x02 */ fpTRUNC_INIT,         fpROUND_INIT, 
  /* 0x04 */ fpADD_RI_INIT,        fpSUB_RI_INIT, 
  /* 0x06 */ fpMUL_RI_INIT,        fpDIV_RI_INIT, 
  /* 0x00 */ fpMOD_RI_INIT,        ILLEGAL_BUILTIN_INIT,
  /* 0x0a */ fpEQU_RI_INIT,        fpNEQ_RI_INIT, 
  /* 0x0c */ fpLT_RI_INIT,         fpGTE_RI_INIT, 
  /* 0x0e */ fpGT_RI_INIT,         fpLTE_RI_INIT, 
  /* 0x10 */ fpNEG_I_INIT,         fpABS_I_INIT, 
  /* 0x12 */ fpSQR_I_INIT,         fpSQRT_I_INIT, 
  /* 0x14 */ fpSIN_I_INIT,         fpCOS_I_INIT, 
  /* 0x16 */ fpATAN_I_INIT,        fpLN_I_INIT, 
  /* 0x18 */ fpEXP_I_INIT
};

static const struct regm_builtin_s g_rgIrFopBuiltIns[MAX_FOP] =
{
  /* 0x00 */ ILLEGAL_BUILTIN_INIT, fpFLOAT_INIT,
  /* 0x02 */ fpTRUNC_INIT,         fpROUND_INIT, 
  /* 0x04 */ fpADD_IR_INIT,        fpSUB_IR_INIT, 
  /* 0x06 */ fpMUL_IR_INIT,        fpDIV_IR_INIT, 
  /* 0x00 */ fpMOD_IR_INIT,        ILLEGAL_BUILTIN_INIT,
  /* 0x0a */ fpEQU_IR_INIT,        fpNEQ_IR_INIT, 
  /* 0x0c */ fpLT_IR_INIT,         fpGTE_IR_INIT, 
  /* 0x0e */ fpGT_IR_INIT,         fpLTE_IR_INIT, 
  /* 0x10 */ fpNEG_R_INIT,         fpABS_R_INIT, 
  /* 0x12 */ fpSQR_R_INIT,         fpSQRT_R_INIT, 
  /* 0x14 */ fpSIN_R_INIT,         fpCOS_R_INIT, 
  /* 0x16 */ fpATAN_R_INIT,        fpLN_R_INIT, 
  /* 0x18 */ fpEXP_R_INIT
};

static const struct regm_builtin_s g_rgIiFopBuiltIns[MAX_FOP] =
{
  /* 0x00 */ ILLEGAL_BUILTIN_INIT, fpFLOAT_INIT,
  /* 0x02 */ fpTRUNC_INIT,         fpROUND_INIT, 
  /* 0x04 */ fpADD_II_INIT,        fpSUB_II_INIT, 
  /* 0x06 */ fpMUL_II_INIT,        fpDIV_II_INIT, 
  /* 0x00 */ fpMOD_II_INIT,        ILLEGAL_BUILTIN_INIT,
  /* 0x0a */ fpEQU_II_INIT,        fpNEQ_II_INIT, 
  /* 0x0c */ fpLT_II_INIT,         fpGTE_II_INIT, 
  /* 0x0e */ fpGT_II_INIT,         fpLTE_II_INIT, 
  /* 0x10 */ fpNEG_I_INIT,         fpABS_I_INIT, 
  /* 0x12 */ fpSQR_I_INIT,         fpSQRT_I_INIT, 
  /* 0x14 */ fpSIN_I_INIT,         fpCOS_I_INIT, 
  /* 0x16 */ fpATAN_I_INIT,        fpLN_I_INIT, 
  /* 0x18 */ fpEXP_I_INIT
};

static const struct regm_builtin_s *g_prgFopBuiltIns[4] =
{
  /* Real    - Real    */ g_rgRrFopBuiltIns,
  /* Integer - Real    */ g_rgRiFopBuiltIns,
  /* Real    - Ingeter */ g_rgIrFopBuiltIns,
  /* Integer - Integer */ g_rgIiFopBuiltIns
};

/**********************************************************************
 * Private Functions
 **********************************************************************/

/***********************************************************************/

static void regm_NoOperation(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode, struct procdata_s *pNode)
{
  TRACE(stderr, "[regm_NoOperation]");

  /* Do nothing */
}

/***********************************************************************/
/* These pcodes are all binary operations in the sense that they take
 * one input and produce one output:
 *
 * INPUT:  TOS(0)
 * OUTPUT: TOS(0)
 * Stack is unchanged.
 */

static void regm_UnaryOperation(const struct regm_opmap_s *pEntry,
                                OPTYPE *pOpCode, struct procdata_s *pNode)
{
  uint32_t dwUnaryRegister = MKCCREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwCcRegister    = MKCCREG(g_dwStackOffset);

  TRACE(stderr, "[regm_UnaryOperation]");

  switch (GETOP(pOpCode))
    {
    case oABS:
      regm_GenerateForm1ICc(rCMPI, dwUnaryRegister, 0, dwCcRegister);
      regm_GenerateForm4ICc(rBGTE, 2, dwCcRegister);

    default:
      regm_GenerateForm3I(pEntry->chOpCode, dwUnaryRegister,
                          dwUnaryRegister, pEntry->chImmediate);
      break;
    }
}

/***********************************************************************/
/* These pcodes are all binary operations in the sense that they take
 * two input:
 *
 * INPUT:  TOS(0), TOS(-1)
 * OUTPUT: TOS(0)
 * Stack reduced by one.
 *
 * These all generate form 3r instructions:
 */

static void regm_BinaryOperation(const struct regm_opmap_s *pEntry,
                                 OPTYPE *pOpCode,
                                 struct procdata_s *pNode)
{
  uint32_t dwROperand1 = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwROperand2 = MKREG(g_dwStackOffset - 2*sINT_SIZE);
  uint32_t dwRDest     = dwROperand2;

  TRACE(stderr, "[regm_BinaryOperation]");

  /* Generate the binary operation */

  regm_GenerateForm3R(pEntry->chOpCode, dwRDest, dwROperand1, dwROperand2);
      
  /* Reduce stack */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* These pcodes are all boolean unary operations in the sense that
 * the pcode form takes one input and generates one output:
 *
 * INPUT:  TOS(0)
 * OUTPUT: TOS(0)
 * Stack unchanged
 *
 * The complication is that the resulting boolean is not represented by
 * data in the register model but, rather, as a condition code setting.
 * For now we can, however, force a large number of condition code
 * registers; during a later fixup pass, we can force this to a single
 * condition code register.
 */

static void regm_CompareVsZero(const struct regm_opmap_s *pEntry,
                               OPTYPE *pOpCode,
                               struct procdata_s *pNode)
{
  uint32_t dwUnaryRegister = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwCcRegister    = MKCCREG(g_dwStackOffset);

  TRACE(stderr, "[regm_CompareVsZero]");

  regm_GenerateForm1ICc(rCMPI, dwUnaryRegister, 0, dwCcRegister);
  regm_GenerateForm2I(rMOVI, dwUnaryRegister, 0);
  regm_GenerateForm4ICc(pEntry->chOpCode, 2, dwCcRegister);
  regm_GenerateForm2I(rMOVI, dwUnaryRegister, 1);
}

/***********************************************************************/
/* These pcodes are all boolean binary operations in the sense that
 * the pcode form takes two inputs and generates one output:
 *
 * INPUT:  TOS(0), TOS(-1)
 * OUTPUT: TOS(0)
 * Stack reduced by one.
 *
 * The complication is that the resulting boolean not represented by
 * data in the register model but, rather, as a condition code setting.
 * For now we can, however, force a large number of condition code
 * registers; during a later fixup pass, we can force this to a single
 * condition code register.
 */

static void regm_BinaryComparison(const struct regm_opmap_s *pEntry,
                                  OPTYPE *pOpCode,
                                  struct procdata_s *pNode)
{
  uint32_t dwROperand1  = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwROperand2  = MKREG(g_dwStackOffset - 2*sINT_SIZE);
  uint32_t dwRDest      = dwROperand2;
  uint32_t dwCcRegister = MKCCREG(g_dwStackOffset);

  TRACE(stderr, "[regm_BinaryComparison]");


  switch (GETOP(pOpCode))
    {
    case oBIT:
      regm_GenerateForm3R(rAND, dwRDest, dwROperand1, dwROperand2);
      regm_GenerateForm1ICc(rCMPI, dwRDest, 0, dwCcRegister);
      break;

    default:
      regm_GenerateForm1RCc(rCMP, dwROperand1, dwROperand2, dwCcRegister);
      break;
    }

  regm_GenerateForm2I(rMOVI, dwRDest, 0);
  regm_GenerateForm4ICc(pEntry->chOpCode, 2, dwCcRegister);
  regm_GenerateForm2I(rMOVI, dwRDest, 1);
      
  /* Reduce stack */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* Load from the address on the stack.  Stack is unchanged */

static void regm_LoadImmediate(const struct regm_opmap_s *pEntry,
                               OPTYPE *pOpCode,
                               struct procdata_s *pNode)
{
  uint32_t dwROperand1 = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRDest     = dwROperand1;

  TRACE(stderr, "[regm_LoadImmediate]");

  /* Use the immediate value as an index against the SPB/LSP */

  regm_GenerateForm3R(pEntry->chOpCode, dwRDest, dwROperand1,
                      MKSPECIAL(pEntry->chSpecial));
}

/***********************************************************************/
/* Generic load multiple logic */

static void regm_LoadMultiple(uint32_t dwRDest, uint32_t dwRSrc)
{
  TRACE(stderr, "[regm_LoadMultiple]");

  if (g_bRegisterCountValid)
    {
      regm_GenerateForm3I(rLDM, dwRDest, dwRSrc, g_dwRegisterCount);

      /* Adjust the stack for the g_dwRegisterCount values added to the
       * stack.
       */

      g_dwStackOffset += g_dwRegisterCount * sINT_SIZE;
      g_bRegisterCountValid = 0;
    }
  else
    {
      fatal(ePOFFCONFUSION);
    }
}

/***********************************************************************/
/* SPB/LSP relative source offset is on the stack.  Stack increase determined
 * by content of DC register.
 */

static void regm_LoadMultipleImmediate(const struct regm_opmap_s *pEntry,
                                       OPTYPE *pOpCode,
                                       struct procdata_s *pNode)
{
  uint32_t dwRSrc  = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRDest = dwRSrc;

  TRACE(stderr, "[regm_LoadMultipleImmediate]");

  /* Adjust the src for the SPB/LSP value and generate the multiple load */

  regm_GenerateForm3R(rADD, dwRSrc, dwRSrc, MKSPECIAL(pEntry->chSpecial));
  regm_LoadMultiple(dwRSrc, dwRDest);

  /* Stack will be increased by an amount determined by DC in
   * regm_LoadMultiple.  However, we need to also account for the
   * immediate stack value that we consume here.
   */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* Store value on stack to address on stack.  Stack is reduced by two */

static void regm_StoreImmediate(const struct regm_opmap_s *pEntry,
                                OPTYPE *pOpCode,
                                struct procdata_s *pNode)
{
  uint32_t dwRSrc      = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwROperand1 = MKREG(g_dwStackOffset - 2*sINT_SIZE);

  TRACE(stderr, "[regm_StoreImmediate]");

  /* Use the immediate value as an index against the SPB/LSP */

  regm_GenerateForm3R(pEntry->chOpCode, dwRSrc, dwROperand1,
                      MKSPECIAL(pEntry->chSpecial));
  
  /* Reduce stack */

  g_dwStackOffset -= 2*sINT_SIZE;
}

/***********************************************************************/
/* Generic store multiple logic */

static void regm_StoreMultiple(uint32_t dwRDest, uint32_t dwRSrc)
{
  TRACE(stderr, "[regm_StoreMultiple]");

  if (g_bRegisterCountValid)
    {
      regm_GenerateForm3I(rSTM, dwRSrc, dwRDest, g_dwRegisterCount);

      /* Adjust the stack for the g_dwRegisterCount values added to the
       * stack.
       */

      g_dwStackOffset -= g_dwRegisterCount * sINT_SIZE;
      g_bRegisterCountValid = 0;
    }
  else
    {
      fatal(ePOFFCONFUSION);
    }
}

/***********************************************************************/
/* Store multiple values on stack to address on stack.  Stack is reduced
 * by an amount determined by the content of DC.
 */

static void regm_StoreMultipleImmediate(const struct regm_opmap_s *pEntry,
                                        OPTYPE *pOpCode,
                                        struct procdata_s *pNode)
{
  uint32_t dwRDest = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRSrc  = MKREG(g_dwStackOffset - (g_dwRegisterCount + 1)*sINT_SIZE);

  TRACE(stderr, "[regm_StoreMultipleImmediate]");

  /* Adjust the src for the SPB/LSP value and generate the multiple load */

  regm_GenerateForm3R(rADD, dwRDest, dwRDest, MKSPECIAL(pEntry->chSpecial));
  regm_StoreMultiple(dwRSrc, dwRDest);

  /* Stack will be increased by an amount determined by DC in
   * regm_StoreMultiple.  However, we need to also account for the
   * immediate stack value that we consume here.
   */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* Duplicate the TOS.  stack increases by one */

static void regm_Duplicate(const struct regm_opmap_s *pEntry,
                           OPTYPE *pOpCode,
                           struct procdata_s *pNode)
{
  uint32_t dwROperand1 = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRDest     = MKREG(g_dwStackOffset);

  TRACE(stderr, "[regm_Duplicate]");

  /* Generate the binary operation */

  regm_GenerateForm2R(rMOV, dwRDest, dwROperand1);

  /* Increment the stack */

  g_dwStackOffset += sINT_SIZE;
}

/***********************************************************************/
/* Put the immediate value at the top of the stack.  Increment stack */

static void regm_PushImmediate(const struct regm_opmap_s *pEntry,
                               OPTYPE *pOpCode,
                               struct procdata_s *pNode)
{
  uint32_t dwRDest = g_dwStackOffset;

  TRACE(stderr, "[regm_PushImmediate]");

  /* The value may be too large to represent with a MOVI, but we'll handle
   * that later.
   */

  regm_GenerateForm2I(rMOVI, dwRDest, GETARG(pOpCode));

  /* Increment the stack */

  g_dwStackOffset += sINT_SIZE;
}

/***********************************************************************/
/* Push the special register onto the stack.  Stack increments by one */

static void regm_PushSpecial(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode,
                             struct procdata_s *pNode)
{
  uint32_t dwRDest = g_dwStackOffset;

  TRACE(stderr, "[regm_PushSpecial]");

  regm_GenerateForm2R(rMOV, dwRDest, MKSPECIAL(pEntry->chSpecial));

  /* Increment the stack */

  g_dwStackOffset += sINT_SIZE;
}

/***********************************************************************/
/* Pop the TOS into the special register.  Stack decrements by one */

static void regm_PopSpecial(const struct regm_opmap_s *pEntry,
                            OPTYPE *pOpCode,
                            struct procdata_s *pNode)
{
  uint32_t dwROperand1 = MKREG(g_dwStackOffset - 1*sINT_SIZE);

  TRACE(stderr, "[regm_PopSpecial]");

  regm_GenerateForm2R(rMOV, MKSPECIAL(pEntry->chSpecial), dwROperand1);

  /* Decrement the stack */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* Save the immediate value in the data count register */

static void regm_SetDataCount(const struct regm_opmap_s *pEntry, 
                              OPTYPE *pOpCode,
                              struct procdata_s *pNode)
{
  /* We don't acutally use the DC register.  It is an artifact just
   * get here.  We save the byte count as a even number of registers.
   */

  g_dwRegisterCount     = (GETARG(pOpCode) + 3) >> 2;
  g_bRegisterCountValid = 1;
}

/***********************************************************************/

static void regm_Return(const struct regm_opmap_s *pEntry,
                        OPTYPE *pOpCode,
                        struct procdata_s *pNode)
{
  TRACE(stderr, "[regm_Return]");

  /* This should have been processed by the prologue/epilogue logic */

  fatal(ePOFFCONFUSION);
}

/***********************************************************************/
/* Load at offset from SPB/LSP.  Stack increases by one */

static void regm_LoadOffset(const struct regm_opmap_s *pEntry,
                            OPTYPE *pOpCode,
                            struct procdata_s *pNode)
{
  uint32_t dwRDest = g_dwStackOffset;

  TRACE(stderr, "[regm_LoadOffset]");

  /* Use the immediate value as an index against the SPB/LSP */

  regm_GenerateForm3I(pEntry->chOpCode, dwRDest, MKSPECIAL(pEntry->chSpecial),
                      GETARG(pOpCode) >> pEntry->chImmediate);
  
  /* Increment the stack */

  g_dwStackOffset += sINT_SIZE;
}

/***********************************************************************/
/* Load multiple registgers at offset from SPB/LSP.  Stack increase depends
 * on value in DC (only)
 */

static void regm_LoadMultipleOffset(const struct regm_opmap_s *pEntry,
                                    OPTYPE *pOpCode,
                                    struct procdata_s *pNode)
{
  uint32_t dwRSrc  = MKREG(g_dwStackOffset);
  uint32_t dwRDest = dwRSrc;

  TRACE(stderr, "[regm_LoadMultipleOffset]");

  regm_GenerateForm3R(rADD, dwRSrc, MKSPECIAL(pEntry->chSpecial),
                      GETARG(pOpCode));
  regm_LoadMultiple(dwRSrc, dwRDest);
}

/***********************************************************************/
/* Store at offset from SPB/LSP.  Stack decreases by one */

static void regm_StoreOffset(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode,
                             struct procdata_s *pNode)
{
  uint32_t dwRSrc = MKREG(g_dwStackOffset - 1*sINT_SIZE);

  TRACE(stderr, "[regm_StoreOffset]");

  /* Use the immediate value as an index against the SPB/LSP */

  regm_GenerateForm3I(pEntry->chOpCode, dwRSrc,
                      MKSPECIAL(pEntry->chSpecial),
                      GETARG(pOpCode) >> pEntry->chImmediate);
  
  /* Decrement the stack */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* Store multiple at offset from SPB/LSP.  Stack decreases an amount
 * determined by the content of the DC regsiter.
 */

static void regm_StoreMultipleOffset(const struct regm_opmap_s *pEntry,
                                     OPTYPE *pOpCode,
                                     struct procdata_s *pNode)
{
  uint32_t dwRSrc = MKREG(g_dwStackOffset - g_dwRegisterCount*sINT_SIZE);
  uint32_t dwRDest;

  TRACE(stderr, "[regm_StoreMultipleOffset]");

  regm_GenerateForm3R(rADD, dwRDest, MKSPECIAL(pEntry->chSpecial),
                      GETARG(pOpCode));
  regm_StoreMultiple(dwRSrc, dwRDest);
}

/***********************************************************************/
/* Load value using index on stack + argument + SPB/LSP. Stack is unchanged */

static void regm_LoadIndexed(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode,
                             struct procdata_s *pNode)
{
  uint32_t dwRIndex = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRDest  = dwRIndex;

  TRACE(stderr, "[regm_LoadIndexed]");

  /* Add the SPB/LSP to the index to make it relative to the stack,
   * then use this with the immediate values to obtain the data.
   */

  regm_GenerateForm3R(rADD, dwRIndex, dwRIndex,
                      MKSPECIAL(pEntry->chSpecial));
  regm_GenerateForm3I(pEntry->chOpCode, dwRDest, dwRDest,
                      GETARG(pOpCode) >> pEntry->chImmediate);
}

/***********************************************************************/
/* Load multiple values using index on stack + argument + SPB/LSP. Stack
 * will increase my an amount that depends on the contents of DC.
 */

static void regm_LoadMultipleIndexed(const struct regm_opmap_s *pEntry,
                                     OPTYPE *pOpCode,
                                     struct procdata_s *pNode)
{
  uint32_t dwRIndex = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRSrc   = dwRIndex;
  uint32_t dwRDest  = dwRIndex;

  TRACE(stderr, "[regm_LoadMultipleIndexed]");

  /* Add the SPB/LSP to the index to make it relative to the stack,
   * add the offset, then generate the multple load.
   */

  regm_GenerateForm3R(rADD, dwRSrc, dwRIndex,
                      MKSPECIAL(pEntry->chSpecial));
  regm_GenerateForm3I(rADDI, dwRSrc, dwRSrc, GETARG(pOpCode));
  regm_LoadMultiple(dwRSrc, dwRDest);

  /* Stack will be increased by an amount determined by DC in
   * regm_LoadMultiple.  However, we need to also account for the
   * index stack value that we consume here.
   */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* Store value at TOS to index + offset + SPB/LSP.  Stack decreases by two */

static void regm_StoreIndexed(const struct regm_opmap_s *pEntry,
                              OPTYPE *pOpCode,
                              struct procdata_s *pNode)
{
  uint32_t dwRSrc   = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRIndex = MKREG(g_dwStackOffset - 2*sINT_SIZE);

  TRACE(stderr, "[regm_StoreIndexed]");

  /* Add the LSP to the index to make it relative to the stack,
   * then use this with the immediate values to obtain the data.
   */

  regm_GenerateForm3R(rADD, dwRIndex, dwRIndex,
                      MKSPECIAL(pEntry->chSpecial));
  regm_GenerateForm3I(pEntry->chOpCode, dwRSrc, dwRIndex,
                      GETARG(pOpCode) >> pEntry->chImmediate);

  /* Decrement the stack */

  g_dwStackOffset -= 2*sINT_SIZE;
}

/***********************************************************************/
/* Store values at TOS to index + offset + SPB/LSP.  Stack decreases by
 * amount determined by the content of the DC register.
 */

static void regm_StoreMultipleIndexed(const struct regm_opmap_s *pEntry,
                                      OPTYPE *pOpCode,
                                      struct procdata_s *pNode)
{
  uint32_t dwRIndex = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRSrc   = MKREG(g_dwStackOffset - (g_dwRegisterCount + 1)*sINT_SIZE);
  uint32_t dwRDest;

  TRACE(stderr, "[regm_StoreMultipleIndexed]");

  /* Adjust the src for the SPB/LSP value and generate the multiple load */

  regm_GenerateForm3R(rADD, dwRDest, dwRIndex,
                      MKSPECIAL(pEntry->chSpecial));
  regm_GenerateForm3I(rADDI, dwRDest, dwRDest, GETARG(pOpCode));
  regm_StoreMultiple(dwRSrc, dwRDest);

  /* Stack will be increased by an amount determined by DC in
   * regm_StoreMultiple.  However, we need to also account for the
   * immediate stack value that we consume here.
   */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* These pcodes are all conditional branch operations. The pcode form
 * takes one input (that is compared with zero) and branches based
 * the result.  The stack is decremented by one.
 */

static void regm_ConditionalBranchVsZero(const struct regm_opmap_s *pEntry,
                                         OPTYPE *pOpCode,
                                         struct procdata_s *pNode)
{
  uint32_t dwUnaryRegister = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwCcRegister    = MKCCREG(g_dwStackOffset);

  TRACE(stderr, "[regm_ConditionalBranchVsZero]");


  regm_GenerateForm1ICc(rCMPI, dwUnaryRegister, 0, dwCcRegister);
  regm_GenerateForm4ICc(pEntry->chOpCode, GETARG(pOpCode), dwCcRegister);

  /* Decrement the stack */

  g_dwStackOffset -= sINT_SIZE;
}

/***********************************************************************/
/* These pcodes are all conditional branch operations. The pcode form
 * takes two inputs that are compared.  The pcode branches on the result
 * of the comparison.  The stack is reduced by two.
 */

static void regm_ConditionalBranchBinary(const struct regm_opmap_s *pEntry,
                                         OPTYPE *pOpCode,
                                         struct procdata_s *pNode)
{
  uint32_t dwROperand1  = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwROperand2  = MKREG(g_dwStackOffset - 2*sINT_SIZE);
  uint32_t dwCcRegister = MKCCREG(g_dwStackOffset);

  TRACE(stderr, "[regm_BinaryComparison]");

  /* Generate the compare and branch */

  regm_GenerateForm1RCc(rCMP, dwROperand1, dwROperand2, dwCcRegister);
  regm_GenerateForm4ICc(pEntry->chOpCode, GETARG(pOpCode), dwCcRegister);
      
  /* Reduce stack */

  g_dwStackOffset -= 2*sINT_SIZE;
}

/***********************************************************************/
/* Branch unconditionally.  The stack is not changed */

static void regm_UnconditionalBranch(const struct regm_opmap_s *pEntry,
                                     OPTYPE *pOpCode,
                                     struct procdata_s *pNode)
{
  TRACE(stderr, "[regm_UnconditionalBranch]");
  regm_GenerateForm4I(rB, GETARG(pOpCode));
}

/***********************************************************************/
/* Add constant value to special register.  Stack does not change */

static void regm_IncrementSpecial(const struct regm_opmap_s *pEntry,
                                  OPTYPE *pOpCode,
                                  struct procdata_s *pNode)
{
  int32_t dwIncrement = (int32_t)(GETOP(pOpCode));
  uint32_t dwRSpecial  = MKSPECIAL(pEntry->chSpecial);
  TRACE(stderr, "[regm_IncrementSpecial]");

  /* The value may be too large to represent with a MOVI, but we'll handle
   * that later.
   */

  if (dwIncrement < 0)
    {
      regm_GenerateForm3I(rSUBI, dwRSpecial, dwRSpecial, -dwIncrement);
    }
  else if (dwIncrement > 0)
    {
      regm_GenerateForm3I(rADDI, dwRSpecial, dwRSpecial, dwIncrement);
    }

  if (pEntry->chSpecial == SP)
    {
      g_dwStackOffset += dwIncrement;
    }
}

/***********************************************************************/
/* Load address at offset from special register.  Stack increases by one */

static void regm_LoadAddress(const struct regm_opmap_s *pEntry,
                             OPTYPE *pOpCode,
                             struct procdata_s *pNode)
{
  uint32_t dwRDest = g_dwStackOffset;

  TRACE(stderr, "[regm_LoadAddress]");

  /* Use the immediate value as an index against the SPB/LSP */

  regm_GenerateForm3I(rADD, dwRDest, MKSPECIAL(pEntry->chSpecial),
                      GETARG(pOpCode));
  
  /* Increment the stack */

  g_dwStackOffset += sINT_SIZE;
}

/***********************************************************************/
/* Load address at indexed offset from special register.  Stack is unchanged */

static void regm_LoadAddressIndexed(const struct regm_opmap_s *pEntry,
                                    OPTYPE *pOpCode,
                                    struct procdata_s *pNode)
{
  uint32_t dwRIndex = MKREG(g_dwStackOffset - 1*sINT_SIZE);
  uint32_t dwRDest  = dwRIndex;

  TRACE(stderr, "[regm_LoadAddressIndexed]");

  /* Add the LSP or SPB to the index to make it relative to the stack,
   * then use this with the immediate values to obtain the data.
   */

  regm_GenerateForm3R(rADD, dwRIndex, dwRIndex,
                      MKSPECIAL(pEntry->chSpecial));
  regm_GenerateForm3I(rADD, dwRDest, dwRIndex, GETARG(pOpCode));
}

/***********************************************************************/

static void regm_SetupOutArgs(uint32_t nParms, const uint32_t *pwArgSize)
{
  int nArgRegs;
  int32_t dwOffset;
  int i;

  for (i = 0, nArgRegs = 0; i < nParms; i++)
    {
      nArgRegs += (pwArgSize[i] + 3) >> 2;
    }

  /* Emit move instructions to handle each */
    
  dwOffset = g_dwStackOffset  - sINT_SIZE;
  for (i = 0;  i < nArgRegs; i++)
    {
      uint32_t dwDest = MKOUTARG(i);
      uint32_t dwSrc  = MKREG(dwOffset);

      regm_GenerateForm2R(rMOV, dwDest, dwSrc);
      dwOffset -= sINT_SIZE;
    }
}

/***********************************************************************/

static void regm_MapInRet(uint32_t wRetSize)
{
  int nRetRegs;
  int32_t dwOffset;
  int i;

  /* Get the number of registers that are returned */

  nRetRegs += (wRetSize + 3) >> 2;

  /* Emit move instructions to handle each */
#warning "This offset is not correct"
  dwOffset = g_dwStackOffset  - sINT_SIZE;
  for (i = 0;  i < nRetRegs; i++)
    {
      uint32_t dwSrc  = MKINRET(i);
      uint32_t dwDest = MKREG(dwOffset);

      regm_GenerateForm2R(rMOV, dwDest, dwSrc);
      dwOffset -= sINT_SIZE;
    }
}

/***********************************************************************/

static void regm_PCal(const struct regm_opmap_s *pEntry, OPTYPE *pOpCode,
                      struct procdata_s *pNode)
{
  poffLibDebugFuncInfo_t *pFuncInfo = pNode->pFuncInfo;

  TRACE(stderr, "[regm_PCal]");

  if (!pFuncInfo)
    {
      fatal(ePOFFCONFUSION);
    }

  /* Map the "output" parameter stack to a set of "output" argument
   * registers.
   */

  regm_SetupOutArgs(pFuncInfo->nparms, pFuncInfo->argsize);

  regm_GenerateForm3I(rADDI, MKSPECIAL(SP),  MKSPECIAL(SP),  3*sINT_SIZE);
  regm_GenerateForm3I(rST,   MKSPECIAL(LSP), MKSPECIAL(SP), -3);
  regm_GenerateForm3I(rST,   MKSPECIAL(BRG), MKSPECIAL(SP), -2);
  regm_GenerateForm4I(rBL,   GETARG(pOpCode));

#warning "This is in the wrong place"
  regm_MapInRet(pFuncInfo->retsize);

  /* Increment the stack */

  g_dwStackOffset += 3*sINT_SIZE;
}

/***********************************************************************/

static void regm_SysIo(const struct regm_opmap_s *pEntry, OPTYPE *pOpCode,
                       struct procdata_s *pNode)
{
  const struct regm_builtin_s *pBuiltIn;
  uint32_t xop;

  TRACE(stderr, "[regm_SysIo]");

  /* Get the function information for this sysio xop */

  xop = GETARG(pOpCode);
  if (xop >= MAX_XOP)
    {
      fatal(ePOFFCONFUSION);
    }

  pBuiltIn = &g_rgSysIoBuiltIns[xop];

  /* Map the "output" parameter stack to a set of "output" argument
   * registers.
   */

  regm_SetupOutArgs(pBuiltIn->nParms, pBuiltIn->wArgSize);

  /* Generate a call to the runtime library */

#warning "Not implemented"

  /* Handled returned values */

  regm_MapInRet(pBuiltIn->wRetSize);
}

/***********************************************************************/

static void regm_LibCall(const struct regm_opmap_s *pEntry, OPTYPE *pOpCode,
                         struct procdata_s *pNode)
{
  const struct regm_builtin_s *pBuiltIn;
  uint32_t lbop;

  TRACE(stderr, "[regm_LibCall]");

  /* Get the function information for this library op */

  lbop = GETARG(pOpCode);
  if (lbop >= MAX_LBOP)
    {
      fatal(ePOFFCONFUSION);
    }

  pBuiltIn = &g_rgLibCallBuiltIns[lbop];

  /* Map the "output" parameter stack to a set of "output" argument
   * registers.
   */

  regm_SetupOutArgs(pBuiltIn->nParms, pBuiltIn->wArgSize);

  /* Generate a call to the runtime library */

#warning "Not implemented"

  /* Handled returned values */

  regm_MapInRet(pBuiltIn->wRetSize);
}

/***********************************************************************/

static void regm_Float(const struct regm_opmap_s *pEntry, OPTYPE *pOpCode,
                       struct procdata_s *pNode)
{
  static const struct regm_builtin_s *pFopBuiltIns;
  const struct regm_builtin_s *pBuiltIn;
  uint32_t foptab;
  uint32_t fop;

  TRACE(stderr, "[regm_FLoat]");

  /* Select the correct table for the builtin */

  foptab = (GETARG(pOpCode) & ~fpMASK) >> fpSHIFT;
  pFopBuiltIns = g_prgFopBuiltIns[foptab];

  /* Select the correct function from the table for this floating
   * point operation.
   */

  fop = GETARG(pOpCode) & fpMASK;
  if (fop >= MAX_FOP)
    {
      fatal(ePOFFCONFUSION);
    }
  pBuiltIn = &pFopBuiltIns[fop];

  /* Map the "output" parameter stack to a set of "output" argument
   * registers.
   */

  regm_SetupOutArgs(pBuiltIn->nParms, pBuiltIn->wArgSize);

  /* Generate a call to the runtime library */

#warning "Not implemented"

  /* Handled returned values */

  regm_MapInRet(pBuiltIn->wRetSize);
}

/***********************************************************************/

static void regm_IllegalPCode(const struct regm_opmap_s *pEntry,
                              OPTYPE *pOpCode,
                              struct procdata_s *pNode)
{
  TRACE(stderr, "[regm_IllegalPCode]");
  fatal(eILLEGALOPCODE);
}

/***********************************************************************/

static void regm_GenerateRegm(struct procdata_s *pNode, void *pvArg)
{
  int32_t dwFrameSize = 0;
  int i, j;

  TRACE(stderr, "[regm_GenerateRegm]");

  /* Analyze the proc/func prologue */

  i = 0; j = pNode->nPCodes;
  if (GETOP(&pNode->pPCode[0]) == oINDS)
    {
      dwFrameSize = GETARG(&pNode->pPCode[0]);
      i++; j--;
    }
  regm_GeneratePrologue(dwFrameSize);

  /* Set the initial stack offset. Parameters will look like
   * negative offsets; local stack will look positive.
   */

  g_dwStackOffset = dwFrameSize;

  /* Generate regm code for each p-code */

  for (; i < j; i++)
    {
      const struct regm_opmap_s *rgOpMap;
      uint8_t chOpCode = GETOP(&pNode->pPCode[i]);

      /* Select the right decode table */
      
      if ((chOpCode & o32) != 0)
        {
          rgOpMap  = vrgOpMap2;
          chOpCode &= ~o32;
        }
      else
        {
          rgOpMap  = vrgOpMap1;
        }

      /* Make sure that the table index is within range */

      if (chOpCode > 63)
        {
          fatal(eBADSHORTINT);
        }

      /* Perform the opcode mapping */

      rgOpMap->pMapper(rgOpMap, &pNode->pPCode[i], pNode);
    }

  /* If a frame was obtained at the beginning, make sure that 
   * there is matching frame release logic at the end.
   */

  if (dwFrameSize > 0)
    {
      if ((GETOP(&pNode->pPCode[i]) != oINDS) ||
          (dwFrameSize != -(int32_t)GETARG(&pNode->pPCode[i])))
        {
          fatal(ePOFFCONFUSION);
        }
      i++;
    }
      
  /* Analyze the proc/func epilogue */

  if ((GETOP(&pNode->pPCode[i]) != oRET) &&
      (GETOP(&pNode->pPCode[i]) != oEND))
    {
      fatal(ePOFFCONFUSION);
    }
  regm_GenerateEpilogue(dwFrameSize);
}

/***********************************************************************/

static int regm_Pass2Node(struct procdata_s *pNode, void *pvArg)
{
  TRACE(stderr, "[regm_Pass2Node]");

  /* Generate code for each child of this proc/func block */

  if (pNode->child)
    {
      (void)regm_ForEachChild(pNode->child, regm_Pass2Node, pvArg);
    }

  /* Generate code for this node */

  regm_GenerateRegm(pNode, pvArg);

  /* Does this node have a peer at the same level?  If so, then
   * do the same for its peer.
   */

  if (pNode->peer)
    {
      (void)regm_Pass2Node(pNode->peer, pvArg);
    }

  return 0;
}

/**********************************************************************
 * Public Functions
 **********************************************************************/

/***********************************************************************/
/* Pass 2: Convert the buffered pcode to the basic register model with
 * an indefinite number of registers (arguments, general, and special
 * registers) and with 32-bit immediate size.
 */

void regm_Pass2(poffHandle_t hPoff)
{
  TRACE(stderr, "[regm_Pass2]");

  /* Initiate traversal at the root node */

  (void)regm_Pass2Node(regm_GetRootNode(), NULL);
}

/***********************************************************************/

