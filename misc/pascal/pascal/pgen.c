/**********************************************************************
 * pgen.c
 * P-Code generation logic
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
#include <string.h>
#include <errno.h>

#include "config.h"   /* Configuration */
#include "keywords.h" /* Standard types */
#include "pasdefs.h"  /* Common types */
#include "ptdefs.h"   /* Token / symbol table definitions */
#include "podefs.h"   /* Logical opcode definitions */
#include "pedefs.h"   /* error code definitions */

#include "pas.h"      /* Global variables */
#include "poff.h"     /* For POFF file format */
#include "pofflib.h"  /* For poff*() functions*/
#include "pinsn.h"    /* (DEBUG only) */
#include "perr.h"     /* error() */

#include "pproc.h"    /* for actualParameterSize */
#include "pgen.h"     /* (to verify prototypes in this file) */

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

#define UNDEFINED_LEVEL (-1)
#define INVALID_PCODE   (-1)

#define LEVEL_DEFINED(l)  ((int32_t)(l) >= 0)
#define PCODE_VALID(p)    ((int32_t)(p) >= 0)

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

static int32_t  g_currentStackLevelReference  = UNDEFINED_LEVEL;
static uint32_t g_nStackLevelReferenceChanges = 0;

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************/
/* Generate a stack reference opcode to a global variable residing at
 * static nesting level zero.
 */

static void
pas_GenerateLevel0StackReference(enum pcode_e eOpCode, STYPE *pVar)
{
  /* Sanity checking.  Double check nesting level and also since this is
   * a level zero reference, then the offset must be positive
   */

  if ((pVar->sLevel != 0) || (pVar->sParm.v.offset < 0))
    {
      error(eHUH);
    }
  else
    {
      /* Generate the P-code */

      insn_GenerateDataOperation(eOpCode, pVar->sParm.v.offset);

      /* If the variable is undefined, also generate a relocation
       * record.
       */

      if ((pVar->sParm.v.flags & SVAR_EXTERNAL) != 0)
        {
          (void)poffAddRelocation(poffHandle, RLT_LDST,
                                  pVar->sParm.v.symIndex, 0);
        }
    }
}

/***********************************************************************/
/* There are some special P-codes for accessing stack data at static
 * nesting level 0.  Check if the specified opcode is one of those.  If
 * so, return the mapped opcode.  Otherwise, return INVALID_PCODE.
 */

static int32_t
pas_GetLevel0Opcode(enum pcode_e eOpCode)
{
  switch (eOpCode)
    {
    case opLDS:   return opLD;
    case opLDSH:  return opLDH;
    case opLDSB:  return opLDB;
    case opLDSM:  return opLDM;
    case opSTS:   return opST;
    case opSTSB:  return opSTB;
    case opSTSM:  return opSTM;
    case opLDSX:  return opLDX;
    case opLDSXB: return opLDXB;
    case opLDSXM: return opLDXM;
    case opSTSX:  return opSTX;
    case opSTSXB: return opSTXB;
    case opSTSXM: return opSTXM;
    case opLAS:   return opLA;
    case opLASX:  return opLAX;
    default:      return INVALID_PCODE;
    }
}

/***********************************************************************/
/* A new static nesting level has been encountered.  Check if we need
 * to reset the level stack pointer (LSP) register (assuming that the
 * architecutre has one).
 */

static void
pas_SetLevelStackPointer(uint32_t dwLevel)
{
  if (dwLevel != g_currentStackLevelReference)
    {
      /* Set the level stack pointer (LSP) register */

      insn_SetStackLevel(dwLevel);

      /* Remember the setting so that we do not reset the LSP until
       * the level changes (or it is invalidated).
       */

      g_currentStackLevelReference = dwLevel;
      g_nStackLevelReferenceChanges++;
    }
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************/
/* Return the current setting of the level stack pointer (LSP) register
 * -- assuming that the underlying architecure may have one.
 */

int32_t pas_GetCurrentStackLevel(void)
{
  return g_currentStackLevelReference;
}

/***********************************************************************/
/* Invalidate the current stack level register setting.  This will cause
 * us to reset the LSP when the next stack level reference is encountered.
 */

void pas_InvalidateCurrentStackLevel(void)
{
  g_currentStackLevelReference = UNDEFINED_LEVEL;
  g_nStackLevelReferenceChanges++;
}

/***********************************************************************/
/* Set the stack level pointer to known value.  This is done when in
 * while and for loop processing.  The value of the LSP will be that
 * as sampled at the top of the lop not necessarily the value at the
 * bottom of the loop.
 */

void pas_SetCurrentStackLevel(int32_t dwLsp)
{
  g_currentStackLevelReference = dwLsp;
  g_nStackLevelReferenceChanges++;
}

/***********************************************************************/
/* Get the number of changes made to the level stack pointer.  This is
 * useful by compiler logic to determine if the stack level pointer was
 * ever changed by any logic path.
 */

uint32_t pas_GetNStackLevelChanges(void)
{
  return g_nStackLevelReferenceChanges;
}

/***********************************************************************/
/* Generate the most simple of all P-codes */

void pas_GenerateSimple(enum pcode_e eOpCode)
{
  insn_GenerateSimple(eOpCode);
}

/***********************************************************************/
/* Generate a P-code with a single data argument */

void pas_GenerateDataOperation(enum pcode_e eOpCode, int32_t dwData)
{
  insn_GenerateDataOperation(eOpCode, dwData);
}

/***********************************************************************/
/* This function is called just before a multiple register operation is
 * is generated.  This should generate logic to specify the size of the
 * multiple register operation (in bytes, not registers). This may translate
 * into different operations on different architectures.  Typically,
 * this would generate a push of the size onto the stack or, perhaps,
 * setting of a dedicated count register.
 */

void pas_GenerateDataSize(int32_t dwDataSize)
{
  insn_GenerateDataSize(dwDataSize);
}

/***********************************************************************/
/* Generate a floating point operation */

void pas_GenerateFpOperation(uint8_t fpOpcode)
{
  insn_GenerateFpOperation(fpOpcode);
}

/***********************************************************************/
/* Generate an IO operation */

void pas_GenerateIoOperation(uint16_t ioOpcode, uint16_t fileNumber)
{
  insn_GenerateIoOperation(ioOpcode, fileNumber);
}

/***********************************************************************/
/* Generate a pseudo call to a built-in, standard pascal function */

void pas_BuiltInFunctionCall(uint16_t libOpcode)
{
  insn_BuiltInFunctionCall(libOpcode);
}

/***********************************************************************/
/* Generate a reference to data on the data stack using the specified
 * level and offset.
 */

void pas_GenerateLevelReference(enum pcode_e eOpCode, uint16_t wLevel,
                                int32_t dwOffset)
{
  /* Is this variable declared at level 0 (i.e., it has global scope)
   * that is being offset via a nesting level?
   */

  if (wLevel == 0)
    {
      int32_t level0Opcode = pas_GetLevel0Opcode(eOpCode);
      if (PCODE_VALID(level0Opcode))
        {
          insn_GenerateDataOperation(level0Opcode, dwOffset);
          return;
        }
    }

  /* We get here if the reference is at some static nesting level
   * other that zero OR if there is no special PCode to reference
   * data at static nesting level 0 for this operation.
   *
   * Check if we have to change the level stack pointer (LSP) register
   * (assuming that the architecture has one).
   */

  pas_SetLevelStackPointer(wLevel);

  /* Then generate the opcode passing the level in the event that the
   * architecture does not have an LSP.
   */

  insn_GenerateLevelReference(eOpCode, wLevel, dwOffset);
}

/***********************************************************************/
/* Generate a stack reference opcode, handling references to undefined
 * stack offsets.
 */

void pas_GenerateStackReference(enum pcode_e eOpCode, STYPE *pVar)
{
  /* Is this variable declared at level 0 (i.e., it has global scope)
   * that is being offset via a nesting level?
   */

  if (pVar->sLevel == 0)
    {
      int32_t level0Opcode = pas_GetLevel0Opcode(eOpCode);
      if (PCODE_VALID(level0Opcode))
        {
          pas_GenerateLevel0StackReference(level0Opcode, pVar);
          return;
        }
    }

  /* We get here if the reference is at some static nesting level
   * other that zero OR if there is no special PCode to reference
   * data at static nesting level 0 for this operation.
   *
   * Check if we have to change the level stack pointer (LSP) register
   * (assuming that the architecture has one).
   */

  pas_SetLevelStackPointer(pVar->sLevel);

  /* Generate the P-Code at the defined offset and with the specified
   * static level offset (in case that the architecture does not have
   * an LSP)
   */

  insn_GenerateLevelReference(eOpCode, (level - pVar->sLevel),
                              pVar->sParm.v.offset);
}

/***********************************************************************/
/* Generate a procedure call and an associated relocation record if the
 * called procedure is external.
 */

void
pas_GenerateProcedureCall(STYPE *pProc)
{
  /* sLevel is the level at which the procedure was declared.  We need
   * to set the SLP to this value prior to the call (on some architectures
   * where the SLP is pushed onto the stack by the procedure
   * call).
   */

  pas_SetLevelStackPointer(pProc->sLevel);

  /* Then generate the procedure call (passing the level again for those
   * architectures that do not support the SLP.
   */

  insn_GenerateProcedureCall(pProc->sLevel, pProc->sParm.p.label);

  /* If the variable is undefined, also generate a relocation
   * record.
   */

#if 0 /* Not yet */
  if ((pVar->sParm.p.flags & SVAR_EXTERNAL) != 0)
    {
      /* For now */
# error "Don't know what last parameter should be"
      (void)poffAddRelocation(poffHandle, RLT_PCAL,
                              pVar->sParm.p.symIndex,
                              0);
    }
#endif

  /* Any logic after the procedure/function call return must assume
   * that the last level reference is unknown.
   */

  pas_InvalidateCurrentStackLevel();
}

/***********************************************************************/

void pas_GenerateLineNumber(uint16_t wIncludeNumber, uint32_t dwLineNumber)
{
  insn_GenerateLineNumber(wIncludeNumber, dwLineNumber);
}

/***********************************************************************/

void pas_GenerateDebugInfo(STYPE *pProc, uint32_t dwReturnSize)
{
  int i;

  /* Allocate a container to pass the proc information to the library */

  uint32_t nparms                      = pProc->sParm.p.nParms;
  poffLibDebugFuncInfo_t *pContainer = poffCreateDebugInfoContainer(nparms);

  /* Put the proc information into the container */

  pContainer->value   = pProc->sParm.p.label;
  pContainer->retsize = dwReturnSize;
  pContainer->nparms  = nparms;

  /* Add the argument information to the container */

   for (i = 0; i < nparms; i++)
     {
       pContainer->argsize[i] = actualParameterSize(pProc, i+1);
     }

   /* Add the contained information to the library */

   poffAddDebugFuncInfo(poffHandle, pContainer);

   /* Release the container */

   poffReleaseDebugFuncContainer(pContainer);
}

/***********************************************************************/
/* Generate description of a level 0 stack variable that can be
 * exported by a unit.
 */

void pas_GenerateStackExport(STYPE *pVar)
{
  poffLibSymbol_t symbol;

#if CONFIG_DEBUG
  /* Get the parent type of the variable */

  STYPE *typePtr = pVar->sParm.v.parent;

  /* Perform some sanity checking:
   * - Must have a parent type
   * - Must not be declared external
   * - Must be declared at static nesting level zero
   */

  if ((!typePtr) ||
      ((pVar->sParm.v.flags & SVAR_EXTERNAL) != 0) ||
      (pVar->sLevel != 0))
    {
      error(eSYMTABINTERNAL);
    }
#endif

  /* Create the symbol structure */

  symbol.type  = STT_DATA;
  symbol.align = STA_8BIT; /* for now */
  symbol.flags = STF_NONE;
  symbol.name  = pVar->sName;
  symbol.value = pVar->sParm.v.offset;
  symbol.size  = pVar->sParm.v.size;

  /* Add the symbol to the symbol table */

  (void)poffAddSymbol(poffHandle, &symbol);
}

/***********************************************************************/
/* Generate description of a level 0 stack variable that must be
 * imported by a program or unit from a unit.
 */

void pas_GenerateStackImport(STYPE *pVar)
{
  poffLibSymbol_t symbol;

#if CONFIG_DEBUG
  /* Get the parent type of the variable */

  STYPE *typePtr = pVar->sParm.v.parent;

  /* Perform some sanity checking
   * - Must have a parent type
   * - Must be declared external
   * - Must be declared at static nesting level zero
   */

  if ((!typePtr) ||
      ((pVar->sParm.v.flags & SVAR_EXTERNAL) == 0) ||
      (pVar->sLevel != 0))
    {
      error(eSYMTABINTERNAL);
    }
#endif

  /* Create the symbol structure */

  symbol.type  = STT_DATA;
  symbol.align = STA_8BIT; /* for now */
  symbol.flags = STF_UNDEFINED;
  symbol.name  = pVar->sName;
  symbol.value = pVar->sParm.v.offset; /* for now */
  symbol.size  = pVar->sParm.v.size;

  /* Add the symbol to the symbol table */

  pVar->sParm.v.symIndex = poffAddSymbol(poffHandle, &symbol);
}

/***********************************************************************/
/* Generate description of a level 0 procedure or function that can be
 * exported by a unit.
 */

void pas_GenerateProcExport(STYPE *pProc)
{
  poffLibSymbol_t symbol;

#if CONFIG_DEBUG
  /* Get the parent type of the function (assuming it is a function) */

  STYPE *typePtr = pProc->sParm.p.parent;

  /* Perform some sanity checking */

  /* Check for a function reference which must have a valid parent type */

  if ((pProc->sKind == sFUNC) && (typePtr != NULL));

  /* Check for a procedure reference which must not have a valid type */

  else  if ((pProc->sKind == sPROC) && (typePtr == NULL));

  /* Anything else is an error */

  else
    error(eSYMTABINTERNAL);

  /* The function / procedure should NOT be declared external and
   * only procedures declared at static nesting level zero can
   * be exported.
   */

  if (((pProc->sParm.p.flags & SPROC_EXTERNAL) != 0) ||
      (pProc->sLevel != 0))
    error(eSYMTABINTERNAL);
#endif

  /* Everthing looks okay.  Create the symbol structure */

  if (pProc->sKind == sPROC)
    symbol.type  = STT_PROC;
  else
    symbol.type  = STT_FUNC;

  symbol.align = STA_NONE;
  symbol.flags = STF_NONE;
  symbol.name  = pProc->sName;
  symbol.value = pProc->sParm.p.label;
  symbol.size  = 0;

  /* Add the symbol to the symbol table */

  (void)poffAddSymbol(poffHandle, &symbol);
}

/***********************************************************************/
/* Generate description of a level 0 procedure or function that must be
 * imported by a program or unit from a unit.
 */

void pas_GenerateProcImport(STYPE *pProc)
{
  poffLibSymbol_t symbol;

#if CONFIG_DEBUG
  /* Get the parent type of the function (assuming it is a function) */

  STYPE *typePtr = pProc->sParm.p.parent;

  /* Perform some sanity checking */

  /* Check for a function reference which must have a valid parent type */

  if ((pProc->sKind == sFUNC) && (typePtr != NULL));

  /* Check for a procedure reference which must not have a valid type */

  else  if ((pProc->sKind == sPROC) && (typePtr == NULL));

  /* Anything else is an error */

  else
    error(eSYMTABINTERNAL);

  /* The function / procedure should also be declared external and
   * only procedures declared at static nesting level zero can
   * be exported.
   */

  if (((pProc->sParm.p.flags & SPROC_EXTERNAL) == 0) ||
      (pProc->sLevel != 0))
    error(eSYMTABINTERNAL);
#endif

  /* Everthing looks okay.  Create the symbol structure */

  if (pProc->sKind == sPROC)
    symbol.type  = STT_PROC;
  else
    symbol.type  = STT_FUNC;

  symbol.align = STA_NONE;
  symbol.flags = STF_UNDEFINED;
  symbol.name  = pProc->sName;
  symbol.value = pProc->sParm.p.label;
  symbol.size  = 0;

  /* Add the symbol to the symbol table */

  pProc->sParm.p.symIndex = poffAddSymbol(poffHandle, &symbol);
}
