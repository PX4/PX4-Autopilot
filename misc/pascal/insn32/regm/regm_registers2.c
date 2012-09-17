/**********************************************************************
 * regm_registers.c
 * Pass 2 register management functions
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
#include "pofflib.h"
#include "pedefs.h"
#include "perr.h"

#include "rinsn32.h"

#include "regm.h"
#include "regm_pass2.h"
#include "regm_registers2.h"

/**********************************************************************
 * Definitions
 **********************************************************************/

#define INITIAL_RCODE2_ALLOC 150
#define RCODE2_REALLOC        50

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

/**********************************************************************
 * Public Variables
 **********************************************************************/

struct regm_rcode2_s *g_pRCode2 = NULL;
uint32_t              g_nRCode2 = 0;

/**********************************************************************
 * Private Variables
 **********************************************************************/

static uint32_t g_nRCode2Alloc = 0;

static const char * const g_prgReg2Names[NREGISTER_TYPES] =
  { "Z", "X", "CC", "A", "R", "V", "V", "S" };

static const char * const g_prgSpecialReg2Names[NSPECIAL_REGISTERS2] =
  { "SPB", "SP", "BRG", "LSP", "CSB", "CSP", "PC", "DC", "LR", "CC"};

/**********************************************************************
 * Private Functions
 **********************************************************************/

/***********************************************************************/

static void regm_CheckRCode2Alloc(void)
{
  /* If the RCode buffer has never been allocated, allocate it now. */

  if (!g_pRCode2)
    {
      /* Allocate an inital buffer to hold the instructions */

      g_pRCode2 = (struct regm_rcode2_s*)
        malloc(INITIAL_RCODE2_ALLOC*sizeof(struct regm_rcode2_s));
      if (!g_pRCode2)
        {
          fatal(eNOMEMORY);
        }

      g_nRCode2Alloc = INITIAL_RCODE2_ALLOC;
    }

  /* The buffer has already been allocated, is there space for one
   * more RCode?
   */

  else if (g_nRCode2 >= g_nRCode2Alloc)
    {
      /* If not, then reallocate the array */

      g_pRCode2 = (struct regm_rcode2_s*)
        realloc(g_pRCode2, g_nRCode2Alloc*sizeof(struct regm_rcode2_s));
      if (!g_pRCode2)
        {
          fatal(eNOMEMORY);
        }

      g_nRCode2Alloc += RCODE2_REALLOC;
    }
}

/***********************************************************************/

struct regm_rcode2_s *regm_AllocateRCode2(void)
{
  struct regm_rcode2_s *pRetSlot;

  /* Make sure we have memory allocated in the array for another
   * RCode.
   */

  regm_CheckRCode2Alloc();

  /* Return the reference to the next RCode slot */

  pRetSlot = &g_pRCode2[g_nRCode2];
  g_nRCode2++;
  return pRetSlot;
}

static void regm_PrintSpecialReg2(FILE *pStream, uint32_t dwRegister)
{
  int wRegNo = regm_GetRegNo(dwRegister);
  if (wRegNo >= NSPECIAL_REGISTERS2)
    fputc('?', pStream);
  else
    fputs(g_prgSpecialRegNames[wRegNo], pStream);
}

static void regm_PrintReg2(FILE *pStream, uint32_t dwRegister)
{
  int wKind  = regm_GetKind(dwRegister);
  int wRegNo = regm_GetRegNo(dwRegister)
  if (wKind >= REGISTERS_TYPES)
    fprintf(pStream, "?%d", wRegNo);
  else if (wKind == SPECIAL_REG)
    regm_PrintSpecialReg(fg, dwRegister);
  else
    fprintf(pStream, "%s%d", g_prgRegNames[wKind], wRegNo);
}

static void regm_PrintDebugReg(const char *string, uint32_t dwRegister)
{
  if (vRegmDebug)
    {
      fputs(string, DEBUG_FILE);
      regm_PrintReg2(DEBUG_FILE, dwRegister);
      fputc('\n', DEBUG_FILE);
    }
}

/***********************************************************************/

static void regm_MarkRegisterUsed(struct regm_rcode2_s *pReg,
                                  uint32_t dwRegister)
{
  regm_PrintDebugReg("Register used: ", dwRegister);

  switch (regm_GetKind(dwRegister))
    {
    case SPECIAL_REG :     /* Special "global" registers */
      break;               /* (ignored) */

    case CC_REG :          /* Condition code register instance (fake) */
      break;

    case INARG_REG :       /* Volatile register for input arguments */
                           /*   and output values from/to callee */
      break;

    case OUTARG_REG  :     /* Volatile register for output arguments */
                           /*   and input values to/from called */
                           /*   function (fake) */
      break;

    case SCRATCH_REG :     /* Volatile register for general usage */
      break;

    case VOLATILE_REG :    /* Volatile registers in general */
    case STATIC_REG :      /* Static register */
    default:
      fatal(ePOFFCONFUSION);
      break;
    }
#warning "Not Implemeted"
}

/***********************************************************************/

static void regm_MarkRegisterModified(struct regm_rcode2_s *pReg,
                                      uint32_t dwRegister)
{
  regm_PrintDebugReg("Register modified: ", dwRegister);

  switch (regm_GetKind(dwRegister))
    {
    case SPECIAL_REG :     /* Special "global" registers */
      break;               /* (ignored) */

    case CC_REG :          /* Condition code register instance (fake) */
      break;

    case INARG_REG :       /* Volatile register for input arguments */
                           /*   and output values from/to callee */
      break;

    case OUTARG_REG  :     /* Volatile register for output arguments */
                           /*   and input values to/from called */
                           /*   function (fake) */
      break;

    case SCRATCH_REG :     /* Volatile register for general usage */
      break;

    case VOLATILE_REG :    /* Volatile registers in general */
    case STATIC_REG :      /* Static register */
    default:
      fatal(ePOFFCONFUSION);
      break;
    }
#warning "Not Implemeted"
}

/**********************************************************************
 * Public Functions
 **********************************************************************/

/***********************************************************************/
/* Generate function prologue: Save return address, create stack frame
 * for local variables, and save static registers that till be used.
 */

void regm_GeneratePrologue(uint32_t dwFrameSize)
{
#warning "Not implemented"
}

/***********************************************************************/
/* Restore static registers, release stack frame and return */

void regm_GenerateEpilogue(uint32_t dwFrameSize)
{
#warning "Not implemented"
}

/***********************************************************************/
/* Register-to-register comparisons (e.g., cmp r1, r2)
 *
 * FORM 1R: <op> <roperand1>, <roperand2>
 */

void regm_GenerateForm1RCc(uint8_t chOp, uint32_t dwROperand1,
                           uint32_t dwROperand2, uint32_t dwRCc)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_1RCc;
  pReg->chOp                 = chOp;
  pReg->u.f1rcc.dwROperand1  = dwROperand1;
  pReg->u.f1rcc.dwROperand2  = dwROperand2;
  pReg->u.f1rcc.dwRCc        = dwRCc;

  regm_MarkRegisterUsed(pReg, dwROperand1);
  regm_MarkRegisterUsed(pReg, dwROperand2);
  regm_MarkRegisterModified(pReg, dwRCc);
}

/***********************************************************************/
/* Register-to-immediate comparisons (e.g., cmpi r1, 1)
 *
 * FORM 1I: <op> <roperand1>, <immediate>
 */

void regm_GenerateForm1ICc(uint8_t chOp, uint32_t dwROperand1,
                           uint32_t dwImmediate, uint32_t dwRCc)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_1ICc;
  pReg->chOp                 = chOp;
  pReg->u.f1icc.dwROperand1  = dwROperand1;
  pReg->u.f1icc.dwImmediate  = dwImmediate;
  pReg->u.f1icc.dwRCc        = dwRCc;

  regm_MarkRegisterUsed(pReg, dwROperand1);
  regm_MarkRegisterModified(pReg, dwRCc);
}

/***********************************************************************/
/* Register-to-register mov instructions (e.g, mov r1, r2)
 *
 * FORM 2R: <op> <rdest>, <roperand2>
 */

void regm_GenerateForm2R(uint8_t chOp, uint32_t dwRDest,
                         uint32_t dwROperand2)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_2R;
  pReg->chOp                 = chOp;
  pReg->u.f2r.dwRDest        = dwRDest;
  pReg->u.f2r.dwROperand2    = dwROperand2;

  regm_MarkRegisterModified(pReg, dwRDest);
  regm_MarkRegisterUsed(pReg, dwROperand2);
}

/***********************************************************************/
/* Immediate-to-register mov instructions (e.g, movi r1, 1)
 *
/* FORM 2I: <op> <rdest>, <immediate>
*/

void regm_GenerateForm2I(uint8_t chOp, uint32_t dwRDest,
                         uint32_t dwImmediate)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_2I;
  pReg->chOp                 = chOp;
  pReg->u.f2i.dwRDest        = dwRDest;
  pReg->u.f2i.dwImmediate    = dwImmediate;

  regm_MarkRegisterModified(pReg, dwRDest);
}

/***********************************************************************/
/* Binary operations (e.g., add r0, r1, r2)
 * Load operations   (e.g., ld r0, [r1, r2])
 * Store operations  (e.g., st r0, [r1, r2])
 *
 * FORM 3R: <op> <rdest>, <roperand1>, <roperand2>
 *               <rsrc>,  <roperand1>, <roperand2>
 */

void regm_GenerateForm3R(uint8_t chOp, uint32_t dwRSrcDest,
                         uint32_t dwROperand1, uint32_t dwROperand2)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_3R;
  pReg->chOp                 = chOp;
  pReg->u.f3r.dwRSrcDest     = dwRSrcDest;
  pReg->u.f3r.dwROperand1    = dwROperand1;
  pReg->u.f3r.dwROperand2    = dwROperand2;

  switch (chOp)
    {
      /* FORM 3: Arithmetic and logical operations */
      /* FORM 3: Loads */

    case rADD   :
    case rSUB   :
    case rRSB   :
    case rMUL   :
    case rDIV   :
    case rMOD   :
    case rSLL   :
    case rSRL   :
    case rSRA   :
    case rOR    :
    case rAND   :
    case rXOR   :
    case rANDN  :
    case rLD    :
    case rLDH   :
    case rLDB   :
      regm_MarkRegisterModified(pReg, dwRSrcDest);
      break;

      /* FORM 3: Loads multiple */
    case rLDM   :
      {
        uint32_t dwRDest = dwRSrcDest;
        int i;
        for (i = 0; i < g_dwRegisterCount; i++)
          {
            regm_MarkRegisterModified(pReg, dwRDest);
            dwRDest++;
          }
      }
      break;

      /* FORM 3: Stores */
    case rST    :
    case rSTH   :
    case rSTB   :
      regm_MarkRegisterUsed(pReg, dwRSrcDest);
      break;

      /* FORM 3: Store multipole */
    case rSTM   :
      {
        uint32_t dwRSrc = dwRSrcDest;
        int i;
        for (i = 0; i < g_dwRegisterCount; i++)
          {
            regm_MarkRegisterUsed(pReg, dwRSrc);
            dwRSrc++;
          }
      }
      break;

    default:
      fatal(ePOFFCONFUSION);
    }

  regm_MarkRegisterUsed(pReg, dwROperand1);
  regm_MarkRegisterUsed(pReg, dwROperand2);
}

/***********************************************************************/
/* Binary operations (e.g., addi r0, r1, 1)
 * Load operations   (e.g., ldi r0, [r1, 4])
 * Store operations  (e.g., sti r0, [r1, 4])
 *
 * FORM 3I: <op> <rdest>, <roperand1>, <immediate>
 *                <rsrc>,  <roperand1>, <immediate>
 */

void regm_GenerateForm3I(uint8_t chOp, uint32_t dwRSrcDest,
                         uint32_t dwROperand1, uint32_t dwImmediate)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_3I;
  pReg->chOp                 = chOp;
  pReg->u.f3i.dwRSrcDest     = dwRSrcDest;
  pReg->u.f3i.dwROperand1    = dwROperand1;
  pReg->u.f3i.dwImmediate    = dwImmediate;

  switch (chOp)
    {
      /* FORM 3: Arithmetic and logical operations */
      /* FORM 3: Loads */

    case rADDI  :
    case rSUBI  :
    case rRSBI  :
    case rMULI  :
    case rDIVI  :
    case rMODI  :
    case rSLLI  :
    case rSRLI  :
    case rSRAI  :
    case rORI   :
    case rANDI  :
    case rXORI  :
    case rANDNI :
    case rLDI   :
    case rLDIH  :
    case rLDIB  :
      regm_MarkRegisterModified(pReg, dwRSrcDest);
      break;

      /* FORM 3: Stores */
    case rSTI   :
    case rSTIH  :
    case rSTIB  :
      regm_MarkRegisterUsed(pReg, dwRSrcDest);
      break;

    default:
      fatal(ePOFFCONFUSION);
    }

  regm_MarkRegisterUsed(pReg, dwROperand1);
}

/***********************************************************************/
/* Unconditional branch operations (b offset, bl offset)
 *
 * FORM 4I: <op> <pc-offset>
 */

void regm_GenerateForm4I(uint8_t chOp, uint32_t dwOffset)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_4I;
  pReg->chOp                 = chOp;
  pReg->u.f4i.dwOffset       = dwOffset;
}

/***********************************************************************/
/* Conditional branch operations (e.g., beq offset)
 *
 * FORM 4I: <op> <pc-offset>
 */

void regm_GenerateForm4ICc(uint8_t chOp, uint32_t dwOffset,
                           uint32_t dwRCc)
{
  struct regm_rcode2_s *pReg = regm_AllocateRCode2();
  pReg->eForm                = eFORM_4ICc;
  pReg->chOp                 = chOp;
  pReg->u.f4icc.dwOffset     = dwOffset;
  pReg->u.f4icc.dwRCc        = dwRCc;

  regm_MarkRegisterUsed(pReg, dwRCc);
}

/***********************************************************************/

int regm_ForEachRCode2(regm_rcode2_node_t pNode, void *arg)
{
  int ret;
  int i;
  for (i = 0; i < g_nRCode2; i++)
    {
      ret = pNode(&g_pRCode2[i], arg);
      if (ret != 0)
        {
          return ret;
        }
    }
  return 0;
}

/***********************************************************************/
