/**********************************************************************
 * pfopt.c
 * Finalization of optimized image
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
#include "podefs.h"
#include "pedefs.h"
#include "pinsn16.h"
#include "poff.h"
#include "paslib.h"
#include "pofflib.h"

#include "popt.h"
#include "pfopt.h"
#include "pinsn.h"
#include "perr.h"

/**********************************************************************
 * Definitions
 **********************************************************************/

/**********************************************************************
 * Private Types
 **********************************************************************/

/**********************************************************************
 * Private Data
 **********************************************************************/

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

static void pass1(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle)
{
  OPTYPE   op;
  uint32_t pc;
  uint32_t opsize;

  /* Build label / line number reference table
   *
   * CASE 1: LABEL
   *   Add label number + PC to table
   *   discard
   * CASE 2: LINE
   *   genereate a line number reference
   *   discard
   * ELSE:
   *   pass through with no additional action
   */

  pc = 0;
  do
    {
      opsize = insn_GetOpCode(poffHandle, &op);
      if (op.op == oLABEL)
        {
          poffAddToDefinedLabelTable(op.arg2, pc);
        }
      else if (op.op == oLINE)
        {
          poffAddLineNumber(poffHandle, op.arg2, op.arg1, pc);
        }
      else
        {
          insn_AddTmpOpCode(poffProgHandle, &op);
          pc += opsize;
        }
    }
  while (op.op != oEND);

  /* Replace the original program data with the new program data */

  poffReplaceProgData(poffHandle, poffProgHandle);
}

/**********************************************************************/

static void pass2(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle)
{
  poffSymHandle_t poffSymHandle;
  int32_t symIndex;
  int32_t nchanges = 0;

  /* Get a container to temporarily hold any modifications that we
   * make to the symbol table.
   */

  poffSymHandle = poffCreateSymHandle();
  if (poffSymHandle == NULL)
    {
      fatal(eNOMEMORY);
    }

  /* Now read all of the symbols.  (1) Add each undefined code reference
   * to the label reference table, and (2) Change each defined code
   * reference from a label to a program data section offset.
   */

  do
    {
      poffLibSymbol_t symbol;
      symIndex = poffGetSymbol(poffHandle, &symbol);
      if (symIndex >= 0)
        {
          if ((symbol.type == STT_PROC) || (symbol.type == STT_FUNC))
            {
              /* It is a symbol associated with the program data section.
               * Has is value been defined?
               */

              if ((symbol.flags & STF_UNDEFINED) != 0)
                {
                  /* No... Add it to the list of undefined labels */

                  poffAddToUndefinedLabelTable(symbol.value, symIndex);
                }
              else
                {
                  /* It is a defined symbol. In this case, we should have
                   * encountered its LABEL marker in the pass1 processing
                   * and the following look up should not fail.
                   */
                  int32_t value = poffGetPcForDefinedLabel(symbol.value);
                  if (value < 0)
                    {
                      DEBUG(stdout, "Failed to find label L%04lx\n", symbol.value);
                      fatal(ePOFFCONFUSION);
                    }
                  else
                    {
                      /* Replace the label value with the section offset
                       * (pc) value.
                       */

                      symbol.value = value;
                      nchanges++;
                    }
                }
            }

          /* In either event, we will want to save the symbol in case
           * we need to re-write the symbol table.
           */

          (void)poffAddTmpSymbol(poffHandle, poffSymHandle, &symbol);
        }
    }
  while (symIndex >= 0);

  /* We any changes made to the symbol table in the temporary container? */

  if (nchanges != 0)
    {
      /* Yes, update the symbol table */

      poffReplaceSymbolTable(poffHandle, poffSymHandle);

    }

  /* Release the symbol container. */

  poffDestroySymHandle(poffSymHandle);
}

/**********************************************************************/

static void pass3(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle)
{
  OPTYPE   op;
  uint32_t pc;
  uint32_t opsize;

  /* Read each opcode, generate relocation information and
   * replace label references with program section offsets.
   *
   * CASE 1: LAC
   *   generate RODATA relocation entry
   * CASE 2: PCAL instructions
   *   replace label with I-space offset, OR
   *   generate a PROGRAM relocation entry
   * CASE 3: J* instructions
   *   replace label with I-space offset
   * CASE 4: LDS*, STS*, and LAS* instructions
   *   generate a STACK relocation (if imported?)
   * ELSE:
   *   pass through with no additional action
   */

  pc = 0;
  do
    {
      opsize = insn_GetOpCode(poffHandle, &op);
      switch (op.op)
        {
          /* Load of an address in the rodata section */

        case oLAC:
          /* We are referencing something from the rodata section.
           * No special action need be taken.
           */
          break;

          /* Call to a procedure or function. */

        case oPCAL:
          {
            /* Check if this is a defined label, i.e., a call to
             * procedure or function in the same file.
             */

            int32_t value = poffGetPcForDefinedLabel(op.arg2);
            if (value >= 0)
              {
                /* Yes... replace the label reference with
                 * a text section offset.  No relocation record
                 * is needed in this case.  The only relocation
                 * may be performed is a subsequent program data
                 * section offset.
                 */

                op.arg2 = (uint16_t)value;
              }
            else
              {
                /* Check if this is a undefined label.  This would
                 * occur for a call to a procedure or a function that
                 * is defined in some other unit file.
                 */

                value = poffGetSymIndexForUndefinedLabel(op.arg2);
                if (value >= 0)
                  {
                    /* Use the value zero now */

                    op.arg2 = 0;

                    /* And generate a symbol-based relocation */

                    (void)poffAddRelocation(poffHandle, RLT_PCAL, value, pc);
                  }
                else
                  {
                    DEBUG(stdout, "Failed to find call label L%04x\n", op.arg2);
                    fatal(ePOFFCONFUSION);
                  }
              }
          }
          break;

          /* Jumps to "nearby" addresses */

        case oJMP:   /* Unconditional */
        case oJEQUZ: /* Unary comparisons with zero */
        case oJNEQZ:
        case oJLTZ:
        case oJGTEZ:
        case oJGTZ:
        case oJLTEZ:
        case oJEQU:  /* Binary comparisons */
        case oJNEQ:
        case oJLT:
        case oJGTE:
        case oJGT:
        case oJLTE:
          {
            /* Check if this is a defined label.  This must be the case
             * because there can be no jumps into a unit file.
             */

            int32_t value = poffGetPcForDefinedLabel(op.arg2);
            if (value >= 0)
              {
                /* Yes... replace the label reference with
                 * a text section offset.  No relocation record
                 * is needed in this case.  The only relocation
                 * may be performed is a subsequent program data
                 * sectioin offset.
                 */

                op.arg2 = (uint16_t)value;
              }
            else
              {
                DEBUG(stdout, "Failed to find jump label L%04x\n", op.arg2);
                fatal(ePOFFCONFUSION);
              }
          }
          break;

          /* References to stack via level offset */

        case oLAS:   /* Load stack address */
        case oLASX:
        case oLDS:   /* Load value */
        case oLDSH:
        case oLDSB:
        case oLDSM:
        case oSTS:   /* Store value */
        case oSTSH:
        case oSTSB:
        case oSTSM:
        case oLDSX:
        case oLDSXH: /* Load value indexed */
        case oLDSXB:
        case oLDSXM:
        case oSTSX:  /* Store value indexed */
        case oSTSXH:
        case oSTSXB:
        case oSTSXM:
          {
#warning REVISIT
          }
          break;

          /* Otherwise, it is not an interesting opcode */
        default:
          break;
        }

      /* Save the potentially modified opcode in the temporary
       * program data container.
       */

      insn_AddTmpOpCode(poffProgHandle, &op);
      pc += opsize;
    }
  while (op.op != oEND);

  /* Replace the original program data with the new program data */

  poffReplaceProgData(poffHandle, poffProgHandle);
}

/**********************************************************************/

static void pass4(poffHandle_t poffHandle)
{
  uint32_t entryLabel;
  int32_t  entryOffset;
  uint8_t  fileType;

  /* What kind of a file did we just process.  Was it a program file?
   * or was it a unit file?
   */

  fileType = poffGetFileType(poffHandle);
  if (fileType == FHT_PROGRAM)
    {
      /* It is a program file.  In this case, it must have a valid
       * entry point label.  Get it.
       */

      entryLabel = poffGetEntryPoint(poffHandle);

      /* Convert the label into a program data section offset */

      entryOffset = poffGetPcForDefinedLabel(entryLabel);
      if (entryOffset < 0)
        {
          fatal(ePOFFCONFUSION);
        }

      /* Replace file header entry point with the program data
       * section offset
       */

      poffSetEntryPoint(poffHandle, entryOffset);
    }
}

/**********************************************************************
 * Global Functions
 **********************************************************************/

void optFinalize(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle)
{
  /* Build label / line number reference table */

  pass1(poffHandle, poffProgHandle);

  /* Reset for next pass */

  insn_ResetOpCodeRead(poffHandle);
  insn_ResetTmpOpCodeWrite(poffProgHandle);

  /* Now process all of the symbols */

  pass2(poffHandle, poffProgHandle);

  /* We do not use the debug function information so we do not bother
   * to fixup the label references.  Just discard this information.
   */

  poffDiscardDebugFuncInfo(poffHandle);

  /* Reset for next pass */

  insn_ResetOpCodeRead(poffHandle);

  /* Generate relocation information and replace all label references
   * in the code with actual program section data offsets.
   */

  pass3(poffHandle, poffProgHandle);

  /* Reset for next pass */

  insn_ResetOpCodeRead(poffHandle);
  insn_ResetTmpOpCodeWrite(poffProgHandle);

  /* Finally, replace file header entry point with the I-space offset */

  pass4(poffHandle);

  /* Clean up after ourselves */

  poffReleaseLabelReferences();
}

/**********************************************************************/
