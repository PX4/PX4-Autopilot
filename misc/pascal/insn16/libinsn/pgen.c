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
#include "pdefs.h"    /* Common types */
#include "pofflib.h"  /* POFF library definitions */
#include "podefs.h"   /* Logical opcode definitions */
#include "pedefs.h"   /* Error codes */
#include "pinsn16.h"  /* 16-bit target INSN opcode definitions */
#include "perr.h"     /* Error handling logic */

#include "pinsn.h"    /* (to verify prototypes in this file) */

/**********************************************************************
 * Definitions
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

extern poffHandle_t poffHandle; /* Handle to POFF object */
extern FILE *lstFile;           /* LIST file pointer */
extern int16_t level;            /* Static nesting level */

/**********************************************************************
 * Private Variables
 **********************************************************************/

static const uint16_t opmap[NUM_OPCODES] =
{
  oNOP,    /* opNOP */
  oNEG,    /* opNEG */
  oABS,    /* opABS */
  oINC,    /* opINC */
  oDEC,    /* opDEC */
  oNOT,    /* opNOT */
  oADD,    /* opADD */
  oSUB,    /* opSUB */
  oMUL,    /* opMUL */
  oDIV,    /* opDIV */
  oMOD,    /* opMOD */
  oSLL,    /* opSLL */
  oSRL,    /* opSRL */
  oSRA,    /* opSRA */
  oOR,     /* opOR */
  oAND,    /* opAND */
  oEQUZ,   /* opEQUZ */
  oNEQZ,   /* opNEQZ */
  oLTZ,    /* opLTZ */
  oGTEZ,   /* opGTEZ */
  oGTZ,    /* opGTZ */
  oLTEZ,   /* opLTEZ */
  oEQU,    /* opEQU */
  oNEQ,    /* opNEQ */
  oLT,     /* opLT */
  oGTE,    /* opGTE */
  oGT,     /* opGT */
  oLTE,    /* opLTE */
  oBIT,    /* opBIT */
  oLDIH,   /* opLDI -- integer load maps to 16-bit load */
  oLDIB,   /* opLDIB */
  oLDIM,   /* opLDIM */
  oSTIH,   /* opSTI - integer store maps to 16-bit store */
  oSTIB,   /* opSTIB */
  oSTIM,   /* opSTIM */
  oDUPH,   /* opDUP -- integer duplicate maps to 16-bit duplicate */
  oPUSHS,  /* opPUSHS */
  oPOPS,   /* opPOPS */
  oRET,    /* opRET */
  oEND,    /* opEND */
  oFLOAT,  /* opFLOAT */
  oJEQUZ,  /* opJEQUZ */
  oJNEQZ,  /* opJNEQZ */
  oJMP,    /* opJMP */
  oJEQU,   /* opJEQU */
  oJNEQ,   /* opJNEQ */
  oJLT,    /* opJLT */
  oJGTE,   /* opJGTE */
  oJGT,    /* opJGT */
  oJLTE,   /* opJLTE */
  oLDH,    /* opLD -- integer load maps to 16-bit load */
  oLDH,    /* opLDH */
  oLDB,    /* opLDB */
  oLDM,    /* opLDM */
  oSTH,    /* opST -- integer store maps to 16-bit store */
  oSTB,    /* opSTB */
  oSTM,    /* opSTM */
  oLDXH,   /* opLDX -- integer load maps to 16-bit load */
  oLDXB,   /* opLDXB */
  oLDXM,   /* opLDXM */
  oSTXH,   /* opSTX -- integer store maps to 16-bit store */
  oSTXB,   /* opSTXB */
  oSTXM,   /* opSTXM */
  oLA,     /* opLA */
  oLAC,    /* opLAC */
  oPUSH,   /* opPUSH */
  oINDS,   /* opINDS */
  oLAX,    /* opLAX */
  oLIB,    /* opLIB */
  oLABEL,  /* opLABEL */
  oPCAL,   /* opPCAL */
  oLDSH,   /* opLDS -- integer store maps to 16-bit load */
  oLDSH,   /* opLDSH */
  oLDSB,   /* opLDSB */
  oLDSM,   /* opLDSM */
  oSTSH,   /* opSTS -- integer store maps to 16-bit store */
  oSTSB,   /* opSTSB */
  oSTSM,   /* opSTSM */
  oLDSXH,  /* opLDSX -- integer load maps to 16-bit load */
  oLDSXB,  /* opLDSXB */
  oLDSXM,  /* opLDSXM */
  oSTSXH,  /* opSTSX -- integer store maps to 16-bit store */
  oSTSXB,  /* opSTSXB */
  oSTSXM,  /* opSTSXM */
  oLAS,    /* opLAS */
  oLASX,   /* opLASX */
  oSYSIO,  /* opSYSIO */
  oLINE,   /* opLINE */
};

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static void
insn16_Generate(enum pcode_e opcode, uint16_t arg1, int32_t arg2);

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************/
/* Disassemble an Op-code */

#if CONFIG_DEBUG
static inline void
insn16_DisassemblePCode(uint8_t opcode, uint8_t arg1, uint16_t arg2)
{
  OPTYPE op;

  op.op   = opcode;
  op.arg1 = arg1;
  op.arg2 = arg2;

  insn_DisassemblePCode(lstFile, &op);
}
#else
# define insn16_DisassemblePCode(op,a1,a2)
#endif

/***********************************************************************/
/* Generate an Op-Code */

static void
insn16_Generate(enum pcode_e opcode, uint16_t arg1, int32_t arg2)
{
  uint16_t insn_opcode = opmap[opcode];
  uint16_t arg16;
  TRACE(lstFile,"[insn16_Generate:0x%02x->0x%04x]", opcode, insn_opcode);

  poffAddProgByte(poffHandle, insn_opcode);
  if (insn_opcode & o8)
    {
      if (arg1 > 0xff) error(eINTOVF);
      poffAddProgByte(poffHandle, (uint8_t)arg1);
    } /* End if */

  if (insn_opcode & o16)
    {
      if ((arg2 < -32768) || (arg2 > 65535)) error(eINTOVF);
      arg16 = (uint16_t)arg2;
      poffAddProgByte(poffHandle, (uint8_t)(arg16 >> 8));
      poffAddProgByte(poffHandle, (uint8_t)(arg16 & 0xff));
    } /* End if */

  /* Now, add the disassembled PCode to the list file. */

  insn16_DisassemblePCode(insn_opcode, arg1, arg2);

} /* end insn16_Generate */

/***********************************************************************
 * Public Functions
 ***********************************************************************/

void
insn_GenerateSimple(enum pcode_e opcode)
{
  insn16_Generate(opcode, 0, 0);
}

/***********************************************************************/

void
insn_GenerateDataOperation(enum pcode_e opcode, int32_t data)
{
  insn16_Generate(opcode, 0, data);
}

/***********************************************************************/
/* Data size for a multiple register operation (in bytes) is simply
 * represented by that value at the top of the stack.
 */

void insn_GenerateDataSize(uint32_t dwDataSize)
{
  insn16_Generate(opPUSH, 0, dwDataSize);
}

/***********************************************************************/

void
insn_GenerateFpOperation(uint8_t fpOpcode)
{
  insn16_Generate(opFLOAT, fpOpcode, 0);
}

/***********************************************************************/

void
insn_GenerateIoOperation(uint16_t ioOpcode, uint16_t fileNumber)
{
  insn16_Generate(opSYSIO, fileNumber, (int32_t)ioOpcode);
}

/***********************************************************************/

void
insn_BuiltInFunctionCall(uint16_t libOpcode)
{
  insn16_Generate(opLIB, 0, (int32_t)libOpcode);
}

/***********************************************************************/

void
insn_GenerateLevelReference(enum pcode_e opcode, uint16_t level, int32_t offset)
{
  insn16_Generate(opcode, level, offset);
}

/***********************************************************************/

void
insn_GenerateProcedureCall(uint16_t level, int32_t offset)
{
  insn16_Generate(opPCAL, level, offset);
}

/***********************************************************************/

void
insn_GenerateLineNumber(uint16_t includeNumber, uint32_t lineNumber)
{
  insn16_Generate(opLINE, includeNumber, lineNumber);
}

/***********************************************************************/

void
insn_SetStackLevel(uint32_t level)
{
  /* Do nothing */
}
