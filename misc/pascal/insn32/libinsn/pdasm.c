/**********************************************************************
 * pdasm.c
 * P-Code Disassembler
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

#include "keywords.h"
#include "podefs.h"
#include "pinsn32.h"
#include "pfdefs.h"
#include "pxdefs.h"
#include "paslib.h"

#include "pinsn.h"

/**********************************************************************
 * Private Types
 **********************************************************************/

/* These are all the format codes that apply to opcodes with an argument */

enum {
  SIMPLE = 0,        /* No argument */
  HEX,               /* Hexadecimal argument */
  DECIMAL,           /* Signed Decimal argument (w/shift) */
  UDECIMAL,          /* Unsigned Decimal argument (w/shift) */
  LABEL_DEC,         /* Label number */
  xOP, lbOP, fpOP,   /* Sub opcode */
  FILENO, LINENO     /* File number, line number */
};

/* The following table defines everything that is needed to disassemble
 * a P-Code.  NOTE:  The order of definition in this table must exactly
 * match the declaration sequence in pinsn.h. */

static const char invOp[] = "Invalid Opcode";
struct optab_s
{
  const char *opName;       /* Opcode mnemonics */
  uint8_t format;           /* arg16 format */
};

/******************** OPCODES WITH NO ARGUMENTS *************************/

static const struct optab_s g_sNoArgOpTable[64] =
{
  /* Program control (No stack arguments) */

  /* 0x00 */ { "NOP  ", SIMPLE },

  /* Arithmetic & logical & and integer conversions (One stack argument) */

  /* 0x01 */ { "NEG  ", SIMPLE },
  /* 0x02 */ { "ABS  ", SIMPLE },
  /* 0x03 */ { "INC  ", SIMPLE },
  /* 0x04 */ { "DEC  ", SIMPLE },
  /* 0x05 */ { "NOT  ", SIMPLE },

  /* Arithmetic & logical (Two stack arguments) */

  /* 0x06 */ { "ADD  ", SIMPLE },
  /* 0x07 */ { "SUB  ", SIMPLE },
  /* 0x08 */ { "MUL  ", SIMPLE },
  /* 0x09 */ { "DIV  ", SIMPLE },
  /* 0x0a */ { "MOD  ", SIMPLE },
  /* 0x0b */ { "SLL  ", SIMPLE },
  /* 0x0c */ { "SRL  ", SIMPLE },
  /* 0x0d */ { "SRA  ", SIMPLE },
  /* 0x0e */ { "OR   ", SIMPLE },
  /* 0x0f */ { "AND  ", SIMPLE },

  /* Comparisons (One stack argument) */

  /* 0x10 */ { "EQUZ ", SIMPLE },
  /* 0x11 */ { "NEQZ ", SIMPLE },
  /* 0x12 */ { "LTZ  ", SIMPLE },
  /* 0x13 */ { "GTEZ ", SIMPLE },
  /* 0x14 */ { "GTZ  ", SIMPLE },
  /* 0x15 */ { "LTEZ ", SIMPLE },
  /* 0x16 */ { invOp,   SIMPLE },
  /* 0x17 */ { invOp,   SIMPLE },

  /* Comparisons (Two stack arguments) */

  /* 0x18 */ { "EQU  ", SIMPLE },
  /* 0x19 */ { "NEQ  ", SIMPLE },
  /* 0x1a */ { "LT   ", SIMPLE },
  /* 0x1b */ { "GTE  ", SIMPLE },
  /* 0x1c */ { "GT   ", SIMPLE },
  /* 0x1d */ { "LTE  ", SIMPLE },
  /* 0x1e */ { invOp,   SIMPLE },
  /* 0x1f */ { "BIT  ", SIMPLE },

  /* Load (One) or Store (Two stack argument) */

  /* 0x20 */ { "LDI ", SIMPLE },
  /* 0x21 */ { "LDIH", SIMPLE },
  /* 0x22 */ { "LDIB", SIMPLE },
  /* 0x23 */ { "LDIM", SIMPLE },
  /* 0x24 */ { "STI ", SIMPLE },
  /* 0x25 */ { "STIH", SIMPLE },
  /* 0x26 */ { "STIB", SIMPLE },
  /* 0x27 */ { "STIM", SIMPLE },

  /* Data stack operations */

  /* 0x28 */ { "DUP  ", SIMPLE },
  /* 0x29 */ { "DUPH ", SIMPLE },
  /* 0x2a */ { "PUSHS", SIMPLE },
  /* 0x2b */ { "POPS",  SIMPLE },
  /* 0x2c */ { invOp,   SIMPLE },
  /* 0x2d */ { invOp,   SIMPLE },
  /* 0x2e */ { invOp,   SIMPLE },
  /* 0x2f */ { "RET  ", SIMPLE },

  /* 0x30 */ { invOp,   SIMPLE },
  /* 0x31 */ { invOp,   SIMPLE },
  /* 0x32 */ { invOp,   SIMPLE },
  /* 0x33 */ { invOp,   SIMPLE },
  /* 0x34 */ { invOp,   SIMPLE },
  /* 0x35 */ { invOp,   SIMPLE },
  /* 0x36 */ { invOp,   SIMPLE },
  /* 0x37 */ { invOp,   SIMPLE },

  /* System Functions (No stack arguments) */

  /* 0x38 */ { invOp,   SIMPLE },
  /* 0x39 */ { invOp,   SIMPLE },
  /* 0x3a */ { invOp,   SIMPLE },
  /* 0x3b */ { invOp,   SIMPLE },
  /* 0x3c */ { invOp,   SIMPLE },
  /* 0x3d */ { invOp,   SIMPLE },
  /* 0x3e */ { invOp,   SIMPLE },
  /* 0x3f */ { "EXIT ", SIMPLE }
};

/****************** OPCODES WITH 25-BIT ARGUMENT ************************/

static const struct optab_s g_sArg32OpTable[64] =
{
  /* Load:  arg = unsigned base offset */

  /* 0x80 */ { "LD   ", UDECIMAL },  /* No stack arguments */
  /* 0x81 */ { "LDH  ", UDECIMAL },  /* No stack arguments */
  /* 0x82 */ { "LDB  ", UDECIMAL },  /* No stack arguments */
  /* 0x83 */ { "LDM  ", UDECIMAL },  /* One 32-bit stack argument */

  /* Store: arg = unsigned base offset (One stack arguments) */

  /* 0x84 */ { "ST   ", UDECIMAL },  /* One 32-bit stack argument */
  /* 0x85 */ { "STH  ", UDECIMAL },  /* One 32-bit stack argument */
  /* 0x86 */ { "STB  ", UDECIMAL },  /* One 32-bit stack argument */
  /* 0x87 */ { "STM  ", UDECIMAL },  /* One+n 32-bit stack argument */

  /* Load Indexed: arg = unsigned base offset */

  /* 0x88 */ { "LDX  ", UDECIMAL },  /* One 32-bit stack argument */
  /* 0x89 */ { "LDXH ", UDECIMAL },  /* One 32-bit stack argument */
  /* 0x8a */ { "LDXB ", UDECIMAL },  /* One 32-bit stack argument */
  /* 0x8b */ { "LDXM ", UDECIMAL },  /* Two 32-bit stack argument */

  /* Store Indexed: arg = unsigned base offset */

  /* 0x8c */ { "STX  ", UDECIMAL },  /* Two 32-bit stack argument */
  /* 0x8d */ { "STXH ", UDECIMAL },  /* Two 32-bit stack argument */
  /* 0x8e */ { "STXB ", UDECIMAL },  /* Two 32-bit stack argument */
  /* 0x8f */ { "STXM ", UDECIMAL },  /* Two+n 32-bit stack argument */

  /* Program control:  arg = unsigned label (One stack argument) */

  /* 0x90 */ { "JEQUZ", HEX },
  /* 0x91 */ { "JNEQZ", HEX },
  /* 0x92 */ { "JLTZ ", HEX },
  /* 0x93 */ { "JGTEZ", HEX },
  /* 0x94 */ { "JGTZ ", HEX },
  /* 0x95 */ { "JLTEZ", HEX },

  /* Program control:  arg = unsigned label (no stack arguments) */

  /* 0x96 */ { "JMP  ", HEX },
  /* 0x97 */ { "PUSH ", DECIMAL },

  /* Program control:  arg = unsigned label (One stack argument) */

  /* 0x98 */ { "JEQU ", HEX },
  /* 0x99 */ { "JNEQ ", HEX },
  /* 0x9a */ { "JLT  ", HEX },
  /* 0x9b */ { "JGTE ", HEX },
  /* 0x9c */ { "JGT  ", HEX },
  /* 0x9d */ { "JLTE ", HEX },
  /* 0x9e */ { invOp,   SIMPLE },
  /* 0x9f */ { "INDS ", DECIMAL },

  /* Load:  Uses LSP; arg = signed frame offset */

  /* 0xa0 */ { "LDS  ", DECIMAL },  /* No stack arguments */
  /* 0xa1 */ { "LDSH ", DECIMAL },  /* No stack arguments */
  /* 0xa2 */ { "LDSB ", DECIMAL },  /* No stack arguments */
  /* 0xa3 */ { "LDSM ", DECIMAL },  /* One 32-bit stack argument */

  /* Store: Uses LSP; arg = signed frame offset */

  /* 0xa4 */ { "STS  ", DECIMAL },  /* One 32-bit stack argument */
  /* 0xa5 */ { "STSH ", DECIMAL },  /* One 32-bit stack argument */
  /* 0xa6 */ { "STSB ", DECIMAL },  /* One 32-bit stack argument */
  /* 0xa7 */ { "STSM ", DECIMAL },  /* One+n 32-bit stack argument */

  /* Load Indexed: Uses LSP; arg = signed frame offset */

  /* 0xa8 */ { "LDSX ", DECIMAL },  /* One 32-bit stack argument */
  /* 0xa9 */ { "LDSXH", DECIMAL },  /* One 32-bit stack argument */
  /* 0xaa */ { "LDSXB", DECIMAL },  /* One 32-bit stack argument */
  /* 0xab */ { "LDSXM", DECIMAL },  /* Two 32-bit stack argument */

  /* Store Indexed: Uses LSP; arg = signed frame offset */

  /* 0xac */ { "STSX ", DECIMAL },  /* Two 32-bit stack argument */
  /* 0xad */ { "STSXH", DECIMAL },  /* Two 32-bit stack argument */
  /* 0xae */ { "STSXB", DECIMAL },  /* Two 32-bit stack argument */
  /* 0xaf */ { "STSXM", DECIMAL },  /* Two+n 32-bit stack argument */

  /* Load address relative to stack base: arg = unsigned offset */

  /* 0xb0 */ { "LA   ", UDECIMAL },

  /* Load address: Uses SLP, arg = signed frame offset */

  /* 0xb1 */ { "LAS  ", DECIMAL },

  /* Load absolute stack address:  arg = RODATA offset (No stack arguments) */

  /* 0xb2 */ { "LAC  ", HEX,    },
  /* 0xb3 */ { invOp,   SIMPLE },

  /* Load address relative to stack base: arg = unsigned offset, TOS=index */

  /* 0xb4 */ { "LAX  ", UDECIMAL },

  /* Load address indexed: Uses SLP, arg = signed frame offset */

  /* 0xb5 */ { "LASX ", DECIMAL },

  /* Set LSP:  arg = new level that evaluates to LSP value */

  /* 0xb6 */ { "SLSP  ", UDECIMAL },
  /* 0xb7 */ { "SDC   ", UDECIMAL },
  /* 0xb8 */ { invOp,   SIMPLE },

  /* Program Control:  Uses LSP; arg = unsigned label (No stack arguments) */

  /* 0xb9 */ { "PCAL ", HEX },

  /* System calls:   arg = file number | sub-function code */

  /* 0xba */ { "SYSIO", xOP },

  /* System functions: arg = 32-bit library call identifier */

  /* 0xbb */ { "LIB  ", lbOP },

  /* Floating point operations: arg = FP op-code */

  /* 0xbc */ { "FLOAT", fpOP },

  /* Program control:  arg = unsigned label (no stack arguments) */

  /* 0xbd */ { NULL, LABEL_DEC },

  /* Pseudo-operations:  arg = file number OR line number */

  /* 0xbe */ { "INCLUDE ", FILENO },
  /* 0xbf */ { "LINE ", LINENO },
};

static const char invXOp[] = "Invalid SYSIO";
static const char *xName[MAX_XOP] = { /* SYSIO opcode mnemonics */
/* 0x00 */ invXOp,      "EOF",       "EOLN",       "RESET",
/* 0x04 */ "REWRITE",   invXOp,      invXOp,       invXOp,
/* 0x08 */ invXOp,      invXOp,      invXOp,       invXOp,
/* 0x0c */ invXOp,      invXOp,      invXOp,       invXOp,
/* 0x10 */ "READLN",    "READPG",    "READBIN",    "READINT",
/* 0x14 */ "READCHR",   "READSTR",   "READRL",     invXOp,
/* 0x18 */ invXOp,      invXOp,      invXOp,       invXOp,
/* 0x1c */ invXOp,      invXOp,      invXOp,       invXOp,
/* 0x20 */ "WRITELN",   "WRITEPG",   "WRITEBIN",   "WRITEINT",
/* 0x24 */ "WRITECHR",  "WRITESTR",  "WRITERL" };

static const char invLbOp[] = "Invalid runtime code";
static const char *lbName[MAX_LBOP] = { /* LIB opcode mnemonics */
/* 0x00 */ "GETENV",     "STR2STR",   "CSTR2STR",  "STR2RSTR",
/* 0x04 */ "CSTR2RSTR",  "VAL",       "MKSTK",     "MKSTKSTR",
/* 0x08 */ "MKSTKC",     "STRCAT",    "STRCATC",   "STRCMP" };

#define MAX_FOP 16
static const char invFpOp[] = "Invalid FP Operation";
static const char *fpName[MAX_FOP] = {
/* 0x00 */ invFpOp,    "FLOAT",    "TRUNC",    "ROUND",
/* 0x04 */ "NEG",      "ADD",      "SUB",      "MUL",
/* 0x08 */ "DIV",      "MOD",      "EQU",      "NEQ",
/* 0x0c */ "LT",       "GTE",      "GT",       "LTE" };

/***********************************************************************/

void insn_DisassemblePCode(FILE* lfile, OPTYPE *pop)
{
  const struct optab_s *opTable;
  int idx;

  /* Select table based upon whether an opcode is included or not */

  if (pop->op & o32)
    {
      opTable = g_sArg32OpTable;
      idx = pop->op & ~o32;
    }
  else
    {
      opTable = g_sNoArgOpTable;
      idx = pop->op;
    }

  /* Indent, comment or label as appropriate */

  switch (opTable[idx].format)
    {
    case LABEL_DEC :
      fprintf(lfile, "L%08lx:\n", pop->arg);
      return;
    case FILENO   :
    case LINENO   :
      fprintf(lfile, "; ");
      break;
    default   :
      fprintf(lfile, "        ");
    } /* end switch */

  /* Print the opcode mnemonic */

  fprintf(lfile, "%s ", opTable[idx].opName);

  /* Print the argument (if present) */

  if (pop->op & o32)
    {
      switch (opTable[idx].format)
        {
        case HEX       :
          fprintf(lfile, "0x%08lx", pop->arg);
          break;

        case FILENO    :
        case LINENO    :
        case DECIMAL   :
          fprintf(lfile, "%ld", (int32_t)pop->arg);
          break;

        case UDECIMAL  :
          fprintf(lfile, "%1lu", pop->arg);
          break;

        case fpOP       :
          if ((pop->arg & 0x3f) < MAX_FOP)
            fprintf(lfile, "%s", fpName[(pop->arg & 0x3f)]);
          else
            fprintf(lfile, "%s", invFpOp);
          break;

        case xOP       :
          {
            unsigned fileno = pop->arg >> 16;
            unsigned xop    = pop->arg & 0xffff;
            fprintf(lfile, "%d, ", fileno);
            if (xop < MAX_XOP)
              fprintf(lfile, "%s", xName[xop]);
            else
              fprintf(lfile, "%s", invXOp);
          }
          break;

        case lbOP :
          if (pop->arg < MAX_LBOP)
            fprintf(lfile, "%s", lbName[pop->arg]);
          else
            fprintf(lfile, "%s", invLbOp);
          break;

        case LABEL_DEC :
        default        :
          break;
        }
    }

  /* Don't forget the newline! */

  fputc('\n', lfile);

} /* end dissassemblePcode */

/***********************************************************************/
