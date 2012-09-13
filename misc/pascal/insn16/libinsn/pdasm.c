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

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdint.h>
#include <stdio.h>

#include "keywords.h"
#include "podefs.h"
#include "pinsn16.h"
#include "pfdefs.h"
#include "pxdefs.h"
#include "paslib.h"

#include "pinsn.h"

/***********************************************************************
 * Pre-processor Definitions
 ***********************************************************************/

/* These are all the format codes that apply to opcodes the an arg16 */

#define NOARG16     0
#define HEX         1
#define DECIMAL     2
#define UDECIMAL    3
#define LABEL_DEC   4
#define xOP         5
#define lbOP        6
#define fpOP        7
#define COMMENT     8

/* The following table defines everything that is needed to disassemble
 * a P-Code.  NOTE:  The order of definition in this table must exactly
 * match the declaration sequence in pinsn.h. */

static const char invOp[] = "Invalid Opcode";
static const struct
{
  const char *opName;       /* Opcode mnemonics */
  uint8_t format;           /* arg16 format */
} opTable[256] =
{

/******************* OPCODES WITH NO ARGUMENTS ************************/
/* Program control (No stack arguments) */

/* 0x00 */ { "NOP  ", NOARG16 },

/* Arithmetic & logical & and integer conversions (One stack argument) */

/* 0x01 */ { "NEG  ", NOARG16 },
/* 0x02 */ { "ABS  ", NOARG16 },
/* 0x03 */ { "INC  ", NOARG16 },
/* 0x04 */ { "DEC  ", NOARG16 },
/* 0x05 */ { "NOT  ", NOARG16 },

/* Arithmetic & logical (Two stack arguments) */

/* 0x06 */ { "ADD  ", NOARG16 },
/* 0x07 */ { "SUB  ", NOARG16 },
/* 0x08 */ { "MUL  ", NOARG16 },
/* 0x09 */ { "DIV  ", NOARG16 },
/* 0x0a */ { "MOD  ", NOARG16 },
/* 0x0b */ { "SLL  ", NOARG16 },
/* 0x0c */ { "SRL  ", NOARG16 },
/* 0x0d */ { "SRA  ", NOARG16 },
/* 0x0e */ { "OR   ", NOARG16 },
/* 0x0f */ { "AND  ", NOARG16 },

/* Comparisons (One stack argument) */

/* 0x10 */ { "EQUZ ", NOARG16 },
/* 0x11 */ { "NEQZ ", NOARG16 },
/* 0x12 */ { "LTZ  ", NOARG16 },
/* 0x13 */ { "GTEZ ", NOARG16 },
/* 0x14 */ { "GTZ  ", NOARG16 },
/* 0x15 */ { "LTEZ ", NOARG16 },
/* 0x16 */ { invOp,   NOARG16 },
/* 0x17 */ { invOp,   NOARG16 },

/* Comparisons (Two stack arguments) */

/* 0x18 */ { "EQU  ", NOARG16 },
/* 0x19 */ { "NEQ  ", NOARG16 },
/* 0x1a */ { "LT   ", NOARG16 },
/* 0x1b */ { "GTE  ", NOARG16 },
/* 0x1c */ { "GT   ", NOARG16 },
/* 0x1d */ { "LTE  ", NOARG16 },
/* 0x1e */ { invOp,   NOARG16 },
/* 0x1f */ { "BIT  ", NOARG16 },

/* Load (One) or Store (Two stack argument) */

/* 0x20 */ { "LDI ", NOARG16 },
/* 0x21 */ { "LDIH", NOARG16 },
/* 0x22 */ { "LDIB", NOARG16 },
/* 0x23 */ { "LDIM", NOARG16 },
/* 0x24 */ { "STI ", NOARG16 },
/* 0x25 */ { "STIH", NOARG16 },
/* 0x26 */ { "STIB", NOARG16 },
/* 0x27 */ { "STIM", NOARG16 },

/* Data stack operations */

/* 0x29 */ { "DUP  ", NOARG16 },
/* 0x29 */ { "DUPH ", NOARG16 },
/* 0x2a */ { "PUSHS", NOARG16 },
/* 0x2b */ { "POPS",  NOARG16 },
/* 0x2c */ { invOp,   NOARG16 },
/* 0x2d */ { invOp,   NOARG16 },
/* 0x2e */ { invOp,   NOARG16 },
/* 0x2f */ { "RET  ", NOARG16 },

/* 0x30 */ { invOp,   NOARG16 },
/* 0x31 */ { invOp,   NOARG16 },
/* 0x32 */ { invOp,   NOARG16 },
/* 0x33 */ { invOp,   NOARG16 },
/* 0x34 */ { invOp,   NOARG16 },
/* 0x35 */ { invOp,   NOARG16 },
/* 0x36 */ { invOp,   NOARG16 },
/* 0x37 */ { invOp,   NOARG16 },
/* 0x38 */ { invOp,   NOARG16 },

/* System Functions (No stack arguments) */

/* 0x39 */ { invOp,   NOARG16 },
/* 0x3a */ { invOp,   NOARG16 },
/* 0x3b */ { invOp,   NOARG16 },
/* 0x3c */ { invOp,   NOARG16 },
/* 0x3d */ { invOp,   NOARG16 },
/* 0x3e */ { invOp,   NOARG16 },
/* 0x3f */ { "EXIT ", NOARG16 },

/************** OPCODES WITH SINGLE BYTE ARGUMENT (arg8) ***************/

/* 0x40 */ { invOp,   NOARG16 },
/* 0x41 */ { invOp,   NOARG16 },
/* 0x42 */ { invOp,   NOARG16 },
/* 0x43 */ { invOp,   NOARG16 },
/* 0x44 */ { invOp,   NOARG16 },
/* 0x45 */ { invOp,   NOARG16 },
/* 0x46 */ { invOp,   NOARG16 },
/* 0x47 */ { invOp,   NOARG16 },
/* 0x48 */ { invOp,   NOARG16 },
/* 0x49 */ { invOp,   NOARG16 },
/* 0x4a */ { invOp,   NOARG16 },
/* 0x4b */ { invOp,   NOARG16 },
/* 0x4c */ { invOp,   NOARG16 },
/* 0x4d */ { invOp,   NOARG16 },
/* 0x4e */ { invOp,   NOARG16 },
/* 0x4f */ { invOp,   NOARG16 },

/* 0x50 */ { invOp,   NOARG16 },
/* 0x51 */ { invOp,   NOARG16 },
/* 0x52 */ { invOp,   NOARG16 },
/* 0x53 */ { invOp,   NOARG16 },
/* 0x54 */ { invOp,   NOARG16 },
/* 0x55 */ { invOp,   NOARG16 },
/* 0x56 */ { invOp,   NOARG16 },
/* 0x57 */ { invOp,   NOARG16 },
/* 0x58 */ { invOp,   NOARG16 },
/* 0x59 */ { invOp,   NOARG16 },
/* 0x5a */ { invOp,   NOARG16 },
/* 0x5b */ { invOp,   NOARG16 },
/* 0x5c */ { invOp,   NOARG16 },
/* 0x5d */ { invOp,   NOARG16 },
/* 0x5e */ { invOp,   NOARG16 },
/* 0x5f */ { invOp,   NOARG16 },

/* Data stack:  arg8 = 8 bit unsigned data (no stack arguments) */

/* 0x60 */ { invOp,   NOARG16 },
/* 0x61 */ { invOp,   NOARG16 },
/* 0x62 */ { invOp,   NOARG16 },
/* 0x63 */ { invOp,   NOARG16 },
/* 0x64 */ { invOp,   NOARG16 },
/* 0x65 */ { invOp,   NOARG16 },
/* 0x66 */ { invOp,   NOARG16 },
/* 0x67 */ { invOp,   NOARG16 },
/* 0x68 */ { invOp,   NOARG16 },
/* 0x69 */ { invOp,   NOARG16 },
/* 0x6a */ { invOp,   NOARG16 },
/* 0x6b */ { invOp,   NOARG16 },
/* 0x6c */ { invOp,   NOARG16 },
/* 0x6d */ { invOp,   NOARG16 },
/* 0x6e */ { invOp,   NOARG16 },
/* 0x6f */ { invOp,   NOARG16 },

/* Floating Point Operations:  arg8 = FP op-code */

/* 0x70 */ { "FLOAT", fpOP    },
/* 0x71 */ { invOp,   NOARG16 },
/* 0x72 */ { invOp,   NOARG16 },
/* 0x73 */ { invOp,   NOARG16 },
/* 0x74 */ { "PUSHB", NOARG16 },
/* 0x75 */ { invOp,   NOARG16 },
/* 0x76 */ { invOp,   NOARG16 },
/* 0x77 */ { invOp,   NOARG16 },

/* 0x78 */ { invOp,   NOARG16 },
/* 0x79 */ { invOp,   NOARG16 },
/* 0x7a */ { invOp,   NOARG16 },
/* 0x7b */ { invOp,   NOARG16 },
/* 0x7c */ { invOp,   NOARG16 },
/* 0x7d */ { invOp,   NOARG16 },
/* 0x7e */ { invOp,   NOARG16 },
/* 0x7f */ { invOp,   NOARG16 },

/************ OPCODES WITH SINGLE 16-BIT ARGUMENT (arg16) ************/

/* 0x80 */ { invOp,   NOARG16 },
/* 0x81 */ { invOp,   NOARG16 },
/* 0x82 */ { invOp,   NOARG16 },
/* 0x83 */ { invOp,   NOARG16 },
/* 0x84 */ { invOp,   NOARG16 },
/* 0x85 */ { invOp,   NOARG16 },
/* 0x86 */ { invOp,   NOARG16 },
/* 0x87 */ { invOp,   NOARG16 },
/* 0x88 */ { invOp,   NOARG16 },
/* 0x89 */ { invOp,   NOARG16 },
/* 0x8a */ { invOp,   NOARG16 },
/* 0x8b */ { invOp,   NOARG16 },
/* 0x8c */ { invOp,   NOARG16 },
/* 0x8d */ { invOp,   NOARG16 },
/* 0x8e */ { invOp,   NOARG16 },
/* 0x8f */ { invOp,   NOARG16 },

/* Program control:  arg16 = unsigned label (One stack argument) */

/* 0x90 */ { "JEQUZ", HEX },
/* 0x91 */ { "JNEQZ", HEX },
/* 0x92 */ { "JLTZ ", HEX },
/* 0x93 */ { "JGTEZ", HEX },
/* 0x94 */ { "JGTZ ", HEX },
/* 0x95 */ { "JLTEZ", HEX },

/* Program control:  arg16 = unsigned label (no stack arguments) */

/* 0x96 */ { "JMP  ", HEX },
/* 0x97 */ { invOp,   NOARG16 },

/* Program control:  arg16 = unsigned label (One stack argument) */

/* 0x98 */ { "JEQU ", HEX },
/* 0x99 */ { "JNEQ ", HEX },
/* 0x9a */ { "JLT  ", HEX },
/* 0x9b */ { "JGTE ", HEX },
/* 0x9c */ { "JGT  ", HEX },
/* 0x9d */ { "JLTE ", HEX },
/* 0x9e */ { invOp,   NOARG16 },
/* 0x9f */ { invOp,   NOARG16 },

/* Data stack:  arg16 = 16 bit signed data (no stack arguments) */

/* Load:  arg16 = unsigned base offset (no stack arguments) */

/* 0xa0 */ { "LD   ", UDECIMAL },
/* 0xa1 */ { "LDH  ", UDECIMAL },
/* 0xa2 */ { "LDB  ", UDECIMAL },
/* 0xa3 */ { "LDM  ", UDECIMAL },

/* Store: arg16 = unsigned base offset (One stack arguments) */

/* 0xa4 */ { "ST   ", UDECIMAL },
/* 0xa5 */ { "STH  ", UDECIMAL },
/* 0xa6 */ { "STB  ", UDECIMAL },
/* 0xa7 */ { "STM  ", UDECIMAL },

/* Load Indexed: arg16 = unsigned base offset (One stack arguments) */

/* 0xa8 */ { "LDX  ", UDECIMAL },
/* 0xa9 */ { "LDXH ", UDECIMAL },
/* 0xaa */ { "LDXB ", UDECIMAL },
/* 0xab */ { "LDXM ", UDECIMAL },

/* Store Indexed: arg16 = unsigned base offset (Two stack arguments) */

/* 0xac */ { "STX  ", UDECIMAL },
/* 0xad */ { "STXH ", UDECIMAL },
/* 0xae */ { "STXB ", UDECIMAL },
/* 0xaf */ { "STXM ", UDECIMAL },

/* 0xb0 */ { "LA   ", UDECIMAL },
/* 0xb1 */ { "LAC  ", HEX,    },
/* 0xb2 */ { invOp,   NOARG16 },
/* 0xb3 */ { invOp,   NOARG16 },
/* 0xb4 */ { "PUSH ", DECIMAL },
/* 0xb5 */ { "INDS ", DECIMAL },
/* 0xb6 */ { invOp,   NOARG16 },
/* 0xb7 */ { invOp,   NOARG16 },
/* 0xb8 */ { "LAX  ", UDECIMAL },

/* System operations: arg16 = 16-bit library function identifer */

/* 0xb9 */ { "LIB  ", lbOP,   },
/* 0xba */ { invOp,   NOARG16 },
/* 0xbb */ { invOp,   NOARG16 },
/* 0xbc */ { invOp,   NOARG16 },
/* 0xbd */ { invOp,   NOARG16 },
/* 0xbe */ { invOp,   NOARG16 },

/* Program control:  arg16 = unsigned label (no stack arguments) */

/* 0xbf */ { "LABEL", LABEL_DEC },

/**** OPCODES WITH BYTE ARGUMENT (arg8) AND 16-BIT ARGUMENT (arg16) ****/

/* 0xc0 */ { invOp,   NOARG16 },
/* 0xc1 */ { invOp,   NOARG16 },
/* 0xc2 */ { invOp,   NOARG16 },
/* 0xc3 */ { invOp,   NOARG16 },
/* 0xc4 */ { invOp,   NOARG16 },
/* 0xc5 */ { invOp,   NOARG16 },
/* 0xc6 */ { invOp,   NOARG16 },
/* 0xc7 */ { invOp,   NOARG16 },

/* Program Control:  arg8 = level; arg16 = unsigned label (No stack
 * arguments */

/* 0xc8 */ { "PCAL ", HEX },
/* 0xc9 */ { invOp,   NOARG16 },
/* 0xca */ { invOp,   NOARG16 },
/* 0xcb */ { invOp,   NOARG16 },
/* 0xcc */ { invOp,   NOARG16 },
/* 0xcd */ { invOp,   NOARG16 },
/* 0xce */ { invOp,   NOARG16 },
/* 0xcf */ { invOp,   NOARG16 },

/* 0xd0 */ { invOp,   NOARG16 },
/* 0xd1 */ { invOp,   NOARG16 },
/* 0xd2 */ { invOp,   NOARG16 },
/* 0xd3 */ { invOp,   NOARG16 },
/* 0xd4 */ { invOp,   NOARG16 },
/* 0xd5 */ { invOp,   NOARG16 },
/* 0xd6 */ { invOp,   NOARG16 },
/* 0xd7 */ { invOp,   NOARG16 },
/* 0xd8 */ { invOp,   NOARG16 },
/* 0xd9 */ { invOp,   NOARG16 },
/* 0xda */ { invOp,   NOARG16 },
/* 0xdb */ { invOp,   NOARG16 },
/* 0xdc */ { invOp,   NOARG16 },
/* 0xdd */ { invOp,   NOARG16 },
/* 0xde */ { invOp,   NOARG16 },
/* 0xdf */ { invOp,   NOARG16 },

/* Load:  arg8 = level; arg16 = signed frame offset (no stack arguments) */

/* 0xe0 */ { "LDS  ", DECIMAL },
/* 0xe1 */ { "LDSH ", DECIMAL },
/* 0xe2 */ { "LDSB ", DECIMAL },
/* 0xe3 */ { "LDSM ", DECIMAL },

/* Store: arg8 = level; arg16 = signed frame offset (One stack arguments) */

/* 0xe4 */ { "STS  ", DECIMAL },
/* 0xe5 */ { "STSH ", DECIMAL },
/* 0xe6 */ { "STSB ", DECIMAL },
/* 0xe7 */ { "STSM ", DECIMAL },

/* Load Indexed: arg8 = level; arg16 = signed frame offset (One stack arguments) */

/* 0xe8 */ { "LDSX ", DECIMAL },
/* 0xe9 */ { "LDSXH", DECIMAL },
/* 0xea */ { "LDSXB", DECIMAL },
/* 0xeb */ { "LDSXM", DECIMAL },

/* Store Indexed: arg8 = level; arg16 = signed frame offset (Two stack arguments) */

/* 0xec */ { "STSX ", DECIMAL },
/* 0xed */ { "STSXH", DECIMAL },
/* 0xee */ { "STSXB", DECIMAL },
/* 0xef */ { "STSXM", DECIMAL },

/* Load Address:  arg8 = level; arg16 = signed frame offset (no stack arguments) */

/* 0xf0 */ { "LAS  ", DECIMAL },
/* 0xf1 */ { invOp,   NOARG16 },
/* 0xf2 */ { invOp,   NOARG16 },
/* 0xf3 */ { invOp,   NOARG16 },
/* 0xf4 */ { invOp,   NOARG16 },
/* 0xf5 */ { invOp,   NOARG16 },
/* 0xf6 */ { invOp,   NOARG16 },
/* 0xf7 */ { invOp,   NOARG16 },
/* 0xf8 */ { "LASX ", DECIMAL },

/* System Functions:  (No stack arguments)
 * For SYSIO:        arg8 = file number; arg16 = sub-function code
 */

/* 0xf9 */ { "SYSIO", xOP,    },
/* 0xfa */ { invOp,   NOARG16 },
/* 0xfb */ { invOp,   NOARG16 },
/* 0xfc */ { invOp,   NOARG16 },
/* 0xfd */ { invOp,   NOARG16 },
/* 0xfe */ { invOp,   NOARG16 },

/* Pseudo-operations:
 * For LINE:         arg8 = file number; arg16 = line number
 */

/* 0xff */ { "LINE ", COMMENT },
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

static const char invFpOp[] = "Invalid FP Operation";
static const char *fpName[MAX_FOP] = {
/* 0x00 */ invFpOp,    "FLOAT",    "TRUNC",    "ROUND",
/* 0x04 */ "ADD",      "SUB",      "MUL",      "DIV",
/* 0x08 */ "MOD",      invFpOp,    "EQU",      "NEQ",
/* 0x0c */ "LT",       "GTE",      "GT",       "LTE",
/* 0x10 */ "NEG",      "ABS",      "SQR",      "SQRT",
/* 0x14 */ "SIN",      "COS",      "ATAN",     "LN",
/* 0x18 */ "EXP" };

/***********************************************************************/

void insn_DisassemblePCode(FILE* lfile, OPTYPE *pop)
{
  /* Indent, comment or label */

  switch (opTable[pop->op].format)
    {
    case LABEL_DEC :
      fprintf(lfile, "L%04x:  ", pop->arg2);
      break;
    case COMMENT   :
      fprintf(lfile, "; ");
      break;
    default   :
      fprintf(lfile, "        ");
    } /* end switch */

  /* Special Case Comment line format */

  if (opTable[pop->op].format == COMMENT)
    {
      fprintf(lfile, "%s ", opTable[pop->op].opName);
      if (pop->op & o8)
        {
          fprintf(lfile, "%d", pop->arg1);
          if (pop->op & o16)
            fprintf(lfile, ":%d", pop->arg2);
        } /* end if */
      else if (pop->op & o16)
        fprintf(lfile, "%d", pop->arg2);
    } /* end if */

  /* Print normal opCode mnemonic */

  else
    {
      fprintf(lfile, "%s ", opTable[pop->op].opName);

      /* Print pop->arg1 (if present) */

      if (pop->op & o8) fprintf(lfile, "%d", pop->arg1);

      /* Print ar16 (if present) */

      if (pop->op & o16)
        {
          switch (opTable[pop->op].format)
            {
            case HEX       :
              if (pop->op & o8) fprintf(lfile, ", ");
              fprintf(lfile, "0x%04x", pop->arg2);
              break;

            case COMMENT   :
            case DECIMAL   :
              if (pop->op & o8) fprintf(lfile, ", ");
              fprintf(lfile, "%ld", signExtend16(pop->arg2));
              break;

            case UDECIMAL   :
              if (pop->op & o8) fprintf(lfile, ", ");
              fprintf(lfile, "%u", pop->arg2);
              break;

            case fpOP       :
              if ((pop->arg1 & fpMASK) < MAX_FOP)
                fprintf(lfile, " %s", fpName[(pop->arg1 & 0x3f)]);
              else
                fprintf(lfile, " %s", invFpOp);
              break;

            case xOP       :
              if (pop->arg2 < MAX_XOP)
                fprintf(lfile, ", %s", xName[pop->arg2]);
              else
                fprintf(lfile, ", %s", invXOp);
              break;

            case lbOP :
              if (pop->arg2 < MAX_LBOP)
                fprintf(lfile, "%s", lbName[pop->arg2]);
              else
                fprintf(lfile, "%s", invLbOp);
              break;

            case LABEL_DEC :
            default        :
              break;
            } /* end switch */
        } /* end if */
    } /* end else */

  /* Don't forget the newline! */

  fputc('\n', lfile);

} /* end dissassemblePcode */

/***********************************************************************/
