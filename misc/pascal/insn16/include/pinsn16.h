/****************************************************************************
 * pinsn16.h
 * 16-bit P-code operation code definitions
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __PINSN16_H
#define __PINSN16_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Op-code bit definitions */

#define o16    (0x80)
#define o8     (0x40)

/* Opcode Encoding Summary:
 *
 *            NO ARGS    arg8 ONLY      arg16 ONLY     BOTH
 *            00xx xxxx  01xx xxxx      10xx xxxx      11xx xxxx
 * xx00 0000  NOP        ---            ---            ---
 * xx00 0001  NEG        ---            ---            ---
 * xx00 0010  ABS        ---            ---            ---
 * xx00 0011  INC        ---            ---            ---
 * xx00 0100  DEC        ---            ---            ---
 * xx00 0101  NOT        ---            ---            ---
 * xx00 0110  ADD        ---            ---            ---
 * xx00 0111  SUB        ---            ---            ---
 * xx00 1000  MUL        ---            ---            PCAL l,ilbl
 * xx00 1001  DIV        ---            ---            ---
 * xx00 1010  MOD        ---            ---            ---
 * xx00 1011  SLL        ---            ---            ---
 * xx00 1100  SRL        ---            ---            ---
 * xx00 1101  SRA        ---            ---            ---
 * xx00 1110  OR         ---            ---            ---
 * xx00 1111  AND        ---            ---            ---
 *
 * xx01 0000  EQUZ       ---            JEQUZ ilbl     ---
 * xx01 0001  NEQZ       ---            JNEQZ ilbl     ---
 * xx01 0010  LTZ        ---            JLTZ  ilbl     ---
 * xx01 0011  GTEZ       ---            JGTEZ ilbl     ---
 * xx01 0100  GTZ        ---            JGTZ  ilbl     ---
 * xx01 0101  LTEZ       ---            JLTEZ ilbl     ---
 * xx01 0110  ---        ---            JMP   ilbl     ---
 * xx01 0111  ---        ---            ---            ---
 * xx01 1000  EQU        ---            JEQU  ilbl     ---
 * xx01 1001  NEQ        ---            JNEQ  ilbl     ---
 * xx01 1010  LT         ---            JLT   ilbl     ---
 * xx01 1011  GTE        ---            JGTE  ilbl     ---
 * xx01 1100  GT         ---            JGT   ilbl     ---
 * xx01 1101  LTE        ---            JLTE  ilbl     ---
 * xx01 1110  ---        ---            ---            ---
 * xx01 1111  BIT        ---            ---            ---
 *
 * xx10 0000  LDI        ---            LD uoffs       LDS lvl,offs
 * xx10 0001  LDIH       ---            LDH uoffs      LDSH lvl,offs
 * xx10 0010  LDIB       ---            LDB uoffs      LDSB lvl,offs
 * xx10 0011  LDIM       ---            LDM uoffs      LDSM lvl,offs
 * xx10 0100  STI        ---            ST uoffs       STS lvl,offs
 * xx10 0101  STIH       ---            STH uoffs      STSH lvl,offs
 * xx10 0110  STIB       ---            STB uoffs      STSB lvl,offs
 * xx10 0111  STIM       ---            STM uoffs      STSM lvl,offs
 * xx10 1000  DUP        ---            LDX uoffs      LDSX lvl,offs
 * xx10 1001  DUPH       ---            LDXH uoffs     LDSXH lvl,offs
 * xx10 1010  PUSHS      ---            LDXB uoffs     LDSXB lvl,offs
 * xx10 1011  POPS       ---            LDXM uoffs     LDSXM lvl,offs
 * xx10 1100  ---        ---            STX uoffs      STSX lvl,offs
 * xx10 1101  ---        ---            STXH uoffs     STSXH lvl,offs
 * xx10 1110  ---        ---            STXB uoffs     STSXB lvl,offs
 * xx10 1111  RET        ---            STXM uoffs     STSXM lvl,offs
 *
 * xx11 0000  ---        FLOAT fop      LA uoffs       LAS lvl,offs
 * xx11 0001  ---        ---            LAC dlbl       ---
 * xx11 0010  ---        ---            ---            ---
 * xx11 0011  ---        ---            ---            ---
 * xx11 0100  ---        PUSHB n        PUSH nn        ---
 * xx11 0101  ---        ---            INDS nn        ---
 * xx11 0110  ---        ---            ---            ---
 * xx11 0111  ---        ---            ---            ---
 * xx11 1000  ---        ---            LAX uoffs      LASX lvl,offs
 * xx11 1001  ---        ---            LIB lop        SYSIO fn,sop
 * xx11 1010  ---        ---            ---            ---
 * xx11 1011  ---        ---            ---            ---
 * xx11 1100  ---        ---            ---            ---
 * xx11 1101  ---        ---            ---            ---
 * xx11 1110  ---        ---            ---            ---
 * xx11 1111  END        ---           *LABEL ilbl    *LINE fn,lineno 
 *
 * KEY:
 *   n     = 8-bit value (unsigned)
 *   lvl   = 8-bit static nesting level offset (unsigned)
 *   vt    = 8-bit type code (unsigned)
 *   nn    = 16-bit value (signed)
 *   fop   = 8-bit floating point operation
 *   sop   = 16-bit sysio operation
 *   lop   = 16-bit library call identifier
 *   fn    = 8-bit file number
 *   ilbl  = instruction space label
 *   dlbl  = stack data label
 *   offs  = 16-bit frame offset (signed)
 *   uoffs = 16-bit base offset (unsigned)
 *   c     = string follows pseudo-operation
 *   *     = Indicates pseudo-operations (these are removed
 *           after final fixup of the object file).
 */

/** OPCODES WITH NO ARGUMENTS ***********************************************/

/* Program control (No stack arguments) */

#define oNOP   (0x00)

/* Arithmetic & logical & and integer conversions (One 16-bit stack argument) */

#define oNEG   (0x01)
#define oABS   (0x02)
#define oINC   (0x03)
#define oDEC   (0x04)
#define oNOT   (0x05)

/* Arithmetic & logical (Two 16-bit stack arguments) */

#define oADD   (0x06)
#define oSUB   (0x07)
#define oMUL   (0x08)
#define oDIV   (0x09)
#define oMOD   (0x0a)
#define oSLL   (0x0b)
#define oSRL   (0x0c)
#define oSRA   (0x0d)
#define oOR    (0x0e)
#define oAND   (0x0f)

/* Comparisons (One 16-bit stack argument) */

#define oEQUZ  (0x10)
#define oNEQZ  (0x11)
#define oLTZ   (0x12)
#define oGTEZ  (0x13)
#define oGTZ   (0x14)
#define oLTEZ  (0x15)

/* 0x16-0x17 -- unassigned */

/* Comparisons (Two 16-bit stack arguments) */

#define oEQU   (0x18)
#define oNEQ   (0x19)
#define oLT    (0x1a)
#define oGTE   (0x1b)
#define oGT    (0x1c)
#define oLTE   (0x1d)

/* 0x1e -- unassigned */

#define oBIT   (0x1f)

/* Load Immediate */

#define oLDI   (0x20)	/* (One 16-bit stack argument) */
#define oLDIH  (0x21)	/* (One 16-bit stack argument) */
#define oLDIB  (0x22)   /* (One 16-bit stack argument) */
#define oLDIM  (0x23)	/* (Two 16-bit stack arguments) */

/* Store Immediate */

#define oSTI   (0x24)	/* (One 32-bit and one 16-bit stack arguments) */
#define oSTIH  (0x25)	/* (Two 16-bit stack arguments) */
#define oSTIB  (0x26)   /* (Two 16-bit stack arguments) */
#define oSTIM  (0x27)	/* (Two + n 16-bit stack arguments) */

/* Data stack */

#define oDUP   (0x28)   /* (One 32-bit stack argument */
#define oDUPH  (0x29)   /* (One 16-bit stack argument) */

/* 0x2a - 0x2b -- unassigned */

#define oPUSHS (0x2a)   /* No arguments */
#define oPOPS  (0x2b)   /* (One 16-bit stack argument) */

/* 0x2c - 0x2e -- unassigned */

/* Program control (No stack arguments)
 * Behavior:
 *   Pop return address
 *   Pop saved base register (BR)
 *   Discard saved base address
 *   Set program counter (PC) to return address
 */

#define oRET   (0x2f)

/* 0x30 - 0x3e -- unassigned */

/* System Functions (No stack arguments) */

#define oEND   (0x3f)

/** OPCODES WITH SINGLE BYTE ARGUMENT (arg8) ********************************/

/* (o8|0x00)-(o8|0x2f) -- unassigned */

/* Floating point operations: arg8 = FP op-code */

#define oFLOAT  (o8|0x30)

/* (o8|0x31)-(o8|0x33) -- unassigned */

/* Data stack:  arg8 = 8 bit unsigned data (no stack arguments) */

#define oPUSHB  (o8|0x34)

/* (o8|0x35)-(o8|0x3f) -- unassigned */

/** OPCODES WITH SINGLE 16-BIT ARGUMENT (arg16) *****************************/

/* (o16|0x00)-(o16|0x0f) -- unassigned */

/* Program control:  arg16 = unsigned label (One 16-bit stack argument) */

#define oJEQUZ (o16|0x10)
#define oJNEQZ (o16|0x11)
#define oJLTZ  (o16|0x12)
#define oJGTEZ (o16|0x13)
#define oJGTZ  (o16|0x14)
#define oJLTEZ (o16|0x15)

/* Program control:  arg16 = unsigned label (no stack arguments) */

#define oJMP   (o16|0x16)

/* (o16|0x17) -- unassigned */

/* Program control:  arg16 = unsigned label (One 16-bit stack argument) */

#define oJEQU  (o16|0x18)
#define oJNEQ  (o16|0x19)
#define oJLT   (o16|0x1a)
#define oJGTE  (o16|0x1b)
#define oJGT   (o16|0x1c)
#define oJLTE  (o16|0x1d)

/* (o16|0x1e)-(o16|0x1f) -- unassigned */

/* Load:  arg16 = unsigned base offset */

#define oLD    (o16|0x20)       /* (no stack arguments) */
#define oLDH   (o16|0x21)       /* (no stack arguments) */
#define oLDB   (o16|0x22)       /* (no stack arguments) */
#define oLDM   (o16|0x23)       /* (One 16-bit stack argument) */

/* Store: arg16 = unsigned base offset */

#define oST    (o16|0x24)       /* (One 32-bit stack argument) */
#define oSTH   (o16|0x25)       /* (One 16-bit stack argument) */
#define oSTB   (o16|0x26)       /* (One 16-bit stack argument) */
#define oSTM   (o16|0x27)       /* (One+n 16-bit stack arguments) */

/* Load Indexed: arg16 = unsigned base offset */

#define oLDX   (o16|0x28)       /* (One 16-bit stack argument) */
#define oLDXH  (o16|0x29)       /* (One 16-bit stack argument) */
#define oLDXB  (o16|0x2a)       /* (One 16-bit stack argument) */
#define oLDXM  (o16|0x2b)       /* (Two 16-bit stack arguments) */

/* Store Indexed: arg16 = unsigned base offset */

#define oSTX   (o16|0x2c)       /* (One 32-bit + one 16-bit stack arguments) */
#define oSTXH  (o16|0x2d)       /* (Two 16-bit stack arguments) */
#define oSTXB  (o16|0x2e)       /* (Two 16-bit stack arguments) */
#define oSTXM  (o16|0x2f)       /* (Two+n 16-bit stack arguments) */

/* Load address relative to stack base: arg16 = unsigned offset */

#define oLA    (o16|0x30)

/* Load absolute stack address:  arg16 = RODATA offset (No stack arguments) */

#define oLAC   (o16|0x31)

/* (o16|0x32)-(o16|0x33) -- unassigned */

/* Data stack:  arg16 = 16 bit signed data (no stack arguments) */

#define oPUSH  (o16|0x34)
#define oINDS  (o16|0x35)

/* (o16|0x34)-(o16|0x37) -- unassigned */

/* Load address relative to stack base: arg16 = unsigned offset, TOS=index */

#define oLAX   (o16|0x38)

/* System functions: arg16 = 16-bit library call identifier */

#define oLIB   (o16|0x39)

/* (o16|0x3a)-(o16|0x3e) -- unassigned */

/* Program control:  arg16 = unsigned label (no stack arguments) */

#define oLABEL (o16|0x3f)

/** OPCODES WITH 24-BITS OF ARGUMENET (arg8 + arg16) ************************/

/* (o16|o8|0x00)-(o8|o16|0x07) -- unassigned */

/* Program Control:  arg8 = level; arg16 = unsigned label
 *                  (No stack arguments)
 * Behavior:
 *   Push base address of level
 *   Push base register (BR) value
 *   Set new base register value (BR) as top of stack
 *   Push return address
 *   Set program counter (PC) for address associated with label
 */

#define oPCAL  (o16|o8|0x08)

/* (o16|o8|0x09)-(o8|o16|0x1f) -- unassigned */

/* Load:  arg8 = level; arg16 = signed frame offset */

#define oLDS   (o16|o8|0x20)	/* (no stack arguments) */
#define oLDSH  (o16|o8|0x21)	/* (no stack arguments) */
#define oLDSB  (o16|o8|0x22)    /* (no stack arguments) */
#define oLDSM  (o16|o8|0x23)	/* (One 16-bit stack argument) */

/* Store: arg8 = level; arg16 = signed frame offset */

#define oSTS   (o16|o8|0x24)	/* (One 32-bit stack argument) */
#define oSTSH  (o16|o8|0x25)	/* (One 16-bit stack argument) */
#define oSTSB  (o16|o8|0x26)    /* (One 16-bit stack argument) */
#define oSTSM  (o16|o8|0x27)	/* (One+n 16-bit stack arguments) */

/* Load Indexed: arg8 = level; arg16 = signed frame offset */

#define oLDSX  (o16|o8|0x28)	/* (One 16-bit stack argument) */
#define oLDSXH (o16|o8|0x29)	/* (One 16-bit stack argument) */
#define oLDSXB (o16|o8|0x2a)    /* (One 16-bit stack argument) */
#define oLDSXM (o16|o8|0x2b)    /* (Two 16-bit stack arguments) */

/* Store Indexed: arg8 = level; arg16 = signed frame offset */

#define oSTSX  (o16|o8|0x2c)    /* (One 32-bit + one 16-bit stack arguments) */
#define oSTSXH (o16|o8|0x2d)    /* (Two 16-bit stack arguments) */
#define oSTSXB (o16|o8|0x2e)    /* (Two 16-bit stack arguments) */
#define oSTSXM (o16|o8|0x2f)    /* (Two+n 16-bit stack arguments) */

/* FOR LAS/LASX    arg8 = level; arg16 = signed frame offset
 *                          (no stack arguments)
 */

#define oLAS   (o16|o8|0x30)
#define oLASX  (o16|o8|0x38)

/* System calls:
 * For SYSIO:        arg8 = file number; arg16 = sub-function code
 */

#define oSYSIO  (o16|o8|0x39)

/* (o16|o8|0x3a)-(o8|o16|0x3e) -- unassigned */

/* Pseudo-operations:
 * For LINE:         arg8 = file number; arg16 = line number
 */

#define oLINE    (o16|o8|0x3f)

#endif /* __PINSN16_H */
