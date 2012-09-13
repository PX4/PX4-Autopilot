/****************************************************************************
 * pinsn32.h
 * 32-bit P-code operation code definitions
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

#ifndef __PINSN32_H
#define __PINSN32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* 32-bit op-code bit definitions
 *
 * Machine model:
 *
 *   SPB 32-bit Pascal stack base address
 *   SP  32-bit Pascal stack pointer
 *   LSP 32-bit Level stack pointer
 *   CSB 32-bit Character stack base address
 *   CSP 32-bit Character stack pointer
 *   DS  32-bit Data size register (for multiple reg transfers)
 *   PC  32-bit Program Counter
 *   CC  Condition code register
 *   --- Volatile general purpose registers
 *   --- Static general purpose registers
 *
 * Condition codes:  Z(ero), N(egative)
 *
 *        +=====+=====+
 *        |  Z  |  N  |
 *  +=====+=====+=====+
 *  | EQ  |  1  |  -  |
 *  | NEQ |  0  |  -  |
 *  | LT  |  -  |  1  |
 *  | GTE |  -  |  0  |
 *  | GT  |  0  |  0  |
 *  | LTE |  1  |  1  |
 *  +=====+=====+=====+
 *   
 * Opcode Encoding Summary:
 *
 *            0rxx xxxx  1rxxx xxxx
 * xr00 0000  NOP        LD   uoffs4
 * xr00 0001  NEG        LDH  uoffs3
 * xr00 0010  ABS        LDB  uoffs
 * xr00 0011  INC        LDM  uoffs4
 * xr00 0100  DEC        ST   uoffs4
 * xr00 0101  NOT        STH  uoffs2
 * xr00 0110  ADD        STB  uoffs
 * xr00 0111  SUB        STM  uoffs4
 * xr00 1000  MUL        LDX  uoffs4
 * xr00 1001  DIV        LDXH uoffs2
 * xr00 1010  MOD        LDXB uoffs
 * xr00 1011  SLL        LDXM uoffs4
 * xr00 1100  SRL        STX  uoffs4
 * xr00 1101  SRA        STXH uoffs2
 * xr00 1110  OR         STXB uoffs
 * xr00 1111  AND        STXM uoffs
 *
 * xr01 0000  EQUZ       JEQUZ ilbl
 * xr01 0001  NEQZ       JNEQZ ilbl
 * xr01 0010  LTZ        JLTZ  ilbl
 * xr01 0011  GTEZ       JGTEZ ilbl
 * xr01 0100  GTZ        JGTZ  ilbl
 * xr01 0101  LTEZ       JLTEZ ilbl
 * xr01 0110  ---        JMP   ilbl
 * xr01 0111  ---        PUSH  nn
 * xr01 1000  EQU        JEQU  ilbl
 * xr01 1001  NEQ        JNEQ  ilbl
 * xr01 1010  LT         JLT   ilbl
 * xr01 1011  GTE        JGTE  ilbl    
 * xr01 1100  GT         JGT   ilbl
 * xr01 1101  LTE        JLTE  ilbl
 * xr01 1110  ---        ---
 * xr01 1111  BIT        INDS  nn
 *
 * xr10 0000  LDI        LDS   offs4
 * xr10 0001  LDIH       LDSH  offs3
 * xr10 0010  LDIB       LDSB  offs
 * xr10 0011  LDIM       LDSM  offs4
 * xr10 0100  STI        STS   offs4
 * xr10 0101  STIH       STSH  offs2
 * xr10 0110  STIB       STSB  offs
 * xr10 0111  STIM       STSM  offs4
 * xr10 1000  DUP        LDSX  offs4
 * xr10 1001  ---        LDSXH offs2
 * xr10 1010  PUSHS      LDSXB offs
 * xr10 1011  POPS       LDSXM offs4
 * xr10 1100  ---        STSX  offs4
 * xr10 1101  ---        STSXH offs2
 * xr10 1110  ---        STSXB offs
 * xr10 1111  RET        STSXM offs
 *
 * xr11 0000  ---        LA    uoffs       
 * xr11 0001  ---        LAS   offs
 * xr11 0010  ---        LAC   dlbl
 * xr11 0011  ---        ---
 * xr11 0100  ---        LAX   uoffs
 * xr11 0101  ---        LASX  offs
 * xr11 0110  ---        SLSP  level
 * xr11 0111  ---        SDC   uu
 * xr11 1000  ---        ---
 * xr11 1001  ---        PCAL  ilbl
 * xr11 1010  ---        SYSIO fn,sop
 * xr11 1011  ---        LIB   lop
 * xr11 1100  ---        FLOAT fop
 * xr11 1101  ---       *LABEL ilbl
 * xr11 1110  ---       *INCLUDE fn
 * xr11 1111  END       *LINE  lineno 
 *
 * KEY:
 *   r      = Reserved bit (must be zero)
 *   fn     = 8-bit file number
 *   lvl    = 8-bit static nexting level
 *   sop    = 17-bit sysio operation
 *   lineno = 17-bit line number
 *   nn     = 32-bit constant value (signed)
 *   uu     = 32-bit constant value (unsigned)
 *   fop    = 32-bit floating point operation
 *   lop    = 32-bit library call identifier
 *   ilbl   = 32-bit Instruction space label number
 *   dlbl   = 32-stack data label
 *   offs4  = 32-bit word offset with respect to LSP (signed)
 *   offs2  = 32-bit halfword offset with respect to LSP (signed)
 *   offs   = 32-bit byte offset with respect to LSP (signed)
 *   uoffs4 = 32-bit word offset with respect to SPB (unsigned)
 *   uoffs2 = 32-bit halfword offset with respect to SPB (unsigned)
 *   uoffs  = 32-bit byte offset with respect to SPB (unsigned)
 *   c      = string follows pseudo-operation
 *   *      = Indicates pseudo-operations (these are removed
 *           after final fixup of the object file).
 */

#define o32         (0x80)

#define GETOP(o)    ((o)->op)
#define PUTOP(o,v)   do { (o)->op = (v); } while (0)

#define GETARG(o)   ((o)->arg)
#define PUTARG(o,a)  do { (o)->arg = (a); } while (0)

#define ARGONES      0xffffffff

/* The opcode binary is stored in big endian order (so that the opcode
 * always comes first).  The following definitions simplify ordering
 * of byte accesses.
 */

#ifdef CONFIG_ENDIAN_BIG
# define opB1 0
# define opB2 1
# define opB3 2
# define opB4 3
#else
# define opB1 3
# define opB2 2
# define opB3 1
# define opB4 0
#endif

/** 1-BYTE OPCODES WITH NO ARGUMENTS (other than stack arguments) ***********/

/* Program control (No stack arguments) */

#define oNOP   (0x00)

/* Arithmetic & logical & and integer conversions (One 32-bit stack
 * argument)
 */

#define oNEG   (0x01)
#define oABS   (0x02)
#define oINC   (0x03)
#define oDEC   (0x04)
#define oNOT   (0x05)
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

/* Comparisons (One 32-bit stack argument) */

#define oEQUZ  (0x10)
#define oNEQZ  (0x11)
#define oLTZ   (0x12)
#define oGTEZ  (0x13)
#define oGTZ   (0x14)
#define oLTEZ  (0x15)

/* Comparisons (Two 32-bit stack arguments) */

#define oEQU   (0x18)
#define oNEQ   (0x19)
#define oLT    (0x1a)
#define oGTE   (0x1b)
#define oGT    (0x1c)
#define oLTE   (0x1d)
#define oBIT   (0x1f)

/* Load Immediate */

#define oLDI   (0x20)	/* (One 32-bit stack argument) */
#define oLDIH  (0x21)	/* (One 32-bit stack argument) */
#define oLDIB  (0x22)	/* (One 32-bit stack argument) */
#define oLDIM  (0x23)	/* (Two 32-bit stack argument) */

/* Store Immediate */

#define oSTI   (0x24)	/* (Two 32-bit stack argument) */
#define oSTIH  (0x25)	/* (Two 32-bit stack argument) */
#define oSTIB  (0x26)	/* (Two 32-bit stack argument) */
#define oSTIM  (0x27)	/* (Two+n 32-bit stack argument) */

/* Data stack */

#define oDUP   (0x28)	/* (One 32-bit stack argument) */
#define oDUPH  (0x29)	/* (One 32-bit stack argument) */
#define oPUSHS (0x2a)  /* No stack arguments */
#define oPOPS  (0x2b)	/* (One 32-bit stack argument) */

/* Program control (No stack arguments)
 * Behavior:
 *   Pop return address
 *   Pop saved base register (BR)
 *   Discard saved base address
 *   Set program counter (PC) to return address
 */

#define oRET   (0x2f)

/* System Functions (No stack arguments) */

#define oEND   (0x3f)

/** 4-BYTE OPCODES INCLUDING ONE 32-BIT ARGUMENT ****************************/

/* Load:  arg = unsigned base offset */

#define oLD    (o32|0x00) /* No stack arguments */
#define oLDH   (o32|0x01) /* No stack arguments */
#define oLDB   (o32|0x02) /* No stack arguments */
#define oLDM   (o32|0x03) /* One 32-bit stack argument */

/* Store: arg = unsigned base offset */

#define oST    (o32|0x04) /* One 32-bit stack argument */
#define oSTH   (o32|0x05) /* One 32-bit stack argument */
#define oSTB   (o32|0x06) /* One 32-bit stack argument */
#define oSTM   (o32|0x07) /* One+n 32-bit stack argument */

/* Load Indexed: arg = unsigned base offset */

#define oLDX   (o32|0x08) /* One 32-bit stack argument */
#define oLDXH  (o32|0x09) /* One 32-bit stack argument */
#define oLDXB  (o32|0x0a) /* One 32-bit stack argument */
#define oLDXM  (o32|0x0b) /* Two 32-bit stack argument */

/* Store Indexed: arg = unsigned base offset */

#define oSTX   (o32|0x0c) /* Two 32-bit stack argument */
#define oSTXH  (o32|0x0d) /* Two 32-bit stack argument */
#define oSTXB  (o32|0x0e) /* Two 32-bit stack argument */
#define oSTXM  (o32|0x0f) /* Two+n 32-bit stack argument */

/* Program control:  arg = unsigned label (One 32-bit stack argument) */

#define oJEQUZ (o32|0x10)
#define oJNEQZ (o32|0x11)
#define oJLTZ  (o32|0x12)
#define oJGTEZ (o32|0x13)
#define oJGTZ  (o32|0x14)
#define oJLTEZ (o32|0x15)

/* Program control:  arg = unsigned label (no stack arguments) */

#define oJMP   (o32|0x16)

/* Data stack:  arg = 32 bit signed data (no stack arguments) */

#define oPUSH  (o32|0x17)

/* Program control:  arg = unsigned label (One 32-bit stack argument) */

#define oJEQU  (o32|0x18)
#define oJNEQ  (o32|0x19)
#define oJLT   (o32|0x1a)
#define oJGTE  (o32|0x1b)
#define oJGT   (o32|0x1c)
#define oJLTE  (o32|0x1d)

/* Data stack:  arg = 32 bit signed data (no stack arguments) */

#define oINDS  (o32|0x1f)

/* Load:  Uses LSP; arg = signed frame offset */

#define oLDS   (o32|0x20) /* No stack arguments */
#define oLDSH  (o32|0x21) /* No stack arguments */
#define oLDSB  (o32|0x22) /* No stack arguments */
#define oLDSM  (o32|0x23) /* One 32-bit stack argument */

/* Store: Uses LSP; arg = signed frame offset */

#define oSTS   (o32|0x24) /* One 32-bit stack argument */
#define oSTSH  (o32|0x25) /* One 32-bit stack argument */
#define oSTSB  (o32|0x26) /* One 32-bit stack argument */
#define oSTSM  (o32|0x27) /* One+n 32-bit stack argument */

/* Load Indexed: Uses LSP; arg = signed frame offset */

#define oLDSX  (o32|0x28) /* One 32-bit stack argument */
#define oLDSXH (o32|0x29) /* One 32-bit stack argument */
#define oLDSXB (o32|0x2a) /* One 32-bit stack argument */
#define oLDSXM (o32|0x2b) /* Two 32-bit stack argument */

/* Store Indexed: Uses LSP; arg = signed frame offset */

#define oSTSX  (o32|0x2c) /* Two 32-bit stack argument */
#define oSTSXH (o32|0x2d) /* Two 32-bit stack argument */
#define oSTSXB (o32|0x2e) /* Two 32-bit stack argument */
#define oSTSXM (o32|0x2f) /* Two+n 32-bit stack argument */

/* Load address relative to stack base: arg = unsigned offset */

#define oLA    (o32|0x30)

/* Load address: Uses SLP, arg = signed frame offset */

#define oLAS   (o32|0x31) /* No stack arguments */

/* Load absolute stack address:  arg = RODATA offset (No stack arguments) */

#define oLAC   (o32|0x32)

/* Load address relative to stack base: arg = unsigned offset, TOS=index */

#define oLAX   (o32|0x34)

/* Load address indexed: Uses SLP, arg = signed frame offset */

#define oLASX  (o32|0x35) /* No stack arguments */

/* Set LSP:  arg = new level that evaluates to LSP value */

#define oSLSP  (o32|0x36)

/* Set DS:  arg = new byte count in DS register */

#define oSDC   (o32|0x37)

/* Program Control:  Uses LSP; arg = unsigned label
 *                  (No stack arguments)
 * Behavior:
 *   Push base address of level
 *   Push base register (BR) value
 *   Set new base register value (BR) as top of stack
 *   Push return address
 *   Set program counter (PC) for address associated with label
 */

#define oPCAL  (o32|0x39)

/* System calls:   arg = file number | sub-function code */

#define oSYSIO (o32|0x3a)

/* System functions: arg = 32-bit library call identifier */

#define oLIB   (o32|0x3b)

/* Floating point operations: arg = FP op-code */

#define oFLOAT (o32|0x3c)

/* Program control:  arg = unsigned label (no stack arguments) */

#define oLABEL (o32|0x3d)

/* Pseudo-operations:  arg = file number OR line number */

#define oINCLUDE (o32|0x3e)
#define oLINE    (o32|0x3f)

#endif /* __PINSN32_H */
