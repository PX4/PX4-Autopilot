/***********************************************************************
 * podefs.h
 * Logical P-code operation code definitions
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
 ***********************************************************************/

#ifndef __PODEFS_H
#define __PODEFS_H

/* These definitions represent logical operations as needed by the
 * the compiler.  The specific INSN generation layer must interpret
 * these requests as is appropriate to the supported INSNS.
 */

enum pcode_e
{

  /**-------------------------------------------------------------------
   * OPCODES WITH NO ARGUMENTS
   **-------------------------------------------------------------------**/

  /* Program control (No stack arguments) */

  opNOP = 0,

  /* Arithmetic & logical & and integer conversions (One stack argument) */

  opNEG, opABS, opINC, opDEC, opNOT,

  /* Arithmetic & logical (Two stack arguments) */

  opADD, opSUB, opMUL, opDIV, opMOD, opSLL, opSRL, opSRA, opOR, opAND,

  /* Comparisons (One stack argument) */

  opEQUZ, opNEQZ, opLTZ, opGTEZ, opGTZ, opLTEZ,

  /* Comparisons (Two stack arguments) */

  opEQU, opNEQ, opLT, opGTE, opGT, opLTE,
  opBIT,

  /* Load Immediate */

  opLDI, opLDIB, opLDIM,

  /* Store Immediate */

  opSTI, opSTIB, opSTIM,

  /* Data stack */

  opDUP, opPUSHS, opPOPS,

  /* Program control (No stack arguments)
   * Behavior:
   *   Pop return address
   *   Pop saved base register (BR)
   *   Discard saved base address
   *   Set program counter (PC) to return address
   */

  opRET,

  /* System Functions (No stack arguments) */

  opEND,

  /**-------------------------------------------------------------------
   ** OPCODES WITH ONE ARGUMENT
   **-------------------------------------------------------------------**/

  /* Floating point operations: arg = FP op-code */

  opFLOAT,

  /* Program control:  arg = unsigned label (One stack argument) */

  opJEQUZ, opJNEQZ,

  /* Program control:  arg = unsigned label (no stack arguments) */

  opJMP,

  /* Program control:  arg = unsigned label (One stack argument) */

  opJEQU, opJNEQ, opJLT, opJGTE, opJGT, opJLTE,

  /* Load:  arg = unsigned base offset */

  opLD, opLDH, opLDB, opLDM,

  /* Store: arg = unsigned base offset */

  opST, opSTB, opSTM,

  /* Load Indexed: arg = unsigned base offset */

  opLDX, opLDXB, opLDXM,

  /* Store Indexed: arg16 = unsigned base offset */

  opSTX, opSTXB, opSTXM,

  /* Load address relative to stack base: arg = unsigned offset */

  opLA,

  /* Load absolute stack address:  arg = RODATA offset (No stack arguments) */

  opLAC,

  /* Data stack:  arg = 16 bit signed data (no stack arguments) */

  opPUSH, opINDS,

  /* Load address relative to stack base: arg1 = unsigned offset, TOS=index */

  opLAX,

  /* System functions: arg = 16-bit library call identifier */

  opLIB,

  /* Program control:  arg = unsigned label (no stack arguments) */

  opLABEL,

  /**-------------------------------------------------------------------
   ** OPCODES WITH TWO ARGUMENTS
   **-------------------------------------------------------------------**/

  /* Program Control:  arg1 = level; arg2 = unsigned label */

  opPCAL,

  /* Load:  arg1 = level; arg2 = signed frame offset */

  opLDS, opLDSH, opLDSB, opLDSM,

  /* Store: arg1 = level; arg2 = signed frame offset */

  opSTS, opSTSB, opSTSM, 

  /* Load Indexed: arg1 = level; arg2 = signed frame offset */

  opLDSX, opLDSXB, opLDSXM,

  /* Store Indexed: arg1 = level; arg2 = signed frame offset */

  opSTSX, opSTSXB, opSTSXM,

  /* FOR LAS/LASX    arg1 = level; arg2 = signed frame offset
   *                          (no stack arguments)
   */

  opLAS, opLASX,

  /* System calls:
   * For SYSIO:        arg1 = file number; arg2 = sub-function code
   */

  opSYSIO,

  /* Pseudo-operations:
   * For LINE:         arg1 = file number; arg2 = line number
   */

  opLINE,

  NUM_OPCODES
};

#endif /* __PODEFS_H */
