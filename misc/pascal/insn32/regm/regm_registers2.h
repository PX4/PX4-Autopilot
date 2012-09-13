/***************************************************************************
 * regm_registers2.h
 * Definitions for management of registers
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
 ***************************************************************************/

#ifndef __REGM_REGISTERS2_H
#define __REGM_REGISTERS2_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <stdint.h>

/***************************************************************************
 * Definitions
 ***************************************************************************/

/* Register types */

#define NULL_REG     0
#define SPECIAL_REG  1            /* Special "global" registers */
#define CC_REG       2            /* Condition code register instance (fake) */
#define ARG_REG      3            /* Volatile register for input arguments */
#define INARG_REG    ARG_REG      /*   from callee */
#define OUTARG_REG   4            /* Volatile register for output arguments */
                                  /*   to called function (fake) */
#define RET_REG      ARG_REG      /* Volatile register for output values */
#define OUTRET_REG   ARG_REG      /*   to callee */
#define INRET_REG    OUTARG_REG   /* Volatile register for input return */
                                  /*   from called functions (fake) */
#define SCRATCH_REG  5            /* Volatile register for general usage */
#define VOLATILE_REG 6            /* Volatile registers in general */
#define STATIC_REG   7            /* Static register */

#define NREGISTER_TYPES   8

/* Special registers */

#define SPB          0 /* 32-bit Pascal stack base address */
#define SP           1 /* 32-bit Pascal stack pointer */
#define BRG          2 /* 32-bit base register (related to parent level) */
#define LSP          3 /* 32-bit Level stack pointer */
#define CSB          4 /* 32-bit Character stack base address */
#define CSP          5 /* 32-bit Character stack pointer */
#define PC           6 /* 32-bit Program counter */

#define NSPECIAL_REGISTERS 7

#define DC           7 /* 32-bit Data count register -- disappears quickly */
#define LR           8 /* 32-bit Link register      -- same as SCRATCH(0) */
#define CC           9 /* Condition code register   -- not a normal register */

#define NSPECIAL_REGISTERS2 10

/* During pass2, we allow an indefinite number of registers.  Eventually,
 * these need to be compressed into a smaller number of registers.  These
 * define the number of registers available in the architecture.
 */

#define NTOTAL_REGISTERS      32
#define NGENERAL_REGISTERS    (NTOTAL_REGISTERS-NSPECIAL_REGISTERS)
#define NARGUMENT_REGISTERS   4
#define NSCRATCH_REGISTERS    4
#define NVOLATILE_REGISTERS   (NARGUMENT_REGISTERS+NSCRATCH_REGISTERS)
#define NSTATIC_REGISTERS     (NGENERAL_REGISTERS-NVOLATILE_REGISTERS)

/* Register creation macros */

#define MKSPECIAL(n)          regm_MkRegister(SPECIAL_REG, (n))
#define MKCC(n)               regm_MkRegister(CC_REG, (n))
#define MKINARG(n)            regm_MkRegister(INARG_REG, (n))
#define MKOUTARG(n)           regm_MkRegister(OUTARG_REG, (n))
#define MKINRET(n)            regm_MkRegister(INRET_REG, (n))
#define MKOUTRET(n)           regm_MkRegister(OUTRET_REG, (n))
#define MKSCRATCH(n)          regm_MkRegister(SCRATCH_REG, (n))
#define MKVOLATILE(n)         regm_MkRegister(VOLATILE_REG, (n))

/* Make a register from a stack offset */

#define MKREG(n)              MKSCRATCH((n) >> 2)
#define MKCCREG(n)            MKCC((n) >> 2)

/* Check register type */

#define ISSPECIAL(n)          regm_IsKind(SPECIAL_REG, (n))
#define ISCC(n)               regm_IsKind(CC_REG, (n))
#define ISARG(n)              regm_IsKind(ARG_REG, (n))
#define ISSCATCH(n)           regm_IsKind(SCRATCH_REG, (n))
#define ISVOLATILE(n)         regm_IsKind(VOLATILE_REG, (n))

/***************************************************************************
 * Global Types
 ***************************************************************************/

struct regm32_t
{
  uint32_t kind  :  3; /* Kind of register */
  uint32_t regno : 29; /* Register identifier */
};

union regm32_u
{
  struct regm32_t f;
  uint32_t        dw;
};

enum regm_formtag_e
{
  eFORM_1RCc, /*  <op> <roperand1>, <roperand2> */
  eFORM_1ICc, /*  <op> <roperand1>, <immediate> */
  eFORM_2R,   /*  <op> <rdest>, <roperand2> */
  eFORM_2I,   /*  <op> <rdest>, <immediate> */
  eFORM_3R,   /*  <op> <rsrc/rdest>, <roperand1>, <roperand2> */
  eFORM_3I,   /*  <op> <rsrc/rdest>, <roperand1>, <immediate> */
  eFORM_4I,   /*  <op> <pc-offset> */
  eFORM_4ICc, /*  <op> <pc-offset> */
};

struct regm_form1rcc_s
{
  uint32_t dwROperand1, dwROperand2, dwRCc;
};

struct regm_form1icc_s
{
  uint32_t dwROperand1, dwImmediate, dwRCc;
};

struct regm_form2r_s
{
  uint32_t dwRDest, dwROperand2;
};

struct regm_form2i_s
{
  uint32_t dwRDest, dwImmediate;
};

struct regm_form3r_s
{
  uint32_t dwRSrcDest, dwROperand1, dwROperand2;
};

struct regm_form3i_s
{
  uint32_t dwRSrcDest, dwROperand1, dwImmediate;
};

struct regm_form4i_s
{
  uint32_t dwRDest, dwOffset;
};

struct regm_form4icc_s
{
  uint32_t dwRDest, dwOffset, dwRCc;
};

struct regm_rcode2_s
{
  uint8_t eForm; /* See enum regm_formtag_e */
  uint8_t chOp;  /* Regm opcode */
  union
  {
    struct regm_form1rcc_s f1rcc;
    struct regm_form1icc_s f1icc;
    struct regm_form2r_s   f2r;
    struct regm_form2i_s   f2i;
    struct regm_form3r_s   f3r;
    struct regm_form3i_s   f3i;
    struct regm_form4i_s   f4i;
    struct regm_form4icc_s f4icc;
  } u;
};

typedef int (*regm_rcode2_node_t)(struct regm_rcode2_s*, void*);

/***************************************************************************
 * Global Variables
 ***************************************************************************/

extern struct regm_rcode2_s *g_pRCode2;
extern uint32_t g_nRCode2;

/***************************************************************************
 * Inline Functions
 ***************************************************************************/

static inline uint32_t regm_MkRegister(int wKind, int wRegNo)
{
  union regm32_u u;
  u.f.kind  = wKind;
  u.f.regno = wRegNo;
  return u.dw;
}

static inline int regm_IsKind(int wKind, uint32_t dwRegister)
{
  union regm32_u u;
  u.dw = dwRegister;
  return (u.f.kind == wKind);
}

static inline int regm_GetKind(uint32_t dwRegister)
{
  union regm32_u u;
  u.dw = dwRegister;
  return u.f.kind;
}

static inline void regm_SetKind(int wKind, uint32_t *pdwRegister)
{
  union regm32_u *pu = (union regm32_u *)pdwRegister;
  pu->f.kind = wKind;
}

static inline int regm_GetRegNo(uint32_t dwRegister)
{
  union regm32_u u;
  u.dw = dwRegister;
  return u.f.regno;
}

static inline void regm_SetRegNo(int wRegNo, uint32_t *pdwRegister)
{
  union regm32_u *pu = (union regm32_u *)pdwRegister;
  pu->f.regno = wRegNo;
}

/***************************************************************************
 * Global Function Prototypes
 ***************************C***********************************************/

/* Generate function prologue: Save return address, create stack frame
 * for local variables, and save static registers that till be used.
 */

extern void regm_GeneratePrologue(uint32_t dwFrameSize);

/* Restore static registers, release stack frame and return */

extern void regm_GenerateEpilogue(uint32_t dwFrameSize);

/* FORM 1R: <op> <roperand1>, <roperand2> */

extern void regm_GenerateForm1RCc(uint8_t chOp, uint32_t dwROperand1,
                                  uint32_t dwROperand2, uint32_t dwRCc);

/* FORM 1I: <op> <roperand1>, <immediate> */

extern void regm_GenerateForm1ICc(uint8_t chOp, uint32_t dwROperand1,
                                  uint32_t dwImmediate, uint32_t dwRCc);

/* FORM 2R: <op> <rdest>, <roperand2> */

extern void regm_GenerateForm2R(uint8_t chOp, uint32_t dwRDest,
                                uint32_t dwROperand2);

/* FORM 2I: <op> <rdest>, <immediate> */

extern void regm_GenerateForm2I(uint8_t chOp, uint32_t dwRDest,
                                uint32_t dwImmediate);

/* FORM 3R: <op> <rdest>, <roperand1>, <roperand2>
 *               <rsrc>,  <roperand1>, <roperand2>
 */

extern void regm_GenerateForm3R(uint8_t chOp, uint32_t dwRSrcDest,
                                uint32_t dwROperand1, uint32_t dwROperand2);

/* FORM 3I: <op> <rdest>, <roperand1>, <immediate>
 *                <rsrc>,  <roperand1>, <immediate>
 */

extern void regm_GenerateForm3I(uint8_t chOp, uint32_t dwRSrcDest,
                                uint32_t dwROperand1, uint32_t dwImmediate);

/* FORM 4I: <op> <pc-offset> */

extern void regm_GenerateForm4I(uint8_t chOp, uint32_t dwOffset);

extern void regm_GenerateForm4ICc(uint8_t chOp, uint32_t dwOffset,
                                  uint32_t dwRCc);

extern int  regm_ForEachRCode2(regm_rcode2_node_t pNode, void *arg);

#endif /* __REGM_REGISTERS2_H */
