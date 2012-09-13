/****************************************************************************
 * arch/mips/include/mips32/registers.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_INCLUDE_MIPS32_REGISTERS_H
#define __ARCH_MIPS_INCLUDE_MIPS32_REGISTERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Standard synonmyms for MIPS registers */

#ifdef __ASSEMBLY__
/* Zero register:  Always returns 0 */

#define zero               $0

/* Assembler temporary register:  Reserved for use by the assembler */

#define at_reg             $1

/* Return value registers:  Value returned by function */

#define v0                 $2
#define v1                 $3

/* Argument registers:  First four parameters to a function */

#define a0                 $4
#define a1                 $5
#define a2                 $6
#define a3                 $7

/* Volatile registers: Registers that can be used without saving */

#define t0                 $8
#define t1                 $9
#define t2                 $10
#define t3                 $11
#define t4                 $12
#define t5                 $13
#define t6                 $14
#define t7                 $15
#define t8                 $24
#define t9                 $25

/* Static registers:  Registers that must be saved and restored if used */
 
#define s0                 $16
#define s1                 $17
#define s2                 $18
#define s3                 $19
#define s4                 $20
#define s5                 $21
#define s6                 $22
#define s7                 $23

/* Reserved for use by interrupt/trap handling logic */

#define k0                 $26
#define k1                 $27

/* Global pointer register */

#define gp                 $28

/* Stack pointer register:  Stack pointer */

#define sp                 $29

/* Register 30 may be either an additional static register or a frame pointer */

#define s8                 $30
#define fp                 $30

/* Return address register:  Contains the function return address */

#define ra                 $31
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_MIPS_INCLUDE_MIPS32_REGISTERS_H */

