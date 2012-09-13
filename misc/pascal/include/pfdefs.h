/***********************************************************************
 * pfdefs.h
 * Floating point operation codes
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

#ifndef __PFDEFS_H
#define __PFDEFS_H

/***********************************************************************
 * FLOATING POINT SUB-OPCODES
 ***********************************************************************/

/* This bit may be set in the opcode to indicate that the arguments
 * is(are) type integer and require(s) conversion. */

#define fpARG1  (0x40)
#define fpARG2  (0x80)
#define fpMASK  (0x3f)
#define fpSHIFT  6

/* The "INVALID" floating point operation */

#define fpINVLD (0x00)

/* Floating Pointer Conversions (On 16-bit stack argument:  FP or Integer) */

#define fpFLOAT (0x01)
#define fpTRUNC (0x02)
#define fpROUND (0x03)

/* Floating Point arithmetic instructions (Two FP 16-bit stack arguments) */

#define fpADD   (0x04)
#define fpSUB   (0x05)
#define fpMUL   (0x06)
#define fpDIV   (0x07)
#define fpMOD   (0x08)

/* Floating Point Comparisons (Two FP 16-bit stack arguments) */

#define fpEQU   (0x0a)
#define fpNEQ   (0x0b)
#define fpLT    (0x0c)
#define fpGTE   (0x0d)
#define fpGT    (0x0e)
#define fpLTE   (0x0f)

/* Floating Point arithmetic instructions (One FP 16-bit stack arguments) */

#define fpNEG   (0x10)
#define fpABS   (0x11)
#define fpSQR   (0x12)
#define fpSQRT  (0x13)
#define fpSIN   (0x14)
#define fpCOS   (0x15)
#define fpATAN  (0x16)
#define fpLN    (0x17)
#define fpEXP   (0x18)

#define MAX_FOP (0x19) /* Number of floating point operations */

#endif /* __PFDEFS_H */
