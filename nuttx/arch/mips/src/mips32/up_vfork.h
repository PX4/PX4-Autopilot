/****************************************************************************
 * arch/mips/src/mips/up_vfork.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_SRC_MIPS32_VFORK_H
#define __ARCH_MIPS_SRC_MIPS32_VFORK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/mips32/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Register r30 may be a frame pointer in some ABIs.  Or may just be saved
 * register s8.  It makes a difference for vfork handling.
 */

#undef VFORK_HAVE_FP

/* r0      zero   Always has the value 0.
 * r1      at     Temporary generally used by assembler.
 * r2-r3   v0-v1  Used for expression evaluations and to hold the integer and
 *                pointer type function return values.
 * r4-r7   a0-a3  Used for passing arguments to functions; values are not
 *                preserved across function calls.
 * r8-r15  t0-t7  Temporary registers used for expression evaluation; values
 *                are not preserved across function calls.
 * r16-r23 s0-s7  Saved registers; values are preserved across function calls.
 * r24-r25 t8-t9  Temporary registers used for expression evaluations; values
 *                are not preserved across function calls. When calling
 *                position independent functions r25 must contain the address
 *                of the called function.
 * r26-r27 k0-k1  Used only by the operating system.
 * r28     gp     Global pointer and context pointer.
 * r29     sp     Stack pointer.
 * r30     s8     Saved register (like s0-s7).  If a frame pointer is used,
 *                then this is the frame pointer.
 * r31     ra     Return address.
 */
 
#define VFORK_S0_OFFSET   (0*4)   /* Saved register s0 */
#define VFORK_S1_OFFSET   (1*4)   /* Saved register s1 */
#define VFORK_S2_OFFSET   (2*4)   /* Saved register s2 */
#define VFORK_S3_OFFSET   (3*4)   /* Saved register s3 */
#define VFORK_S4_OFFSET   (4*4)   /* Saved register s4 */
#define VFORK_S5_OFFSET   (5*4)   /* Saved register s5 */
#define VFORK_S6_OFFSET   (6*4)   /* Saved register s6 */
#define VFORK_S7_OFFSET   (7*4)   /* Saved register s7 */

#ifdef CONFIG_MIPS32_FRAMEPOINTER
#  define VFORK_FP_OFFSET (8*4)   /* Frame pointer */
#else
#  define VFORK_S8_OFFSET (8*4)   /* Saved register s8 */
#endif

#define VFORK_SP_OFFSET   (9*4)   /* Stack pointer*/
#define VFORK_RA_OFFSET   (10*4)  /* Return address*/
#ifdef MIPS32_SAVE_GP
#  define VFORK_GP_OFFSET (11*4)   /* Global pointer */
#  define VFORK_SIZEOF    (12*4)
#else
#  define VFORK_SIZEOF    (11*4)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct vfork_s
{
  /* CPU registers */

  uint32_t s0;   /* Saved register s0 */
  uint32_t s1;   /* Saved register s1 */
  uint32_t s2;   /* Saved register s2 */
  uint32_t s3;   /* Saved register s3 */
  uint32_t s4;   /* Saved register s4 */
  uint32_t s5;   /* Saved register s5 */
  uint32_t s6;   /* Saved register s6 */
  uint32_t s7;   /* Saved register s7 */
#ifdef CONFIG_MIPS32_FRAMEPOINTER
  uint32_t fp;   /* Frame pointer */
#else
  uint32_t s8;   /* Saved register s8 */
#endif
  uint32_t sp;   /* Stack pointer*/
  uint32_t ra;   /* Return address*/
#ifdef MIPS32_SAVE_GP
  uint32_t gp;   /* Global pointer */
#endif

  /* Floating point registers (not yet) */
};
#endif

#endif /* __ARCH_MIPS_SRC_MIPS32_VFORK_H */
