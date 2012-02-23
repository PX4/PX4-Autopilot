/************************************************************************************
 * arch/arm/src/armv7-m/stm32_fpu.S
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/
/*
 * When this file is assembled, it will require the following GCC options:
 *
 * -mcpu=cortex-m3 -mfloat-abi=hard -mfpu=vfp -meabi=5
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#ifdef CONFIG_ARCH_FPU

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		up_savefpu
	.globl		up_restorefpu

	.syntax		unified
	.thumb
	.file		"up_fpu.S"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_restorefpu
 *
 * Description:
 *   Given the pointer to a register save area (in R0), save the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void up_savefpu(uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area in which to save the floating point
 *     registers
 *
 * Returned Value:
 *   None
 * 
 ************************************************************************************/

	.thumb_func
	.type	up_savefpu, function
up_savefpu:

	add		r1, r0, #(4*REG_S0)		/* R1=Address of FP register storage */

	/* Some older GNU assemblers don't support all the newer UAL mnemonics. */

#if 1 /* Use UAL mnemonics */
	/* Store all floating point registers */

	vstmia	r1!, {s0-s31}			/* Save the full FP context */

	/* Store the floating point control and status register */

	vmrs	r2, fpscr				/* Fetch the FPCSR */
	str		r2, [r1], #4			/* Save the floating point control and status register */
#else
	/* Store all floating point registers */

#if 1 /* Use store multiple */
	fstmias	r1!, {s0-s31}			/* Save the full FP context */
#else
	vmov	r2, r3, d0				/* r2, r3 = d0 */
	str		r2, [r1], #4			/* Save S0 and S1 values */
	str		r3, [r1], #4
	vmov	r2, r3, d1				/* r2, r3 = d1 */
	str		r2, [r1], #4			/* Save S2 and S3 values */
	str		r3, [r1], #4
	vmov	r2, r3, d2				/* r2, r3 = d2 */
	str		r2, [r1], #4			/* Save S4 and S5 values */
	str		r3, [r1], #4
	vmov	r2, r3, d3				/* r2, r3 = d3 */
	str		r2, [r1], #4			/* Save S6 and S7 values */
	str		r3, [r1], #4
	vmov	r2, r3, d4				/* r2, r3 = d4 */
	str		r2, [r1], #4			/* Save S8 and S9 values */
	str		r3, [r1], #4
	vmov	r2, r3, d5				/* r2, r3 = d5 */
	str		r2, [r1], #4			/* Save S10 and S11 values */
	str		r3, [r1], #4
	vmov	r2, r3, d6				/* r2, r3 = d6 */
	str		r2, [r1], #4			/* Save S12 and S13 values */
	str		r3, [r1], #4
	vmov	r2, r3, d7				/* r2, r3 = d7 */
	str		r2, [r1], #4			/* Save S14 and S15 values */
	str		r3, [r1], #4
	vmov	r2, r3, d8				/* r2, r3 = d8 */
	str		r2, [r1], #4			/* Save S16 and S17 values */
	str		r3, [r1], #4
	vmov	r2, r3, d9				/* r2, r3 = d9 */
	str		r2, [r1], #4			/* Save S18 and S19 values */
	str		r3, [r1], #4
	vmov	r2, r3, d10				/* r2, r3 = d10 */
	str		r2, [r1], #4			/* Save S20 and S21 values */
	str		r3, [r1], #4
	vmov	r2, r3, d11				/* r2, r3 = d11 */
	str		r2, [r1], #4			/* Save S22 and S23 values */
	str		r3, [r1], #4
	vmov	r2, r3, d12				/* r2, r3 = d12 */
	str		r2, [r1], #4			/* Save S24 and S25 values */
	str		r3, [r1], #4
	vmov	r2, r3, d13				/* r2, r3 = d13 */
	str		r2, [r1], #4			/* Save S26 and S27 values */
	str		r3, [r1], #4
	vmov	r2, r3, d14				/* r2, r3 = d14 */
	str		r2, [r1], #4			/* Save S28 and S29 values */
	str		r3, [r1], #4
	vmov	r2, r3, d15				/* r2, r3 = d15 */
	str		r2, [r1], #4			/* Save S30 and S31 values */
	str		r3, [r1], #4
#endif

	/* Store the floating point control and status register */

	fmrx	r2, fpscr				/* Fetch the FPCSR */
	str		r2, [r1], #4			/* Save the floating point control and status register */
#endif
	bx		lr

	.size	up_savefpu, .-up_savefpu

/************************************************************************************
 * Name: up_restorefpu
 *
 * Description:
 *   Given the pointer to a register save area (in R0), restore the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void up_restorefpu(const uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area containing the floating point
 *     registers
 *
 * Returned Value:
 *   None
 * 
 ************************************************************************************/

	.thumb_func
	.type	up_restorefpu, function
up_restorefpu:

	add		r1, r0, #(4*REG_S0)		/* R1=Address of FP register storage */

	/* Some older GNU assemblers don't support all the newer UAL mnemonics. */

#if 1 /* Use UAL mnemonics */
	/* Load all floating point registers */

	vldmia	r1!, {s0-s31}			/* Restore the full FP context */

	/* Load the floating point control and status register */

	ldr		r2, [r1], #4			/* Fetch the floating point control and status register */
	vmsr	fpscr, r2				/* Restore the FPCSR */
#else
	/* Load all floating point registers */

#if 1 /* Use load multiple */
	fldmias	r1!, {s0-s31}			/* Restore the full FP context */
#else
	ldr		r2, [r1], #4			/* Fetch S0 and S1 values */
	ldr		r3, [r1], #4
	vmov	d0, r2, r3				/* Save as d0 */
	ldr		r2, [r1], #4			/* Fetch S2 and S3 values */
	ldr		r3, [r1], #4
	vmov	d1, r2, r3				/* Save as d1 */
	ldr		r2, [r1], #4			/* Fetch S4 and S5 values */
	ldr		r3, [r1], #4
	vmov	d2, r2, r3				/* Save as d2 */
	ldr		r2, [r1], #4			/* Fetch S6 and S7 values */
	ldr		r3, [r1], #4
	vmov	d3, r2, r3				/* Save as d3 */
	ldr		r2, [r1], #4			/* Fetch S8 and S9 values */
	ldr		r3, [r1], #4
	vmov	d4, r2, r3				/* Save as d4 */
	ldr		r2, [r1], #4			/* Fetch S10 and S11 values */
	ldr		r3, [r1], #4
	vmov	d5, r2, r3				/* Save as d5 */
	ldr		r2, [r1], #4			/* Fetch S12 and S13 values */
	ldr		r3, [r1], #4
	vmov	d6, r2, r3				/* Save as d6 */
	ldr		r2, [r1], #4			/* Fetch S14 and S15 values */
	ldr		r3, [r1], #4
	vmov	d7, r2, r3				/* Save as d7 */
	ldr		r2, [r1], #4			/* Fetch S16 and S17 values */
	ldr		r3, [r1], #4
	vmov	d8, r2, r3				/* Save as d8 */
	ldr		r2, [r1], #4			/* Fetch S18 and S19 values */
	ldr		r3, [r1], #4
	vmov	d9, r2, r3				/* Save as d9 */
	ldr		r2, [r1], #4			/* Fetch S20 and S21 values */
	ldr		r3, [r1], #4
	vmov	d10, r2, r3				/* Save as d10 */
	ldr		r2, [r1], #4			/* Fetch S22 and S23 values */
	ldr		r3, [r1], #4
	vmov	d11, r2, r3				/* Save as d11 */
	ldr		r2, [r1], #4			/* Fetch S24 and S25 values */
	ldr		r3, [r1], #4
	vmov	d12, r2, r3				/* Save as d12 */
	ldr		r2, [r1], #4			/* Fetch S26 and S27 values */
	ldr		r3, [r1], #4
	vmov	d13, r2, r3				/* Save as d13 */
	ldr		r2, [r1], #4			/* Fetch S28 and S29 values */
	ldr		r3, [r1], #4
	vmov	d14, r2, r3				/* Save as d14 */
	ldr		r2, [r1], #4			/* Fetch S30 and S31 values */
	ldr		r3, [r1], #4
	vmov	d15, r2, r3				/* Save as d15 */
#endif

	/* Load the floating point control and status register */

	ldr		r2, [r1], #4			/* Fetch the floating point control and status register */
	fmxr	fpscr, r2				/* Restore the FPCSR */
#endif
	bx		lr

	.size	up_restorefpu, .-up_restorefpu
#endif /* CONFIG_ARCH_FPU */
	.end

