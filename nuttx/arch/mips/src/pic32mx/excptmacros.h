/********************************************************************************************
 * arch/mips/src/pic32mx/excptmacros.h
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
 ********************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_EXCPTMACROS_H
#define __ARCH_MIPS_SRC_PIC32MX_EXCPTMACROS_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/pic32mx/cp0.h>

#ifdef __ASSEMBLY__

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Global Symbols
 ********************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.global		g_intstackbase
#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS
	.global		g_nestlevel
#endif
#endif

/********************************************************************************************
 * Assembly Language Macros
 ********************************************************************************************/

/********************************************************************************************
 * General Usage Example:
 *
 * my_exception:
 *		EXCPT_PROLOGUE t0			- Save registers on stack, enable nested interrupts
 *		move a0, sp					- Pass register save structure as the parameter 1
 *		USE_INTSTACK t0, t1, t2		- Switch to the interrupt stack
 *		jal	handler					- Handle the exception IN=old regs OUT=new regs
 *		di							- Disable interrupts
 *		RESTORE_STACK t0, t1		- Undo the operations of USE_STACK
 *		EXCPT_EPILOGUE v0			- Return to the context returned by handler()
 *
 ********************************************************************************************/
/********************************************************************************************
 * Name: EXCPT_PROLOGUE
 *
 * Description:
 *   Provides the "prologue" logic that should appear at the beginning of every exception
 *   handler.
 *
 * On Entry:
 *   sp - Points to the top of the stack
 *   tmp - Is a register the can be modified for scratch usage (after it has been saved)
 *   k0 and k1 - Since we are in an exception handler, these are available for use
 *
 * At completion:
 *   Register state is saved on the stack; All registers are available for usage except sp
 *   and k1:
 *
 *   - sp points the beginning of the register save area
 *   - k1 holds the value of the STATUS register
 *
 *   The following registers are modified: k0, k1, sp, a0
 *
 ********************************************************************************************/

	.macro	EXCPT_PROLOGUE, tmp
	.set	noat

	/* Get the SP from the previous shadow set */

#if 0
	rdpgpr	sp, sp
#endif

	/* "When entering the interrupt handler routine, the interrupt controller must first
	 *  save the current priority and exception PC counter from Interrupt  Priority (IPL)
	 *  bits (Status<15:10>) and the ErrorEPC register, respectively, on the stack. ..."
	 */

#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS // Does not work!
	mfc0	k0, MIPS32_CP0_CAUSE
	mfc0 	k1, MIPS32_CP0_EPC

	/* Isolate the pending interrupt level in bits 0-5 of k0 */

	srl		k0, k0, CP0_CAUSE_IP_SHIFT

	/* Create the register context stack frame large enough to hold the entire register save
	 * array.
	 */

	addiu	sp, sp, -XCPTCONTEXT_SIZE

	/* Save the EPC and STATUS in the register context array */

	sw		k1, REG_EPC(sp)
	mfc0	k1, MIPS32_CP0_STATUS
	sw		k1, REG_STATUS(sp)

	/* Then insert pending interrupt level as the current mask level in the CP0 status
	 * register. Also clear bits 1-4 in new value of the status register:
	 *
	 *   Bit 1: Exception Level
	 *   Bit 2: Error Level
	 *   Bit 3: (not used in PIC32MX)
	 *   Bit 4: Operating mode == USER
	 */

	ins 	k1, k0, CP0_STATUS_IPL_SHIFT, 6
	ins		k1, zero, 1, 4

	/* And Enable interrupts */

	mtc0	k1, MIPS32_CP0_STATUS
#else
	/* Get the EPC and STATUS register (Don't bother with the CAUSE register if we are
	 * not supporting nested interrupts)
	 */

	mfc0 	k0, MIPS32_CP0_EPC
	mfc0	k1, MIPS32_CP0_STATUS

	/* Create the register context stack frame large enough to hold the entire register
	 * save array.
	 */

	addiu	sp, sp, -XCPTCONTEXT_SIZE

	/* Save the EPC and STATUS in the register context array */

	sw		k0, REG_EPC(sp)
	sw		k1, REG_STATUS(sp)
#endif

	/* Save floating point registers */

	mfhi	k0
	sw		k0, REG_MFHI(sp)
	mflo	k0
	sw		k0, REG_MFLO(sp)

	/* Save general purpose registers */
	/* $1: at_reg, assembler temporary */

	sw		$1, REG_AT(sp)

	/* $2-$3 = v0-v1: Return value registers */

	sw		v0, REG_V0(sp)
	sw		v1, REG_V1(sp)

	/* $4-$7 = a0-a3: Argument registers */

	sw		a0, REG_A0(sp)
	sw		a1, REG_A1(sp)
	sw		a2, REG_A2(sp)
	sw		a3, REG_A3(sp)

	/* $8-$15 = t0-t7: Volatile registers */

	sw		t0, REG_T0(sp)
	sw		t1, REG_T1(sp)
	sw		t2, REG_T2(sp)
	sw		t3, REG_T3(sp)
	sw		t4, REG_T4(sp)
	sw		t5, REG_T5(sp)
	sw		t6, REG_T6(sp)
	sw		t7, REG_T7(sp)

	/* $16-$23 = s0-s7: Static registers */

	sw		s0, REG_S0(sp)
	sw		s1, REG_S1(sp)
	sw		s2, REG_S2(sp)
	sw		s3, REG_S3(sp)
	sw		s4, REG_S4(sp)
	sw		s5, REG_S5(sp)
	sw		s6, REG_S6(sp)
	sw		s7, REG_S7(sp)

	/* $24-25 = t8-t9: More Volatile registers */

	sw		t8, REG_T8(sp)
	sw		t9, REG_T9(sp)

	/* $26-$27 = ko-k1: Reserved for use in exeption handers.  These do not need to be
	 * saved.
	 *
	 * $28 = gp: Only needs to be saved under conditions where there are multiple, per-
	 * thread values for the GP.
	 */

#ifdef MIPS32_SAVE_GP
	sw		gp, REG_GP(sp)
#endif

	/* $30 = either s8 or fp:  Depends if a frame pointer is used or not */

	sw		s8, REG_S8(sp)

	/* $31 = ra: Return address */

	sw		ra, REG_RA(sp)

	/* $29 = sp:  The value of the stack pointer on return from the exception.  a0 is
	 * used as a temporary
	 */

	addiu	\tmp, sp, XCPTCONTEXT_SIZE
	sw		\tmp, REG_SP(sp)
	.endm

/********************************************************************************************
 * Name: EXCPT_EPILOGUE
 *
 * Description:
 *   Provides the "epilogue" logic that should appear at the end of every exception handler.
 *
 * On input:
 *   regs - points to the register save structure.  NOTE:  This *may not* be an address
 *          lying in a stack!  It might be an address in a TCB!
 *   Interrupts are disabled (via 'di')
 *
 * On completion:
 *   All registers restored
 *	 eret is executed to return from the exception
 *
 ********************************************************************************************/

	.macro	EXCPT_EPILOGUE, regs
	.set	noat

	/* Since interrupts are disabled via di can now use k0 and k1 again.  Use k1 as the
	 * pointer to the register save array.
	 */

	move	k1, \regs

	/* Restore the floating point register state */

	lw		k0, REG_MFLO(k1)
	mtlo	k0
	lw		k0, REG_MFHI(k1)
	mthi	k0

	/* Restore general purpose registers */
	/* $1: at_reg, assembler temporary */

	lw		$1, REG_AT(k1)

	/* $2-$3 = v0-v1: Return value registers */

	lw		v0, REG_V0(k1)
	lw		v1, REG_V1(k1)

	/* $4-$7 = a0-a3: Argument registers */

	lw		a0, REG_A0(k1)
	lw		a1, REG_A1(k1)
	lw		a2, REG_A2(k1)
	lw		a3, REG_A3(k1)

	/* $8-$15 = t0-t7: Volatile registers */

	lw		t0, REG_T0(k1)
	lw		t1, REG_T1(k1)
	lw		t2, REG_T2(k1)
	lw		t3, REG_T3(k1)
	lw		t4, REG_T4(k1)
	lw		t5, REG_T5(k1)
	lw		t6, REG_T6(k1)
	lw		t7, REG_T7(k1)

	/* $16-$23 = s0-s7: Static registers */

	lw		s0, REG_S0(k1)
	lw		s1, REG_S1(k1)
	lw		s2, REG_S2(k1)
	lw		s3, REG_S3(k1)
	lw		s4, REG_S4(k1)
	lw		s5, REG_S5(k1)
	lw		s6, REG_S6(k1)
	lw		s7, REG_S7(k1)

	/* $24-25 = t8-t9: More Volatile registers */

	lw		t8, REG_T8(k1)
	lw		t9, REG_T9(k1)

	/* $26-$27 = ko-k1: Reserved for use in exeption handers.  These do not need to be
	 * saved.
	 *
	 * $28 = gp: Only needs to be saved under conditions where there are multiple, per-
	 * thread values for the GP.
	 */

#ifdef MIPS32_SAVE_GP
	lw		gp, REG_GP(k1)
#endif

	/* $29 = sp: Stack pointer */

	lw		sp, REG_SP(k1)

	/* $30 = either s8 or fp:  Depends if a frame pointer is used or not */

	lw		s8, REG_S8(k1)

	/* $31 = ra: Return address */

	lw		ra, REG_RA(k1)

	/* Finally, restore CP status and the EPC */

	lw		k0, REG_STATUS(k1)
	lw		k1, REG_EPC(k1)
	mtc0	k0, MIPS32_CP0_STATUS
	ehb
	mtc0 	k1, MIPS32_CP0_EPC
	eret
	nop
	.endm

/********************************************************************************************
 * Name: USE_INTSTACK
 *
 * Description:
 *   Switch to the interrupt stack (if enabled in the configuration).
 *
 * On Entry:
 *   sp - Current value of the user stack pointer
 *   tmp1, tmp2, and tmp3 are registers that can be used temporarily.
 *   All interrupts should still be disabled.
 *
 * At completion:
 *   If the nesting level is 0, then (1) the user stack pointer is saved at the base of the
 *   interrupt stack and sp points to the interrupt stack.
 *   The values of tmp1, tmp2, tmp3, and sp have been altered
 *
 ********************************************************************************************/

	.macro	USE_INTSTACK, tmp1, tmp2, tmp3

#if CONFIG_ARCH_INTERRUPTSTACK > 3
#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS

	/* Check the nesting level.  If there are no nested interrupts, then we can
	 * claim the interrupt stack.
	 */

	la		\tmp1, g_nestlevel
	lw		\tmp2, (\tmp1)
	bne		1f
	nop
#endif

	/* Use the interrupt stack, pushing the user stack pointer onto the interrupt
	 * stack first.
	 */

	la		\tmp3, g_intstackbase
	lw		\tmp, (\tmp3)
	sw		sp, (\tmp3)
	move	sp, \tmp3

#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS
1:
	/* Increment the interrupt nesting level */

	addiu	\tmp2, \tmp2, 1
	sw		\tmp2, 0(\tmp1)
#endif
#endif
	.endm

/********************************************************************************************
 * Name: RESTORE_STACK
 *
 * Description:
 *   Restore the user stack.  Not really.. actually only decrements the nesting level.  We
 *   always get the new stack pointer for the register save array.
 *
 * On Entry:
 *   tmp1 and tmp2 are registers that can be used temporarily.
 *   All interrupts must be disabled.
 *
 * At completion:
 *   Current nesting level is decremented
 *   The values of tmp1 and  tmp2 have been altered
 *
 ********************************************************************************************/

	.macro		RESTORE_STACK, tmp1, tmp2

#if CONFIG_ARCH_INTERRUPTSTACK > 3
#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS

	/* Decrement the nesting level */

	la		\tmp1, g_nestlevel
	lw		\tmp2, (\tmp1)
	addiu	\tmp2, \tmp2, -1
	sw		\tmp2, 0(\tmp1)

#endif
#endif
	.endm

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_EXCPTMACROS_H */
