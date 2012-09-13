/********************************************************************************************
 * arch/avr/src/avr/excptmacros.h
 *
 *	Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *	Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#ifndef __ARCH_AVR_SRC_AVR_EXCPTMACROS_H
#define __ARCH_AVR_SRC_AVR_EXCPTMACROS_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#ifdef __ASSEMBLY__

#include <arch/irq.h>

#include <avr/io.h>
#include <avr/sfr_defs.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Global Symbols
 ********************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.global		g_intstackbase
	.global		g_nestlevel
#endif

/********************************************************************************************
 * Assembly Language Macros
 ********************************************************************************************/

/********************************************************************************************
 * General Exception Handling Example:
 *
 *		HANDLER IRQ_X, my_exception
 *		...
 * my_exception:
 *		EXCPT_PROLOGUE				- Save registers on stack
 *		in	r22, _SFR_IO_ADDR(SPL)	- Pass register save structure as the parameter 2
 *		in	r23, _SFR_IO_ADDR(SPH)	- (Careful, push post-decrements)
 *		USE_INTSTACK rx, ry, rz		- Switch to the interrupt stack
 *		call handler				- Handle the exception IN=old regs OUT=new regs
 *		RESTORE_STACK rx, ry		- Undo the operations of USE_INTSTACK
 *		EXCPT_EPILOGUE				- Return to the context returned by handler()
 *		reti						- Return from interrupt
 *
 ********************************************************************************************/

/********************************************************************************************
 * Name: HANDLER
 *
 * Description:
 *	This macro provides the exception entry logic.  It is called with the PC already on the
 *	stack.  It simply saves one register on the  stack (r24) and passes the IRQ number to 
 *	common logic (see EXCPT_PROLOGUE).
 *
 * On Entry:
 *	sp - Points to the top of the stack.  The PC is already on the stack.
 *	Only the stack is available for storage
 *
 *	 PCL
 *	 PCH
 *	 --- <- SP
 *
 * At completion:
 *	Stack pointer is incremented by one, the saved r24 is on the stack, r24 now contains the
 *	IRQ number
 *
 *	 PCL
 *	 PCH
 *	 R0
 *	 --- <- SP
 *
 ********************************************************************************************/

	.macro	HANDLER, label, irqno, common
	.global	\label
\label:
	push	r24
	ldi		r24, \irqno
	rjmp	\common
	.endm

/********************************************************************************************
 * Name: EXCPT_PROLOGUE
 *
 * Description:
 *	Provides the common "prologue" logic that should appear at the beginning of the exception
 *	handler.
 *
 * On Entry:
 *	r24 - has already been pushed onto the stack and now holds the IRQ number
 *	sp - Points to the top of the stack
 *	Only the stack is available for storage
 *
 *	 PCL
 *	 PCH
 *	 R24
 *	 --- <- SP
 *
 * At completion:
 *	Register state is saved on the stack; All registers are available for usage except sp and
 *  r24 which still contains the IRQ number as set by the HANDLER macro.
 *
 ********************************************************************************************/

	.macro	EXCPT_PROLOGUE

	/* Save R25  */

	push	r25

	/* Save the status register on the stack */

	in		r25, _SFR_IO_ADDR(SREG)	/* Save the status register */
	cli								/* Disable interrupts (not necessary) */
	ori		r25, (1 << SREG_I)		/* Interrupts re-enabled on restore */
	push	r25

	/* Save R0 -- the scratch register and the zero register (which may not be zero).  R1
	 * must be zero for our purposes
	 */

	push	r0
	push	r1
	clr		r1

	/* Save r2-r17 - Call-saved, "static" registers */

	push	r2
	push	r3
	push	r4
	push	r5
	push	r6
	push	r7
	push	r8
	push	r9
	push	r10
	push	r11
	push	r12
	push	r13
	push	r14
	push	r15
	push	r16
	push	r17

	/* Save r18-r27 - Call-used, "volatile" registers (r24 was saved by
	 * HANDLER, r15 was saved above, and r26-r27 saved later, out of sequence)
	 */

	push	r18
	push	r19
	push	r20
	push	r21
	push	r22
	push	r23

	/* Save r28-r29 - Call-saved, "static" registers */

	push	r28
	push	r29

	/* Save r30-r31 - Call-used, "volatile" registers */

	push	r30
	push	r31

	/* Now save r26-r27 */

	push	r26
	push	r27

	/* Finally, save the stack pointer.  BUT we want the value of the stack pointer as
	 * it was just BEFORE the exception.  We'll have to add to get that value.
	 * The value to add is the size of the register save area including the bytes
	 * pushed by the interrupt handler (2), by the HANDLER macro (1), and the 32
	 * registers pushed above.  That is, the entire size of the register save structure
	 * MINUS two bytes for the stack pointer which has not yet been saved.
	 */

	in		r26, _SFR_IO_ADDR(SPL)
	in		r27, _SFR_IO_ADDR(SPH)
	adiw	r26, XCPTCONTEXT_REGS-2

	push	r26					/* SPL then SPH */
	push	r27
	.endm

/********************************************************************************************
 * Name: EXCPT_EPILOGUE
 *
 * Description:
 *	Provides the "epilogue" logic that should appear at the end of every exception handler.
 *
 * On input:
 *	sp points to the address of the register save area (just as left by EXCPT_PROLOGUE).
 *	All registers are available for use.
 *	Interrupts are disabled.
 *
 * On completion:
 *	All registers restored except the PC which remains on the stack so that a return
 *	via reti can be performed.
 *
 ********************************************************************************************/

	.macro	EXCPT_EPILOGUE, regs

	/* We don't need to restore the stack pointer */

	pop		r27					/* Discard SPH */
	pop		r26					/* Discard SPL */

	/* Restore r26-r27 */

	pop		r27
	pop		r26

	/* Restore r30-r31 - Call-used, "volatile" registers */

	pop		r31
	pop		r30

	/* Restore r28-r29 - Call-saved, "static" registers */

	pop		r29
	pop		r28

	/* Restore r18-r27 - Call-used, "volatile" registers (r26-r27 already
	 * restored, r24 and r25 will be restored later)
	 */

	pop		r23
	pop		r22
	pop		r21
	pop		r20
	pop		r19
	pop		r18

	/* Restore r2-r17 - Call-saved, "static" registers */

	pop		r17
	pop		r16
	pop		r15
	pop		r14
	pop		r13
	pop		r12
	pop		r11
	pop		r10
	pop		r9
	pop		r8
	pop		r7
	pop		r6
	pop		r5
	pop		r4
	pop		r3
	pop		r2

	/* Restore r0 - the scratch register and  r1- the "zero" register (that may not be zero) */

	pop		r1
	pop		r0

	/* Restore the status register (probably enabling interrupts) */

	pop		r24					/* Restore the status register */
	andi	r24, ~(1 << SREG_I)	/* but keeping interrupts disabled until the reti */
	out		_SFR_IO_ADDR(SREG), r24

	/* Finally, restore r24-r25 - the temporary and IRQ number registers */

	pop		r25
	pop		r24
	.endm

/********************************************************************************************
 * Name: USER_SAVE
 *
 * Description:
 *	Similar to EXPCT_PROLOGUE except that (1) this saves values into a register save
 *	data structure instead of on the stack, (2) the pointer is in r26;r27, and (3)
 *	Call-used registers are not saved.
 *
 * On Entry:
 *	X [r26:r27] - Points to the register save structure.
 *	Return address is already on the stack (due to CALL or RCALL instruction)/.
 *	Interrupts are disabled.
 *
 * At completion:
 *	Register state is saved on the stack; All registers are available for usage except sp.
 *
 ********************************************************************************************/

	.macro	USER_SAVE

	/* Pop the return address from the stack (PCH then PCL).  R18:19 are Call-used */

	pop		r19			/* r19=PCH */
	pop		r18			/* r18=PCL */

	/* Save the current stack pointer  as it would be after the return(SPH then SPL). */

	in		r25, _SFR_IO_ADDR(SPH)
	st		x+, r25
	in		r24, _SFR_IO_ADDR(SPL)
	st		x+, r24

	/* Skip over r26-r27 and r30-r31 - Call-used, "volatile" registers */

	adiw	r26, 4		/* Four registers: r26-r27 and r30-r31*/

	/* Save r28-r29 - Call-saved, "static" registers */

	st		x+, r29
	st		x+, r28

	/* Skip over r18-r27 - Call-used, "volatile" registers (r26-r27 have
	 * already been skipped, r24 and r25 are saved elsewhere)
	 */

	adiw	r26, 6		/* Seven registers: r18-23 */

	/* Save r2-r17 - Call-saved, "static" registers */

	st		x+, r17
	st		x+, r16
	st		x+, r15
	st		x+, r14
	st		x+, r13
	st		x+, r12
	st		x+, r11
	st		x+, r10
	st		x+, r9
	st		x+, r8
	st		x+, r7
	st		x+, r6
	st		x+, r5
	st		x+, r4
	st		x+, r3
	st		x+, r2

	/* Set r1  to zero - Function calls must return with r1=0 */

	clr		r1
	st		x+, r1

	/* Skip over r0 -- the scratch register */

	adiw	r26, 1

	/* Save the status register (probably not necessary since interrupts are disabled) */

	in		r0, _SFR_IO_ADDR(SREG)
	st		x+, r0

	/* Skip r24-r25 - These are scratch register and Call-used, "volatile" registers */

	adiw	r26, 2		/* Two registers: r24-r25 */

	/* Save the return address that we have saved in r18:19*/

	st		x+, r19		/* r19=PCH */
	st		x+, r18		/* r18=PCL */
	.endm

/********************************************************************************************
 * Name: TCB_RESTORE
 *
 * Description:
 *	Functionally equivalent to EXCPT_EPILOGUE excetp that register save area is not on the
 *	stack but is held in a data structure.
 *
 * On input:
 *	X [r26:r27] points to the data structure.
 *	All registers are available for use.
 *	Interrupts are disabled.
 *
 * On completion:
 *	All registers restored except for the PC with now resides at the top of the new stack
 *	so that ret can be used to switch to the new context. (ret, not reti, becaue ret
 * 	will preserve the restored interrupt state).
 *
 ********************************************************************************************/

	.macro	TCB_RESTORE, regs

	/* X [r36:27] points to the register save block.  Get an offset pointer to the PC in
	 * Y [r28:29]
	 */

	movw	r28, r26			/* Get a pointer to the PCH/PCL storage location */
	adiw	r28, REG_PCH

	/* Fetch and set the new stack pointer */

	ld		r25, x+				/* Fetch stack pointer (post-incrementing) */
	out		_SFR_IO_ADDR(SPH), r25		/* (SPH then SPL) */
	ld		r24, x+
	out		_SFR_IO_ADDR(SPL), r24
 
	/* Fetch the return address and save it at the bottom of the new stack so
	 * that we can iret to switch contexts.  The new stack is now:
	 *
	 *  PCL
	 *  PCH
	 *  --- <- SP
	 */

	ld		r25, y+				/* Load PCH (r25) then PCL (r24) */
	ld		r24, y+
	push	r24					/* Push PCH and PCL on the stack (PCL then PCH) */
	push	r25

	/* Then get value of X [r26:r27].  Save X on the new stack where we can
	 * recover it later.  The new stack is now:
	 *
	 *  PCL
	 *  PCH
	 *  R26
	 *  R27
	 *  --- <- SP
	 */

	ld		r25, x+				/* Fetch r26-r27 and save to the new stack */
	ld		r24, x+
	push	r24					/* r26 then r27 */
	push	r25

	/* Restore r30-r31 - Call-used, "volatile" registers */

	ld		r31, x+
	ld		r30, x+

	/* Restore r28-r29 - Call-saved, "static" registers */

	ld		r29, x+
	ld		r28, x+

	/* Restore r18-r27 - Call-used, "volatile" registers (r26-r27 have been
	 * moved and r24-r25 will be restore later)
	 */

	ld		r23, x+
	ld		r22, x+
	ld		r21, x+
	ld		r20, x+
	ld		r19, x+
	ld		r18, x+

	/* Restore r2-r17 - Call-saved, "static" registers */

	ld		r17, x+
	ld		r16, x+
	ld		r15, x+
	ld		r14, x+
	ld		r13, x+
	ld		r12, x+
	ld		r11, x+
	ld		r10, x+
	ld		r9, x+
	ld		r8, x+
	ld		r7, x+
	ld		r6, x+
	ld		r5, x+
	ld		r4, x+
	ld		r3, x+
	ld		r2, x+

	/* Restore r1 - zero register (which may not be zero) */

	ld		r1, x+

	/* Restore r0 - the scratch register */

	ld		r0, x+

	/* Restore the status register (probably re-enabling interrupts) */

	ld		r24, x+
	out		_SFR_IO_ADDR(SREG), r24

	/* Restore r24-r25 - The temporary and IRQ number registers */

	ld		r25, x+
	ld		r24, x+

	/* Finally, recover X [r26-r27] from the the new stack.  The PC remains on the new
	 * stack so that the user of this macro can return with ret (not reti, ret will
	 * preserve the restored interrupt state).
	 */

	pop		r27					/* R27 then R26 */
	pop		r26
	.endm

/********************************************************************************************
 * Name: USE_INTSTACK
 *
 * Description:
 *	Switch to the interrupt stack (if enabled in the configuration) and if the nesting level
 *	is equal to 0.  Increment the nesting level in any event.
 *
 * On Entry:
 *	sp - Current value of the user stack pointer
 *	tmp1, tmp2, and tmp3 are registers that can be used temporarily.
 *	All interrupts should still be disabled.
 *
 * At completion:
 *	If the nesting level is 0, then (1) the user stack pointer is saved at the base of the
 *	interrupt stack and sp points to the interrupt stack.
 *	The values of tmp1, tmp2, tmp3, and sp have been altered
 *
 ********************************************************************************************/

	.macro	USE_INTSTACK, tmp1, tmp2, tmp3
#if CONFIG_ARCH_INTERRUPTSTACK > 0
# warning "Not implemented"
#endif
	.endm

/********************************************************************************************
 * Name: RESTORE_STACK
 *
 * Description:
 *	Restore the user stack.  Not really.. actually only decrements the nesting level.  We
 *	always get the new stack pointer for the register save array.
 *
 * On Entry:
 *	tmp1 and tmp2 are registers that can be used temporarily.
 *	All interrupts must be disabled.
 *
 * At completion:
 *	Current nesting level is decremented
 *	The values of tmp1 and  tmp2 have been altered
 *
 ********************************************************************************************/

	.macro		RESTORE_STACK, tmp1, tmp2
#if CONFIG_ARCH_INTERRUPTSTACK > 0
#  warning "Not implemented"
#endif
	.endm

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVR_EXCPTMACROS_H */
