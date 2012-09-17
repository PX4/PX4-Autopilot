/****************************************************************************
 * arch/mips/src/mips32/up_syscall0.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/mips32/registers.h>
#include <arch/mips32/cp0.h>

/****************************************************************************
 * Global Symbols
 ****************************************************************************/

	.file	"up_syscall0.S"
	.global sys_call0
	.global sys_call1
	.global sys_call2
	.global sys_call3

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_syscall0, up_syscall1, up_syscall2, up_syscall3
 *
 * Description:
 *   up_syscall0 - System call SYS_ argument and no additional parameters.
 *   up_syscall1 - System call SYS_ argument and one additional parameter.
 *   up_syscall2 - System call SYS_ argument and two additional parameters.
 *   up_syscall3 - System call SYS_ argument and three additional parameters.
 *
 * Assumption:
 *   All interrupts are disabled except for the software interrupts.
 *
 ****************************************************************************/

	.text
	.set	noreorder
	.ent	sys_call0
sys_call0:	/* r4 holds the syscall number */
sys_call1:	/* r4 holds the syscall number, argument in r5 */
sys_call2:	/* r4 holds the syscall number, arguments in r5 and r6 */
sys_call3:	/* r4 holds the syscall number, arguments in r5, r6, and r7 */

	.set	push
	.set	noat

	/* Set Bit 8 to request the software interrupt */

	mfc0	t3, MIPS32_CP0_CAUSE	/* t3 = CP0 cause register */
	ori		t3, (1 << 8)			/* Bit 8: Request software interrupt 0 */
	.set	noreorder
	mtc0	t3, MIPS32_CP0_CAUSE	/* Trigger the software interrupt */

	/* The actual interrupt will not a occur for a few more cycles.  Let's
	 * put a few nop's here in hope that the SW interrupt occurs during
	 * the sequence of nops.
	 */

	nop
	nop
	nop
	nop

	/* Then return with the result of the software interrupt in v0 */

	j		ra
	nop
	.end	sys_call0
