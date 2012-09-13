/****************************************************************************
 * arch/arm/src/arm/up_vectortab.S
 *
 *   Copyright (C) 2007, 2009-2010 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Data
 ****************************************************************************/

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Name: _vector_start
 *
 * Description:
 *   Vector initialization block
 ****************************************************************************/

	.globl	_vector_start

/* These will be relocated to VECTOR_BASE. */

_vector_start:
	ldr	pc, .Lresethandler		/* 0x00: Reset */
	ldr	pc, .Lundefinedhandler		/* 0x04: Undefined instruction */
	ldr	pc, .Lswihandler		/* 0x08: Software interrupt */
	ldr	pc, .Lprefetchaborthandler	/* 0x0c: Prefetch abort */
	ldr	pc, .Ldataaborthandler		/* 0x10: Data abort */
	ldr	pc, .Laddrexcptnhandler		/* 0x14: Address exception */
	ldr	pc, .Lirqhandler		/* 0x18: IRQ */
	ldr	pc, .Lfiqhandler		/* 0x1c: FIQ */

	.globl   __start
	.globl	up_vectorundefinsn
	.globl	up_vectorswi
	.globl	up_vectorprefetch
	.globl	up_vectordata
	.globl	up_vectoraddrexcptn
	.globl	up_vectorirq
	.globl	up_vectorfiq

.Lresethandler:
	.long   __start
.Lundefinedhandler:
	.long	up_vectorundefinsn
.Lswihandler:
	.long	up_vectorswi
.Lprefetchaborthandler:
	.long	up_vectorprefetch
.Ldataaborthandler:
	.long	up_vectordata
.Laddrexcptnhandler:
	.long	up_vectoraddrexcptn
.Lirqhandler:
	.long	up_vectorirq
.Lfiqhandler:
	.long	up_vectorfiq

	.globl	_vector_end
_vector_end:
	.end
