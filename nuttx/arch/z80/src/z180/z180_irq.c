/****************************************************************************
 * arch/z80/src/z180/z180_irq.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/io.h>

#include "switch.h"
#include "z180_iomap.h"
#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

volatile chipreg_t *current_regs;

/* This holds the value of the MMU's CBR register.  This value is set to the
 * interrupted tasks's CBR on interrupt entry, changed to the new task's CBR if
 * an interrrupt level context switch occurs, and restored on interrupt exit.  In
 * this way, the CBR is always correct on interrupt exit.
 */

uint8_t current_cbr;

/* The interrupt vector table is exported by z180_vectors.asm or
 * z180_romvectors.asm with the name up_vectors:
 */

extern uintptr_t up_vectors[16];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_seti
 *
 * Description:
 *   Input byte from port p
 *
 ****************************************************************************/

static void z180_seti(uint8_t value) __naked
{
  __asm
	ld      a, 4(ix)	;value
	ld      l, a
  __endasm;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irqsave
 *
 * Description:
 *   Disable all interrupts; return previous interrupt state
 *
 ****************************************************************************/

irqstate_t irqsave(void) __naked
{
  __asm
	ld	a, i		; AF Parity bit holds interrupt state
	di			; Interrupts are disabled
	push	af		; Return AF in HL
	pop	hl		;
	ret			;
  __endasm;
}

/****************************************************************************
 * Name: irqrestore
 *
 * Description:
 *   Restore previous interrupt state
 *
 ****************************************************************************/

void irqrestore(irqstate_t flags) __naked
{
  __asm
	di			; Assume disabled
	pop	hl		; HL = return address
	pop	af		; AF Parity bit holds interrupt state
	jp	po, statedisable
	ei
statedisable:
	push	af		; Restore stack
	push	hl		;
	ret			; and return
  __endasm;
}

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Initialize and enable interrupts
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint16_t vectaddr = (uint16_t)up_vectors;
  uint8_t regval;

  /* Initialize the I and IL registers so that the interrupt vector table
   * is used.
   */

  regval = (uint8_t)(vectaddr >> 8);
  z180_seti(regval);

  regval = (uint8_t)(vectaddr & IL_MASK);
  outp(Z180_INT_IL, regval);

  /* Disable external interrupts */

  outp(Z180_INT_ITC, 0);

  /* And finally, enable interrupts (including the timer) */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqrestore(Z180_C_FLAG);
#endif
}
