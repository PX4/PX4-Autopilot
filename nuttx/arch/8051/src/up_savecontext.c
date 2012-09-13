/**************************************************************************
 * up_savecontext.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_savestack
 *
 * Description:
 *   Save the entire interrupt stack contents in the provided context
 *   structure.
 *
 * Inputs:
 *   context - the context structure in which to save the stack info
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 **************************************************************************/

static void up_savestack(FAR struct xcptcontext *context, uint8_t tos)
{
  /* Copy the current stack frame from internal RAM to XRAM. */

  uint8_t nbytes     = tos - (STACK_BASE-1);
  NEAR uint8_t *src  = (NEAR uint8_t*)STACK_BASE;
  FAR  uint8_t *dest = context->stack;

  context->nbytes = nbytes;
  while (nbytes--)
    {
      *dest++ = *src++;
    }
}

/**************************************************************************
 * Name: up_saveregs
 *
 * Description:
 *   Save the interrupt registers into the TCB.
 *
 * Inputs:
 *   context - the context structure in which to save the register info
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 **************************************************************************/

static void up_saveregs(FAR struct xcptcontext *context, uint8_t tos)
{
  /* Copy the irq register save area into the TCB */

  FAR uint8_t *src  = g_irqregs;
  FAR uint8_t *dest = context->regs;
  uint8_t nbytes    = REGS_SIZE;

  while (nbytes--)
    {
      *dest++ = *src++;
    }
}

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_saveregisters
 *
 * Description:
 *   Save the current registers in the context save area.  This function
 *   is called from up_savecontext (below) and also from interrupt
 *   handling logic.
 *
 *   Note that this function does not save:
 *   a, dptr, ie - these are saved in the stack area
 *   sp - this can be inferred from g_irqtos or struct xcptontext.nbytes.
 *
 * Inputs:
 *   regs - the context register array in which to save the register info
 *
 * Return:
 *   None
 *
 **************************************************************************/

void up_saveregisters(FAR uint8_t *regs) _naked
{
 _asm
	mov	a, b
	movx	@dptr, a
	inc	dptr
	mov	a, r2
	movx	@dptr, a
	inc	dptr
	mov	a, r3
	movx	@dptr, a
	inc	dptr
	mov	a, r4
	movx	@dptr, a
	inc	dptr
	mov	a, r5
	movx	@dptr, a
	inc	dptr
	mov	a, r6
	movx	@dptr, a
	inc	dptr
	mov	a, r7
	movx	@dptr, a
	inc	dptr
	mov	a, r0
	movx	@dptr, a
	inc	dptr
	mov	a, r1
	movx	@dptr, a
	inc	dptr
	mov	a, psw
	movx	@dptr, a
	clr	psw
	inc	dptr
	mov	a, _bp
	movx	@dptr, a
	ret
  _endasm;
}

/**************************************************************************
 * Name: up_savecontext
 *
 * Description:
 *   Push the current execution context onto the stack, then save the
 *   entire stack contents in the provided context structure.
 *
 * Inputs:
 *   context - the context structure in which to save the stack info
 *
 * Return:
 *   0 = Normal state save return
 *   1 = This is the matching return from up_restorecontext()
 *
 **************************************************************************/

uint8_t up_savecontext(FAR struct xcptcontext *context) _naked
{
 _asm
	/* Create the stack frame that we want when it is time to restore
	 * this context.  The return address will be the return address
	 * of this function, the return value will be zero.
         *
         * ...
         * return address (2 bytes, already on the stack)
         * register a=0   (1 byte)
         * register ie    (1 byte)
         * register dptr  (2 bytes)
	 */

	clr	a
	push	acc	/* ACC = 0 */
	push	ie
	mov	a, #1
	push	acc	/* DPL = 1 */
	clr	a
	push	acc	/* DPH = 0 */

	/* Dump the stack contents before they are occupied into XRAM */

#ifdef CONFIG_SWITCH_FRAME_DUMP
	push	dpl
	push	dph
	lcall	_up_dumpstack
	pop	dph
	pop	dpl
#endif
	/* Disable interrupts while we create a snapshot of the stack
         * and registers.  At this point, we have 5 bytes on the stack
	 * to account for.
         */

	push	ie
	mov	ea, 0

	/* Save the registers in the context save area */

	push	dpl
	push	dph
	mov	a, #XCPT_REGS
	add	a, dpl
	mov	dpl, a
	clr	a
	addc	a, dph
	mov	dph, a
	lcall	_up_saveregisters
	pop	dph
	pop	dpl

#ifdef CONFIG_SWITCH_FRAME_DUMP
	/* Save the address of the context structure.  We will
	 * need this later to dump the saved frame.  Now we have
	 * 7 bytes on the stack to account for.
	 */

	push	dpl
	push	dph

	/* Push the top of frame stack pointer.  We need to
	 * decrement the current SP value by three to account
	 * for dptr+IE on the stack above the end of the frame.
	 */

	mov	a, sp
	subb	a, #3
#else
	/* Push the top of frame stack pointer.  We need to
	 * decrement the current stack pointer by one to account
	 * for IE that we saved on the stack.
	 */

	mov	a, sp
	dec	a
#endif
	push	acc

	/* Copy the current stack frame from internal RAM to XRAM. */

	lcall	_up_savestack
	pop	acc

	/* Dump the contents of the saved frame after it has been
	 * copied from  memory/registers.
	 */

#ifdef CONFIG_SWITCH_FRAME_DUMP
	pop	dph
	pop	dpl
	push	dpl
	push	dph
	lcall	_up_dumpframe
	pop	dph
	pop	dpl
	lcall	_up_dumpstack
#endif

	/* Restore the interrupt state */

	pop	ie

	/* Now that we have a snapshot of the desired stack frame saved,
	 * we can release the stack frame (all but the return address)
	 */

	mov	a, sp
	subb	a, #4
	mov	sp, a
	mov	dpl,#0
	ret
  _endasm;
}

/**************************************************************************
 * Name: up_saveirqcontext
 *
 * Description:
 *   The interrupt context was saved in g_irqtos and g_irqregs when the
 *   interrupt was taken.  If a context switch from the interrupted task
 *   will be made at the interrupt level, then these saved values must be
 *   copied into the TCB.
 *
 * Inputs:
 *   context - the structure in which to save the context info
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 **************************************************************************/

void up_saveirqcontext(FAR struct xcptcontext *context)
{
  /* Save the number of bytes in the stack */

   context->nbytes = g_irqtos - (STACK_BASE-1);

  /* Copy the current stack frame from internal RAM to XRAM. */

  up_savestack(context, g_irqtos);

  /* Copy the saved registers into the TCB */

  up_saveregisters(context->regs);
}
