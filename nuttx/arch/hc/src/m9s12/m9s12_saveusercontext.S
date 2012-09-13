/**************************************************************************
 * arch/arm/src/m9s12/m9s12_saveusercontext.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "up_internal.h"
#include "m9s12_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

#ifdef CONFIG_HCS12_NONBANKED
#  define CALL   jsr
#  define RETURN rts
#else
#  define CALL   call
#  define RETURN rtc
#endif

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
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *	 Create this state save strucure:
 *   Low Address  [PPAGE]
 *                [soft regisers]
 *                XYH
 *                XYL
 *                ZH
 *                ZL
 *                TMPH
 *                TMPL
 *                FRAMEH
 *                FRAMEL
 *                SP    <-- SP after interrupt
 *                CCR
 *                B
 *                A
 *                XH
 *                XL
 *                YH
 *                YL
 *                PCH
 *   High Address PCL
 *
 *   On entry:
 *    D=Pointer to save save structure
 *    TOS=return address
 *
 **************************************************************************/

	.text
	.globl	up_saveusercontext
	.type	up_saveusercontext, function
up_saveusercontext:
	/* Exchange D with X.  Now X points to the save structure. */

	xgdx

	/* Save the PPAGE register */

#ifndef CONFIG_HCS12_NONBANKED
	movb	HCS12_MMC_PPAGE, 1, x+
#endif

	/* Save the soft registers */

#if CONFIG_HCS12_MSOFTREGS > 2
#  error "Need to save more registers"
#endif
#if CONFIG_HCS12_MSOFTREGS > 1
	movw	_.d2, 2, x+
#endif
#if CONFIG_HCS12_MSOFTREGS > 0
	movw	_.d1, 2, x+
#endif

	/* It is not necessary to save the value of _.tmp, _.z, or _.xy */

	ldd		#0
	std		2, x+			/* Save _.xy = 0 */
	std		2, x+			/* Save _.z = 0 */
	std		2, x+			/* Save _.tmp = 0 */

	/* Save _.frame */

	movw	_.frame, 2, x+

	/* Save the value of the stack "before" this function was called */

	tfr		sp, d			/* D = current SP */
	addd	#(TOTALFRAME_SIZE-INTFRAME_SIZE)
	std		2, x+			/* Save the value of SP on entry */

	/* Save the CCR */

	tpa						/* A = CCR */
	staa	1, x+			/* Save CCR in the structure */

	/* D (A:B) is the return value.  Save 1 as the new return value as it
	 * will appear after a context switch back to the current thread.
	 */

	ldd		#1
	std		2, x+			/* Save D = 1 */

	/* X, Y do not need to be preserved.  Write zeros to these locations */

	ldd		#0
	std		2, x+			/* Save X = 0 */
	std		2, x+			/* Save Y = 0 */

	/* Fetch the 2-byte return address from the stack and save it at the
	 * end of the state save area
	 */

	movw	0, sp, 2, x+	/* Save PCH and PCL */

#if __INT__ == 32 /* 32-bit ABI */
	ldx		#0
#endif
	clra
	clrb
	RETURN
	.size	up_saveusercontext, . - up_saveusercontext
	.end
