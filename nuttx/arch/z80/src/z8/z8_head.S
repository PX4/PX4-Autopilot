/**************************************************************************
 * arch/z80/src/z8/z8_head.S
 * ez8 Reset Entry Point
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS or IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER or CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, or CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS or SERVICES; LOSS
 * OF USE, DATA, or PROFITS; or BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, or TORT (INCLUDING NEGLIGENCE or OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <ez8.inc>
#include <configl.inc>
#include <vect.inc>

/**************************************************************************
 * Definitions
 **************************************************************************/

/* Assume the large model */

#if !defined(CONFIG_Z8_MODEL_LARGE) && !defined(CONFIG_Z8_MODEL_SMALL) 
#  define CONFIG_Z8_MODEL_LARGE 1
#  undef  CONFIG_Z8_MODEL_SMALL
#endif

#ifdef __Z8F1680
#  define CONFIG_Z8_COPYPRAM
#else
#  undef CONFIG_Z8_COPYPRAM
#endif


/**************************************************************************
 * External References / External Definitions
 **************************************************************************/

	xref	_z16f_clkinit:ROM
	xref	_z16f_lowinit:ROM
#ifdef CONFIG_ARCH_LEDS
	xref	_up_ledinit:ROM
#endif
	xref	_os_start:ROM
	xref	_up_doirq:ROM
	xref	_low_nearbss
	xref	_len_nearbss
	xref	_low_farbss
	xref	_len_farbss
	xref	_low_neardata
	xref	_len_neardata
	xref	_low_near_romdata
 	xref	_low_fardata
	xref	_len_fardata
	xref	_low_far_romdata
#ifdef CONFIG_Z8_COPYPRAM
	xref	_low_pramseg
	xref	_len_pramseg
	xref	_low_pram_romdata
#endif
	xref	_far_stacktop
	xdef	_z8_reset
	xdef	__intrp

/**************************************************************************
 * Code
 **************************************************************************/

 	segment	CODE
 
/**************************************************************************
 * Interrupt Vectors
 **************************************************************************/

 	/* Reset vector */

	vector	RESET = _z8_reset

/**************************************************************************
 * Name: _z16f_reset
 *
 * Description:
 *   Reset entry point
 *
 **************************************************************************/

	define startup, space=rom
	segment startup
_z8_reset:
	/* Set the register pointer for working registers e0-ef */

	srp     #%e0

	/* Initialize the stack pointer */

	ldx     spl, #low(_far_stacktop+1)
	ldx     sph, #high(_far_stacktop+1)

	/* Clear internal register ram area (c_nearbss) */

	ld	r0, #_low_nearbss
	ld	r2, #_len_nearbss
	cp	r2, #0
	jr	z, _z8_reset2

_z8_reset1:
	clr	@r0
	inc	r0
	djnz	r2, _z8_reset1

	/* Clear extended ram area (c_farbss) */

_z8_reset2:
	ld	r2, #high(_low_farbss)
	ld	r3, #low(_low_farbss)
	ld	r0, #high(_len_farbss)
	ld	r1, #low(_len_farbss)

	ld	r4, r0
	or	r4, r1
	jr	z, _z8_reset4
	clr	r4

_z8_reset3:
	ldx     @rr2,r4
	incw	rr2
	decw	rr0
	jr	nz, _z8_reset3

	/* Copy ROM data into internal RAM */

_z8_reset4:
#ifdef CONFIG_Z8_COPYNEARDATA
	ld	r0, #high(_low_near_romdata)
	ld	r1, #low(_low_near_romdata)
	ld	r3, #_len_neardata
	ld	r4, #_low_neardata
	cp	r3, #0
	jr	z, _z8_reset6

_z8_reset5:
	ldci	@r4, @rr0
	djnz    r3, _z8_reset5

_z8_reset6:
#endif
	/* Copy ROM data into extended RAM */

	ld	r0, #high(_low_fardata)
	ld	r1, #low(_low_fardata)
	ld	r2, #high(_low_far_romdata)
	ld	r3, #low(_low_far_romdata)
	ld	r4, #high(_len_fardata)
	ld	r5, #low(_len_fardata)

	ld	r6, r4
	or	r6, r5
	jr	z, _z8_reset8

_z8_reset7:
	ldc	r6, @rr2
	ldx     @rr0, r6
	incw	rr0
	incw	rr2
	decw	rr4
	jr	nz, _z8_reset7

	/* Copy ROM copy of code into Program RAM */

_z8_reset8:
#ifdef CONFIG_Z8_COPYPRAM
	ld	r0, #high(_low_pramseg)
	ld	r1, #low(_low_pramseg)
	ld	r2, #high(_low_pram_romdata)
	ld	r3, #low(_low_pram_romdata)
	ld	r4, #high(_len_pramseg)
	ld	r5, #low(_len_pramseg)

	ld	r6, r4
	or	r6, r5
	jr	z, _z8_reset10

_z8_reset9:
	ldc	r6, @rr2
	ldc     @rr0, r6
	incw	rr0
	incw	rr2
	decw	rr4
	jr	nz, _z8_reset9

_z8_reset10:
#endif

	/* Start NuttX */

	ldx     __intrp,#0   
	xor     r15, r15
	xor     r14, r14
	call	_os_start

	/* We should never get here */

_z8_reset_halt:
	jr	_z8_reset_halt

/**************************************************************************
 * Data
 **************************************************************************/

#ifdef CONFIG_Z8_MODEL_LARGE
	segment FAR_BSS
__intrp	ds	1
#else
	segment NEAR_BSS
__intrp	ds	1
#endif

	/* Set aside area for working registers */

	define workingreg, space=rdata, org=%e0
	segment workingreg
	ds  %10

	end	_z8_reset
