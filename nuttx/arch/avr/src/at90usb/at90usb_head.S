/****************************************************************************
 * arch/avr32/src/at90usb/at90usb_head.S
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <avr/io.h>
#include <avr/sfr_defs.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Stack is allocated just after uninitialized data and just before the heap */

#define STACKBASE (_enoinit+CONFIG_IDLETHREAD_STACKSIZE-1)

/* The RAMPZ register is only available for CPUs with more than 64Kb of FLASH.
 * Only the AT90USB646, 647, 1286, and 1287 are supported by this file.
 *
 * - Support for the EPLMX instructions is assumed if RAMPZ is present
 * - If RAMPZ is not present, support for LPMX is assumed
 */

#if defined(CONFIG_ARCH_CHIP_AT90USB1286) || defined(CONFIG_ARCH_CHIP_AT90USB1286)
#  define HAVE_RAMPZ 1
#endif

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.file	"up_nommuhead.S"
	.global	__start				/* Entry point */
	.global	_sbss				/* Start of .bss.  Defined by ld.script */
	.global	_ebss				/* End of .bss.  Defined by ld.script */
	.global	_sdata				/* Start of .data section in RAM */
	.global	_edata				/* End of .data section in RAM */
	.global	_eronly				/* Start of .data section in FLASH */
	.global	_enoinit			/* End of uninitialized data.  Defined by ld.script */
	.global	up_lowinit			/* Perform low level initialization */
	.global	os_start			/* NuttX entry point */

	.global	vectortab
	.global	at90usb_int0		/* External interrupt request 0 */
	.global	at90usb_int1		/* External interrupt request 1 */
	.global	at90usb_int2		/* External interrupt request 2 */
	.global	at90usb_int3		/* External interrupt request 3 */
	.global	at90usb_int4		/* External interrupt request 4 */
	.global	at90usb_int5		/* External interrupt request 5 */
	.global	at90usb_int6		/* External interrupt request 6 */
	.global	at90usb_int7		/* External interrupt request 7 */
	.global	at90usb_pcint0		/* Pin Change Interrupt Request 0 */
	.global	at90usb_usbgen		/* USB General USB General Interrupt request */
	.global	at90usb_usbep		/* USB Endpoint/Pipe USB ENdpoint/Pipe Interrupt request */
	.global	at90usb_wdt			/* Watchdog Time-out Interrupt */
	.global	at90usb_t2compa		/* TIMER2 COMPA Timer/Counter2 Compare Match A */
	.global	at90usb_t2compb		/* TIMER2 COMPB Timer/Counter2 Compare Match B */
	.global	at90usb_t2ovf		/* TIMER2 OVF Timer/Counter2 Overflow */
	.global	at90usb_t1capt		/* TIMER1 CAPT Timer/Counter1 Capture Event */
	.global	at90usb_t1compa		/* TIMER1 COMPA Timer/Counter1 Compare Match A */
	.global	at90usb_t1compb		/* TIMER1 COMPB Timer/Counter1 Compare Match B */
	.global	at90usb_t1compc		/* TIMER1 COMPC Timer/Counter1 Compare Match c */
	.global	at90usb_t1ovf		/* TIMER1 OVF Timer/Counter1 Overflow */
	.global	at90usb_t0compa		/* TIMER0 COMPA Timer/Counter0 Compare Match A */
	.global	at90usb_t0compb		/* TIMER0 COMPB Timer/Counter0 Compare Match B */
	.global	at90usb_t0ovf		/* TIMER0 OVF Timer/Counter0 Overflow */
	.global	at90usb_spi			/* STC SPI Serial Transfer Complete */
	.global	at90usb_u1rx		/* USART1 Rx Complete */
	.global	at90usb_u1dre		/* USART1 Data Register Empty */
	.global	at90usb_u1tx		/* USART1 Tx Complete */
	.global	at90usb_anacomp		/* ANALOG COMP Analog Comparator */
	.global	at90usb_adc			/* ADC Conversion Complete */
	.global	at90usb_ee			/* EEPROM Ready */
	.global	at90usb_t3capt		/* TIMER3 CAPT Timer/Counter3 Capture Event */
	.global	at90usb_t3compa		/* TIMER3 COMPA Timer/Counter3 Compare Match A */
	.global	at90usb_t3compb		/* TIMER3 COMPB Timer/Counter3 Compare Match B */
	.global	at90usb_t3compc		/* TIMER3 COMPC Timer/Counter3 Compare Match C */
	.global	at90usb_t3ovf		/* TIMER3 OVF Timer/Counter3 Overflow */
	.global	at90usb_twi			/* TWI Two-wire Serial Interface */
	.global	at90usb_spmrdy		/* Store Program Memory Ready */

/****************************************************************************
 * Macros
 ****************************************************************************/

	.macro	vector name
	jmp		\name
	.endm

/****************************************************************************
 * Vector Table
 ****************************************************************************/

/* The AT90USB has 38 interrupt vectors including vector 0, the reset
 * vector.
 */

	.section .vectors, "ax", @progbits
	.func	vectortab
vectortab:
	jmp		__start				/*  0: Vector 0 is the reset vector */
	vector	at90usb_int0		/*  1: External interrupt request 0 */
	vector	at90usb_int1		/*  2: External interrupt request 1 */
	vector	at90usb_int2		/*  3: External interrupt request 2 */
	vector	at90usb_int3		/*  4: External interrupt request 3 */
	vector	at90usb_int4		/*  5: External interrupt request 4 */
	vector	at90usb_int5		/*  6: External interrupt request 5 */
	vector	at90usb_int6		/*  7: External interrupt request 6 */
	vector	at90usb_int7		/*  8: External interrupt request 7 */
	vector	at90usb_pcint0		/*  9: PCINT0 Pin Change Interrupt Request 0 */
	vector	at90usb_usbgen		/* 10: USB General USB General Interrupt request */
	vector	at90usb_usbep		/* 11: USB Endpoint/Pipe USB ENdpoint/Pipe Interrupt request */
	vector	at90usb_wdt			/* 12: Watchdog Time-out Interrupt */
	vector	at90usb_t2compa		/* 13: TIMER2 COMPA Timer/Counter2 Compare Match A */
	vector	at90usb_t2compb		/* 14: TIMER2 COMPB Timer/Counter2 Compare Match B */
	vector	at90usb_t2ovf		/* 15: TIMER2 OVF Timer/Counter2 Overflow */
	vector	at90usb_t1capt		/* 16: TIMER1 CAPT Timer/Counter1 Capture Event */
	vector	at90usb_t1compa		/* 17: TIMER1 COMPA Timer/Counter1 Compare Match A */
	vector	at90usb_t1compb		/* 18: TIMER1 COMPB Timer/Counter1 Compare Match B */
	vector	at90usb_t1compc		/* 19: TIMER1 COMPC Timer/Counter1 Compare Match c */
	vector	at90usb_t1ovf		/* 20: TIMER1 OVF Timer/Counter1 Overflow */
	vector	at90usb_t0compa		/* 21: TIMER0 COMPA Timer/Counter0 Compare Match A */
	vector	at90usb_t0compb		/* 22: TIMER0 COMPB Timer/Counter0 Compare Match B */
	vector	at90usb_t0ovf		/* 23: TIMER0 OVF Timer/Counter0 Overflow */
	vector	at90usb_spi			/* 24: STC SPI Serial Transfer Complete */
	vector	at90usb_u1rx		/* 25: USART1 Rx Complete */
	vector	at90usb_u1dre		/* 26: USART1 Data Register Empty */
	vector	at90usb_u1tx		/* 27: USART1 Tx Complete */
	vector	at90usb_anacomp		/* 28: ANALOG COMP Analog Comparator */
	vector	at90usb_adc			/* 29: ADC Conversion Complete */
	vector	at90usb_ee			/* 30: EEPROM Ready */
	vector	at90usb_t3capt		/* 31: TIMER3 CAPT Timer/Counter3 Capture Event */
	vector	at90usb_t3compa		/* 32: TIMER3 COMPA Timer/Counter3 Compare Match A */
	vector	at90usb_t3compb		/* 33: TIMER3 COMPB Timer/Counter3 Compare Match B */
	vector	at90usb_t3compc		/* 34: TIMER3 COMPC Timer/Counter3 Compare Match C */
	vector	at90usb_t3ovf		/* 35: TIMER3 OVF Timer/Counter3 Overflow */
	vector	at90usb_twi			/* 36: TWI Two-wire Serial Interface */
	vector	at90usb_spmrdy		/* 37: Store Program Memory Ready */
	.endfunc

/****************************************************************************
 * Reset Entry Point
 ****************************************************************************/

	.section .init, "ax", @progbits
	.func	__start
__start:

	/* Clear the zero register, clear the status register and initialize the
	 * IDLE thread stack
	 */

	clr		r1
	out		_SFR_IO_ADDR(SREG), r1
	ldi		r28, lo8(STACKBASE)
	ldi		r29, hi8(STACKBASE)
	out		_SFR_IO_ADDR(SPH), r29
	out		_SFR_IO_ADDR(SPL), r28

	/* Copy initial global data values from FLASH into RAM */

    .global __do_copy_data;     /* Required to suppress dragging in logic from libgcc */
__do_copy_data:

#ifdef HAVE_RAMPZ
	ldi		r17, hi8(_edata)
	ldi		r26, lo8(_sdata)
	ldi		r27, hi8(_sdata)
	ldi		r30, lo8(_eronly)
	ldi		r31, hi8(_eronly)
	ldi		r16, hh8(_eronly)
	out		_SFR_IO_ADDR(RAMPZ), r16
	rjmp	.Lcopystart

.Lcopyloop:
	elpm	r0, Z+
	st		X+, r0

.Lcopystart:
	cpi		r26, lo8(_edata)
	cpc		r27, r17
	brne	.Lcopyloop
#else
	ldi		r17, hi8(_edata)
	ldi		r26, lo8(_sdata)
	ldi		r27, hi8(_sdata)
	ldi		r30, lo8(_eronly)
	ldi		r31, hi8(_eronly)
	rjmp	.Lcopystart

.Lcopyloop:
	lpm     r0, Z+
	st      X+, r0
 
.Lcopystart:
	cpi     r26, lo8(_edata)
	cpc     r27, r17
	brne    .Lcopyloop
#endif

	/* Clear uninitialized data */

    .global __do_clear_bss;     /* Required to suppress dragging in logic from libgcc */
__do_clear_bss:

	ldi		r17, hi8(_ebss)
	ldi		r26, lo8(_sbss)
	ldi		r27, hi8(_sbss)
	rjmp	.Lclearstart

.Lclearloop:
	st		X+, r1
 
.Lclearstart:
	cpi		r26, lo8(_ebss)
	cpc		r27, r17
	brne	.Lclearloop

	/* Perform any low-level initialization */

	call	up_lowinit

	/* Now start NuttX */

	call	os_start
	jmp		exit
	.endfunc

/****************************************************************************
 * Heap Base
 ****************************************************************************/

	/* This global variable is unsigned long g_heapbase and is exported from
	 * here only because of its coupling to other uses of _enoinit in this file
	 */

	.data
	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.word	_enoinit+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase
	.end
