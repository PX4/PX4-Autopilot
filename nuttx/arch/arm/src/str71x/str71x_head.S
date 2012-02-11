/*****************************************************************************
 * arch/arm/src/str71x/str71x_head.S
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>	/* NuttX configuration settings */
#include <arch/board/board.h>	/* Board-specific settings */

#include "arm.h"		/* ARM-specific settings */
#include "chip.h"		/* Chip-specific settings */
#include "up_internal.h"
#include "up_arch.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/* This file holds the NuttX start logic that runs when the STR711
 * is reset.  This logic must be located at address 0x4000:0000 in
 * flash.  It will also be mapped to address zero when the STR711 is
 * reset.
 */

/*****************************************************************************
 * External references
 *****************************************************************************/

	.globl	str71x_prccuinit	/* Clock initialization */
	.globl	up_lowsetup		/* Early initialization of UART */
#ifdef USE_EARLYSERIALINIT
	.globl	up_earlyserialinit	/* Early initialization of serial driver */
#endif
#ifdef CONFIG_ARCH_LEDS
	.globl	up_ledinit		/* Boot LED setup */
#endif
#ifdef CONFIG_DEBUG
	.globl	up_lowputc		/* Low-level debug output */
#endif
	.globl	os_start		/* NuttX entry point */

/*****************************************************************************
 * Macros
 *****************************************************************************/

/*****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status. This macro will
 *   modify r0, r1, r2 and r14
 *
 *****************************************************************************/

	.macro	showprogress, code
#ifdef CONFIG_DEBUG
	mov	r0, #\code
	bl	up_lowputc
#endif
	.endm

/*****************************************************************************
 * Name: emiinit
 *
 * Description:
 *   Initialize external memory banks 0-3 as configured
 *
 *****************************************************************************/

	.macro	emiinit, base, value
#if defined(CONFIG_STR71X_BANK0) || defined(CONFIG_STR71X_BANK1) || \
    defined(CONFIG_STR71X_BANK2) || defined(CONFIG_STR71X_BANK3)

	/* In order to use the external memory, certain GPIO pins must be
	 * configured in the alternate function:
	 *
         * GPIO    ALT     Description
	 * P2.0-3  CS.0-3  External memory chip select for banks 0,1,3,4
	 * P2.4-7  A.20-23 External memory extended address bus (needed for
	 *                 address space > 1Mb)
	 */

#ifdef CONFIG_STR71X_BIGEXTMEM
#  define EXTMEM_GPIO_BITSET 0x000000ff	/* P2.0-7 */
#else
#  define EXTMEM_GPIO_BITSET 0x0000000f	/* P2.0-3 */
#endif

	ldr	\base, =STR71X_GPIO_BASE	; Configure P2.0 to P2.3/7 in AF_PP mode
	ldr	\value, [\base, #STR71X_GPIO_PC0_OFFSET]
	orr	\value, \value, #EXTMEM_GPIO_BITSET
	str	\value, [\base, #STR71X_GPIO_PC0_OFFSET]
	ldr	\value, [\base, #STR71X_GPIO_PC1_OFFSET]
	orr	\value, \value, #EXTMEM_GPIO_BITSET
	str	\value, [\base, #STR71X_GPIO_PC1_OFFSET]
	ldr	\value, [\base, #STR71X_GPIO_PC2_OFFSET]
	orr	\value, \value, #EXTMEM_GPIO_BITSET
	str	\value, [\base, #STR71X_GPIO_PC2_OFFSET]

	/* Enable bank 0 */

	ldr	\base, =STR71X_EMI_BASE

#ifdef CONFIG_STR71X_BANK0

	/* Get the bank 0 size */

#  if CONFIG_STR71X_BANK0_SIZE == 8
#    define EXTMEM_BANK0_SIZE STR71X_EMIBCON_BSIZE8
#  elif CONFIG_STR71X_BANK0_SIZE == 16
#    define EXTMEM_BANK0_SIZE STR71X_EMIBCON_BSIZE16
#  else
#    error "CONFIG_STR71X_BANK0_SIZE invalid"
#  endif

	/* Get the bank 0 waitstates */

#  if !defined(CONFIG_STR71X_BANK0_WAITSTATES) || \
      CONFIG_STR71X_BANK0_WAITSTATES < 0 || CONFIG_STR71X_BANK0_WAITSTATES > 15
#    error "CONFIG_STR71X_BANK0_WAITSTATES invalid"
#  else
     /* Bits 2-5: wait states */
#    define EXTMEM_BANK0_WAITSTATES (CONFIG_STR71X_BANK0_WAITSTATES << 2)
#  endif

	ldr	\value, =(STR71X_EMIBCON_ENABLE|EXTMEM_BANK0_WAITSTATES|EXTMEM_BANK0_SIZE)
#else
	mov	\value, #0
#endif
	str	\value, [\base, #STR71X_EMI_BCON0_OFFSET]

	/* Enable bank 1 */

#ifdef CONFIG_STR71X_BANK1

	/* Get the bank 1 size */

#  if CONFIG_STR71X_BANK1_SIZE == 8
#    define EXTMEM_BANK1_SIZE STR71X_EMIBCON_BSIZE8
#  elif CONFIG_STR71X_BANK1_SIZE == 16
#    define EXTMEM_BANK1_SIZE STR71X_EMIBCON_BSIZE16
#  else
#    error "CONFIG_STR71X_BANK1_SIZE invalid"
#  endif

	/* Get the bank 1 waitstates */

#  if !defined(CONFIG_STR71X_BANK1_WAITSTATES) || \
      CONFIG_STR71X_BANK1_WAITSTATES < 0 || CONFIG_STR71X_BANK1_WAITSTATES > 15
#    error "CONFIG_STR71X_BANK1_WAITSTATES invalid"
#  else
     /* Bits 2-5: wait states */
#    define EXTMEM_BANK1_WAITSTATES (CONFIG_STR71X_BANK1_WAITSTATES << 2)
#  endif

	ldr	\value, =(STR71X_EMIBCON_ENABLE|EXTMEM_BANK1_WAITSTATES|EXTMEM_BANK1_SIZE)
#else
	mov	\value, #0
#endif
	str	\value, [\base, #STR71X_EMI_BCON1_OFFSET]

	/* Enable bank 2 */

#ifdef CONFIG_STR71X_BANK2

	/* Get the bank 2 size */

#  if CONFIG_STR71X_BANK2_SIZE == 8
#    define EXTMEM_BANK2_SIZE STR71X_EMIBCON_BSIZE8
#  elif CONFIG_STR71X_BANK2_SIZE == 16
#    define EXTMEM_BANK2_SIZE STR71X_EMIBCON_BSIZE16
#  else
#    error "CONFIG_STR71X_BANK2_SIZE invalid"
#  endif

	/* Get the bank 2 waitstates */

#  if !defined(CONFIG_STR71X_BANK2_WAITSTATES) || \
      CONFIG_STR71X_BANK2_WAITSTATES < 2 || CONFIG_STR71X_BANK2_WAITSTATES > 15
#    error "CONFIG_STR71X_BANK2_WAITSTATES invalid"
#  else
     /* Bits 2-5: wait states */
#    define EXTMEM_BANK2_WAITSTATES (CONFIG_STR71X_BANK2_WAITSTATES << 2)
#  endif

	ldr	\value, =(STR71X_EMIBCON_ENABLE|EXTMEM_BANK2_WAITSTATES|EXTMEM_BANK2_SIZE)
#else
	mov	\value, #0
#endif
	str	\value, [\base, #STR71X_EMI_BCON2_OFFSET]

	/* Enable bank 3 */

#ifdef CONFIG_STR71X_BANK3

	/* Get the bank 3 size */

#  if CONFIG_STR71X_BANK3_SIZE == 8
#    define EXTMEM_BANK3_SIZE STR71X_EMIBCON_BSIZE8
#  elif CONFIG_STR71X_BANK3_SIZE == 16
#    define EXTMEM_BANK3_SIZE STR71X_EMIBCON_BSIZE16
#  else
#    error "CONFIG_STR71X_BANK3_SIZE invalid"
#  endif

	/* Get the bank 3 waitstates */

#  if !defined(CONFIG_STR71X_BANK3_WAITSTATES) || \
      CONFIG_STR71X_BANK3_WAITSTATES < 3 || CONFIG_STR71X_BANK3_WAITSTATES > 15
#    error "CONFIG_STR71X_BANK3_WAITSTATES invalid"
#  else
     /* Bits 2-5: wait states */
#    define EXTMEM_BANK3_WAITSTATES (CONFIG_STR71X_BANK3_WAITSTATES << 2)
#  endif

	ldr	\value, =(STR71X_EMIBCON_ENABLE|EXTMEM_BANK3_WAITSTATES|EXTMEM_BANK3_SIZE)
#else
	mov	\value, #0
#endif
	str	\value, [\base, #STR71X_EMI_BCON3_OFFSET]
#endif
	.endm

/*****************************************************************************
 * Name: eicinit
 *
 * Description:
 *   The EIC is initialized for use with NuttX.  This initialization does not
 *   take advantage of the high performance capabilities of the EIC.  Instead,
 *   The EIC is only used to to provide NuttX IRQ numbers.  Here is what is
 *   done:
 *
 *     IRQs and FIQs are disabled
 *     IVR set to zero
 *     All channels are disabled 
 *     Channels set to priority 0
 *     All SIR[n] registers contain the NuttX IRQ number in the MS 16-bits
 *
 *   At the time of IRQ processing, the IVR will contain the decoded IRQ
 *   number needed by NuttX.
 *
 *****************************************************************************/

	.macro	eicinit, eicbase, value, irqno, offset
	/* Disable and clear all interrupts */

	ldr	\eicbase, =STR71X_EIC_BASE

	/* Disable FIQ and IRQ */

	mov	\value, #0
	str	\value, [\eicbase, #STR71X_EIC_ICR_OFFSET]

	/* Disable all channel interrupts */

	str	\value, [\eicbase, #STR71X_EIC_IER_OFFSET]

	/* Clear all pending IRQs */

	ldr	\value, =0xffffffff
	str	\value, [\eicbase, #STR71X_EIC_IPR_OFFSET]

	/* Disable FIQ channels/clear pending FIQs */

	mov	\value, #0x0c
	str	\value, [\eicbase, #STR71X_EIC_FIR_OFFSET]

	/* Reset the current priority register */

	mov	\value, #0
	str     \value, [\eicbase, #STR71X_EIC_CIPR_OFFSET]

	/* Zero IVR 31:16 */

	str	\value, [\eicbase, #STR71X_EIC_IVR_OFFSET]

	/* Set up the loop to initialize each SIR register.  Start
	 * with IRQ number 0 and SIR0
	 */

	mov	\irqno, #0
	ldr	\offset, =STR71X_EIC_SIR_OFFSET

	/* Then loop for each EIC channel */
eicloop:
	/* Shift the IRQ number to bits 16-31 and save the shifted IRQ
	 * number as SIR[irqno].  This will appear as bits 0:15 in the
	 * IVR during IRQ processing.
	 *
	 * NOTE that the initial priority is set to zero -- the current
	 * interrupt priority (CIP) is always zero, so these interrupts
	 * are all disabled.
	 */

	mov	\value, \irqno, lsl #16
	str	\value, [\eicbase, \offset]

	/* Increment the offset to the next SIR register and inrement
	 * the IRQ number.
	 */

	add	\offset, \offset, #4
	add	\irqno, \irqno, #1

	/* Continue to loop until all of the SIR registers have been
	 * intialized.
	 */

	cmp	\irqno, #STR71X_EIC_NCHANNELS
	blt	eicloop	
	.endm

/*****************************************************************************
 * Name: periphinit
 *
 * Description"
 *   Disable all perfipherals (except EIC)
 *
 *****************************************************************************/

	.macro	periphinit, value, base1, base2
#ifndef CONFIG_STR71X_DISABLE_PERIPHINIT
	/* Set up APB1 and APB2 addresses */

	ldr	\base1, =STR71X_APB1_BASE
	ldr	\base2, =STR71X_APB2_BASE

	/* Disable all APB1 peripherals */

	ldr	\value, =STR71X_APB1_APB1ALL
	strh	\value, [\base1, #STR71X_APB_CKDIS_OFFSET]

	/* Disable all(or most) APB2 peripherals */
	ldr	\value, =(STR71X_APB2_APB2ALL & ~STR71X_APB2_EIC)
	strh	\value, [\base2, #STR71X_APB_CKDIS_OFFSET]

	/* Allow EMI and USB */

	ldr	\base1, =STR71X_RCCU_BASE
#ifdef CONFIG_STR71X_USB
	ldr	\value, =(STR71X_RCCUPER_EMI|STR71X_RCCUPER_USBKERNEL)
#else
	ldr	\value, =STR71X_RCCUPER_EMI
#endif
	strh	\value, [\base1, #STR71X_RCCU_PER_OFFSET]
#endif
	.endm

/*****************************************************************************
 * Name: remap
 *
 * Description:
 *   Remap memory at address 0x0000000 to either FLASH.  The system always
 *   boots at Bank0, sector 0 of FLASH.  Part of the initial setup will be to
 *   map the memory appropriately for the execution configuration.  Various
 *   options are possible, but only boot from FLASH is currently supported.
 *
 *****************************************************************************/

	.macro	remap, base, value
	/* Read the PCU BOOTCR register */

	ldr	\base, =STR71X_PCU_BASE
	ldrh	\value, [\base, #STR71X_PCU_BOOTCR_OFFSET]

	/* Mask out the old boot mode bits and set the boot mode to FLASH */

	bic	\value, \value, #STR71X_PCUBOOTCR_BOOTMASK
	orr	\value, \value, #STR71X_PCUBOOTCR_BMFLASH

	/* Save the modified BOOTCR register */

	strh	\value, [\base, #STR71X_PCU_BOOTCR_OFFSET]
	.endm

/*****************************************************************************
 * Text
 *****************************************************************************/

	.text

/*****************************************************************************
 * Name: _vector_table
 *
 * Description:
 *   Interrrupt vector table.  This must be located at the beginning
 *   of the memory space (at the beginning FLASH which will be mapped to
 *   address 0x00000000).  The first entry in the vector table is the reset
 *   vector and this is the code that will execute whn the processor is reset.
 *
 *****************************************************************************/

	.globl	_vector_table
	.type	_vector_table, %function
_vector_table:
	ldr	pc, .Lresethandler		/* 0x00: Reset */
	ldr	pc, .Lundefinedhandler		/* 0x04: Undefined instruction */
	ldr	pc, .Lswihandler		/* 0x08: Software interrupt */
	ldr	pc, .Lprefetchaborthandler	/* 0x0c: Prefetch abort */
	ldr	pc, .Ldataaborthandler		/* 0x10: Data abort */
	.long	0				/* 0x14: Reserved vector */
	ldr	pc, .Lirqhandler		/* 0x18: IRQ */
	ldr	pc, .Lfiqhandler		/* 0x1c: FIQ */

	.globl   __start
	.globl	up_vectorundefinsn
	.globl	up_vectorswi
	.globl	up_vectorprefetch
	.globl	up_vectordata
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
.Lirqhandler:
	.long	up_vectorirq
.Lfiqhandler:
	.long	up_vectorfiq
	.size	_vector_table, . - _vector_table

/*****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry point.  This is the first function to execute when
 *   the processor is reset.  It initializes hardware and then gives
 *   control to NuttX.
 *
 *****************************************************************************/

	.global __start
	.type	__start, #function

__start:
	/* On reset, an aliased copy of FLASH is mapped to address 0x00000000.
	 *  Continue execution in the 'real' FLASH address space rather than
	 *  the aliased copy
	 */

	ldr     pc, =__flashstart
__flashstart:
	.rept 9
	nop		 /* Wait for OSC stabilization*/
	.endr

	/* Setup the initial processor mode */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT )
	msr	cpsr, r0

	/* Initialize the external memory interface (EMI) */

	emiinit	r0, r1

	/* Initialize the enhanced interrupt controller (EIC) */

	eicinit	r0, r1, r2, r3

	/* Disable all peripherals except EIC */

	periphinit r0, r1, r2

	/* Map memory appropriately for configuration */

	remap	r0, r1

	/* Setup system stack (and get the BSS range) */

	adr	r0, LC0
	ldmia   r0, {r4, r5, sp}

	/* Clear system BSS section */

	mov	r0, #0
1:	cmp	r4, r5
	strcc	r0, [r4], #4
	bcc	1b

	/* Copy system .data sections from FLASH to new home in RAM. */

	adr	r3, LC2
	ldmia	r3, {r0, r1, r2}

2:	ldmia	r0!, {r3 - r10}
	stmia	r1!, {r3 - r10}
	cmp	r1, r2
	blt	2b

	/* Initialize clocking */

	bl	str71x_prccuinit

	/* Configure the uart so that we can get debug output as soon
	 * as possible.
	 */

        bl	up_lowsetup
	showprogress 'A'

	/* Perform early serial initialization */

	mov	fp, #0
#ifdef USE_EARLYSERIALINIT
	bl	up_earlyserialinit
#endif

	showprogress 'B'

	/* Call C++ constructors */

#ifdef CONFIG_CPLUSPLUS
	ldr 	r0, =__ctors_start__
	ldr 	r1, =__ctors_end__
ctor_loop:
	cmp 	r0, r1
	beq 	ctor_end
	ldr 	r2, [r0], #4
	stmfd 	sp!, {r0-r1}
	mov 	lr, pc
	mov 	pc, r2
	ldmfd 	sp!, {r0-r1}
	b 	ctor_loop
ctor_end:

	showprogress 'C'
#endif
	showprogress '\n'

	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	bl	up_ledinit
#endif

	/* Then jump to OS entry */

	b	os_start

	/* Call destructors  -- never get here */

#if 0 /* CONFIG_CPLUSPLUS */
	ldr	r0, =__dtors_start__
	ldr	r1, =__dtors_end__
dtor_loop:
	cmp	r0, r1
	beq	dtor_end
	ldr	r2, [r0], #4
	stmfd	sp!, {r0-r1}
	mov	lr, pc
	mov	pc, r2
	ldmfd	sp!, {r0-r1}
	b	dtor_loop
dtor_end:
#endif

	/* Variables:
	 * _sbss is the start of the BSS region (see ld.script)
	 * _ebss is the end of the BSS regsion (see ld.script)
	 * The idle task stack starts at the end of BSS and is
	 * of size CONFIG_IDLETHREAD_STACKSIZE.  The heap continues
	 * from there until the end of memory.  See g_heapbase
	 * below.
	 */

LC0:	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4

LC2:	.long	_eronly	/* Where .data defaults are stored in FLASH */
	.long	_sdata	/* Where .data needs to reside in SDRAM */
	.long	_edata
	.size	__start, .-__start

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to LCO
	 * above.
	 */

	.data
	.align	4
	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase

	.end

