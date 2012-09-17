/*****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_head.S
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "arm.h"
#include "chip.h"
#include "lpc214x_pll.h"
#include "lpc214x_apb.h"
#include "lpc214x_pinsel.h"
#include "up_internal.h"
#include "up_arch.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/* This file holds the NuttX start logic that runs when the LPC2148
 * is reset.  This logic must be located at address 0x0000:0000 in
 * flash but may be linked to run at different locations based on
 * the selected mode:
 *
 * default: Executes from 0x0000:0000.  In non-default modes, the
 *   MEMAP register is set override the settings of the CPU configuration
 *   pins.
 *
 *  CONFIG_EXTMEM_MODE: Code executes from external memory starting at
 *    address 0x8000:0000.
 *
 *  CONFIG_RAM_MODE: Code executes from on-chip RAM at address
 *     0x4000:0000.
 *
 * Starupt Code must be linked to run at the correct address
 * corresponding to the selected mode.
 */

#if defined(CONFIG_EXTMEM_MODE)
#  if CONFIG_CODE_BASE != LPC214X_EXTMEM_BASE
#    error "CONFIG_CODE_BASE must be 0x80000000 in EXTMEM mode"
#  endif
#elif defined(CONFIG_RAM_MODE)
#  if CONFIG_CODE_BASE != LPC214X_ONCHIP_RAM_BASE
#    error "CONFIG_CODE_BASE must be 0x40000000 in EXTMEM mode"
#  endif
#else
#  if CONFIG_CODE_BASE != LPC214X_FLASH_BASE
#    error "CONFIG_CODE_BASE must be 0x00000000 in default mode"
#  endif
#endif

/* Phase Locked Loop (PLL) initialization values
 *
 * Bit 0:4 MSEL: PLL Multiplier "M" Value
 *               CCLK = M * Fosc
 * Bit 5:6 PSEL: PLL Divider "P" Value 
 *               Fcco = CCLK * 2 * P
 *               156MHz <= Fcco <= 320MHz
 */

/* PLL0 provides CCLK and must always be configured */

#ifndef CONFIG_PLLCFG_VALUE /* board.h values can be supeceded config file */
#  ifdef LPC214X_PLL_M
#    define CONFIG_PLLCFG_MSEL (LPC214X_PLL_M-1)
#  else
#    warning "PLL_M not specified"
#    define CONFIG_PLLCFG_MSEL (5-1)
#  endif
#  ifdef LPC214X_PLL_P
#    if LPC214X_PLL_P == 1
#      define CONFIG_PLLCFG_PSEL LPC214X_PLL_CFG_PSEL1
#    elif LPC214X_PLL_P == 2
#      define CONFIG_PLLCFG_PSEL LPC214X_PLL_CFG_PSEL2
#    elif LPC214X_PLL_P == 4
#      define CONFIG_PLLCFG_PSEL LPC214X_PLL_CFG_PSEL4
#    elif LPC214X_PLL_P == 8
#      define CONFIG_PLLCFG_PSEL LPC214X_PLL_CFG_PSEL8
#    else
#      error "Unrecognized value for PLL_P"
#    endif
#  else
#    warning "PLL_P not specified"
#    define CONFIG_PLLCFG_PSEL LPC214X_PLL_CFG_PSEL2
#  endif
#  define CONFIG_PLLCFG_VALUE (CONFIG_PLLCFG_PSEL|CONFIG_PLLCFG_MSEL)
#endif

/* If  USB is enabled, PLL1 must be configured for 48MHz to provide USB clocking */

#ifdef CONFIG_USBDEV
#  ifndef CONFIG_USBPLLCFG_VALUE /* board.h values can be supeceded config file */
#    ifdef LPC214X_USBPLL_M
#      define LPC214X_USBPLLCFG_MSEL (LPC214X_USBPLL_M-1)
#    else
#      warning "PLL_M not specified"
#      define LPC214X_USBPLLCFG_MSEL 0x00000004
#    endif
#    ifdef LPC214X_USBPLL_P
#      if LPC214X_USBPLL_P == 1
#        define LPC214X_USBPLLCFG_PSEL 0x00000000
#      elif LPC214X_USBPLL_P == 2
#        define LPC214X_USBPLLCFG_PSEL 0x00000020
#      elif LPC214X_USBPLL_P == 4
#        define LPC214X_USBPLLCFG_PSEL 0x00000040
#      elif LPC214X_USBPLL_P == 8
#        define LPC214X_USBPLLCFG_PSEL 0x00000060
#      else
#        error "Unrecognized value for PLL_P"
#      endif
#    endif
#    define CONFIG_USBPLLCFG_VALUE (LPC214X_USBPLLCFG_PSEL|LPC214X_USBPLLCFG_MSEL)
#  endif
#endif

/* Memory Accelerator Module (MAM) initialization values
 *
 * MAM Control Register
 *   Bit 0:1 Mode
 *           0 = Disabled
 *           1 = Partially Enabled
 *           2 = Fully Enabled
 * MAM Timing Register
 *   Bit 0:2 Fetch Cycles
 *           0 = Reserved
 *           1 = 1
 *           2 = 2
 *           3 = 3
 *           4 = 4
 *           5 = 5
 *           6 = 6
 *           7 = 7
 */

#ifndef CONFIG_MAMCR_VALUE /* Can be selected from config file */
#  define CONFIG_MAMCR_VALUE  0x00000002
#endif

#ifndef CONFIG_MAMTIM_VALUE /* Can be selected from config file */
#  define CONFIG_MAMTIM_VALUE 0x00000004
#endif

/* APBDIV initialization values
 *
 * Bits 0:1 APB Peripheral Bus Clock Rate
 *          0 = APB Clock = CPU Clock / 4
 *          1 = APB Clock = CPU Clock
 *          2 = APB Clock = CPU Clock / 2
 */

#ifndef CONFIG_APBDIV_VALUE /* Can be selected from config file */
#  ifdef LPC214X_APB_DIV
#    if LPC214X_APB_DIV == 1
#       define CONFIG_APBDIV_VALUE LPC214X_APBDIV_DIV1
#    elif LPC214X_APB_DIV == 2
#       define CONFIG_APBDIV_VALUE LPC214X_APBDIV_DIV2
#    elif LPC214X_APB_DIV == 4
#       define CONFIG_APBDIV_VALUE LPC214X_APBDIV_DIV4
#    else
#       error "Unrecognized value for APBDIV"
#    endif
#  else
#    define CONFIG_APBDIV_VALUE LPC214X_APBDIV_DIV1
#  endif
#endif

/* External Memory Controller (EMC) initialization values
 *
 * Bank Configuration n (BCFG0..3)
 *    Bit 0:3   IDCY: Idle Cycles (0-15)
 *    Bit 5:9   WST1: Wait States 1 (0-31)
 *    Bit 11:15 WST2: Wait States 2 (0-31)
 *    Bit 10    RBLE: Read Byte Lane Enable
 *    Bit 26    WP: Write Protect
 *    Bit 27    BM: Burst ROM
 *    Bit 28:29 MW: Memory Width  (0=8-bit 1=16-bit 2=32-bit 3=Reserved)
 */

#ifndef CONFIG_BCFG0_VALUE /* Can be selected from config file */
#  define CONFIG_BCFG0_VALUE   0x0000fbef
#endif

#ifndef CONFIG_BCFG1_VALUE /* Can be selected from config file */
#  define CONFIG_BCFG1_VALUE   0x0000fbef
#endif

#ifndef CONFIG_BCFG2_VALUE /* Can be selected from config file */
#  define CONFIG_BCFG2_VALUE   0x0000fbef
#endif

#ifndef CONFIG_BCFG3_VALUE /* Can be selected from config file */
#  define CONFIG_BCFG3_VALUE   0x0000fbef
#endif

/* The following are used to configure the ADC/DAC */
#ifndef CONFIG_AD0CR_VALUE
#  define CONFIG_AD0CR_VALUE   0x00200402; /* Setup A/D: 10-bit AIN0 @ 3MHz */
#endif

/* GIO Pin Selection Register settings
 *
 * PINSEL0 configures GPIO 0.0 through 0.15
 */

#ifndef CONFIG_PINSEL0_VALUE /* Can be selected from config file */
#  define CONFIG_PINSEL0_VALUE 0x00000000 /* Reset value */
#endif

/* PINSEL1 configures GPIO 0.16 through 0.30 and GPO */

#ifndef CONFIG_PINSEL1_VALUE /* Can be selected from the config file */
#  ifdef CONFIG_ADC_SETUP
#    define CONFIG_PINSEL1_ADC    0x01000000 /* Enable DAC */
#  else
#    define CONFIG_PINSEL1_ADC    0x00000000 /* Reset value */
#  endif
#  ifdef CONFIG_USBDEV
#    define CONFIG_PINSEL1_USBDEV 0x80004000 /* Enable Vbus and Connect LED */
#  else
#    define CONFIG_PINSEL1_USBDEV 0x00000000 /* Reset value */
#  endif
#  define CONFIG_PINSEL1_VALUE (CONFIG_PINSEL1_ADC|CONFIG_PINSEL1_USBDEV)
#endif

/* External Memory Pins definitions
 * Bit 0:1  Reserved
 * Bit 2    GPIO/DEBUG
 * Bit 3    GPIO/TRACE
 * Bit 31:4 Reserved
 * CS0..3, OE, WE, BLS0..3,  D0..31, A2..23, JTAG Pins
 */

#ifndef CONFIG_PINSEL2_VALUE /* Can be selected from config file */
#  define CONFIG_PINSEL2_VALUE 0x0e6149e4
#endif

/*****************************************************************************
 * Macros
 *****************************************************************************/

/* Print a character on the UART to show boot status. This macro will
 * modify r0, r1, r2 and r14
 */

	.macro	showprogress, code
#ifdef CONFIG_DEBUG
	mov	r0, #\code
	bl	up_lowputc
#endif
	.endm

/* Configured the PINSEL2 register if EXTMEM mode is selected */

	.macro	configpinsel2, base, val
#ifdef CONFIG_EXTMEM_MODE
	ldr	\base, =LPC214X_PINSEL2
	ldr	\val, =CONFIG_PINSEL2_VALUE
	str	\val, [\base]
#endif
	.endm

/* Configure the external memory controller */

	.macro	configemc, base, val
#ifdef CONFIG_EMC_SETUP
	ldr	\base, =LPC214X_EMC_BASE

#ifdef CONFIG_BCFG0_SETUP
	ldr	\val, =CONFIG_BCFG0_VALUE
	str	\val, [\base, #LPC214X_BCFG0_OFFSET]
#endif

#ifdef CONFIG_BCFG1_SETUP
	ldr	\val, =CONFIG_BCFG1_VALUE
	str	\val, [\base, #LPC214X_BCFG1_OFFSET]
#endif

#ifdef CONFIG_BCFG2_SETUP
	ldr	\val, =CONFIG_BCFG2_VALUE
	str	\val, [\base, #LPC214X_BCFG2_OFFSET]
#endif

#ifdef CONFIG_BCFG3_SETUP
	ldr	\val, =CONFIG_BCFG3_VALUE
	str	\val, [\base, #LPC214X_BCFG3_OFFSET]
#endif
#endif
	.endm

/* Configure APBDIV */

	.macro	configapbdiv, base, val
#ifdef CONFIG_APBDIV_SETUP
	ldr	\base, =LPC214X_APBDIV
	ldr	\val, =CONFIG_APBDIV_VALUE
	strb	\val, [\base]
#endif
	.endm

/* Configure the PLL */

	.macro	configpll, base, val1, val2, val3
#ifdef CONFIG_PLL_SETUP
	ldr	\base, =LPC214X_PLL0_BASE
	mov	\val1, #LPC214X_PLL_FEED1
	mov	\val2, #LPC214X_PLL_FEED2

	/* Configure and Enable PLL */

	mov	\val3, #CONFIG_PLLCFG_VALUE
	str	\val3, [\base, #LPC214X_PLL_CFG_OFFSET]
	mov	\val3, #LPC214X_PLL_CON_PLLE
	str	\val3, [\base, #LPC214X_PLL_CON_OFFSET]
	str	\val1, [\base, #LPC214X_PLL_FEED_OFFSET]
	str	\val2, [\base, #LPC214X_PLL_FEED_OFFSET]

	/* Wait until PLL Locked */
1:
	ldr	\val3, [\base, #LPC214X_PLL_STAT_OFFSET]
	ands	\val3, \val3, #LPC214X_PLL_STAT_PLOCK
	beq	1b

	/* Switch to PLL Clock */

	mov	\val3, #(LPC214X_PLL_CON_PLLE | LPC214X_PLL_CON_PLLC)
	str	\val3, [\base, #LPC214X_PLL_CON_OFFSET]
	str	\val1, [\base, #LPC214X_PLL_FEED_OFFSET]
	str	\val2, [\base, #LPC214X_PLL_FEED_OFFSET]
#endif
	.endm

	.macro	configusbpll, base, val1, val2, val3
#ifdef CONFIG_USBDEV
	ldr	\base, =LPC214X_PLL1_BASE
	mov	\val1, #LPC214X_PLL_FEED1
	mov	\val2, #LPC214X_PLL_FEED2

	/* Configure and Enable PLL */

	mov	\val3, #CONFIG_USBPLLCFG_VALUE
	str	\val3, [\base, #LPC214X_PLL_CFG_OFFSET]
	mov	\val3, #LPC214X_PLL_CON_PLLE
	str	\val3, [\base, #LPC214X_PLL_CON_OFFSET]
	str	\val1, [\base, #LPC214X_PLL_FEED_OFFSET]
	str	\val2, [\base, #LPC214X_PLL_FEED_OFFSET]

	/* Wait until PLL Locked */
1:
	ldr	\val3, [\base, #LPC214X_PLL_STAT_OFFSET]
	ands	\val3, \val3, #LPC214X_PLL_STAT_PLOCK
	beq	1b

	/* Switch to PLL Clock */

	mov	\val3, #(LPC214X_PLL_CON_PLLE | LPC214X_PLL_CON_PLLC)
	str	\val3, [\base, #LPC214X_PLL_CON_OFFSET]
	str	\val1, [\base, #LPC214X_PLL_FEED_OFFSET]
	str	\val2, [\base, #LPC214X_PLL_FEED_OFFSET]
#endif
	.endm


/* Configure the Memory Accelerator Module (MAM) */

	.macro	configmam, base, val
#ifdef CONFIG_MAM_SETUP
	ldr	\base, =LPC214X_MAM_BASE
	mov	\val, #CONFIG_MAMTIM_VALUE
	str	\val, [\base, #LPC214x_MAM_TIM_OFFSET] 
	mov	\val, #CONFIG_MAMCR_VALUE
	str	\val, [\base, #LPC214X_MAM_CR_OFFSET] 
#endif
	.endm

/* Setup MEMMAP for the selected mode of operation */

	.macro	configmemmap, base, val
	ldr	\base, =LPC214X_MEMMAP
#if defined(CONFIG_EXTMEM_MODE)
	mov	\val, #3
#elif defined(CONFIG_RAM_MODE)
	mov	\val, #2
#else /* Setting the default should not be necessary */
	mov	\val, #1
#endif
	str	\val, [\base]
	.endm

	.macro	configdac, base, tmp
#ifdef CONFIG_ADC_SETUP
	ldr	\base, =LPC214X_AD0_BASE
	ldr	\tmp, =CONFIG_AD0CR_VALUE
	str	\tmp, [\base, #LPC214X_AD_ADCR_OFFSET]

	ldr	\base,=LPC214X_PINSEL1
	ldr	\tmp, =CONFIG_PINSEL1_VALUE
	str	\tmp, [\base]
#endif
	.endm

	.macro configfastport, base, tmp
#ifdef CONFIG_LPC214x_FIO
	ldr	\base, =LPC214X_SCS
	mov	\tmp, #0x03
	str	\tmp,[\base]
#endif
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
 *   of the memory space (at CONFIG_CODE_BASE).  The first entry in
 *   the vector table is the reset vector and this is the code that
 *   will execute whn the processor is reset.
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
	.long	0				/* 0x14: Vector checksum */
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
	/* Setup the initial processor mode */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT )
	msr	cpsr, r0

	/* Set up external memory mode (if so selected) */

	configpinsel2	r0, r1

	/* Setup the External Memory Controllor (EMC) as configured */

	configemc	r0, r1

	/* Configure APBDIV */

	configapbdiv	r0, r1

	/* Configure the PLL(s) */

	configpll	r0, r1, r2, r3
	configusbpll	r0, r1, r2, r3

	/* Configure the Memory Accelerator Module (MAM) */

	configmam	r0, r1

	/* Setup MEMMAP for the selected mode of operation */

	configmemmap	r0, r1

	/* Configure the DAC and ADC */
	
	configdac	r0, r1

	/* Configure Fast GPIO Port */

	configfastport  r0, r1

	/* Configure the uart so that we can get debug output as soon
	 * as possible.  Modifies r0, r1, r2, and r14.
	 */

	bl	up_lowsetup
	showprogress 'A'

	/* Setup system stack (and get the BSS range) */

	adr	r0, LC0
	ldmia   r0, {r4, r5, sp}

	/* Clear system BSS section */

	mov	r0, #0
1:	cmp	r4, r5
	strcc	r0, [r4], #4
	bcc	1b

	showprogress 'B'

	/* Copy system .data sections to new home in RAM. */

	adr	r3, LC2
	ldmia	r3, {r0, r1, r2}

1:	ldmia	r0!, {r3 - r10}
	stmia	r1!, {r3 - r10}
	cmp	r1, r2
	blt	1b

	/* Perform early serial initialization */

	mov	fp, #0
#ifdef USE_EARLYSERIALINIT
	bl	up_earlyserialinit
#endif

	showprogress 'C'
	showprogress '\n'

	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	bl	up_ledinit
#endif

	/* Then jump to OS entry */

	b	os_start

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

