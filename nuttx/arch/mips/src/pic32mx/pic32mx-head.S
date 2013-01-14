/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-head.S
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
#include <arch/pic32mx/cp0.h>

#include "pic32mx-config.h"
#include "pic32mx-bmx.h"
#include "excptmacros.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_PIC32MX_MVEC0
#  error "Multi-vectors not supported"
#  ifndef CONFIG_PIC32MX_EBASE
#    error "EBASE address provided"  /* Should come from the linker script */
#  endif
#  ifndef CONFIG_PIC32MX_VECTORSPACING
#    error "No vector spacing provided"
#  endif
#endif

/* Linker memory organization ***********************************************/
/* Data memory is organized as follows:
 *
 * 1) Possible space reserved for debug data
 * 2) Ram functions: (.data):
 *    Start:   _sramfunc
 *    End(+1): _eramfunc
 * 3) Initialized data (.data):
 *    Start:   _sdata
 *    End(+1): _edata
 * 4) Uninitialized data (.bss):
 *    Start:   _sbss
 *    End(+1): _ebss
 *
 * The following are placed outside of the "normal" memory segments -- mostly
 * so that they do not have to be cleared on power up.
 *
 * 5) Idle thread stack:
 *    Start:   _ebss
 *    End(+1): _ebss+CONFIG_IDLETHREAD_STACKSIZE
 * 6) Optional interrupt stack
 *    Start:   _ebss+CONFIG_IDLETHREAD_STACKSIZE
 *    End(+1): _ebss+CONFIG_IDLETHREAD_STACKSIZE+(CONFIG_ARCH_INTERRUPTSTACK & ~3)
 * 6a) Heap (without interupt stack)
 *    Start:   _ebss+CONFIG_IDLETHREAD_STACKSIZE
 *    End(+1): to the end of memory
 * 6b) Heap (with interupt stack)
 *    Start:   _ebss+CONFIG_IDLETHREAD_STACKSIZE+(CONFIG_ARCH_INTERRUPTSTACK & ~3)
 *    End(+1): to the end of memory
 */

#define PIC32MX_STACK_BASE      _ebss
#define PIC32MX_STACK_TOP       _ebss+CONFIG_IDLETHREAD_STACKSIZE-4

#if CONFIG_ARCH_INTERRUPTSTACK > 3
#  define PIC32MX_INTSTACK_BASE PIC32MX_STACK_TOP
#  define PIC32MX_INTSTACK_SIZE (CONFIG_ARCH_INTERRUPTSTACK & ~3)
#  define PIC32MX_INTSTACK_TOP  PIC32MX_STACK_TOP+PIC32MX_INTSTACK_SIZE
#  define PIC32MX_HEAP_BASE     PIC32MX_INTSTACK_TOP
#else
#  define PIC32MX_HEAP_BASE     PIC32MX_STACK_TOP
#endif

/****************************************************************************
 * Global Symbols
 ****************************************************************************/

	.file	"pic32mx-head.S"

	/* Exported symbols */

	.globl	__reset
	.global __start
	.global halt
	.global	devconfig
#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.global g_intstackbase
#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS
	.global	g_nestlevel
#endif
#endif
	.global	g_heapbase

	/* Imported symbols */

	.global os_start
	.global	pic32mx_exception
	.global pic32mx_decodeirq
#ifdef CONFIG_PIC32MX_NMIHANDLER
	.global	pic32mx_donmi
#endif

/****************************************************************************
 * Name: __reset
 *
 * Description:
 *   Reset entry point.  This function is positioned at the beginning of
 *   the boot FLASH by the linker in KSEG1.  Simply jumps to the __start
 *   logic in KSEG0 (also in the boot FLASH).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .reset, "ax", @progbits
	.set	noreorder
	.ent	__reset
__reset:
	la		k0, __start	/* Just jump to the startup initialization code */
	jr		k0
	nop
	.end	__reset

/****************************************************************************
 * Name: _gen_exception
 *
 * Description:
 *   General Exception Vector Handler.  Jumps to _exception_handler. This
 *   vector will be positioned at 0xbfc00180 by the linker script. NOTE:
 *   If we set the BEV bit in the status register so all interrupt vectors
 *   should go through the _bev_exception.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .gen_excpt,"ax",@progbits
	.set	noreorder
	.ent	_gen_exception
_gen_exception:
	la		k0, _exception_handler
	jr		k0
	nop
	.end _gen_exception

/****************************************************************************
 * Name: _ebase_exception
 *
 * Description:
 *   Interrupt Exception Vector Handler.  Jumps to _int_handler.  This
 *   vector will be positioned at 0xbfc00200 by the linker script. NOTE:
 *   Several vectors (JTAG, TLB fills, etc.) could come through this vector.
 *   However, this is intended to serve vectors in PIC32MX single vector
 *   mode:  The EBASE register will be set to 0xbfc00000 and the vector
 *   should go to EBASE + 0x0200.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .ebase_excpt,"ax",@progbits
	.set	noreorder
	.ent	_ebase_exception
_ebase_exception:
	la		k0, _int_handler
	jr		k0
	nop
	.end _ebase_exception

/****************************************************************************
 * Name: _bev_exception
 *
 * Description:
 *   Boot Exception Vector Handler.  Jumps to _exception_handler.  This
 *   vector will be positioned at 0xbfc00380 by the linker script.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .bev_excpt,"ax",@progbits
	.set	noreorder
	.ent	_bev_exception
_bev_exception:
	la		k0, _exception_handler
	jr		k0
	nop
	.end _bev_exception

/****************************************************************************
 * Name: _int_exception
 *
 * Description:
 *   Interrupt Exception Vector Handler.  Jumps to _int_handler.  This
 *   vector will be positioned at 0xbfc00400 by the linker script.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .int_excpt,"ax",@progbits
	.set	noreorder
	.ent	_int_exception
_int_exception:
	la		k0, _int_handler
	jr		k0
	nop
	.end _int_exception

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the KSEG0 startup code.  It receives control from the reset
 *   entry point.  This lgic This prepares the processor to execute
 *   C code, performs some very low-level initialization, then starts NuttX
 *   (via __start_nuttx
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .start, "ax", @progbits
	.set	noreorder
	.ent	__start
__start:

	/* If this function was entered because of an NMI, then turn processing
	 * over to the NMI handler.
	 */

#ifdef CONFIG_PIC32MX_NMIHANDLER
	mfc0	k0, $12				/* Load CP0 status register */
	ext		k0, k0, 19, 1		/* Extract NMI bit */
	beqz	k0, .Lnotnmi
	nop
	la		k0, _nmi_handler
	jr		k0
	nop

	/* This is not an NMI */

.Lnotnmi:
#endif

	/* Initialize the stack pointer */

	la		sp, PIC32MX_STACK_TOP

	/* Initialize the globl pointer (gp).  _gp is initialized by the linker
	 * script to point to the "middle" of the small variables region.
	 */

	la		gp, _gp

	/* Initialize Global Pointer in Shadow Set.  The SRSCtl PSS field must
	 * be set to the shadow set in which to initialize the global pointer.
	 * Since we only have a single shadow set, we will initialize
	 * SRSCtl PSS to SRSCtl HSS.  We then write the global pointer to the
	 * previous shadow set to ensure that on interrupt, the global pointer
	 * has been initialized.
	 */

	mfc0	t1, PIC32MX_CP0_SRSCTL	/* Read SRSCtl register */
	add		t3, t1, zero			/* Save off current SRSCtl */
	ext		t2, t1, 26, 4			/* to obtain HSS field */
	ins		t1, t2, 6, 4			/* Put HSS field */
	mtc0	t1, PIC32MX_CP0_SRSCTL	/* into SRSCtl PSS  */
	wrpgpr	gp, gp					/* Set global pointer in PSS */
	mtc0	t3, PIC32MX_CP0_SRSCTL	/* Restore SRSCtl */

	/* Clear uninitialized data sections */

	la		t0, _sbss
	la		t1, _ebss
	b		.Lbsscheck
	nop

.Lbssloop:	
	sw		zero, 0x0(t0)
	sw		zero, 0x4(t0)
	sw		zero, 0x8(t0)
	sw		zero, 0xc(t0)
	addu	t0, 16

.Lbsscheck:
	bltu	t0, t1, .Lbssloop
	nop

	/* Copy initialized data from program flash to data memory */

	la		t0, _data_loaddr
	la		t1, _sdata
	la		t2, _edata
	b		.Ldatacheck
	nop

.Ldataloop:	
	lw		t3, (t0)
	sw		t3, (t1)
	addu	t0, 4
	addu	t1, 4

.Ldatacheck:
	bltu	t1, t2, .Ldataloop
	nop

	/* If there are no RAM functions, skip the next two sections --
	 * copying RAM functions from program flash to data memory and
	 * initializing bus matrix registers.
	 */

#ifdef CONFIG_ARCH_RAMFUNCS
	la		t1, _ramfunc_sizeof
	beqz	t1, .Lnoramfuncs
	nop

	/* Copy RAM functions from program flash to data memory */

	la		t0, _ramfunc_loadaddr
	la		t1, _sramfunc
	la		t2, _eramfunc

.Lramfuncloop:
	lw		t3,(t0)
	sw		t3,(t1)
	addu	t0,4
	addu	t1,4

	bltu	t1, t2, .Lramfuncloop
	nop

	/* Initialize bus matrix registers if RAM functions exist in the
	 * application
	 */

	la		t1, _bmxdkpba_address
	la		t2, PIC32MX_BMX_DKPBA
	sw		t1, 0(t2)
	la		t1, _bmxdudba_address
	la		t2, PIC32MX_BMX_DUDBA
	sw		t1, 0(t2)
	la		t1, _bmxdupba_address
	la		t2, PIC32MX_BMX_DUPBA
	sw		t1, 0(t2)
.Lnoramfuncs:
#endif

	/* Initialize CP0 Count register */

	mtc0	zero, PIC32MX_CP0_COUNT

	/* Initialize Compare register */

	li		t2, -1
	mtc0	t2, PIC32MX_CP0_COMPARE

	/* Initialize EBase register */

#ifdef CONFIG_PIC32MX_MVEC
	la		t1, CONFIG_PIC32MX_EBASE
	mtc0	t1, PIC32MX_CP0_EBASE

	/* Initialize IntCtl register */

	li		t1, CONFIG_PIC32MX_VECTORSPACING
	li		t2, 0
	ins		t2, t1, CP0_INTCTL_VS_SHIFT, 5
	mtc0	t2, PIC32MX_CP0_INTCTL
#endif

	/* Initialize CAUSE registers
	 * - Enable counting of Count register (DC = 0)
	 * - Use special exception vector (IV = 1)
	 * - Clear pending software interrupts (IP1:IP0 = 0)
	 */

	li		t1, CP0_CAUSE_IV
	mtc0	t1, PIC32MX_CP0_CAUSE

	/* Initialize STATUS register
	 * - Access to Coprocessor 0 not allowed in user mode (CU0 = 0)
	 * - User mode uses configured endianness (RE = 0)
	 * - Preserve Bootstrap Exception vectors (BEV)
	 * - Preserve soft reset (SR) and non-maskable interrupt (NMI)
	 * - CorExtend enabled based on whether CorExtend User Defined
	 *   Instructions have been implemented (CEE = Config(UDI))
	 * - Disable any pending interrups (IM7..IM2 = 0, IM1..IM0 = 0)
	 * - Disable hardware interrupts (IPL7:IPL2 = 0)
	 * - Base mode is Kernel mode (UM = 0)
	 * - Error level is normal (ERL = 0)
	 * - Exception level is normal (EXL = 0)
	 * - Interrupts are disabled (IE = 0)
	 */

	mfc0	t0, PIC32MX_CP0_CONFIG
	ext		t1, t0, 22,1			/* Extract UDI from Config register */
	sll		t1, t1, 17				/* Move UDI to Status.CEE location */
	mfc0	t0, PIC32MX_CP0_STATUS
	and		t0, t0, 0x00580000		/* Preserve SR, NMI, and BEV */
	or		t0, t1, t0				/* Include Status.CEE (from UDI) */
	mtc0	t0, PIC32MX_CP0_STATUS

	/* Initialize Status BEV for normal exception vectors */

	mfc0	t0, PIC32MX_CP0_STATUS
	and		t0, t0, ~CP0_STATUS_BEV	/* Clear BEV */
	mtc0	t0, PIC32MX_CP0_STATUS

	/* Start NuttX. We do this via a thunk in the text section so that
	 * a normal jump and link can be used, enabling the startup code
	 * to work properly whether main is written in MIPS16 or MIPS32
	 * code. I.e., the linker will correctly adjust the JAL to JALX if
	 * necessary
	 */

	la		t0, __start_nuttx
	jr		t0
	nop
	.end __start

/****************************************************************************
 * Name: _exception_handler
 *
 * Description:
 *   BEV/General exception handler.  Calls pic32mx_exception()
 *
 ****************************************************************************/

	.section .bev_handler, "ax", @progbits
	.set	noreorder
	.ent	_exception_handler
_exception_handler:
	EXCPT_PROLOGUE t0				/* Save registers on stack, enable nested interrupts */
	move	a0, sp					/* Pass register save structure as the parameter 1 */
	USE_INTSTACK t0, t1, t2			/* Switch to the interrupt stack */
	la		t0, pic32mx_exception		/* Call pic32mx_exception(regs) */
	jalr	ra, t0
	nop
#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS
	di								/* Prohibit nested interrupts from here */
#endif
	RESTORE_STACK t0, t1			/* Undo the operations of USE_STACK */
	EXCPT_EPILOGUE v0				/* Return to the context returned by pic32mx_exception() */
	.end	_exception_handler

/****************************************************************************
 * Name: _int_handler
 *
 * Description:
 *   Interrupt exception handler.  Calls up_decodeirq()
 *
 ****************************************************************************/

	.section .int_handler, "ax", @progbits
	.set	noreorder
	.ent	_int_handler
_int_handler:
	EXCPT_PROLOGUE t0				/* Save registers on stack, enable nested interrupts */
	move	a0, sp					/* Pass register save structure as the parameter 1 */
	USE_INTSTACK t0, t1, t2			/* Switch to the interrupt stack */
	la		t0, pic32mx_decodeirq	/* Call pic32mx_decodeirq(regs) */
	jalr	ra, t0
	nop
#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS
	di								/* Prohibit nested interrupts from here */
#endif
	RESTORE_STACK t0, t1			/* Undo the operations of USE_STACK */
	EXCPT_EPILOGUE v0				/* Return to the context returned by pic32mx_decodeirq() */
	.end	_int_handler

/****************************************************************************
 * Name: _nmi_handler
 *
 * Description:
 *   NMI exception handler.  Calls pic32mx_donmi
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_NMIHANDLER
	.section .nmi_handler, "ax", @progbits
	.set	noreorder
	.ent	_nmi_handler
_nmi_handler:
	EXCPT_PROLOGUE t0				/* Save registers on stack, enable nested interrupts */
	move	a0, sp					/* Pass register save structure as the parameter 1 */
	USE_INTSTACK t0, t1, t2			/* Switch to the interrupt stack */
	la		t0, pic32mx_donmi		/* Call up_donmi(regs) */
	jalr	ra, t0
	nop
#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS
	di								/* Prohibit nested interrupts from here */
#endif
	RESTORE_STACK t0, t1			/* Undo the operations of USE_STACK */
	EXCPT_EPILOGUE v0				/* Return to the context returned by pic32mx_donmi() */
	.end	_nmi_handler
#endif

/****************************************************************************
 * Name: __start_nuttx
 *
 * Description:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.text
	.ent __start_nuttx
__start_nuttx:
	/* Perform low level initialization */

	la		t0, pic32mx_lowinit
	jalr    ra, t0
	nop

	/* Call os_start */

	la		t0, os_start
	jalr	ra, t0
	nop

	/* Just in case main returns, go into an infinite loop */

halt:
1:
	b	 1b
	nop
	.end __start_nuttx

/****************************************************************************
 * Device Configuration
 ****************************************************************************/

	.section .devcfg, "a"
	.type	devconfig, object
devconfig:
devconfig3:
#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)

	.long	CONFIG_PIC32MX_USERID  << DEVCFG3_USERID_SHIFT | \
			CONFIG_PIC32MX_PMDL1WAY << 28 | CONFIG_PIC32MX_IOL1WAY << 29 | \
			CONFIG_PIC32MX_USBIDO  << 30 | CONFIG_PIC32MX_VBUSIO  << 31 | \
			DEVCFG3_UNUSED

#elif defined(CHIP_PIC32MX3) || defined(CHIP_PIC32MX4)

	.long	CONFIG_PIC32MX_USERID  << DEVCFG3_USERID_SHIFT | \
			DEVCFG3_UNUSED

#elif defined(CHIP_PIC32MX5) || defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)

	.long	CONFIG_PIC32MX_USERID  << DEVCFG3_USERID_SHIFT | \
			CONFIG_PIC32MX_SRSSEL  << DEVCFG3_FSRSSEL_SHIFT | \
			CONFIG_PIC32MX_FMIIEN  << 24 | CONFIG_PIC32MX_FETHIO  << 25 | \
			CONFIG_PIC32MX_FCANIO  << 26 | CONFIG_PIC32MX_FSCM1IO << 29 | \
			CONFIG_PIC32MX_USBIDO  << 30 | CONFIG_PIC32MX_VBUSIO  << 31 | \
			DEVCFG3_UNUSED

#endif

devconfig2:
	.long	CONFIG_PIC32MX_PLLIDIV  | CONFIG_PIC32MX_PLLMULT | \
			CONFIG_PIC32MX_UPLLIDIV | CONFIG_PIC32MX_PLLODIV | \
			CONFIG_PIC32MX_FUPLLEN << 15 | DEVCFG2_UNUSED

devconfig1:
#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)

	.long	CONFIG_PIC32MX_FNOSC    | CONFIG_PIC32MX_FSOSCEN  | \
			CONFIG_PIC32MX_IESO     | CONFIG_PIC32MX_POSCMOD  | \
			CONFIG_PIC32MX_OSCOUT << 10 | \
			CONFIG_PIC32MX_PBDIV    | CONFIG_PIC32MX_FCKSM    | \
			DEVCFG1_WINDIS          | CONFIG_PIC32MX_WDENABLE | \
			DEVCFG1_FWDTWINSZ_75    | DEVCFG1_UNUSED

#else

	.long	CONFIG_PIC32MX_FNOSC    | CONFIG_PIC32MX_FSOSCEN | \
			CONFIG_PIC32MX_IESO     | CONFIG_PIC32MX_POSCMOD | \
			CONFIG_PIC32MX_OSCOUT << 10 | \
			CONFIG_PIC32MX_PBDIV    | CONFIG_PIC32MX_FCKSM   | \
			CONFIG_PIC32MX_WDENABLE | DEVCFG1_UNUSED

#endif

devconfig0:
#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)

	.long	CONFIG_PIC32MX_DEBUGGER    << DEVCFG0_DEBUG_SHIFT | \
			DEVCFG0_JTAGEN | \
			CONFIG_PIC32MX_ICESEL      << DEVCFG0_ICESEL_SHIFT | \
			CONFIG_PIC32MX_PROGFLASHWP << DEVCFG0_PWP_SHIFT | \
			CONFIG_PIC32MX_BOOTFLASHWP << 24 | \
			CONFIG_PIC32MX_CODEWP      << 28 | \
			DEVCFG0_UNUSED

#else

	.long	CONFIG_PIC32MX_DEBUGGER    << DEVCFG0_DEBUG_SHIFT | \
			CONFIG_PIC32MX_ICESEL      << 3 | \
			CONFIG_PIC32MX_PROGFLASHWP << DEVCFG0_PWP_SHIFT | \
			CONFIG_PIC32MX_BOOTFLASHWP << 24 | \
			CONFIG_PIC32MX_CODEWP      << 28 | \
			DEVCFG0_UNUSED

#endif
	.size	devconfig, .-devconfig

/****************************************************************************
 * Global Data
 ****************************************************************************/

/* Interrupt stack variables */

#if CONFIG_ARCH_INTERRUPTSTACK > 3

/* g_instackbase is a pointer to the final, aligned word of the interrupt
 * stack.
 */

	.sdata
	.type	g_intstackbase, object
g_intstackbase:
	.long	PIC32MX_INTSTACK_TOP-4
	.size	g_intstackbase, .-g_intstackbase

/* g_nextlevel is the exception nesting level... the interrupt stack is not
 * available to nested exceptions.
 */

#ifdef CONFIG_PIC32MX_NESTED_INTERRUPTS
	.sbss
	.type	g_nestlevel, object
g_nestlevel:
	.skip		4
#endif
#endif

/* This global variable is unsigned int g_heapbase and is exported here only
 * because of its coupling to idle thread stack.
 */

	.sdata
	.type	g_heapbase, object
g_heapbase:
	.long	PIC32MX_HEAP_BASE
	.size	g_heapbase, .-g_heapbase

