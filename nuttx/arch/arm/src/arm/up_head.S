/****************************************************************************
 * arch/arm/src/arm/up_pghead.S
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

#include "arm.h"
#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#  include "pg_macros.h"
#endif

/**********************************************************************************
 * Configuration
 **********************************************************************************/

#undef ALIGNMENT_TRAP
#undef CPU_DCACHE_WRITETHROUGH
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE

/* There are three operational memory configurations:
 *
 * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM, 
 *    - Initialize the .data section in RAM, and
 *    - Clear .bss section
 */

#ifdef CONFIG_BOOT_RUNFROMFLASH
#  error "Configuration not implemented"
#  define CONFIG_SDRAM 1

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of FLASH is the same as the physical
   * beginning of FLASH.
   */

#  if !defined(CONFIG_FLASH_START) || !defined(CONFIG_FLASH_VSTART)
#    error "CONFIG_FLASH_START or CONFIG_FLASH_VSTART is not defined"
#  endif

#  if CONFIG_FLASH_START == CONFIG_FLASH_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

/* 2. We boot in FLASH but copy ourselves to DRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM, 
 *    - Copy ourself to DRAM (after mapping it), and
 *    - Clear .bss section
 *
 *   In this case, we assume that the logic within this file executes from FLASH.
 */

#elif defined(CONFIG_BOOT_COPYTORAM)
#  error "configuration not implemented
#  define CONFIG_SDRAM 1

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of FLASH is the same as the physical
   * beginning of FLASH.
   */

#  if !defined(CONFIG_FLASH_START) || !defined(CONFIG_FLASH_VSTART)
#    error "CONFIG_FLASH_START or CONFIG_FLASH_VSTART is not defined"
#  endif

#  if CONFIG_FLASH_START == CONFIG_FLASH_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

/* 3. There is bootloader that copies us to DRAM (but probably not to the beginning)
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n). In this case SDRAM
 *    was initialized by the boot loader, and this boot logic must: 
 *
 *    - Clear .bss section
 */

#else

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of RAM is the same as the physical
   * beginning of RAM.
   */

#  if !defined(CONFIG_DRAM_START) || !defined(CONFIG_DRAM_VSTART)
#    error "CONFIG_DRAM_START or CONFIG_DRAM_VSTART is not defined"
#  endif

#  if CONFIG_DRAM_START == CONFIG_DRAM_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

#endif

/* For each page table offset, the following provide (1) the physical address of
 * the start of the page table and (2) the number of page table entries in the
 * first page table.
 *
 * Coarse: PG_L1_PADDRMASK=0xfffffc00
 *         NPAGE1=(256 -((a) & 0x000003ff) >> 2) NPAGE1=1-256
 * Fine:   PG_L1_PADDRMASK=0xfffff000
 *         NPAGE1=(1024 -((a) & 0x00000fff) >> 2) NPAGE1=1-1024
 */

#ifdef CONFIG_PAGING
#  define PG_L2_TEXT_PBASE     (PG_L2_TEXT_PADDR & PG_L1_PADDRMASK)
#  define PG_L2_TEXT_NPAGE1    (PTE_NPAGES - ((PG_L2_TEXT_PADDR & ~PG_L1_PADDRMASK) >> 2))
#  define PG_L2_PGTABLE_PBASE  (PG_L2_PGTABLE_PADDR & PG_L1_PADDRMASK)
#  define PG_L2_PGTABLE_NPAGE1 (PTE_NPAGES - ((PG_L2_PGTABLE_PADDR & ~PG_L1_PADDRMASK) >> 2))
#  define PG_L2_DATA_PBASE     (PG_L2_DATA_PADDR & PG_L1_PADDRMASK)
#  define PG_L2_DATA_NPAGE1    (PTE_NPAGES - ((PG_L2_DATA_PADDR & ~PG_L1_PADDRMASK) >> 2))
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* RX_NSECTIONS determines the number of 1Mb sections to map for the
 * Read/eXecute address region.  This is based on CONFIG_DRAM_SIZE.  For most
 * ARM9 architectures, CONFIG_DRAM_SIZE describes the size of installed SDRAM.
 * But for other architectures, this might refer to the size of FLASH or
 * SRAM regions. (bad choice of naming).
 */

#define RX_NSECTIONS ((CONFIG_DRAM_SIZE+0x000fffff) >> 20)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* The ARM9 L1 page table can be placed at the beginning or at the end of
 * the RAM space.  This decision is based on the placement of the vector
 * area: If the vectors are place in low memory at address 0x0000 0000, then
 * the page table is placed in high memory; if the vectors are placed in
 * high memory at address 0xfff0 0000, then the page table is locating at
 * the beginning of RAM.
 *
 * For the special case where (1) the program executes out of RAM, and (2)
 * the page is located at the beginning of RAM (i.e., the high vector case),
 * then the following macro can easily find the physical address of the
 * section that includes the first part of the text region:  Since the page
 * table is closely related to the NuttX base address in this case, we can
 * convert the page table base address to the base address of the section
 * containing both.
 */

/* REVISIT:  This works now of the low vector case only because the RAM
 * sizes that we have been dealing with are less then 1MB so that both the
 * page table and the vector table are in the same 1MB RAM block.  But
 * this will certainly break later. Hence, the annoying warning.
 */

#ifdef CONFIG_ARCH_LOWVECTORS
#  warning "REVISIT"
#endif

//#ifndef CONFIG_ARCH_LOWVECTORS
	.macro	mksection, section, pgtable
	bic	\section, \pgtable, #0x000ff000
	.endm
//#endif

/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG
	.macro	showprogress, code
	mov	r0, #\code
	bl	up_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * Name: __start
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for us
 * and that only leaves us having to do some os specific things below.
 */
	.text
	.global	__start
	.type	__start, #function
__start:
	/* Make sure that we are in SVC mode with all IRQs disabled */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT)
	msr	cpsr_c, r0

	/* Initialize DRAM using a macro provided by board-specific logic */

#ifdef CONFIG_SDRAM
	config_sdram
#endif
	/* Clear the 16K level 1 page table */

	ldr	r4, .LCppgtable			/* r4=phys. page table */
#ifndef CONFIG_ARCH_ROMPGTABLE
	mov	r0, r4
	mov	r1, #0
	add	r2, r0, #PGTABLE_SIZE
.Lpgtableclear:
	str	r1, [r0], #4
	str	r1, [r0], #4
	str	r1, [r0], #4
	str	r1, [r0], #4
	teq	r0, r2
	bne	.Lpgtableclear

	/* Create identity mapping for first MB of the .text section to support
	 * this startup logic executing out of the physical address space.  This
	 * identity mapping will be removed by .Lvstart (see below).  Of course,
	 * we would only do this if the physical-virtual mapping is not already
	 * the identity mapping.
	 */

#ifndef CONFIG_IDENTITY_TEXTMAP
	mksection r0, r4			/* r0=phys. base section */
	ldr	r1, .LCmmuflags			/* FLGS=MMU_MEMFLAGS */
	add	r3, r1, r0			/* r3=flags + base */
	str	r3, [r4, r0, lsr #18]		/* identity mapping */
#endif

#ifdef CONFIG_PAGING

	/* Map the read-only .text region in place.  This must be done
	 * before the MMU is enabled and the virtual addressing takes
	 * effect.  First populate the L1 table for the locked and paged
	 * text regions.
	 *
	 * We could probably make the the pg_l1span and pg_l2map macros into
	 * call-able subroutines, but we would have to be carefully during
	 * this phase while we are operating in a physical address space.
	 *
	 * NOTE: That the value of r5 (L1 table base address) must be
	 * preserved through the following.
	 */

	adr	r0, .Ltxtspan
	ldmia	r0, {r0, r1, r2, r3, r5}
 	pg_l1span r0, r1, r2, r3, r5, r6
 
	/* Then populate the L2 table for the locked text region only. */

	adr	r0, .Ltxtmap
	ldmia	r0, {r0, r1, r2, r3}
 	pg_l2map r0, r1, r2, r3, r5

	/* Make sure that the page table is itself mapped and and read/write-able.
	 * First, populate the L1 table:
	 */

	adr	r0, .Lptabspan
	ldmia	r0, {r0, r1, r2, r3, r5}
 	pg_l1span r0, r1, r2, r3, r5, r6

	/* Then populate the L2 table. */

	adr	r0, .Lptabmap
	ldmia	r0, {r0, r1, r2, r3}
 	pg_l2map r0, r1, r2, r3, r5

#else /* CONFIG_PAGING */

	/* Create a virtual single section mapping for the first MB of the .text
	 * address space. Now, we have the first 1MB mapping to both phyical and
	 * virtual addresses.  The rest of the .text mapping will be completed in
	 * .Lvstart once we have moved the physical mapping out of the way.
	 *
	 * Here we expect to have:
	 * r4 = Address of the base of the L1 table
	 */

	ldr	r2, .LCvpgtable			/* r2=virt. page table */
	mksection r0, r2			/* r0=virt. base section */
	str	r3, [r4, r0, lsr #18]		/* identity mapping */

	/* NOTE: No .data/.bss access should be attempted.  This temporary mapping
	 * can only be assumed to cover the initial .text region.
	 */

#endif /* CONFIG_PAGING */
#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* The following logic will set up the ARM920/ARM926 for normal operation.
	 *
	 * Here we expect to have:
	 * r4 = Address of the base of the L1 table
	 */

	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		/* Invalidate I,D caches */
	mcr	p15, 0, r0, c7, c10, 4		/* Drain write buffer */
	mcr	p15, 0, r0, c8, c7		/* Invalidate I,D TLBs */
	mcr	p15, 0, r4, c2, c0		/* Load page table pointer */

#ifdef CPU_DCACHE_WRITETHROUGH
	mov	r0, #4				/* Disable write-back on caches explicitly */
	mcr	p15, 7, r0, c15, c0, 0
#endif 

	/* Enable the MMU and caches
	 * lr = Resume at .Lvstart with the MMU enabled
	 */

	ldr	lr, .LCvstart			/* Abs. virtual address */

	mov	r0, #0x1f			/* Domains 0, 1 = client */
	mcr	p15, 0, r0, c3, c0		/* Load domain access register */
	mrc	p15, 0, r0, c1, c0		/* Get control register */
	
	/* Clear bits (see arm.h)
	 *
	 * CR_R - ROM MMU protection
	 * CR_F - Implementation defined
	 * CR_Z - Implementation defined
	 *
	 * CR_A - Alignment abort enable
	 * CR_C - Dcache enable
	 * CR_W - Write buffer enable
	 *
	 * CR_I - Icache enable
	 */

	bic	r0, r0, #(CR_R|CR_F|CR_Z)
	bic	r0, r0, #(CR_A|CR_C|CR_W)
	bic	r0, r0, #(CR_I)

	/* Set bits (see arm.h)
	 *
	 * CR_M - MMU enable
	 * CR_P - 32-bit exception handler
	 * CR_D - 32-bit data address range
	 */

	orr	r0, r0, #(CR_M|CR_P|CR_D)
	
	/* In most architectures, vectors are relocated to 0xffff0000.
	 * -- but not all
	 *
	 * CR_S - System MMU protection
	 * CR_V - Vectors relocated to 0xffff0000
	 */

#ifndef CONFIG_ARCH_LOWVECTORS
	orr	r0, r0, #(CR_S|CR_V)
#else
	orr	r0, r0, #(CR_S)
#endif
	/* CR_RR - Round Robin cache replacement */

#ifdef CPU_CACHE_ROUND_ROBIN
	orr	r0, r0, #(CR_RR)
#endif
	/* CR_C - Dcache enable */

#ifndef CPU_DCACHE_DISABLE
	orr	r0, r0, #(CR_C)
#endif
	/* CR_C - Dcache enable */

#ifndef CPU_ICACHE_DISABLE
	orr	r0, r0, #(CR_I)
#endif
	/* CR_A - Alignment abort enable */

#ifdef ALIGNMENT_TRAP
	orr	r0, r0, #(CR_A)
#endif
	mcr	p15, 0, r0, c1, c0, 0		/* write control reg */

	/* Get TMP=2 Processor ID register */

	mrc	p15, 0, r1, c0, c0, 0		/* read id reg */
	mov	r1,r1				/* Null-avoiding nop */
	mov	r1,r1				/* Null-avoiding nop */

	/* And "jump" to .Lvstart */

	mov	pc, lr

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* Most addresses are all virtual address */
	
	.type	.LCvstart, %object
.LCvstart:
	.long	.Lvstart

#ifndef CONFIG_ARCH_ROMPGTABLE
	.type	.LCmmuflags, %object
.LCmmuflags:
	.long	MMU_MEMFLAGS			/* MMU flags for memory sections */
#endif

	.type	.LCppgtable, %object
.LCppgtable:
	.long	PGTABLE_BASE_PADDR		/* Physical start of page table */

#ifndef CONFIG_ARCH_ROMPGTABLE
	.type	.LCvpgtable, %object
.LCvpgtable:
	.long	PGTABLE_BASE_VADDR		/* Virtual start of page table */
#endif

#ifdef CONFIG_PAGING

.Ltxtspan:
	.long	PG_L1_TEXT_PADDR		/* Physical address in the L1 table */
	.long	PG_L2_TEXT_PBASE		/* Physical address of the start of the L2 page table */
	.long	PG_TEXT_NVPAGES			/* Total (virtual) text pages to be mapped */
	.long	PG_L2_TEXT_NPAGE1		/* The number of text pages in the first page table */
	.long	MMU_L1_TEXTFLAGS		/* L1 MMU flags to use */

.Ltxtmap:
	.long	PG_L2_LOCKED_PADDR		/* Physical address in the L2 table */
	.long	PG_LOCKED_PBASE			/* Physical address of locked base memory */
	.long	CONFIG_PAGING_NLOCKED		/* Number of pages in the locked region */
	.long	MMU_L2_TEXTFLAGS		/* L2 MMU flags to use */

.Lptabspan:
	.long	PG_L1_PGTABLE_PADDR		/* Physical address in the L1 table */
	.long	PG_L2_PGTABLE_PBASE		/* Physical address of the start of the L2 page table */
	.long	PG_PGTABLE_NPAGES		/* Total mapped page table pages */
	.long	PG_L2_PGTABLE_NPAGE1		/* The number of text pages in the first page table */
	.long	MMU_L1_PGTABFLAGS		/* L1 MMU flags to use */

.Lptabmap:
	.long	PG_L2_PGTABLE_PADDR		/* Physical address in the L2 table */
	.long	PGTABLE_BASE_PADDR		/* Physical address of the page table memory */
	.long	PG_PGTABLE_NPAGES		/* Total mapped page table pages */
	.long	MMU_L2_PGTABFLAGS		/* L2 MMU flags to use */

#endif /* CONFIG_PAGING */
	.size	__start, .-__start

/****************************************************************************
 * Name: .Lvstart
 ***************************************************************************/

/* The following is executed after the MMU has been enabled. This uses
 * absolute addresses; this is not position independent.
 */
	.align	5
	.local	.Lvstart
	.type	.Lvstart, %function
.Lvstart:

	/* Remove the temporary mapping (if one was made).  The following assumes
	 * that the total RAM size is > 1Mb and extends that initial mapping to
	 * cover additinal RAM sections.
	 */


#ifndef CONFIG_ARCH_ROMPGTABLE
#ifndef CONFIG_IDENTITY_TEXTMAP
	ldr	r4, .LCvpgtable			/* r4=virtual page table */
	ldr	r1, .LCppgtable			/* r1=phys. page table */
	mksection r3, r1			/* r2=phys. base addr */
	mov	r0, #0				/* flags + base = 0 */
	str	r0, [r4, r3, lsr #18]		/* Undo identity mapping */
#endif

#if defined(CONFIG_PAGING)
	/* Populate the L1 table for the data region */

	adr	r0, .Ldataspan
	ldmia	r0, {r0, r1, r2, r3, r4}
 	pg_l1span r0, r1, r2, r3, r4, r5

	/* Populate the L2 table for the data region */

	adr	r0, .Ldatamap
	ldmia	r0, {r0, r1, r2, r3}
 	pg_l2map r0, r1, r2, r3, r4

#elif defined(CONFIG_BOOT_RUNFROMFLASH)
#  error "Logic not implemented"
#else
	/* Now setup the pagetables for our normal SDRAM mappings mapped region.
	 * We round NUTTX_START_VADDR down to the nearest megabyte boundary.
	 */

	ldr	r1, .LCmmuflags			/* FLGS=MMU_MEMFLAGS */
	add	r3, r3, r1			/* r3=flags + base */

	add	r0, r4, #(NUTTX_START_VADDR & 0xff000000) >> 18
	bic	r2, r3, #0x00f00000
	str	r2, [r0]

	add	r0, r0, #(NUTTX_START_VADDR & 0x00f00000) >> 18
	str	r3, [r0], #4

	/* Now map the remaining RX_NSECTIONS-1 sections of the executable
	 * memory region.
	 */

	.rept	RX_NSECTIONS-1
	add	r3, r3, #SECTION_SIZE
	str	r3, [r0], #4
	.endr

	/* If we are executing from RAM with a fixed page configuration, then
	 * we can assume that the above contiguous mapping included all of the
	 * .text, .data, .bss, heap, etc. But if we are executing from FLASH,
	 * then the RAM area is probably in a separate physical address region
	 * and will require a separate mapping.  Or, if we are supporting on-demand
	 * paging of the .text region, then the RAM-based .data/.bss/heap section
	 * will still probably be located in a separate (virtual) address region.
	 */

#endif /* CONFIG_PAGING */
#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* Zero BSS and set up the stack pointer */

	adr	r0, .Linitparms
	ldmia	r0, {r0, r1, sp}

	/* Clear the frame pointer and .bss */

	mov     fp, #0

.Lbssinit:	
	cmp     r0, r1		       		/* Clear up to _bss_end_ */
	strcc	fp, [r0],#4
	bcc	.Lbssinit

	/* If the .data section is in a separate, unitialized address space,
	 * then we will also need to copy the initial values of of the .data
	 * section from the .text region into that .data region.  This would
	 * be the case if we are executing from FLASH and the .data section
	 * lies in a different physical address region OR if we are support
	 * on-demand paging and the .data section lies in a different virtual
	 * address region.
	 */

#if defined(CONFIG_BOOT_RUNFROMFLASH) || defined(CONFIG_PAGING)
	adr	r3, .Ldatainit
	ldmia	r3, {r0, r1, r2}

1:	ldmia	r0!, {r3 - r10}
	stmia	r1!, {r3 - r10}
	cmp	r1, r2
	blt	1b
#endif

	/* Perform early C-level, platform-specific initialization */

	bl	up_boot

	/* Finally branch to the OS entry point */

	mov	lr, #0
	b	os_start

	/* Text-section constants:
	 *
	 *   _sbss is the start of the BSS region (see ld.script)
	 *   _ebss is the end of the BSS regsion (see ld.script)
	 *
	 * The idle task stack starts at the end of BSS and is of size
	 * CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there until the
	 * end of memory.  See g_heapbase below.
	 */

.Linitparms:
	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4

#ifdef CONFIG_PAGING

.Ldataspan:
	.long	PG_L1_DATA_VADDR	/* Virtual address in the L1 table */
	.long	PG_L2_DATA_PBASE	/* Physical address of the start of the L2 page table */
	.long	PG_DATA_NPAGES		/* Number of pages in the data region */
	.long	PG_L2_DATA_NPAGE1	/* The number of text pages in the first page table */
	.long	MMU_L1_DATAFLAGS	/* L1 MMU flags to use */

.Ldatamap:
	.long	PG_L2_DATA_VADDR	/* Virtual address in the L2 table */
	.long	PG_DATA_PBASE		/* Physical address of data memory  */
	.long	PG_DATA_NPAGES		/* Number of pages in the data region */
	.long	MMU_L2_DATAFLAGS	/* L2 MMU flags to use */

#endif /* CONFIG_PAGING */

#if defined(CONFIG_BOOT_RUNFROMFLASH) || defined(CONFIG_PAGING)
.Ldatainit:
	.long	_eronly			/* Where .data defaults are stored in FLASH */
	.long	_sdata			/* Where .data needs to reside in SDRAM */
	.long	_edata
#endif
	.size	.Lvstart, .-.Lvstart

	 /* Data section variables */

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to .Linitparms
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

