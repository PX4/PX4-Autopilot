/****************************************************************************
 * scripts/script.ld
 *
 *   Copyright (C) 2026 CBUnmanned Ltd. All rights reserved.
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

/* The CBUnmanned H753-SOM uses an STM32H753VI with 2048Kb of main FLASH.
 * The flash memory is partitioned into 16 sectors of 128Kb each:
 *
 *   Sector 0  (128K): Bootloader
 *   Sectors 1-14 (1792K): Firmware
 *   Sector 15 (128K): Flash-based parameter storage
 *
 * The STM32H753VI has 1024Kb of data SRAM:
 *
 *   128Kb DTCM-RAM at 0x20000000
 *    64Kb ITCM-RAM at 0x00000000
 *   512Kb AXI SRAM at 0x24000000
 *   128Kb SRAM1 at 0x30000000
 *   128Kb SRAM2 at 0x30020000
 *    32Kb SRAM3 at 0x30040000
 *    64Kb SRAM4 at 0x38000000
 *     4Kb Backup RAM at 0x38800000
 */

MEMORY
{
	ITCM_RAM  (rwx) : ORIGIN = 0x00000000, LENGTH =   64K
	FLASH      (rx) : ORIGIN = 0x08020000, LENGTH = 1792K

	DTCM1_RAM (rwx) : ORIGIN = 0x20000000, LENGTH =   64K
	DTCM2_RAM (rwx) : ORIGIN = 0x20010000, LENGTH =   64K
	AXI_SRAM  (rwx) : ORIGIN = 0x24000000, LENGTH =  512K /* D1 domain AXI bus */
	SRAM1     (rwx) : ORIGIN = 0x30000000, LENGTH =  128K /* D2 domain AHB bus */
	SRAM2     (rwx) : ORIGIN = 0x30020000, LENGTH =  128K /* D2 domain AHB bus */
	SRAM3     (rwx) : ORIGIN = 0x30040000, LENGTH =   32K /* D2 domain AHB bus */
	SRAM4     (rwx) : ORIGIN = 0x38000000, LENGTH =   64K /* D3 domain */
	BKPRAM    (rwx) : ORIGIN = 0x38800000, LENGTH =    4K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

/*
 * Ensure that abort() is present in the final object.  The exception handling
 * code pulled in by libgcc.a requires it (and that code cannot be easily avoided).
 */
EXTERN(abort)
EXTERN(_bootdelay_signature)

SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(.vectors)
		. = ALIGN(32);
		/*
		This signature provides the bootloader with a way to delay booting
		*/
		_bootdelay_signature = ABSOLUTE(.);
		FILL(0xffecc2925d7d05c5)
		. += 8;
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);

	} > FLASH

	/*
	 * Init functions (static constructors and the like)
	 */
	.init_section : {
		_sinit = ABSOLUTE(.);
		KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
	} > FLASH


	.ARM.extab : {
		*(.ARM.extab*)
	} > FLASH

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > FLASH
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);

		/* Pad out last section as the STM32H7 Flash write size is 256 bits. 32 bytes */
		. = ALIGN(16);
		FILL(0xffff)
		. += 16;
	} > AXI_SRAM AT > FLASH  = 0xffff

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > AXI_SRAM

	/* Emit the the D3 power domain section for locating BDMA data */

	.sram4_reserve (NOLOAD) :
	{
		*(.sram4)
		. = ALIGN(4);
		_sram4_heap_start = ABSOLUTE(.);
	} > SRAM4

	/* Stabs debugging sections. */
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}
