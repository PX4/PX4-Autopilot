/****************************************************************************
 * configs/ne64badge/ostest/ld.script
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
 ****************************************************************************/

/* The NE64 Badge has 64Kb of FLASH and 8Kb of SRAM that are assumed to be
 * paged and positioned as below:
 */

MEMORY
{
  /* The register space resides at address 0x0000-0x03ff.  The following
   * address, 0x0400-0x1fff are unused.
   *
   * The 8Kb SRAM is mapped to 0x2000-0x2fff.
   */

  sram    (rwx) : ORIGIN = 0x2000, LENGTH = 8K

  /* Three fixed text flash pages (corresponding to page 3e, 3d, and 3f) at
   * 16Kb each (minus 256 bytes at the end of page 3f that is reserved for
   * interrupt vectors).  Notice that this is linked as a single contiguous;
   * Post-processing is planned to make the binary to the appropriate flash
   * pages.
   */

  text    (rx)  : ORIGIN = 0x4000, LENGTH = 48K-256		/* Page 3e, 3d, and 3f */

  /* Vectors */

  vectors (rx)  : ORIGIN = 0xff80, LENGTH = 256
}

ENTRY(_stext)
SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(nonbanked)
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
	} > text

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > text

	.vectors : {
		*(vectors)
	} > vectors

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		_ebss = ABSOLUTE(.);
	} > sram

	.vectors : {
		_svectors = ABSOLUTE(.);
		*(.vectors)
		_evectors = ABSOLUTE(.);
	} > vectors

	/* Stabs debugging sections.	*/
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
