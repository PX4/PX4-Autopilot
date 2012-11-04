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

  sram   (rwx)  : ORIGIN = 0x2000, LENGTH = 8K

  /* Two fixed text flash pages (corresponding to page 3e and 3f).  The
   * top 256 bytes of page 3f is reserved to hold the interrupt vectors
   */

  lowtext(rx)   : ORIGIN = 0x4000, LENGTH = 16K			/* Page 3e */
  hitext (rx)   : ORIGIN = 0xc000, LENGTH = 16K-256		/* Page 3f */

  /* Flash memory pages:
   *
   * The MC9S12NE64 implements 6 bits of the PPAGE register which gives it a
   * 1 Mbyte program memory address space that is accessed through the PPAGE
   * window. The lower 768K portion (0x000000-0x0bffff) of the address space
   * is accessed with PPAGE values 0x00 through 0x2f.  This address range
   * is reserved for external memory when the part is operated in expanded
   * mode. The upper 256K of the address space (0x0c0000-0x100000), accessed
   * with PPAGE values 0x30 through 0x3f, is occupied by on chip flash.
   */

  page30 (rx)   : ORIGIN = 0x0c0000, LENGTH = 16K		/* Page 30 */
  page31 (rx)   : ORIGIN = 0x0c4000, LENGTH = 16K		/* Page 31 */
  page32 (rx)   : ORIGIN = 0x0c8000, LENGTH = 16K		/* Page 32 */
  page33 (rx)   : ORIGIN = 0x0cc000, LENGTH = 16K		/* Page 33 */
  page34 (rx)   : ORIGIN = 0x0d0000, LENGTH = 16K		/* Page 34 */
  page35 (rx)   : ORIGIN = 0x0d4000, LENGTH = 16K		/* Page 35 */
  page36 (rx)   : ORIGIN = 0x0d8000, LENGTH = 16K		/* Page 36 */
  page37 (rx)   : ORIGIN = 0x0dc000, LENGTH = 16K		/* Page 37 */
  page38 (rx)   : ORIGIN = 0x0e0000, LENGTH = 16K		/* Page 38 */
  page39 (rx)   : ORIGIN = 0x0e4000, LENGTH = 16K		/* Page 39 */
  page3a (rx)   : ORIGIN = 0x0e8000, LENGTH = 16K		/* Page 3a */
  page3b (rx)   : ORIGIN = 0x0ec000, LENGTH = 16K		/* Page 3b */
  page3c (rx)   : ORIGIN = 0x0f0000, LENGTH = 16K		/* Page 3c */
  page3d (rx)   : ORIGIN = 0x0f4000, LENGTH = 16K		/* Page 3d */

  page3e (rx)   : ORIGIN = 0x0f8000, LENGTH = 16K		/* Page 3e */
  page3f (rx)   : ORIGIN = 0x0fc000, LENGTH = 16K-2K	/* Page 3f */

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
	} > hitext

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > lowtext

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
