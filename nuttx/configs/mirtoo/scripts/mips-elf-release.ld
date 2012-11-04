/****************************************************************************
 * configs/mirtoo/scripts/mips-elf-release.ld
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
/* Memory Regions ***********************************************************/

MEMORY
{
	/* The PIC32MX250F128D has 128Kb of program FLASH at physical address
	 * 0x1d000000 but is always accessed at KSEG0 address 0x9d00:0000
	 */

	kseg0_progmem  (rx) : ORIGIN = 0x9d000000, LENGTH = 128K

	/* The PIC32MX250F128D has 3Kb of boot FLASH at physical addresses
	 * 0x1fc00000-0x1fc00c00.  The initial reset vector is in KSEG1, but
	 * all other accesses are in KSEG0.
	 *
	 *   REGION          PHYSICAL   KSEG   SIZE
	 *   DESCRIPTION     START ADDR        (BYTES)
	 *   --------------- ---------- ------ ----------------------
	 *   Exceptions:*
	 *     Reset         0x1fc00000 KSEG1  384           384 (0.375Kb)
	 *     Gen exception 0x1fc00000 KSEG1  128           512 (0.500Kb)
	 *     TLB Refill    0x1fc00200 KSEG1  256           768 (0.750Kb)
	 *     Cache Error   0x1fc00300 KSEG1  128           896 (0.875Kb)
	 *     Others        0x1fc00380 KSEG1  128          1024 (1.000Kb)
	 *     Interrupt     0x1fc00400 KSEG1  128          1152 (1.125Kb)
	 *     JTAG          0x1fc00480 KSEG1  16           1168 (1.141Kb)
	 *   Exceptions      0x1fc00490 KSEG0  3072-1168-16 3056 (2.984Kb)
	 *   DEVCFG3-0       0x1fc00bf0 KSEG1  16           3072 (3Kb)
	 *
	 * Exceptions assume:
	 *
	 *   STATUS: BEV=0/1 and EXL=0
	 *   CAUSE:  IV=1
	 *   JTAG:   ProbEn=0
	 *   And multi-vector support disabled
	 */

	kseg1_reset    (rx) : ORIGIN = 0xbfc00000, LENGTH = 384
	kseg1_genexcpt (rx) : ORIGIN = 0xbfc00180, LENGTH = 128
	kseg1_ebexcpt  (rx) : ORIGIN = 0xbfc00200, LENGTH = 256
	kseg1_cacherr  (rx) : ORIGIN = 0xbfc00300, LENGTH = 128
	kseg1_bevexcpt (rx) : ORIGIN = 0xbfc00380, LENGTH = 128
	kseg1_intexcpt (rx) : ORIGIN = 0xbfc00400, LENGTH = 128
	kseg1_dbgexcpt (rx) : ORIGIN = 0xbfc00480, LENGTH = 16
	kseg0_bootmem  (rx) : ORIGIN = 0x9fc00490, LENGTH = 3072-1168-16
	kseg1_devcfg    (r) : ORIGIN = 0xbfc00bf0, LENGTH = 16

	/* The PIC32MX250F128D has 32Kb of data memory at physical address
	 * 0x00000000.  Since the PIC32MX has no data cache, this memory is
	 * always accessed through KSEG1.
	 *
	 * NOTE: When used with MPLAB, we need to set aside 512 bytes of memory
	 * at the beginning of this region for use by MPLAB.
	 */

	kseg1_datamem (w!x) : ORIGIN = 0xa0000200, LENGTH = 32K
}

OUTPUT_FORMAT("elf32-littlemips")
OUTPUT_ARCH(mips)
ENTRY(__start)

SECTIONS
{
	/* Boot FLASH sections */

	.reset :
	{
		KEEP (*(.reset))
	} > kseg1_reset

	/* Exception handlers.  The following is assumed:
	 *
	 *   STATUS: BEV=1 and EXL=0
	 *   CAUSE:  IV=1
	 *   JTAG:   ProbEn=0
	 *   And multi-vector support disabled
	 *
	 * In that configuration, the vector locations become:
	 *
	 *   Reset, Soft Reset  bfc0:0000
	 *   TLB Refill         bfc0:0200
	 *   Cache Error        bfc0:0300
	 *   All others         bfc0:0380
	 *   Interrupt          bfc0:0400
	 *   EJTAG Debug        bfc0:0480
	 */

	 /* KSEG1 exception handler "trampolines" */

	.gen_excpt :
	{
		KEEP (*(.gen_excpt))
	} > kseg1_genexcpt

	.ebase_excpt :
	{
		KEEP (*(.ebase_excpt))
	} > kseg1_ebexcpt

	.bev_excpt :
	{
		KEEP (*(.bev_excpt))
	} > kseg1_bevexcpt

	.int_excpt :
	{
		KEEP (*(.int_excpt))
	} > kseg1_intexcpt

	.dbg_excpt = ORIGIN(kseg1_dbgexcpt);

	.start :
	{
		/* KSEG0 Reset startup logic */

		*(.start)

		/* KSEG0 exception handlers */

		*(.nmi_handler)
		*(.bev_handler)
		*(.int_handler)
	} > kseg0_bootmem

	.devcfg :
	{
		KEEP (*(.devcfg))
	} > kseg1_devcfg

	/* Program FLASH sections */

	.text :
	{
		_stext = ABSOLUTE(.);
		*(.text .text.*)
		*(.stub)
		KEEP (*(.text.*personality*))
		*(.gnu.linkonce.t.*)
		*(.gnu.warning)
		*(.mips16.fn.*)
		*(.mips16.call.*)

		/* Read-only data is included in the text section */

		*(.rodata .rodata.*)
		*(.rodata1)
		*(.gnu.linkonce.r.*)

		/* Small initialized constant global and static data */

		*(.sdata2 .sdata2.*)
		*(.gnu.linkonce.s2.*)

		/* Uninitialized constant global and static data */

		*(.sbss2 .sbss2.*)
		*(.gnu.linkonce.sb2.*)
		_etext = ABSOLUTE(.);
	} > kseg0_progmem

	/* Initialization data begins here in progmem */

	_data_loaddr = LOADADDR(.data);

	.eh_frame_hdr : { *(.eh_frame_hdr) }
	.eh_frame : ONLY_IF_RO { KEEP (*(.eh_frame)) }

	/* RAM functions are positioned at the beginning of RAM so that
	 * they can be guaranteed to satisfy the 2Kb alignment requirement.
	 */

/* This causes failures if there are no RAM functions
	.ramfunc ALIGN(2K) :
	{
		_sramfunc =  ABSOLUTE(.);
		*(.ramfunc	.ramfunc.*)
		_eramfunc =  ABSOLUTE(.);
	} > kseg1_datamem AT > kseg0_progmem

	_ramfunc_loadaddr = LOADADDR(.ramfunc);
	_ramfunc_sizeof   = SIZEOF(.ramfunc);
	_bmxdkpba_address = _sramfunc - ORIGIN(kseg1_datamem) ;
	_bmxdudba_address = LENGTH(kseg1_datamem) ;
	_bmxdupba_address = LENGTH(kseg1_datamem) ;
*/

	.data :
	{
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		KEEP (*(.gnu.linkonce.d.*personality*))
		*(.data1)
	} > kseg1_datamem AT > kseg0_progmem

	.eh_frame : ONLY_IF_RW { KEEP (*(.eh_frame)) }
	_gp = ALIGN(16) + 0x7FF0 ;

	.got :
	{
		*(.got.plt) *(.got)
	} > kseg1_datamem AT > kseg0_progmem

	.sdata :
	{
		*(.sdata .sdata.* .gnu.linkonce.s.*)
	} > kseg1_datamem AT > kseg0_progmem

	.lit8 :
	{
		*(.lit8)
	} > kseg1_datamem AT > kseg0_progmem

	.lit4 :
	{
		*(.lit4)
		_edata = ABSOLUTE(.);
	} >kseg1_datamem AT>kseg0_progmem

	.sbss :
	{
		_sbss = ABSOLUTE(.);
		*(.dynsbss)
		*(.sbss .sbss.* .gnu.linkonce.sb.*)
		*(.scommon)
	} >kseg1_datamem

	.bss :
	{
		*(.dynbss)
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		_ebss = ABSOLUTE(.);
	} > kseg1_datamem

	/* Stabs debugging sections */

	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }

	/* DWARF debug sections */
	/* DWARF 1 */

	.debug 0 : { *(.debug) }
	.line 0 : { *(.line) }

	/* GNU DWARF 1 extensions */

	.debug_srcinfo 0 : { *(.debug_srcinfo) }
	.debug_sfnames 0 : { *(.debug_sfnames) }

	/* DWARF 1.1 and DWARF 2 */

	.debug_aranges 0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }

	/* DWARF 2 */

	.debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_line 0 : { *(.debug_line) }
	.debug_frame 0 : { *(.debug_frame) }
	.debug_str 0 : { *(.debug_str) }
	.debug_loc 0 : { *(.debug_loc) }
	.debug_macinfo 0 : { *(.debug_macinfo) }

	/* SGI/MIPS DWARF 2 extensions */

	.debug_weaknames 0 : { *(.debug_weaknames) }
	.debug_funcnames 0 : { *(.debug_funcnames) }
	.debug_typenames 0 : { *(.debug_typenames) }
	.debug_varnames 0 : { *(.debug_varnames) }

	/DISCARD/ : { *(.note.GNU-stack) }
}
