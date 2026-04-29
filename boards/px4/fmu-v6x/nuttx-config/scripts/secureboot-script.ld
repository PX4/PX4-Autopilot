/****************************************************************************
 * scripts/secureboot-script.ld
 *
 * Linker script for the px4_fmu-v6x_secureboot variant.
 *
 * Same layout as script.ld, except:
 *   - The image table-of-contents (.main_toc, defined by src/toc.c)
 *     is placed at a fixed offset past the vector table so the
 *     bootloader can find it via BOARD_IMAGE_TOC_OFFSET (0x800).
 *   - An empty .signature output section reserves the symbol
 *     _boot_signature at the end of the FLASH content. The TOC
 *     references it as the signed-image end address; sign_firmware.py
 *     appends the 64-byte ed25519 signature there.
 *
 ****************************************************************************/

MEMORY
{
	ITCM_RAM  (rwx) : ORIGIN = 0x00000000, LENGTH =   64K
	FLASH      (rx) : ORIGIN = 0x08020000, LENGTH = 1920K

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

EXTERN(abort)
EXTERN(board_get_manifest)
EXTERN(_main_toc)

SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(.vectors)
		/* Place the image TOC at a fixed offset past the vector table
		 * so the bootloader can find it via BOARD_IMAGE_TOC_OFFSET.
		 * 0x800 leaves comfortable room past the STM32H7 vector table
		 * (~0x298 bytes) and lands on a clean 2 KiB boundary.
		 */
		. = ABSOLUTE(ORIGIN(FLASH) + 0x800);
		KEEP(*(.main_toc))
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

	.sram4_reserve (NOLOAD) :
	{
		*(.sram4)
		. = ALIGN(4);
		_sram4_heap_start = ABSOLUTE(.);
	} > SRAM4

	/* Marker for the end of the signed image. sign_firmware.py
	 * appends the 64-byte ed25519 signature at this address; the TOC's
	 * SIG1 entry references the same range so the bootloader hashes
	 * exactly the bytes that were signed.
	 */
	.signature : {
		_boot_signature = ALIGN(4);
	} > FLASH

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
