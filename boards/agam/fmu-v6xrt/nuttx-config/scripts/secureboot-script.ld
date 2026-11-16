/****************************************************************************
 * boards/agam/fmu-v6xrt/nuttx-config/scripts/secureboot-script.ld
 *
 * Linker script for the agam_fmu-v6xrt_secureboot variant.
 *
 * Same layout as script.ld, except:
 *   - The image table-of-contents (.main_toc, defined by src/toc.c)
 *     is placed at a fixed offset past the vector table so the
 *     bootloader can find it via BOARD_IMAGE_TOC_OFFSET (0x2800).
 *     script.ld reserves 0x0-0x2000 for the FlexSPI boot header
 *     (.boot_hdr: IVT/boot data/DCD) and starts .vectors at 0x2000,
 *     so the TOC is placed inside the .vectors output section, right
 *     after the real vector table and before any other code.
 *   - An empty .signature output section reserves the symbol
 *     _boot_signature at the end of the FLASH content. The TOC
 *     references it as the signed-image end address; sign_firmware.py
 *     appends the 64-byte ed25519 signature there.
 *
 ****************************************************************************/

/* Specify the memory areas */

MEMORY
{
  flash (rx)  : ORIGIN = 0x30020000, LENGTH = 4M-128K /* We have 64M but we do not want to wait to program it all */
  sram  (rwx) : ORIGIN = 0x20240000, LENGTH = 2M-256k
  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 256K   /* TODO FlexRAM partition */
  dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(g_flash_config)
EXTERN(g_image_vector_table)
EXTERN(g_boot_data)
EXTERN(board_get_manifest)
EXTERN(imxrt_flexspi_initialize)
EXTERN(_main_toc)

ENTRY(__start)

SECTIONS
{
    /* Image Vector Table and Boot Data for booting from external flash */

    .boot_hdr : ALIGN(4)
    {
        FILL(0xff)
        . = 0x400 ;
        __boot_hdr_start__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.conf))
        . = 0x1000 ;
        KEEP(*(.boot_hdr.ivt))
        . = 0x1020 ;
        KEEP(*(.boot_hdr.boot_data))
        . = 0x1030 ;
        KEEP(*(.boot_hdr.dcd_data))
        __boot_hdr_end__ = ABSOLUTE(.) ;
        . = 0x2000 ;
    } >flash

    .vectors :
    {
        KEEP(*(.vectors))
        /* Place the image TOC at a fixed offset past the vector table
         * so the bootloader can find it via BOARD_IMAGE_TOC_OFFSET.
         * 0x2800 is 2 KiB past the vector table (936 B).
         */
        . = ABSOLUTE(ORIGIN(flash) + 0x2800);
        KEEP(*(.main_toc))
        *(.text .text.__start)
    } >flash

    .itcmfunc :
    {
        . = ALIGN(8);
        _sitcmfuncs = ABSOLUTE(.);
        FILL(0xFF)
        . = 0x40 ;
        *(.ram_vectors)
        INCLUDE "itcm_static_functions.ld"
        INCLUDE "itcm_functions_includes.ld"
        . = ALIGN(8);
        _eitcmfuncs = ABSOLUTE(.);
    } > itcm AT > flash

    _fitcmfuncs = LOADADDR(.itcmfunc);

    .text : ALIGN(4)
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
		. = ALIGN(32);
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        . = ALIGN(4096);
        _etext = ABSOLUTE(.);
        _srodata = ABSOLUTE(.);
        *(.rodata .rodata.*)
        . = ALIGN(4096);
        _erodata = ABSOLUTE(.);
    } > flash

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > flash

    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > flash

    _eronly = ABSOLUTE(.);

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    .ramfunc ALIGN(4):
    {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > sram AT > flash

    _framfuncs = LOADADDR(.ramfunc);

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

    /* Marker for the end of the signed image. sign_firmware.py
     * appends the 64-byte ed25519 signature at this address; the TOC's
     * SIG1 entry references the same range so the bootloader hashes
     * exactly the bytes that were signed.
     */
    .signature : {
        _boot_signature = ALIGN(4);
    } > flash

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

    _boot_loadaddr = ORIGIN(flash);
    _boot_size = LENGTH(flash);
    _ram_size = LENGTH(sram);
    _sdtcm = ORIGIN(dtcm);
    _edtcm = ORIGIN(dtcm) + LENGTH(dtcm);
}
