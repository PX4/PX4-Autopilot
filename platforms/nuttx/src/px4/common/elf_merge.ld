/*
 * Linker script for loadable ELF modules.
 * Merges per-function/data sections (from -ffunction-sections -fdata-sections)
 * into standard sections that the NuttX ELF loader can handle.
 */

SECTIONS
{
    .text : { *(.text .text.*) }
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) }
    .data : { *(.data .data.* .sdata .sdata.*) }
    .bss : { *(.bss .bss.* .sbss .sbss.* COMMON) }
}
