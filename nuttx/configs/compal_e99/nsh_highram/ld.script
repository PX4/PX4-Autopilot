/*
 * Linker script for running from internal SRAM on Compal phones
 *
 * This script is tailored specifically to the requirements imposed
 * on us by the Compal bootloader.
 *
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(__start)
MEMORY
{
    /* 0x800000-0xa00000 */
    /* compal-loaded binary: our text, initialized data */
    LRAM (rw) : ORIGIN = 0x00800000, LENGTH = 0x00020000
    TRAM (rw) : ORIGIN = 0x00820000, LENGTH = 0x0001d000
    /* compal-loaded binary: our unitialized data, stacks, heap */
    IRAM (rw) : ORIGIN = 0x0083d000, LENGTH = 0x00002000
}
SECTIONS
{
    . = 0x800000;

    /* romloader data section, contains passthru interrupt vectors */
    .compal.loader (NOLOAD) : { . = 0x100; } > LRAM

    /* image signature (prepended by osmocon according to phone type) */
    .compal.header (NOLOAD) : { . = 4; } > LRAM

    /* initialization code */
    . = ALIGN(4);
    .text.start : {
        PROVIDE(__start = .);
        KEEP(*(.text.start))
        *(.text.start)
    } > TRAM

    /* exception vectors from 0x80001c to 0x800034 */
    .text.exceptions 0x80001c : AT (LOADADDR(.text.start) + SIZEOF(.text.start)) {
        KEEP(*(.text.exceptions))
        * (.text.exceptions)
        . = ALIGN(4);
    } > LRAM
    PROVIDE(_exceptions = LOADADDR(.text.exceptions));

    /* code */
    . = ALIGN(4);
    .text (LOADADDR(.text.exceptions) + SIZEOF(.text.exceptions)) :
          AT (LOADADDR(.text.exceptions) + SIZEOF(.text.exceptions)) {
        /* regular code */
        *(.text*)
        /* always-in-ram code */
        *(.ramtext*)
        /* gcc voodoo */
        *(.glue_7t) *(.glue_7) *(.vfp11_veneer) *(.v4_bx)
        . = ALIGN(4);
    } > TRAM
    PROVIDE(_text_start = LOADADDR(.text));
    PROVIDE(_text_end = LOADADDR(.text) + SIZEOF(.text));

    /* constructor pointers */
    .ctors : {
        /* ctor count */
        LONG(SIZEOF(.ctors) / 4 - 2)
        /* ctor pointers */
        KEEP(*(SORT(.ctors)))
        /* end of list */
        LONG(0)
    } > TRAM
    PROVIDE(_ctor_start = LOADADDR(.ctors));
    PROVIDE(_ctor_end = LOADADDR(.ctors) + SIZEOF(.ctors));

    /* destructor pointers */
    .dtors : {
        /* dtor count */
        LONG(SIZEOF(.dtors) / 4 - 2)
        /* dtor pointers */
        KEEP(*(SORT(.dtors)))
        /* end of list */
        LONG(0)
    } > TRAM
    PROVIDE(_dtor_start = LOADADDR(.dtors));
    PROVIDE(_dtor_end = LOADADDR(.dtors) + SIZEOF(.dtors));

    /* read-only data */
    . = ALIGN(4);
    .rodata : {
        *(.rodata*)
    } > TRAM
    PROVIDE(_rodata_start = LOADADDR(.rodata));
    PROVIDE(_rodata_end = LOADADDR(.rodata) + SIZEOF(.rodata));

    /* initialized data */
    . = ALIGN(4);
    .data : {
        *(.data)
    } > TRAM
    PROVIDE(_data_start = LOADADDR(.data));
    PROVIDE(_data_end = LOADADDR(.data) + SIZEOF(.data));

    /* pic offset tables */
    . = ALIGN(4);
    .got : {
        *(.got)
        *(.got.plt) *(.igot.plt) *(.got) *(.igot)
    } > TRAM
    PROVIDE(_got_start = LOADADDR(.got));
    PROVIDE(_got_end = LOADADDR(.got) + SIZEOF(.got));

    /* uninitialized data */
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        _sbss = ABSOLUTE(.);
        *(.bss)
        _ebss = ABSOLUTE(.);
    } > IRAM
    . = ALIGN(4);
    __bss_end = .;
    PROVIDE(_bss_start = __bss_start);
    PROVIDE(_bss_end = __bss_end);

    /* end of image */
    . = ALIGN(4);
    _end = .;
    PROVIDE(end = .);
}
