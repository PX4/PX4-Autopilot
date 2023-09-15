/****************************************************************************
 * boards/risc-v/icicle/mpfs/scripts/ld.script
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

MEMORY
{
  progmem (rx) : ORIGIN = 0xACA00000, LENGTH = 6M  /* w/ cache */
  sram   (rwx) : ORIGIN = 0xAD000000, LENGTH = 4M  /* w/ cache */
}

OUTPUT_ARCH("riscv")

ENTRY(__start)
EXTERN(__start)
EXTERN(_main_toc)
EXTERN(_main_toc_sig)
EXTERN(board_get_manifest)

SECTIONS
{
    /* Provide dummy relocation symbols for the dummy ToC */
    PROVIDE(_toc_start = 0);
    PROVIDE(_toc_end = 0);
    PROVIDE(_toc_signature = 0);
    PROVIDE(_app_start = 0);
    PROVIDE(_app_end = 0);
    PROVIDE(_boot_signature = 0);

    /* Make a hole for the ToC and signature */
    .toc (NOLOAD) : {
        *(.main_toc)
        *(.main_toc_sig)
        FILL(0xff);
        . = ALIGN(8);
    } > progmem

    .start : {
        *(.start .start.*)
    } > progmem

    .text : {
        _stext = ABSOLUTE(.);
        /*
        This signature provides the bootloader with a way to delay booting
        */
        . = 0x400;
        _bootdelay_signature = ABSOLUTE(.);
        FILL(0x01ecc2925d7d05c5)
        . += 8;
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.* .srodata .srodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > progmem

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > progmem

    _eronly = ABSOLUTE(.);

    .data : ALIGN(4) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.sdata .sdata.* .sdata2.*)
        *(.gnu.linkonce.d.*)
        *(.gnu.linkonce.s.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > progmem

    PROVIDE(__global_pointer$ = _sdata + ((_edata - _sdata) / 2));

    .bss : ALIGN(4) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(.gnu.linkonce.b.*)
        *(.gnu.linkonce.sb.*)
        *(COMMON)
        . = ALIGN(32);
        _ebss = ABSOLUTE(.);
    } > sram

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
