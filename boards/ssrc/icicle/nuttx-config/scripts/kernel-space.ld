/****************************************************************************
 * boards/risc-v/mpfs/icicle/scripts/ld.script
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

/* NOTE:  This depends on the memory.ld script having been included prior to
 * this script.
 */
 
OUTPUT_ARCH("riscv")

/* Provide these so there is no need for using config files for this */

__uflash_start = ORIGIN(uflash);
__uflash_size = LENGTH(uflash);
__usram_start = ORIGIN(usram);
__usram_size = LENGTH(usram);

/* Provide the kernel boundaries as well */

__kflash_start = ORIGIN(kflash);
__kflash_size = LENGTH(kflash);
__ksram_start = ORIGIN(ksram);
__ksram_size = LENGTH(ksram);
__ksram_end = ORIGIN(ksram) + LENGTH(ksram);

ENTRY(_stext)

EXTERN(__start)

SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
        *(.start .start.*)
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
    } > kflash

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > kflash

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
    } > ksram AT > kflash

    PROVIDE(__global_pointer$ = _sdata + ((_edata - _sdata) / 2));

    .bss : ALIGN(4) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(.gnu.linkonce.b.*)
        *(.gnu.linkonce.sb.*)
        *(COMMON)
    } > ksram

    /* Page tables here, align to 4K boundary */
    .pgtables : ALIGN(0x1000) {
        *(.pgtables)
         . = ALIGN(32);
        _ebss = ABSOLUTE(.);
    } > ksram

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
