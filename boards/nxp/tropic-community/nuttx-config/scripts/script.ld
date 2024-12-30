/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/scripts/flash.ld
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

/* Specify the memory areas */

MEMORY
{
  flash (rx)  : ORIGIN = 0x60000000, LENGTH = 7936K - 128K
  sram  (rwx) : ORIGIN = 0x20200000, LENGTH = 512K
  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 384K
  dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(g_flash_config)
EXTERN(g_image_vector_table)
EXTERN(g_boot_data)

ENTRY(_stext)

SECTIONS
{
    /* Image Vector Table and Boot Data for booting from external flash */

    .boot_hdr : ALIGN(4)
    {
        FILL(0xff)
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
        *(.text .text.__start)
    } >flash

    .itcmfunc :
    {
        . = ALIGN(8);
        _sitcmfuncs = ABSOLUTE(.);
        FILL(0xFF)
        . = 0x40 ;
        INCLUDE "itcm_static_functions.ld"
        INCLUDE "itcm_functions_includes.ld"
        . = ALIGN(8);
        _eitcmfuncs = ABSOLUTE(.);
    } > itcm AT > flash

    _fitcmfuncs = LOADADDR(.itcmfunc);

    /* The RAM vector table (if present) should lie at the beginning of SRAM */

    .ram_vectors (COPY) : {
        *(.ram_vectors)
    } > dtcm


    /* Workaround for ethernet issue, by placing g_desc_pool into DTCM,
       which effectively puts it into a no-cache region */
    .dtcm : {
        *(.bss.g_desc_pool)
    } > dtcm

    .text :
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
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
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array .ctors))
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
    _sdtcm = ORIGIN(dtcm);
    _edtcm = ORIGIN(dtcm) + LENGTH(dtcm);
}
