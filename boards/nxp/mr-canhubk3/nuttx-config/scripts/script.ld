/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/scripts/flash.ld
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

/* Copyright 2022 NXP */

/* TO DO: ADD DESCRIPTION
 *
 *   0x00400000 - 0x007fffff  4194304  Program Flash (last 64K sBAF)
 *   0x10000000 - 0x1003ffff   262144  Data Flash (last 32K HSE_NVM)
 *   0x20400000 - 0x20408000    32768  Standby RAM_0 (32K)
 *   0x20400000 - 0x20427fff   163840  SRAM_0
 *   0x20428000 - 0x2044ffff   163840  SRAM_1
 *
 *   Last  48 KB of SRAM_1 reserved by HSE Firmware
 *   Last 128 KB of CODE_FLASH_3 reserved by HSE Firmware
 *   Last 128 KB of DATA_FLASH reserved by HSE Firmware (not supported in this linker file)
 *
 *   Note Standby RAM and SRAM overlaps in NuttX since we dont use the Standby functionality
 *
 */

MEMORY
{
  BOOT_HEADER (R)   : ORIGIN = 0x00400000, LENGTH = 0x00001000  /* 0x00400000 - 0x00400fff */
  flash       (rx)  : ORIGIN = 0x00401000, LENGTH = 0x003cffff  /* 0x00401000 - (0x007fffff - 0x20000 (128 KB) = 0x007dffff) */
  sram0_stdby (rwx) : ORIGIN = 0x20400000, LENGTH = 32K
  sram        (rwx) : ORIGIN = 0x20400000, LENGTH = 272K
  itcm        (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
  dtcm        (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(boot_header)
ENTRY(_stext)

SECTIONS
{

  .boot_header :
  {
    KEEP(*(.boot_header))
  } > BOOT_HEADER

  .text :
  {
    _stext = ABSOLUTE(.);
    *(.vectors)
    *(.text.__start)
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
  } >flash

  .ARM.exidx :
  {
    __exidx_start = ABSOLUTE(.);
    *(.ARM.exidx*)
    __exidx_end = ABSOLUTE(.);
  } >flash

  /* Due ECC initialization sequence __data_start__ and __data_end__ should be aligned on 8 bytes */
  .data :
  {
    . = ALIGN(8);
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(8);
    _edata = ABSOLUTE(.);
  } > sram AT > flash

  _eronly = LOADADDR(.data);

  .ramfunc ALIGN(8):
  {
    _sramfuncs = ABSOLUTE(.);
    *(.ramfunc  .ramfunc.*)
    _eramfuncs = ABSOLUTE(.);
  } > sram AT > flash

  _framfuncs = LOADADDR(.ramfunc);

  /* Due ECC initialization sequence __bss_start__ and __bss_end__ should be aligned on 8 bytes */
  .bss :
  {
    . = ALIGN(8);
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = ABSOLUTE(.);
  } > sram

  CM7_0_START_ADDRESS = ORIGIN(flash);

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

  SRAM_BASE_ADDR         = ORIGIN(sram);
  SRAM_END_ADDR          = ORIGIN(sram) + LENGTH(sram);
  SRAM_STDBY_BASE_ADDR   = ORIGIN(sram0_stdby);
  SRAM_STDBY_END_ADDR    = ORIGIN(sram0_stdby) + LENGTH(sram0_stdby);
  SRAM_INIT_END_ADDR     = ORIGIN(sram) + 320K;
  ITCM_BASE_ADDR         = ORIGIN(itcm);
  ITCM_END_ADDR          = ORIGIN(itcm) + LENGTH(itcm);
  DTCM_BASE_ADDR         = ORIGIN(dtcm);
  DTCM_END_ADDR          = ORIGIN(dtcm) + LENGTH(dtcm);
  FLASH_BASE_ADDR        = ORIGIN(BOOT_HEADER);
  FLASH_END_ADDR         = ORIGIN(flash) + LENGTH(flash);
}
