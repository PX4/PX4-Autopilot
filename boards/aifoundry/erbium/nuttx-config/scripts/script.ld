/****************************************************************************
 * boards/aifoundry/erbium/nuttx-config/scripts/script.ld
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Derived from NuttX boards/risc-v/erbium/minion/scripts/ld.script. PX4
 * links with this file directly, without the C preprocessor, so the
 * NuttX configuration values are written out: CONFIG_RAM_START is
 * 0x40000200 and CONFIG_RAM_SIZE 16776704, ending MRAM at 0x41000000, and
 * CONFIG_IDLETHREAD_STACKSIZE is 2048.
 */

EXTERN(__start)
ENTRY(__start)

PHDRS
{
  text PT_LOAD FLAGS(5);
  data PT_LOAD FLAGS(6);
}

SECTIONS
{
  /* The Erbium simulator reserves the first 512 bytes of MRAM. */
  . = 0x40000200;

  .rom_start : {
    _stext = .;
    KEEP(*(.text.__start))
  } :text

  . = ALIGN(4);

  .exceptions : {
    KEEP(*(.text.exceptions))
  } :text

  .text : {
    *(.text)
    *(.text.*)
    *(.gnu.warning)
    *(.stub)
    *(.gnu.linkonce.t.*)
    *(.init)
    *(.fini)
    _etext = .;
  } :text

  .rodata : {
    _srodata = .;
    *(.rodata)
    *(.rodata.*)
    *(.srodata .srodata.*)
    *(.gnu.linkonce.r*)
    _erodata = .;
  } :text

  .tdata : {
    _stdata = ABSOLUTE(.);
    *(.tdata .tdata.* .gnu.linkonce.td.*);
    _etdata = ABSOLUTE(.);
  } :data

  .tbss : {
    _stbss = ABSOLUTE(.);
    *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
    _etbss = ABSOLUTE(.);
  } :data

  _eronly = ABSOLUTE(.);

  .data : {
    _sdata = .;
    PROVIDE(__global_pointer$ = . + 0x800);
    *(.data)
    *(.data.*)
    *(.sdata .sdata.*)
    *(.gnu.linkonce.d*)
    . = ALIGN(8);
    _edata = .;
  } :data

  .init_array : {
    . = ALIGN(8);
    _sinit = ABSOLUTE(.);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array .ctors))
    _einit = ABSOLUTE(.);
  } :data

  .bss (NOLOAD) : {
    . = ALIGN(8);
    _sbss = .;
    *(.bss)
    *(.bss.*)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.b*)
    *(COMMON)
    . = ALIGN(16);
    _ebss = .;
  } :data

  ASSERT(_ebss + ALIGN(2048, 16) < 0x41000000,
         "Erbium image and idle stack exceed MRAM")

  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .comment 0 : { *(.comment) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_info 0 : { *(.debug_info) }
  .debug_line 0 : { *(.debug_line) }
  .debug_aranges 0 : { *(.debug_aranges) }
}
