/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
