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


/*-----------------------------------------------------------------------------

  -- PolarFire SoC Memorp map


     0x0800_0000  +-----------------------+
                  |   DTIM  Len=8K        |
                  +-----------------------+

     0x0180_0000  +-----------------------+
                  |   ITIM Hart 0         |
                  +-----------------------+
                                                        eNVM detail
     0x0180_8000  +-----------------------+        +-------------------+   -
                  |   ITIM Hart 1 Len=28K |        |Sector 2           |   ^
                  +-----------------------+        |0x2022_0000 8K     |   |
                                                   +-------------------+   |
     0x0181_0000  +-----------------------+        |Sector 0           |   1
                  |   ITIM Hart 2 Len=28K |        |0x2022_2000 56K    |   2
                  +-----------------------+        +-------------------+   8
                                                   |Sector 1           |   k
     0x0181_8000  +-----------------------+        |0x2022_3000 56K    |   |
                  |   ITIM Hart 3 Len=28K |        +-------------------+   |
                  +-----------------------+        |Sector 3           |   |
                                                   |0x2022_3E00 8K     |   v
     0x0182_0000  +-----------------------+        +-------------------+   -
                  |   ITIM Hart 4 Len=28K |
                  +-----------------------+

     0x0800_0000  +-----------------------+
                  |   LIM  Len max=1920K  |  +------>+------+
                  +-----------------------+  |       |      |
                                             |       |      |
     0x2022_0000  +-----------------------+  |       |      |
                  |   eNVM 128K           |  |       |      |
                  +-----------------------+  |       |      |
                                             |       |      |
     0x8000_0000  +-----------------------+--+       |  DDR |
  0x10_0000_0000  | DDR cached            |          |      |
                  |   SEG0                |     +--->|      |
                  +-----------------------+     |    |      |
                                                |    |      |
     0xC000_0000  +-----------------------+-----+    |      |
  0x14_0000_0000  | DDR non-cached        |          |      |
                  |    SEG1               |          |      |
                  +-----------------------+      +-->+      |
                                                 |   |      |
     0xD000_0000  +-----------------------+------+   |      |
  0x18_0000_0000  | Write Combine buffer  |          +------+
                  |    SEG1               |
                  +-----------------------+
  ----------------------------------------------------------------------------*/

MEMORY
{
    ddr (rx)  : ORIGIN = 0x80000000, LENGTH = 2M    /* w/ cache */
    envm (rx) : ORIGIN = 0x20220100, LENGTH = 128K - 256  /* 256 reserved for hss headers */
    lim (rwx) : ORIGIN  = 0x08000000, LENGTH = 1024k
}

OUTPUT_ARCH("riscv")

ENTRY(_stext)

EXTERN(exception_common) /* todo: what was the magic with this again? */
EXTERN(_main_toc)

SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
        *(.text.vec)
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
    } > envm

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > envm

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
    } > lim AT > envm

    PROVIDE(__global_pointer$ = _sdata + ((_edata - _sdata) / 2));

    .bss : ALIGN(4) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(.gnu.linkonce.b.*)
        *(.gnu.linkonce.sb.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
        . = ALIGN(32);
        _default_stack_limit = ABSOLUTE(.);
    } > lim

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
