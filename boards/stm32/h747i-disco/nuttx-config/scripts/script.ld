/****************************************************************************
 * boards/arm/stm32h7/stm32h747i-disco/scripts/flash.ld
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

/* The STM32H747XI has 2048Kb of main FLASH memory. The flash memory is
 * partitioned into a User Flash memory and a System Flash memory. Each
 * of these memories has two banks:
 *
 *   1) User Flash memory:
 *
 *      Bank 1: Start address 0x0800:0000 to 0x080F:FFFF with 8 sectors, 128Kb each
 *      Bank 2: Start address 0x0810:0000 to 0x081F:FFFF with 8 sectors, 128Kb each
 *
 *   2) System Flash memory:
 *
 *      Bank 1: Start address 0x1FF0:0000 to 0x1FF1:FFFF with 1 x 128Kb sector
 *      Bank 1: Start address 0x1FF4:0000 to 0x1FF5:FFFF with 1 x 128Kb sector
 *
 *   3) User option bytes for user configuration, only in Bank 1.
 *
 * In the STM32H747XI, two different boot spaces can be selected through
 * the BOOT pin and the boot base address programmed in the BOOT_ADD0 and
 * BOOT_ADD1 option bytes:
 *
 *   1) BOOT=0: Boot address defined by user option byte BOOT_ADD0[15:0].
 *      ST programmed value: Flash memory at 0x0800:0000
 *   2) BOOT=1: Boot address defined by user option byte BOOT_ADD1[15:0].
 *      ST programmed value: System bootloader at 0x1FF0:0000
 *
 * NuttX does not modify these option bytes. On the unmodified STM32H747I-DISCO
 * board, the BOOT0 pin is at ground so by default, the STM32 will boot
 * to address 0x0800:0000 in FLASH.
 *
 * The STM32H747XI also has 1024Kb of data SRAM.
 * SRAM is split up into several blocks and into three power domains:
 *
 *   1) TCM SRAMs are dedicated to the Cortex-M7 and are accessible with
 *      0 wait states by the Cortex-M7 and by MDMA through AHBS slave bus
 *
 *      1.1) 128Kb of DTCM-RAM beginning at address 0x2000:0000
 *
 *           The DTCM-RAM is organized as 2 x 64Kb DTCM-RAMs on 2 x 32 bit
 *           DTCM ports. The DTCM-RAM could be used for critical real-time
 *           data, such as interrupt service routines or stack / heap memory.
 *           Both DTCM-RAMs can be used in parallel (for load/store operations)
 *           thanks to the Cortex-M7 dual issue capability.
 *
 *      1.2)  64Kb of ITCM-RAM beginning at address 0x0000:0000
 *
 *           This RAM is connected to ITCM 64-bit interface designed for
 *           execution of critical real-times routines by the CPU.
 *
 *   2) AXI SRAM (D1 domain) accessible by all system masters except BDMA
 *      through D1 domain AXI bus matrix
 *
 *      2.1) 512Kb of SRAM beginning at address 0x2400:0000
 *
 *   3) AHB SRAM (D2 domain) accessible by all system masters except BDMA
 *      through D2 domain AHB bus matrix
 *
 *      3.1) 128Kb of SRAM1 beginning at address 0x3000:0000
 *      3.2) 128Kb of SRAM2 beginning at address 0x3002:0000
 *      3.3)  32Kb of SRAM3 beginning at address 0x3004:0000
 *
 *      SRAM1 - SRAM3 are one contiguous block: 288Kb at address 0x3000:0000
 *
 *   4) AHB SRAM (D3 domain) accessible by most of system masters
 *      through D3 domain AHB bus matrix
 *
 *      4.1)  64Kb of SRAM4 beginning at address 0x3800:0000
 *      4.1)   4Kb of backup RAM beginning at address 0x3880:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 */

MEMORY
{
  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH =   64K
  flash (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  dtcm1 (rwx) : ORIGIN = 0x20000000, LENGTH =   64K
  dtcm2 (rwx) : ORIGIN = 0x20010000, LENGTH =   64K
  sram  (rwx) : ORIGIN = 0x24000000, LENGTH =  512K
  sram1 (rwx) : ORIGIN = 0x30000000, LENGTH =  128K
  sram2 (rwx) : ORIGIN = 0x30020000, LENGTH =  128K
  sram3 (rwx) : ORIGIN = 0x30040000, LENGTH =   32K
  sram4 (rwx) : ORIGIN = 0x38000000, LENGTH =   64K
  bbram (rwx) : ORIGIN = 0x38800000, LENGTH =    4K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
SECTIONS
{
    .text :
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
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
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

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

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

    /* Emit the the D3 power domain section for locating BDMA data
     *
     * Static data with locate_data(".sram4") will be located
     * at start of SRAM4; the rest of SRAM4 will be added to the heap.
     */

    .sram4_reserve (NOLOAD) :
    {
        *(.sram4)
        . = ALIGN(4);
        _sram4_heap_start = ABSOLUTE(.);
    } > sram4

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
