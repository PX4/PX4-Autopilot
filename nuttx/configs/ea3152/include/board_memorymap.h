/************************************************************************************
 * configs/ea3152/include/board_memorymap.h
 * include/arch/board/board_memorymap.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __ARCH_BOARD_BOARD_MEMORYMAP_H
#define __ARCH_BOARD_BOARD_MEMORYMAP_H

/* This file should never be included directly, but only indirectly via
 * lpc31_memorymap.h.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* If the LPC31xx ROM page table is selected, then the board-logic is required
 * to provide:
 *
 *   PGTABLE_BASE_PADDR - The physical address of the page table in ROM,
 *   PGTABLE_BASE_VADDR - The mapped address of the page table in ROM, and
 *   Mappings for each of the PSECTIONS in lpc31_memorymap.h
 */
 
#ifdef CONFIG_ARCH_ROMPGTABLE
   /* The LPC31xx ROM page table uses a 1-1 physical to virtual memory mapping */

#  define LPC31_SHADOWSPACE_VSECTION 0x00000000 /* 0x00000000-0x00000fff: Shadow Area 4Kb */
#  define LPC31_INTSRAM_VSECTION     0x11028000 /*                        Internal SRAM 96Kb-192Kb */
#    define LPC31_INTSRAM0_VADDR     0x11028000 /* 0x11028000-0x1103ffff: Internal SRAM 0 96Kb */
#    define LPC31_INTSRAM1_VADDR     0x11040000 /* 0x11040000-0x11057fff: Internal SRAM 1 96Kb */
#  define LPC31_INTSROM0_VSECTION    0x12000000 /* 0x12000000-0x1201ffff: Internal SROM 0 128Kb */
#  define LPC31_APB01_VSECTION       0x13000000 /* 0x13000000-0x1300bfff: APB0 32Kb  APB1 16Kb*/
#    define LPC31_APB0_VADDR         0x13000000 /* 0x13000000-0x13007fff: APB0 32Kb */
#    define LPC31_APB1_VADDR         0x13008000 /* 0x13008000-0x1300bfff: APB1 16Kb */
#  define LPC31_APB2_VSECTION        0x15000000 /* 0x15000000-0x15003fff: APB2 16Kb */
#  define LPC31_APB3_VSECTION        0x16000000 /* 0x16000000-0x160003ff: APB3 1Kb */
#  define LPC31_APB4MPMC_VSECTION    0x17000000 /*                        8Kb */
#    define LPC31_APB4_VADDR         0x17000000 /* 0x17000000-0x17000fff: APB4 4Kb */
#    define LPC31_MPMC_VADDR         0x17008000 /* 0x17008000-0x17008fff: MPMC cfg 4Kb */
#  define LPC31_MCI_VSECTION         0x18000000 /* 0x18000000 0x180003ff: MCI/SD/SDIO 1Kb */
#  define LPC31_USBOTG_VSECTION      0x19000000 /* 0x19000000-0x19000fff: USB OTG 4Kb */
#  define LPC31_EXTSRAM_VSECTION     0x20020000 /*                        64-128Kb */
#    define LPC31_EXTSRAM0_VADDR     0x20000000 /* 0x20000000-0x2001ffff: External SRAM 0 64-128Kb */
#    define LPC31_EXTSRAM1_VADDR     0x20020000 /* 0x20020000-0x2003ffff: External SRAM 1 64-128Kb */
#  define LPC31_EXTSDRAM0_VSECTION   0x30000000 /* 0x30000000-0x37ffffff: External SDRAM 0 128Mb */
#  define LPC31_INTC_VSECTION        0x60000000 /* 0x60000000-0x60000fff: Interrupt controller 4Kb */
#  define LPC31_NAND_VSECTION        0x70000000 /* 0x70000000-0x700007ff: NANDFLASH Ctrl 2Kb */

   /* Define the address of the page table within the ROM */

#  define ROMPGTABLE_OFFSET          0x0001c000  /* Offset of the ROM page table in ROM */
#  define PGTABLE_BASE_PADDR         (LPC31_INTSROM0_PSECTION+ROMPGTABLE_OFFSET)
#  define PGTABLE_BASE_VADDR         (LPC31_INTSROM0_VSECTION+ROMPGTABLE_OFFSET)
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_MEMORYMAP_H */
