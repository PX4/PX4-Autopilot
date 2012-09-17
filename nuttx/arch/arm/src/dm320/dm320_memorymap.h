/************************************************************************************
 * dm320/dm320_memorymap.h
 *
 *   Copyright (C) 2007, 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __DM320_MEMORYMAP_H
#define __DM320_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>
#include "arm.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Mapped base of all registers *****************************************************/

/* DM320 Physical Memory Map, where:
 *
 *   CW = cachable with write buffering
 *   -W = Write buffering only
 *   -- = Neither
 *
 * NOTE:
 * 1. Most DM320 memory sections can be programmed to lie at different locations in
 *    the memory map. Therefore, much of the DM320 physical memory map is really
 *    board-specific and, as such, really belongs in the configs/<board>/include/board.h
 *    file rather than here.
 *
 *    To handle all cases, this file defines a "default" physical memory map, but
 *    section address for most regions can be overriden if the same setting is
 *    defined in the board.h file (These defaults correspond to the product Neuros
 *    OSD memory configuration).
 *    
 * 2. The DM320 only has a single control line for external peripherals. To support
 *    more than one peripheral, most hardware will use external memory decode logic,
 *    so that physical memory regions is in the board-specific files.
 */

/* Section/Region Name             Phys Address    Size  TLB Enty        CW */
#define DM320_PERIPHERALS_PSECTION   0x00000000 /*   1Mb   1 section     -- */
#define   DM320_IRAM_PADDR           0x00000000 /*  16Kb   1 large page  CW */
#define   DM320_PERIPHERALS_PADDR    0x00030000 /*   4Kb   1 small pages -- */
#define   DM320_DSP_ONCHIP_RAM_PADDR 0x00040000 /* 128Kb   1 large page  -- */
#define   DM320_AHB_PADDR            0x00060000 /*   4Kb   1 small page  -- */
#define   DM320_COPRO_SUB_PADDR      0x00080000 /* 128Kb                 -- */

#ifndef DM320_FLASH_PSECTION
#  define DM320_FLASH_PSECTION       0x00100000 /*  16Mb  many sections  -- */
#  define   DM320_EXT_MEM_PADDR      0x00100000 /*  16Mb flash           -- */
#endif
#ifndef DM320_FLASH_PSECTION
#  define DM320_SDRAM_PSECTION       0x01100000 /* 496Mb  many section   -- */
#  define   DM320_SDRAM_PADDR        0x01100000 /* 496Mb  many sections  CW */
#endif
#ifndef DM320_CFI_PSECTION
#  define DM320_CFI_PSECTION         0x40000000 /*  16Mb  16 sections    -- */
#  define   DM320_CFI_PADDR          0x40000000 /*  16Mb  16 sections    -- */
#endif
#ifndef DM320_SSFDC_PSECTION
#  define DM320_SSFDC_PSECTION       0x48000000 /*  16Mb  16 sections    -- */
#  define   DM320_SSFDC_PADDR        0x48000000 /*  16Mb  16 sections    -- */
#endif
#ifndef DM320_CE1_PSECTION
#  define DM320_CE1_PSECTION         0x50000000 /*  16Mb  16 sections    -- */
#  define   DM320_CE1_PADDR          0x50000000 /*  16Mb  16 sections    -- */
#endif
#ifndef DM320_CE2_PSECTION
#  define DM320_CE2_PSECTION         0x60000000 /*  16Mb  16 sections    -- */
#  define   DM320_CE2_PADDR          0x60000000 /*  16Mb  16 sections    -- */
#endif

#define DM320_VLYNQ_PSECTION         0x70000000 /*  64MB  64 sections    -- */
#define   DM320_VLYNQ_PADDR          0x70000000 /*  64MB  64 sections    -- */
#define DM320_USBOTG_PSECTION        0x80000000 /*   1Mb   1 section     -- */
#define   DM320_USBOTG_PADDR         0x80000000 /*   1Kb   1 small page  -- */

/* Sizes of sections/regions */

/* Section / Region Name             Size */
#define DM320_PERIPHERALS_NSECTIONS  1          /*   1Mb   1 section     -- */
#define   DM320_IRAM_SIZE            (16*1024)
#define   DM320_PERIPHERALS_SIZE     (4*1024)
#define   DM320_DSP_ONCHIP_RAM_SIZE  (128*1024)
#define   DM320_AHB_SIZE             (4*1024)
#define   DM320_COPRO_SUB_SIZE       (128*1024)
#define DM320_FLASH_NSECTIONS        16         /*  16Mb  16 sections    -- */
#define   DM320_EXT_MEM_SIZE         (16*1024*1024)
#define DM320_CFI_NSECTIONS          16         /*  16Mb  16 sections    -- */
#define   DM320_CFI_SIZE             (16*1024*1024)
#define DM320_SSFDC_NSECTIONS        16         /*  16Mb  16 sections    -- */
#define   DM320_SSFDC_SIZE           (16*1024*1024) 
#define DM320_CE1_NSECTIONS          16         /*  16Mb  16 sections    -- */
#define   DM320_CE1_SIZE             (16*1024*1024)
#define DM320_CE2_NSECTIONS          16         /*  16Mb  16 sections    -- */
#define   DM320_CE2_SIZE             (16*1024*1024)
#define DM320_VLYNQ_NSECTIONS        64         /*  64MB  64 sections    -- */
#define   DM320_VLYNQ_SIZE           (64*1024*1024)
#define DM320_USBOTG_NSECTIONS       1          /*   1Mb   1 section     -- */
#define   DM320_USBOTG_SIZE          (1024)

/* DM320 Virtual Memory Map */

#if CONFIG_DRAM_VSTART != 0x00000000
# error "Invalid setting for CONFIG_DRAM_VSTART"
#endif

/* Section/Region Name               Virt Address  End        Size  CW */
#define DM320_SDRAM_VSECTION         0x00000000 /* 0x1effffff 496Mb CW */
#define   DM320_SDRAM_VADDR          0x00000000 /* 0x1effffff 496Mb CW */
                                  /* 0x1f000000    0xdfffffff UNMAPPED */
#define DM320_FLASH_VSECTION         0xc0000000 /* 0xc0ffffff  16Mb -- */
#define   DM320_EXT_MEM_VADDR        0xc0000000 /* 0xc0ffffff  16Mb -- */
#define DM320_CFI_VSECTION           0xc4000000 /* 0xc4ffffff  16Mb -- */
#define   DM320_CFI_VADDR            0xc4000000 /* 0xc4ffffff  16Mb -- */
#define DM320_SSFDC_VSECTION         0xc8000000 /* 0xc8ffffff  16Mb -- */
#define   DM320_SSFDC_VADDR          0xc8000000 /* 0xc8ffffff  16Mb -- */
#define DM320_CE1_VSECTION           0xcc000000 /* 0xccffffff  16Mb -- */
#define   DM320_CE1_VADDR            0xcc000000 /* 0xccffffff  16Mb -- */
#define DM320_CE2_VSECTION           0xd0000000 /* 0xd0ffffff  16Mb -- */
#define   DM320_CE2_VADDR            0xd0000000 /* 0xd0ffffff  16Mb -- */
#define DM320_USBOTG_VSECTION        0xd4000000 /* 0xd40fffff   1Mb -- */
#define   DM320_USBOTG_VADDR         0xd4000000 /* 0xd40003ff   1Kb -- */
#define DM320_VLYNQ_VSECTION         0xe0000000 /* 0xefffffff  64Mb -- */
#define   DM320_VLYNQ_VADDR          0xe0000000 /* 0xefffffff  64Mb -- */
#define DM320_PERIPHERALS_VSECTION   0xf0000000 /* 0xf00fffff   1Mb -- */
#define   DM320_IRAM_VADDR           0xf0000000 /* 0xf0003fff  16Kb -- */
#define   DM320_PERIPHERALS_VADDR    0xf0030000 /* 0xf0030fff   4Kb -- */
#define   DM320_DSP_ONCHIP_RAM_VADDR 0xf0040000 /* 0xf005ffff 128Kb -- */
#define   DM320_AHB_VADDR            0xf0060000 /* 0xf0060fff   4Kb -- */
#define   DM320_COPRO_SUB_VADDR      0xf0080000 /* 0xf009ffff 128Kb -- */
                                  /* 0xf0100000    0xffefffff UNMAPPED */
#define DM320_VECTOR_VCOARSE         0xfff00000 /* 0xffffffff   1Mb -- */
                                  /* 0xfff00000    0xfffeffff UNMAPPED */
#define   DM320_VECTOR_VADDR         0xffff0000 /* 0xffff3fff  16Kb -- */
                                  /* 0xffff4000    0xffffffff UNMAPPED */

/* The NuttX entry point starts at an offset from the virtual beginning of DRAM.
 * This offset reserves space for the MMU page cache.
 */

#define NUTTX_START_VADDR            (DM320_SDRAM_VADDR+PGTABLE_SIZE)

/* Section MMU Flags                 Flags              CW */
#define DM320_FLASH_MMUFLAGS         MMU_IOFLAGS     /* -- */
#define DM320_CFI_MMUFLAGS           MMU_IOFLAGS     /* -- */
#define DM320_SSFDC_MMUFLAGS         MMU_IOFLAGS     /* -- */
#define DM320_CE1_MMUFLAGS           MMU_IOFLAGS     /* -- */
#define DM320_CE2_MMUFLAGS           MMU_IOFLAGS     /* -- */
#define DM320_VLYNQ_MMUFLAGS         MMU_IOFLAGS     /* -- */
#define DM320_USBOTG_MMUFLAGS        MMU_IOFLAGS     /* -- */
#define DM320_PERIPHERALS_MMUFLAGS   MMU_IOFLAGS     /* -- */

/* 16Kb of memory is reserved at the beginning of SDRAM to hold the
 * page table for the virtual mappings.  A portion of this table is
 * not accessible in the virtual address space (for normal operation).
 * We will reuse this memory for coarse page tables as follows:
 * FIXME!  Where does that 0x00000800 come from.  I can't remember
 * and it does not feel right!
 */

#define PGTABLE_BASE_PADDR          DM320_SDRAM_PADDR
#define PGTABLE_SDRAM_PADDR         PGTABLE_BASE_PADDR
#define PGTABLE_L2_COARSE_PBASE     (PGTABLE_BASE_PADDR+0x00000800)
#define PGTABLE_L2_FINE_PBASE       (PGTABLE_BASE_PADDR+0x00001000)
#define PGTABLE_L2_END_PADDR        (PGTABLE_BASE_PADDR+PGTABLE_SIZE)

#define PGTABLE_BASE_VADDR          DM320_SDRAM_VADDR
#define PGTABLE_SDRAM_VADDR         PGTABLE_BASE_VADDR
#define PGTABLE_L2_COARSE_VBASE     (PGTABLE_BASE_VADDR+0x00000800)
#define PGTABLE_L2_FINE_VBASE       (PGTABLE_BASE_VADDR+0x00001000)
#define PGTABLE_L2_END_VADDR        (PGTABLE_BASE_VADDR+PGTABLE_SIZE)

/* Page table sizes */

#define PGTABLE_L2_COARSE_ALLOC     (PGTABLE_L2_END_VADDR-PGTABLE_L2_COARSE_VBASE)
#define PGTABLE_COARSE_TABLE_SIZE   (4*256)
#define PGTABLE_NCOARSE_TABLES      (PGTABLE_L2_COARSE_ALLOC / PGTABLE_COARSE_TABLE_SIZE)

#define PGTABLE_L2_FINE_ALLOC       (PGTABLE_L2_END_VADDR-PGTABLE_L2_FINE_VBASE)
#define PGTABLE_FINE_TABLE_SIZE     (4*1024)
#define PGTABLE_NFINE_TABLES        (PGTABLE_L2_FINE_ALLOC / PGTABLE_FINE_TABLE_SIZE)

/* This is the base address of the interrupt vectors on the ARM926 */

#define VECTOR_BASE                 DM320_VECTOR_VADDR

/* DM320 Peripheral Registers */

#define DM320_TIMER0_REGISTER_BASE  (DM320_PERIPHERALS_VADDR + 0x0000) /* Timer 0 */
#define DM320_TIMER1_REGISTER_BASE  (DM320_PERIPHERALS_VADDR + 0x0080) /* Timer 1 */
#define DM320_TIMER2_REGISTER_BASE  (DM320_PERIPHERALS_VADDR + 0x0100) /* Timer 2 */
#define DM320_TIMER3_REGISTER_BASE  (DM320_PERIPHERALS_VADDR + 0x0180) /* Timer 3 */
#define DM320_SERIAL0_REGISTER_BASE (DM320_PERIPHERALS_VADDR + 0x0200) /* Serial port 0 */
#define DM320_SERIAL1_REGISTER_BASE (DM320_PERIPHERALS_VADDR + 0x0280) /* Serial port 1 */
#define DM320_UART0_REGISTER_BASE   (DM320_PERIPHERALS_VADDR + 0x0300) /* UART 0 */
#define DM320_UART1_REGISTER_BASE   (DM320_PERIPHERALS_VADDR + 0x0380) /* UART 1 */
#define DM320_WDT_REGISTER_BASE     (DM320_PERIPHERALS_VADDR + 0x0400) /* Watchdog timer */
#define DM320_MMCSD_REGISTER_BASE   (DM320_PERIPHERALS_VADDR + 0x0480) /* MMC/SD */
#define DM320_INTC_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0500) /* Interrupt controller */
#define DM320_GIO_REGISTER_BASE     (DM320_PERIPHERALS_VADDR + 0x0580) /* GIO */
#define DM320_DSPC_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0600) /* DSP controller */
#define DM320_OSD_REGISTER_BASE     (DM320_PERIPHERALS_VADDR + 0x0680) /* OSD */
#define DM320_CCDC_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0700) /* CCD controller */
#define DM320_VENC_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0800) /* Video encoder */
#define DM320_CLKC_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0880) /* Clock controller */
#define DM320_BUSC_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0900) /* Bus controller */
#define DM320_SDRAMC_REGISTER_BASE  (DM320_PERIPHERALS_VADDR + 0x0980) /* SDRAM controller */
#define DM320_EMIF_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0A00) /* External memory interface */
#define DM320_PREV_REGISTER_BASE    (DM320_PERIPHERALS_VADDR + 0x0A80) /* Preview engine */
#define DM320_AF_REGISTER_BASE      (DM320_PERIPHERALS_VADDR + 0x0B80) /* Hardware 3A (AF/AE/AWB) */
#define DM320_MSTICK_REGISTER_BASE  (DM320_PERIPHERALS_VADDR + 0x0C80) /* Memory stick */
#define DM320_I2C_REGISTER_BASE     (DM320_PERIPHERALS_VADDR + 0x0D80) /* I2C */
#define DM320_USB_REGISTER_BASE     (DM320_USBOTG_VADDR + 0x0000)      /* USB full speed OTG */
#define DM320_USBDMA_REGISTER_BASE  (DM320_USBOTG_VADDR + 0x0200)      /* USB DMA */
#define DM320_VLYNQ_REGISTER_BASE   (DM320_AHB_VADDR + 0x0300)         /* VLYNQ */
#define DM320_AHBBUSC_REGISTER_BASE (DM320_AHB_VADDR + 0x0F00)         /* AHBBUSC */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __DM320_MEMORYMAP_H */
