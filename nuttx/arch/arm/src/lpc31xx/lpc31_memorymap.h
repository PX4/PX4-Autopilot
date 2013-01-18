/************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_memorymap.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* LPC31XX Physical (unmapped) Memory Map */

#define LPC31_FIRST_PSECTION         0x00000000 /* Beginning of the physical address space */
#define LPC31_SHADOWSPACE_PSECTION   0x00000000 /* 0x00000000-0x00000fff: Shadow Area 4Kb */
                                                /* 0x00001000-0xff027fff: Reserved */
#define LPC31_INTSRAM_PSECTION       0x11028000 /*                        Internal SRAM 0+1 192Kb */
#  define LPC31_INTSRAM0_PADDR       0x11028000 /* 0x11028000-0x1103ffff: Internal SRAM 0 96Kb */
#  define LPC31_INTSRAM1_PADDR       0x11040000 /* 0x11040000-0x11057fff: Internal SRAM 1 96Kb */
                                                /* 0x11058000-11ffffffff: Reserved */
#define LPC31_INTSROM0_PSECTION      0x12000000 /* 0x12000000-0x1201ffff: Internal SROM 0 128Kb */
                                                /* 0x12020000-0x12ffffff: Reserved */
#define LPC31_APB01_PSECTION         0x13000000 /* 0x13000000-0x1300bfff: APB0 32Kb APB1 16Kb */
#  define LPC31_APB0_PADDR           0x13000000 /* 0x13000000-0x13007fff: APB0 32Kb */
#  define LPC31_APB1_PADDR           0x13008000 /* 0x13008000-0x1300bfff: APB1 16Kb */
                                                /* 0x1300c000-0x14ffffff: Reserved */
#define LPC31_APB2_PSECTION          0x15000000 /* 0x15000000-0x15003fff: APB2 16Kb */
#define LPC31_APB3_PSECTION          0x16000000 /* 0x16000000-0x160003ff: APB3 1Kb */
#define LPC31_APB4MPMC_PSECTION      0x17000000 /*                        8Kb */
#  define LPC31_APB4_PADDR           0x17000000 /* 0x17000000-0x17000fff: APB4 4Kb */
#  define LPC31_MPMC_PADDR           0x17008000 /* 0x17008000-0x17008fff: MPMC cfg 4Kb */
                                                /* 0x17009000-0x17ffffff: Reserved */
#define LPC31_MCI_PSECTION           0x18000000 /* 0x18000000 0x180003ff: MCI/SD/SDIO 1Kb */
                                                /* 0x18000900-0x18ffffff: Reserved */
#define LPC31_USBOTG_PSECTION        0x19000000 /* 0x19000000-0x19000fff: USB OTG 4Kb */
                                                /* 0x19001000-0x1fffffff: Reserved */
#define LPC31_EXTSRAM_PSECTION       0x20000000 /*                        64-128Kb */
#  define LPC31_EXTSRAM0_PADDR       0x20000000 /* 0x20000000-0x2001ffff: External SRAM 0 64-128Kb */
#  define LPC31_EXTSRAM1_PADDR       0x20020000 /* 0x20020000-0x2003ffff: External SRAM 1 64-128Kb */
#define LPC31_EXTSDRAM0_PSECTION     0x30000000 /* 0x30000000-0x37ffffff: External SDRAM 0 128Mb */
                                                /* 0x40000000-0x5fffffff: Reserved */
#define LPC31_INTC_PSECTION          0x60000000 /* 0x60000000-0x60000fff: Interrupt controller 4Kb */
                                                /* 0x60001000-0x6fffffff: Reserved */
#define LPC31_NAND_PSECTION          0x70000000 /* 0x70000000-0x700007ff: NANDFLASH Ctrl 2Kb */
                                                /* 0x70000800-0xffffffff: Reserved */
#ifdef CONFIG_ARCH_EXTNAND                      /* End of the physical address space */
#  define LPC31_LAST_PSECTION        (LPC31_NAND_PSECTION + (1 << 20))
#else
#  define LPC31_LAST_PSECTION        (LPC31_INTC_PSECTION + (1 << 20))
#endif

/* APB0-4 Domain Offsets */

#define LPC31_APB0_EVNTRTR_OFFSET    0x00000000 /* Event Router */
#define LPC31_APB0_ADC_OFFSET        0x00002000 /* ADC 10-bit */
#define LPC31_APB0_WDT_OFFSET        0x00002400 /* WDT */
#define LPC31_APB0_SYSCREG_OFFSET    0x00002800 /* SYSCREG block */
#define LPC31_APB0_IOCONFIG_OFFSET   0x00003000 /* IOCONFIG */
#define LPC31_APB0_GCU_OFFSET        0x00004000 /* GCU */
#define LPC31_APB0_OTP_OFFSET        0x00005000 /* USB OTG (LPC315x only) */
#define LPC31_APB0_RNG_OFFSET        0x00006000 /* RNG */

#define LPC31_APB1_TIMER0_OFFSET     0x00000000 /* TIMER0 */
#define LPC31_APB1_TIMER1_OFFSET     0x00000400 /* TIMER1 */
#define LPC31_APB1_TIMER2_OFFSET     0x00000800 /* TIMER2 */
#define LPC31_APB1_TIMER3_OFFSET     0x00000c00 /* TIMER3 */
#define LPC31_APB1_PWM_OFFSET        0x00001000 /* PWM */
#define LPC31_APB1_I2C0_OFFSET       0x00002000 /* I2C0 */
#define LPC31_APB1_I2C1_OFFSET       0x00002400 /* I2C1 */

#define LPC31_APB2_PCM_OFFSET        0x00000000 /* PCM */
#define LPC31_APB2_LCD_OFFSET        0x00000400 /* LCD */
                                  /* 0x00000800    Reserved */
#define LPC31_APB2_UART_OFFSET       0x00001000 /* UART */
#define LPC31_APB2_SPI_OFFSET        0x00002000 /* SPI */
                                  /* 0x00003000    Reserved */

#define LPC31_APB3_I2SCONFIG_OFFSET  0x00000000 /* I2S System Configuration */
#define LPC31_APB3_I2STX0_OFFSET     0x00000080 /* I2S TX0 */
#define LPC31_APB3_I2STX1_OFFSET     0x00000100 /* I2S TX1 */
#define LPC31_APB3_I2SRX0_OFFSET     0x00000180 /* I2S RX0 */
#define LPC31_APB3_I2SRX1_OFFSET     0x00000200 /* I2S RX1 */
                                  /* 0x00000280    Reserved */

#define LPC31_APB4_DMA_OFFSET        0x00000000 /* DMA */
#define LPC31_APB4_NAND_OFFSET       0x00000800 /* NAND FLASH Controller */
                                  /* 0x00001000    Reserved */

/* Sizes of memory regions in bytes */

#define LPC31_SHADOWSPACE_SIZE      (4*1024)
#define LPC31_INTSRAM0_SIZE         (96*1024)
#define LPC31_INTSRAM1_SIZE         (96*1024)
#define LPC31_INTSROM0_SIZE         (128*1024)
#define LPC31_APB0_SIZE             (32*1024)
#define LPC31_APB1_SIZE             (16*1024)
#define LPC31_APB2_SIZE             (16*1024)
#define LPC31_APB3_SIZE             (1*1024)
#define LPC31_APB4_SIZE             (4*1024)
#define LPC31_MPMC_SIZE             (4*1024)
#define LPC31_APB4MPMC_SIZE         (LPC31_APB4_SIZE+LPC31_MPMC_SIZE)
#define LPC31_MCI_SIZE              (1*1024)
#define LPC31_USBOTG_SIZE           (4*1024)
#define LPC31_INTC_SIZE             (4*1024)
#define LPC31_NAND_SIZE             (2*1024)

#ifdef HAVE_INTSRAM1
#  define LPC31_ISRAM_SIZE        (LPC31_INTSRAM0_SIZE+LPC31_INTSRAM1_SIZE)
#else
#  define LPC31_ISRAM_SIZE        LPC31_INTSRAM0_SIZE
#endif

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)                 (((b)+0x000fffff) >> 20)

/* Sizes of sections/regions.  The boot logic in lpc31_boot.c, will select
 * 1Mb level 1 MMU mappings to span the entire physical address space.
 * The definitiions below specifiy the number of 1Mb entries that are
 * required to span a particular address region.
 */

#define LPC31_SHADOWSPACE_NSECTIONS  1 /*         4Kb - <1 section  */
#define LPC31_INTSRAM_NSECTIONS      1 /* 96 or 192Kb - <1 section */
#define LPC31_APB01_NSECTIONS        1 /*        32Kb - <1 section */
#define LPC31_INTSROM0_NSECTIONS     1 /*       128Kb - <1 section */
#define LPC31_APB1_NSECTIONS         1 /*        16Kb - <1 section */
#define LPC31_APB2_NSECTIONS         1 /*        16Kb - <1 section */
#define LPC31_APB3_NSECTIONS         1 /*         1Kb - <1 section */
#define LPC31_APB4MPMC_NSECTIONS     1 /*         8Kb - <1 section */
#define LPC31_MCI_NSECTIONS          1 /*         1Kb - <1 section */
#define LPC31_USBOTG_NSECTIONS       1 /*         4Kb - <1 section */
#define LPC31_EXTSRAM_NSECTIONS      1 /*    64-128Kb - <1 section */
#define LPC31_INTC_NSECTIONS         1 /*         4Kb - <1 section */
#define LPC31_NAND_NSECTIONS         1 /*         2Kb - <1 section */

/* External SDRAM is a special case -- the number of sections depends upon
 * the size of the SDRAM installed.
 */

#if defined(CONFIG_ARCH_EXTDRAM) && CONFIG_ARCH_EXTDRAMSIZE > 0
#  define LPC31_EXTSDRAM0_NSECTIONS  _NSECTIONS(CONFIG_ARCH_EXTDRAMSIZE)
#endif

/* Section MMU Flags */

#define LPC31_SHADOWSPACE_MMUFLAGS   MMU_ROMFLAGS
#define LPC31_INTSRAM_MMUFLAGS       MMU_MEMFLAGS
#define LPC31_INTSROM_MMUFLAGS       MMU_MEMFLAGS
#define LPC31_APB01_MMUFLAGS         MMU_IOFLAGS
#define LPC31_APB2_MMUFLAGS          MMU_IOFLAGS
#define LPC31_APB3_MMUFLAGS          MMU_IOFLAGS
#define LPC31_APB4MPMC_MMUFLAGS      MMU_IOFLAGS
#define LPC31_MCI_MMUFLAGS           MMU_IOFLAGS
#define LPC31_USBOTG_MMUFLAGS        MMU_IOFLAGS
#define LPC31_EXTSRAM_MMUFLAGS       MMU_MEMFLAGS
#define LPC31_EXTSDRAM_MMUFLAGS      MMU_MEMFLAGS
#define LPC31_INTC_MMUFLAGS          MMU_IOFLAGS
#define LPC31_NAND_MMUFLAGS          MMU_IOFLAGS

/* board_memorymap.h contains special mappings that are needed when a ROM
 * memory map is used.  It is included in this odd location becaue it depends
 * on some the virtual address definitions provided above.
 */

#include <arch/board/board_memorymap.h>

/* LPC31XX Virtual (mapped) Memory Map.  These are the mappings that will
 * be created if the page table lies in RAM.  If the platform has another,
 * read-only, pre-initialized page table (perhaps in ROM), then the board.h
 * file must provide these definitions.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
# define LPC31_FIRST_VSECTION        0x00000000 /* Beginning of the virtual address space */
#  define LPC31_SHADOWSPACE_VSECTION 0x00000000 /* 0x00000000-0x00000fff: Shadow Area 4Kb */
#  define LPC31_INTSRAM_VSECTION     0x11028000 /*                        Internal SRAM 96Kb-192Kb */
#    define LPC31_INTSRAM0_VADDR     0x11028000 /* 0x11028000-0x1103ffff: Internal SRAM 0 96Kb */
#    define LPC31_INTSRAM1_VADDR     0x11040000 /* 0x11040000-0x11057fff: Internal SRAM 1 96Kb */
#  define LPC31_INTSROM0_VSECTION    0x12000000 /* 0x12000000-0x1201ffff: Internal SROM 0 128Kb */
#  define LPC31_APB01_VSECTION       0x13000000 /* 0x13000000-0x1300bfff: APB0 32Kb APB0 32Kb */
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
#
#  ifdef CONFIG_ARCH_EXTNAND                    /* End of the virtual address space */
#    define LPC31_LAST_VSECTION      (LPC31_NAND_VSECTION + (1 << 20))
#  else
#    define LPC31_LAST_VSECTION      (LPC31_INTC_VSECTION + (1 << 20))
#  endif
#endif

/* The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, NuttX could be running from NOR FLASH,
 * SDRAM, external SRAM, or ISRAM.
 */

#if defined(CONFIG_BOOT_RUNFROMFLASH)
#  define NUTTX_START_VADDR            LPC31_MPMC_VADDR
#elif defined(CONFIG_BOOT_RUNFROMSDRAM)
#  define NUTTX_START_VADDR            LPC31_EXTSDRAM0_VSECTION
#elif defined(CONFIG_BOOT_RUNFROMEXTSRAM)
#  define NUTTX_START_VADDR            LPC31_EXTSRAM0_VADDR
#else /* CONFIG_BOOT_RUNFROMISRAM, CONFIG_PAGING */
#  define NUTTX_START_VADDR            LPC31_INTSRAM0_VADDR
#endif

/* Determine the address of the MMU page table.  We will try to place that page
 * table at the beginng of ISRAM0 if the vectors are at the high address, 0xffff:0000
 * or at the end of ISRAM1 (or ISRAM0 if ISRAM1 is not available in this architecture)
 * if the vectors are at 0x0000:0000
 *
 * Or... the user may specify the address of the page table explicitly be defining
 * CONFIG_PGTABLE_VADDR and CONFIG_PGTABLE_PADDR in the configuration or board.h file.
 */

#undef PGTABLE_IN_HIGHSRAM
#undef PGTABLE_IN_LOWSRAM

#if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)

  /* Sanity check.. if one is undefined, both should be undefined */

#  if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

  /* A sanity check, if the configuration says that the page table is read-only
   * and pre-initialized (maybe ROM), then it should have also defined both of
   * the page table base addresses.
   */

#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "CONFIG_ARCH_ROMPGTABLE defined; PGTABLE_BASE_P/VADDR not defined"
#  else

     /* If CONFIG_PAGING is selected, then parts of the 1-to-1 virtual memory
      * map probably do not apply because paging logic will probably partition
      * the SRAM section differently.  In particular, if the page table is located
      * at the end of SRAM, then the virtual page table address defined below
      * will probably be in error.
      *
      * We work around this header file interdependency by (1) insisting that
      * pg_macros.h be included AFTER this header file, then (2) allowing the
      * pg_macros.h header file to redefine PGTABLE_BASE_VADDR.
      */

#    if defined(CONFIG_PAGING) && defined(__ARCH_ARM_SRC_ARM_PG_MACROS_H)
#       error "pg_macros.h must be included AFTER this header file"
#    endif


     /* We must declare the page table in ISRAM0 or 1.  We decide depending upon
      * where the vector table was place.
      */

#    ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */

       /* In this case, ISRAM0 will be shadowed at address 0x0000:0000.  The page
        * table must lie at the top 16Kb of ISRAM1 (or ISRAM0 if this is a ISRAM1 is
        * not available in this architecture)
        */

#      ifdef HAVE_INTSRAM1
#          define PGTABLE_BASE_PADDR (LPC31_INTSRAM1_PADDR+LPC31_INTSRAM1_SIZE-PGTABLE_SIZE)
#          define PGTABLE_BASE_VADDR (LPC31_INTSRAM1_VADDR+LPC31_INTSRAM1_SIZE-PGTABLE_SIZE)
#      else
#          define PGTABLE_BASE_PADDR (LPC31_INTSRAM0_PADDR+LPC31_INTSRAM0_SIZE-PGTABLE_SIZE)
#          define PGTABLE_BASE_VADDR (LPC31_INTSRAM0_VADDR+LPC31_INTSRAM0_SIZE-PGTABLE_SIZE)
#      endif
#      define PGTABLE_IN_HIGHSRAM    1

       /* If CONFIG_PAGING is defined, insist that pg_macros.h assign the virtual
        * address of the page table.
        */

#      ifdef CONFIG_PAGING
#        undef PGTABLE_BASE_VADDR
#      endif
#    else

       /* Otherwise, ISRAM1 (or ISRAM0 for the is ISRAM1 is not available in this
        * architecture) will be mapped so that the end of the SRAM region will
        * provide memory for the vectors.  The page table will then be places at
        * the first 16Kb of ISRAM0 (which will be in the shadow memory region).
        */

#      define PGTABLE_BASE_PADDR     LPC31_SHADOWSPACE_PSECTION
#      define PGTABLE_BASE_VADDR     LPC31_SHADOWSPACE_VSECTION
#      define PGTABLE_IN_LOWSRAM     1
#    endif
#  endif
#endif

/* Page table start addresses:
 *
 * 16Kb of memory is reserved hold the page table for the virtual mappings.  A
 * portion of this table is not accessible in the virtual address space (for
 * normal operation). We will reuse this memory for coarse page tables as follows:
 *
 * NOTE: If CONFIG_PAGING is defined, pg_macros.h will re-assign the virtual
 * address of the page table.
 */

#define PGTABLE_L2_COARSE_OFFSET    ((((LPC31_LAST_PSECTION >> 20) + 255) & ~255) << 2)
#define PGTABLE_L2_COARSE_PBASE     (PGTABLE_BASE_PADDR+PGTABLE_L2_COARSE_OFFSET)
#define PGTABLE_L2_COARSE_VBASE     (PGTABLE_BASE_VADDR+PGTABLE_L2_COARSE_OFFSET)

#define PGTABLE_L2_FINE_OFFSET      ((((LPC31_LAST_PSECTION >> 20) + 1023) & ~1023) << 2)
#define PGTABLE_L2_FINE_PBASE       (PGTABLE_BASE_PADDR+PGTABLE_L2_FINE_OFFSET)
#define PGTABLE_L2_FINE_VBASE       (PGTABLE_BASE_VADDR+PGTABLE_L2_FINE_OFFSET)

/* Page table end addresses: */

#define PGTABLE_L2_END_PADDR        (PGTABLE_BASE_PADDR+PGTABLE_SIZE)
#define PGTABLE_L2_END_VADDR        (PGTABLE_BASE_VADDR+PGTABLE_SIZE)

/* Page table sizes */

#define PGTABLE_L2_COARSE_ALLOC     (PGTABLE_L2_END_VADDR-PGTABLE_L2_COARSE_VBASE)
#define PGTABLE_COARSE_TABLE_SIZE   (4*256)
#define PGTABLE_NCOARSE_TABLES      (PGTABLE_L2_COARSE_ALLOC / PGTABLE_COARSE_TABLE_SIZE)

#define PGTABLE_L2_FINE_ALLOC       (PGTABLE_L2_END_VADDR-PGTABLE_L2_FINE_VBASE)
#define PGTABLE_FINE_TABLE_SIZE     (4*1024)
#define PGTABLE_NFINE_TABLES        (PGTABLE_L2_FINE_ALLOC / PGTABLE_FINE_TABLE_SIZE)

/* Determine the base address of the vector table:
 *
 *   LPC31_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
 *   LPC31_VECTOR_VSRAM - Virtual address of vector table in SRAM
 *   LPC31_VECTOR_VADDR - Virtual address of vector table (0x00000000 or 0xffff0000)
 */

#define VECTOR_TABLE_SIZE           0x00010000
#ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */
#  define LPC31_VECTOR_PADDR      LPC31_INTSRAM0_PADDR
#  define LPC31_VECTOR_VSRAM      LPC31_INTSRAM0_VADDR
#  define LPC31_VECTOR_VADDR      0x00000000
#  define LPC31_VECTOR_VCOARSE    0x00000000
#else  /* Vectors located at 0xffff:0000 -- this probably does not work */
#  ifdef HAVE_INTSRAM1
#    define LPC31_VECTOR_PADDR    (LPC31_INTSRAM1_PADDR+LPC31_INTSRAM1_SIZE-VECTOR_TABLE_SIZE)
#    define LPC31_VECTOR_VSRAM    (LPC31_INTSRAM1_VADDR+LPC31_INTSRAM1_SIZE-VECTOR_TABLE_SIZE)
#  else
#    define LPC31_VECTOR_PADDR    (LPC31_INTSRAM0_PADDR+LPC31_INTSRAM0_SIZE-VECTOR_TABLE_SIZE)
#    define LPC31_VECTOR_VSRAM    (LPC31_INTSRAM0_VADDR+LPC31_INTSRAM0_SIZE-VECTOR_TABLE_SIZE)
#  endif
#  define LPC31_VECTOR_VADDR      0xffff0000
#  define LPC31_VECTOR_VCOARSE    0xfff00000
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_MEMORYMAP_H */
