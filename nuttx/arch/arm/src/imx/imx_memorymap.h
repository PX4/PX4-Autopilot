/************************************************************************************
 * arch/arm/src/imx/imx_memorymap.h
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

#ifndef __ARCH_ARM_IMX_MEMORYMAP_H
#define __ARCH_ARM_IMX_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "arm.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Physical Memory Map **************************************************************/

                                             /* -0x000fffff Double Map Image    1Mb */
                                             /* -0x001fffff Bootstrap ROM       1Mb */
#define IMX_PERIPHERALS_PSECTION  0x00200000 /* -0x002fffff Peripherals         1Mb */
#define IMX_ESRAM_PSECTION        0x00300000 /* -0x003fffff Embedded SRAM     128Kb */
#define IMX_SDRAM0_PSECTION       0x08000000 /* -0x0bffffff SDRAM0 (CSD0)      64Mb */
#define IMX_SDRAM1_PSECTION       0x0c000000 /* -0x0fffffff SDRAM1 (CSD1)      64Mb */
#define IMX_FLASH_PSECTION        0x10000000 /* -0x11ffffff FLASH  (CS0)       32Mb */
#define IMX_CS1_PSECTION          0x12000000 /* -0x12ffffff CS1                16Mb */
#define IMX_CS2_PSECTION          0x13000000 /* -0x13ffffff CS2                16Mb */
#define IMX_CS3_PSECTION          0x14000000 /* -0x14ffffff CS3                16Mb */
#define IMX_CS4_PSECTION          0x15000000 /* -0x15ffffff CS4                16Mb */
#define IMX_CS5_PSECTION          0x16000000 /* -0x16ffffff CS5                16Mb */

/* Sizes of Address Sections ********************************************************/

/* Mapped sections */
#define IMX_PERIPHERALS_NSECTIONS 1          /*  1Mb  1 section                     */
#define IMX_SDRAM0_NSECTIONS      16         /* 16Mb Based on CONFIG_DRAM_SIZE      */
#define IMX_SDRAM1_NSECTIONS      0          /* 64Mb (Not mapped)                   */
#define IMX_FLASH_NSECTIONS       32         /* 64Mb Based on CONFIG_FLASH_SIZE     */
#define IMX_CS1_NSECTIONS         16         /* 16Mb                                */
#define IMX_CS2_NSECTIONS         16         /* 16Mb                                */
#define IMX_CS3_NSECTIONS         16         /* 16Mb                                */
#define IMX_CS4_NSECTIONS         16         /* 16Mb                                */
#define IMX_CS5_NSECTIONS         16         /* 16Mb                                */

/* Virtual Memory Map ***************************************************************/

/* There are three operational memory configurations:
 *
 * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case:
 *
 *    - Our vectors must be located at the beginning of FLASH and will
 *      also be mapped to address zero (because of the i.MX's "double map image."
 *    - All vector addresses are FLASH absolute addresses,
 *    - DRAM cannot reside at address zero,
 *    - Vectors at address zero (CR_V is not set),
 *    - The boot logic must configure SDRAM and, 
 *    - The .data section in RAM must be initialized.
 *
 * 2. We boot in FLASH but copy ourselves to DRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case:
 *
 *    - Our code image is in FLASH and we boot to FLASH initially, then copy
 *      ourself to DRAM,
 *    - DRAM will be mapped to address zero,
 *    - The RESET vector is a FLASH absolute address,
 *    - All other vectors are absulte and reference functions in the final mapped SDRAM address
 *    - Vectors at address zero (CR_V is not set), and
 *    - The boot logic must configure SDRAM.
 *
 * 3. There is bootloader that copies us to DRAM, but probably not to the beginning
 *    of DRAM (say to 0x0900:0000) (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n).
 *    In this case:
 *
 *    - DRAM will be mapped to address zero,
 *    - Interrupt vectors will be copied to address zero,
 *    - Memory between the end of the vector area (say 0x0800:0400) and the beginning
 *      of the page table (0x0900:0000) will be given to the memory manager as a second
 *      memory region,
 *    - All vectors are absulte and reference functions in the final mapped SDRAM address
 *    - Vectors at address zero (CR_V is not set), and
 *    - We must assume that the bootloader has configured SDRAM.
 */

#ifdef CONFIG_BOOT_RUNFROMFLASH
   /* Use the identity mapping */

#  define IMX_SDRAM_VSECTION      0x08000000 /* -(+CONFIG_DRAM_SIZE)                */
#else
   /* Map SDRAM to address zero */

#  define IMX_SDRAM_VSECTION      0x00000000 /* -(+CONFIG_DRAM_SIZE)                */
#endif

/* We use a identity mapping for other regions */

#define IMX_PERIPHERALS_VSECTION  0x00200000 /* -0x002fffff                     1Mb */
#define IMX_FLASH_VSECTION        0x10000000 /* -(+CONFIG_FLASH_SIZE)               */
#define IMX_CS1_VSECTION          0x12000000 /* -0x12ffffff CS1                32Mb */
#define IMX_CS2_VSECTION          0x13000000 /* -0x13ffffff CS2                32Mb */
#define IMX_CS3_VSECTION          0x14000000 /* -0x14ffffff CS3                32Mb */
#define IMX_CS4_VSECTION          0x15000000 /* -0x15ffffff CS4                32Mb */
#define IMX_CS5_VSECTION          0x16000000 /* -0x16ffffff CS5                32Mb */

/* In any event, the vector base address is 0x0000:0000 */

#define VECTOR_BASE               0x00000000

/* Peripheral Register Offsets ******************************************************/

#define IMX_AIPI1_OFFSET          0x00000000 /* -0x00000fff AIPI1               4Kb */
#define IMX_WDOG_OFFSET           0x00001000 /* -0x00001fff WatchDog            4Kb */          
#define IMX_TIMER1_OFFSET         0x00002000 /* -0x00002fff Timer1              4Kb */
#define IMX_TIMER2_OFFSET         0x00003000 /* -0x00003fff Timer2              4Kb */
#define IMX_RTC_OFFSET            0x00004000 /* -0x00004fff RTC                 4Kb */
#define IMX_LCDC_OFFSET           0x00005000 /* -0x00005fff LCD                 4Kb */
#define   IMX_LCDC_COLORMAP       0x00005800
#define IMX_UART1_OFFSET          0x00006000 /* -0x00006fff UART1               4Kb */
#define IMX_UART2_OFFSET          0x00007000 /* -0x00007fff UART2               4Kb */
#define IMX_PWM1_OFFSET           0x00008000 /* -0x00008fff PWM                 4Kb */
#define IMX_DMA_OFFSET            0x00009000 /* -0x00009fff DMA                 4Kb */
#define IMX_UART3_OFFSET          0x0000a000 /* -0x0000afff UART3               4Kb */
                                             /* -0x0000ffff Reserved           20Kb */
#define IMX_AIPI2_OFFSET          0x00010000 /* -0x00010fff AIPI2               4Kb */
#define IMX_SIM_OFFSET            0x00011000 /* -0x00011fff SIM                 4Kb */
#define IMX_USBD_OFFSET           0x00012000 /* -0x00012fff USBD                4Kb */
#define IMX_CSPI1_OFFSET          0x00013000 /* -0x00013fff CSPI1               4Kb */
#define IMX_MMC_OFFSET            0x00014000 /* -0x00014fff MMC                 4Kb */
#define IMX_ASP_OFFSET            0x00015000 /* -0x00015fff ASP                 4Kb */
#define IMX_BTA_OFFSET            0x00016000 /* -0x00016fff BTA                 4Kb */
#define IMX_I2C_OFFSET            0x00017000 /* -0x00017fff I2C                 4Kb */
#define IMX_SSI_OFFSET            0x00018000 /* -0x00018fff SSI                 4Kb */
#define IMX_CSPI2_OFFSET          0x00019000 /* -0x00019fff CSPI2               4Kb */
#define IMX_MSHC_OFFSET           0x0001a000 /* -0x0001afff Memory Stick        4Kb */
#define IMX_CRM_OFFSET            0x0001b000 /* -0x0001bfff CRM                 4Kb */
#define   IMX_PLL_OFFSET          0x0001b000
#define   IMX_SC_OFFSET           0x0001b800
#define IMX_GPIO_OFFSET           0x0001c000 /* -0x0001cfff GPIO                4Kb */
                                             /* -0x0001ffff Reserved           12Kb */
#define IMX_EIM_OFFSET            0x00020000 /* -0x00020fff EIM                 4Kb */
#define IMX_SDRAMC_OFFSET         0x00021000 /* -0x00021fff SDRAMC              4Kb */
#define IMX_DSPA_OFFSET           0x00022000 /* -0x00022fff DSPA                4Kb */
#define IMX_AITC_OFFSET           0x00023000 /* -0x00023fff AITC                4Kb */
#define IMX_CSI_OFFSET            0x00024000 /* -0x00024fff CSI                 4Kb */
                                             /* -0x000fffff Reserved          876Kb */

/* Peripheral Register Offsets ******************************************************/

#define IMX_AIPI1_VBASE           (IMX_PERIPHERALS_VSECTION + IMX_AIPI1_OFFSET)
#define IMX_WDOG_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_WDOG_OFFSET)
#define IMX_TIMER1_VBASE          (IMX_PERIPHERALS_VSECTION + IMX_TIMER1_OFFSET)
#define IMX_TIMER2_VBASE          (IMX_PERIPHERALS_VSECTION + IMX_TIMER2_OFFSET)
#define IMX_RTC_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_RTC_OFFSET)
#define IMX_LCDC_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_LCDC_OFFSET)
#define IMX_LCDC_COLORMAP_VBASE   (IMX_PERIPHERALS_VSECTION + IMX_LCDC_COLORMAP)
#define IMX_UART1_VBASE           (IMX_PERIPHERALS_VSECTION + IMX_UART1_OFFSET)
#define IMX_UART2_VBASE           (IMX_PERIPHERALS_VSECTION + IMX_UART2_OFFSET)
#define IMX_PWM1_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_PWM1_OFFSET)
#define IMX_DMA_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_DMA_OFFSET)
#define IMX_UART3_VBASE           (IMX_PERIPHERALS_VSECTION + IMX_UART3_OFFSET)
#define IMX_AIP2_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_AIPI2_OFFSET)
#define IMX_SIM_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_SIM_OFFSET)
#define IMX_USBD_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_USBD_OFFSET)
#define IMX_CSPI1_VBASE           (IMX_PERIPHERALS_VSECTION + IMX_CSPI1_OFFSET)
#define IMX_MMC_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_MMC_OFFSET)
#define IMX_ASP_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_ASP_OFFSET)
#define IMX_BTA_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_BTA_OFFSET)
#define IMX_I2C_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_I2C_OFFSET)
#define IMX_SSI_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_SSI_OFFSET)
#define IMX_CSPI2_VBASE           (IMX_PERIPHERALS_VSECTION + IMX_CSPI2_OFFSET)
#define IMX_MSHC_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_MSHC_OFFSET)
#define IMX_CRM_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_CRM_OFFSET)
#define IMX_PLL_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_PLL_OFFSET)
#define IMX_SC_VBASE              (IMX_PERIPHERALS_VSECTION + IMX_SC_OFFSET)
#define IMX_GPIO_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_GPIO_OFFSET)
#define IMX_EIM_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_EIM_OFFSET)
#define IMX_SDRAMC_VBASE          (IMX_PERIPHERALS_VSECTION + IMX_SDRAMC_OFFSET)
#define IMX_DSPA_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_DSPA_OFFSET)
#define IMX_AITC_VBASE            (IMX_PERIPHERALS_VSECTION + IMX_AITC_OFFSET)
#define IMX_CSI_VBASE             (IMX_PERIPHERALS_VSECTION + IMX_CSI_OFFSET)

/* Memory Mapping Info **************************************************************/

/* The NuttX entry point starts at an offset from the virtual beginning of DRAM.
 * This offset reserves space for the MMU page cache.
 */

#define NUTTX_START_VADDR         ((CONFIG_DRAM_NUTTXENTRY & 0xfff00000) | PGTABLE_SIZE)

#if NUTTX_START_VADDR != CONFIG_DRAM_NUTTXENTRY
# error "CONFIG_DRAM_NUTTXENTRY does not have correct offset for page table"
#endif

/* Section MMU Flags  */

#define IMX_FLASH_MMUFLAGS        MMU_IOFLAGS
#define IMX_PERIPHERALS_MMUFLAGS  MMU_IOFLAGS

/* 16Kb of memory is reserved at the beginning of SDRAM to hold the
 * page table for the virtual mappings.  A portion of this table is
 * not accessible in the virtual address space (for normal operation).
 * We will reuse this memory for coarse page tables as follows:
 */

#define PGTABLE_BASE_PADDR        IMX_SDRAM0_PSECTION
#define PGTABLE_SDRAM_PADDR       PGTABLE_BASE_PADDR
#define PGTABLE_COARSE_PBASE      (PGTABLE_BASE_PADDR+0x00000800)
#define PGTABLE_COARSE_PEND       (PGTABLE_BASE_PADDR+0x00003000)
#define PTTABLE_PERIPHERALS_PBASE (PGTABLE_BASE_PADDR+0x00003000)
#define PGTABLE_PEND              (PGTABLE_BASE_PADDR+0x00004000)

#define PGTABLE_BASE_VADDR        IMX_SDRAM_VSECTION
#define PGTABLE_SDRAM_VADDR       PGTABLE_BASE_VADDR
#define PGTABLE_COARSE_VBASE      (PGTABLE_BASE_VADDR+0x00000800)
#define PGTABLE_COARSE_VEND       (PGTABLE_BASE_VADDR+0x00003000)
#define PTTABLE_PERIPHERALS_VBASE (PGTABLE_BASE_VADDR+0x00003000)
#define PGTABLE_VEND              (PGTABLE_BASE_VADDR+0x00004000)

#define PGTABLE_COARSE_TABLE_SIZE (4*256)
#define PGTABLE_COARSE_ALLOC      (PGTABLE_COARSE_VEND-PGTABLE_COARSE_VBASE)
#define PGTABLE_NCOARSE_TABLES    (PGTABLE_COARSE_SIZE / PGTBALE_COARSE_TABLE_ALLOC)

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_MEMORYMAP_H */
