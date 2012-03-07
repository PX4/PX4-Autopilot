/************************************************************************************
 * arch/arm/src/lpc17xx/chip.h
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_LPC1769) || defined(CONFIG_ARCH_CHIP_LPC1768)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have AHB SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1767)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have AHB SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         0  /* No USB device controller */
#  define LPC17_NCAN            0  /* No CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1766)
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have AHB SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1765)
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have AHB SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1764)
#  define LPC17_FLASH_SIZE      (128*1024) /* 128Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_CPUSRAM_SIZE    (16*1024)
#  define LPC17_HAVE_BANK0      1  /* Have AHB SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* No AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            0  /* No DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1759)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have AHB SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1758)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have AHB SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1756)
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_CPUSRAM_SIZE    (16*1024)
#  define LPC17_HAVE_BANK0      1  /* No AHB SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* No AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1754)
#  define LPC17_FLASH_SIZE      (128*1024) /* 128Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_CPUSRAM_SIZE    (16*1024)
#  define  LPC17_HAVE_BANK0     1  /* Have AHB SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* No AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            1  /* One CAN controller */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1752)
#  define LPC17_FLASH_SIZE      (64*1024) /* 65Kb */
#  define LPC17_SRAM_SIZE       (16*1024) /* 16Kb */
#  define LPC17_CPUSRAM_SIZE    (16*1024)
#  undef  LPC17_HAVE_BANK0         /* No AHB SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* No AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            1  /* One CAN controller */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            0  /* No DAC module */
#elif defined(CONFIG_ARCH_CHIP_LPC1751)
#  define LPC17_FLASH_SIZE      (32*1024) /* 32Kb */
#  define LPC17_SRAM_SIZE       (8*1024)  /*  8Kb */
#  define LPC17_CPUSRAM_SIZE    (8*1024)
#  undef  LPC17_HAVE_BANK0         /* No AHB SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* No AHB SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            1  /* One CAN controller */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            0  /* No DAC module */
#else
#  error "Unsupported LPC17xx chip"
#endif

/* Include only the memory map.  Other chip hardware files should then include this
 * file for the proper setup
 */

#include "lpc17_memorymap.h"

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-31. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:3] of each field, bits[2:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xf8 /* All bits[7:3] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_H */
