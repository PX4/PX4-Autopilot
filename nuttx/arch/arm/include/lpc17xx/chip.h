/************************************************************************************
 * arch/arm/include/lpc17xx/chip.h
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           with LPC178x support from Rommel Marcelo
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

#ifndef __ARCH_ARM_INCLUDE_LPC17XX_CHIP_H
#define __ARCH_ARM_INCLUDE_LPC17XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_LPC1751)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1752)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1754)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1756)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1758)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1759)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1764)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1765)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1766)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1767)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1769) || defined(CONFIG_ARCH_CHIP_LPC1768)
#  define LPC176x               1  /* LPC175/6 family */
#  undef  LPC178x                  /* Not LPC177/8 family */
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
#elif defined(CONFIG_ARCH_CHIP_LPC1773)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (128*1024) /* 128Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_CPUSRAM_SIZE    (8*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* No Peripheral SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LPC17_NUSBHOST           /* No USB host controller */
#  undef  LPC17_NUSBOTG            /* No USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_HAVE_SPIFI      1  /* Have SPIFI interface */
#  undef  LPC17_HAVE_LCD           /* No LCD controller */
#  undef  LPC17_HAVE_QEI           /* No QEI interface */
#  undef  LPC17_HAVE_SD            /* No SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1774)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (128*1024) /* 128Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_CPUSRAM_SIZE    (8*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0*/
#  undef  LPC17_HAVE_BANK1         /* Have Peripheral SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LPC17_NUSBHOST           /* One USB host controller */
#  undef  LPC17_NUSBOTG            /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  undef  LPC17_HAVE_LCD           /* One LCD controller */
#  define LPC17_HAVE_QEI        1  /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1776)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE      (16*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0  */
#  undef  LPC17_HAVE_BANK1         /* Have Peripheral SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  undef  LPC17_HAVE_LCD           /* One LCD controller */
#  define LPC17_HAVE_QEI        1  /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1777)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have Peripheral SRAM bank 1 */
#  undef  LPC17_NETHCONTROLLERS    /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  undef  LPC17_HAVE_LCD           /* One LCD controller */
#  define LPC17_HAVE_QEI        1  /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1778)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have Peripheral SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  undef  LPC17_HAVE_LCD           /* One LCD controller */
#  define LPC17_HAVE_QEI        1  /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1785)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (16*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* Have Peripheral SRAM bank 1 */
#  undef  LPC17_NETHCONTROLLERS    /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  define LPC17_HAVE_LCD        1  /* One LCD controller */
#  undef  LPC17_HAVE_QEI           /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1786)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (16*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0 */
#  undef  LPC17_HAVE_BANK1         /* Have Peripheral SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  define LPC17_HAVE_LCD        1  /* One LCD controller */
#  define LPC17_HAVE_QEI        1  /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1787)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have Peripheral SRAM bank 1 */
#  undef  LPC17_NETHCONTROLLERS    /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  define LPC17_HAVE_LCD        1  /* One LCD controller */
#  define LPC17_HAVE_QEI        1  /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#elif defined(CONFIG_ARCH_CHIP_LPC1788)
#  undef  LPC176x                  /* Not LPC175/6 family */
#  define LPC178x               1  /* LPC177/8 family */
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_CPUSRAM_SIZE    (32*1024)
#  define LPC17_HAVE_BANK0      1  /* Have Peripheral SRAM bank 0 */
#  define LPC17_HAVE_BANK1      1  /* Have Peripheral SRAM bank 1 */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  undef  LPC17_HAVE_SPIFI         /* Have SPIFI interface */
#  define LPC17_HAVE_LCD        1  /* One LCD controller */
#  define LPC17_HAVE_QEI        1  /* One QEI interface */
#  define LPC17_HAVE_SD         1  /* One SD controller */
#else
#  error "Unsupported LPC17xx chip"
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

#endif /* __ARCH_ARM_INCLUDE_LPC17XX_CHIP_H */
