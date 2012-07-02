/************************************************************************************
 * arch/arm/include/lpc43xx/chip.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_LPC43XX_CHIP_H
#define __ARCH_ARM_INCLUDE_LPC43XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Per the data sheet: LPC4350/30/20/10 Rev. 3.2 — 4 June 2012 */
/* Get customizations for each supported chip.
 *
 * SRAM Resources
 * --------------------- -------- ------- ------- -------
 * Local SRAM            LPC4310  LPC4320 LPC4330 LPC4350
 * --------------------- -------- ------- ------- -------
 * BANK 0 (0x1000 0000)     96Kb   128Kb   128Kb   128Kb
 * BANK 1 (0x1008 0000)     40Kb    40Kb    72Kb    72Kb
 * --------------------- -------- ------- ------- -------
 * SUBTOTAL                136Kb   168Kb   200Kb   200Kb
 * --------------------- -------- ------- ------- -------
 * AHB SRAM              LPC4310  LPC4320 LPC4330 LPC4350
 * --------------------- -------- ------- ------- -------
 * BANK 0 (0x2000 0000)     16Kb    16Kb   64Kb    64Kb
 * BANK 1 (0x2000 8000)                    NOTE 1  NOTE 1
 * BANK 2 (0x2000 c000)     16Kb    16Kb   NOTE 1  NOTE 1
 * --------------------- -------- ------- ------- -------
 * SUBTOTAL                 32Kb    32Kb   64Kb    64Kb
 * --------------------- -------- ------- ------- -------
 * TOTAL                   168Kb   200Kb  264Kb   264Kb
 * --------------------- -------- ------- ------- -------
 *
 * NOTE 1: The 64Kb of AHB of SRAM on the LPC4330/50 span
 * all AHB SRAM banks and so are treated as a single bank
 * by the NuttX memory manager.
 */

/* Per the user manual: UM10503, Rev. 1.2 — 8 June 2012 */
/* Get customizations for each supported chip.
 *
 * SRAM Resources
 * --------------------- -------- ------- ------- ------- ------- -------
 * Local SRAM            LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357
 * --------------------- -------- ------- ------- ------- ------- -------
 * BANK 0 (0x1000 0000)     96Kb    96Kb   128Kb   128Kb    32Kb    32Kb
 * BANK 1 (0x1008 0000)     40Kb    40Kb    72Kb    72Kb    40Kb    40Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * SUBTOTAL                136Kb   136Kb   200Kb   200Kb    72Kb    72Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * AHB SRAM              LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357
 * --------------------- -------- ------- ------- ------- ------- -------
 * BANK 0 (0x2000 0000)     16Kb    64Kb   64Kb    64Kb     64Kb    64Kb
 * BANK 1 (0x2000 8000)             NOTE 1 NOTE 1  NOTE 1  NOTE 1  NOTE 1
 * BANK 2 (0x2000 c000)     16Kb    NOTE 1 NOTE 1  NOTE 1  NOTE 1  NOTE 1
 * --------------------- -------- ------- ------- ------- ------- -------
 * SUBTOTAL                 32Kb    64Kb   64Kb    64Kb     64Kb    64Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * TOTAL                   168Kb   200Kb  264Kb   264Kb    136Kb   136Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 *
 * --------------------- -------- ------- ------- ------- ------- -------
 * FLASH                 LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357
 * --------------------- -------- ------- ------- ------- ------- -------
 * BANK A (0x1a00 0000)                                    256Kb   512Kb
 * BANK B (0x1b00 8000)                                    256Kb   512Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 * TOTAL                   None    None    None    None    512Kb  1024Kb
 * --------------------- -------- ------- ------- ------- ------- -------
 *
 * NOTE 1: The 64Kb of AHB of SRAM on the LPC4350/30/20 span
 * all AHB SRAM banks and so are treated as a single bank
 * by the NuttX memory manager.
 */

#if defined(CONFIG_ARCH_CHIP_LPC4310FBD144)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (96*1024)   /* 136Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  undef  LPC43_USB0                           /* No USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4310FET100)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (96*1024)   /* 136Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  undef  LPC43_USB0                           /* No USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  undef  LPC43_PWM                            /* No PWM capability */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (4)         /* Four ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4320FBD144)
#  warning "Data sheet and user manual are consistement for the LPC4320"
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 168Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4320FET100)
#  warning "Data sheet and user manual are consistement for the LPC4320"
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 168Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  undef  LPC43_PWM                            /* No PWM capability */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (4)         /* Four ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FBD144)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET100)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  undef  LPC43_PWM                            /* No PWM capability */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (4)         /* Four ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET180)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET256)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4350FBD208)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  define LPC43_NLCD               (1)         /* One LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET180)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  define LPC43_NLCD               (1)         /* One LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET256)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  define LPC43_NLCD               (1)         /* One LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4353FBD208)
#  define LPC43_FLASH_BANKA_SIZE   (256*1025)  /* 512Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (256*1025)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)  /*  72Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET180)
#  define LPC43_FLASH_BANKA_SIZE   (256*1025)  /* 512Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (256*1025)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)  /*  72Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET256)
#  define LPC43_FLASH_BANKA_SIZE   (256*1025)  /* 512Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (256*1025)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)  /*  72Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET180)
#  define LPC43_FLASH_BANKA_SIZE   (512*1025)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1025)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)  /*  72Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4357FBD208)
#  define LPC43_FLASH_BANKA_SIZE   (512*1025)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1025)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)  /*  72Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET256)
#  define LPC43_FLASH_BANKA_SIZE   (512*1025)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1025)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)  /*  72Kb Local SRAM*/
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (64*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (0)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_PWM                (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (8)         /* Eight ADC channels */
#else
#  error "Unsupported LPC43xx chip"
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

#endif /* __ARCH_ARM_INCLUDE_LPC43XX_CHIP_H */
