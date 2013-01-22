/************************************************************************************
 * arch/arm/include/kinetis/chip.h
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_CHIP_H
#define __ARCH_ARM_INCLUDE_KINETIS_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_MK40X64VFX50) || defined(CONFIG_ARCH_CHIP_MK40X64VLH50) || \
    defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)
#  define KINETIS_K40             1          /* Kinetics K40 family */
#  undef  KINETIS_K60                        /* Not Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (64*1024)  /* 64Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)  /* 32Kb */
#  define KINETIS_SRAM_SIZE       (16*1024)  /* 16Kb */
#  undef  KINETIS_MPU                        /* No memory protection unit */
#  undef  KINETIS_EXTBUS                     /* No external bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                      /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  undef  KINETIS_NSDHC                      /* No SD host controller */
#  undef  KINETIS_NTOUCHIF                   /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2          /* Two I2C modules */
#  undef  KINETIS_NISO7816                   /* No UART with ISO-786 */
#  define KINETIS_NUART           6          /* Six UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  if defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)
#    define KINETIS_NCAN          2          /* Two CAN controllers */
#  else
#    undef KINETIS_NCAN                      /* No CAN in 64-pin chips */
#  endif
#  define KINETIS_NI2S            1          /* One I2S module */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 25x8/29x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            2          /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  define KINETIS_NDAC6           3          /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Two 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X128VFX50) || defined(CONFIG_ARCH_CHIP_MK40X128VLH50) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLK50) || defined(CONFIG_ARCH_CHIP_MK40X128VMB50) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLL50) || defined(CONFIG_ARCH_CHIP_MK40X128VML50) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VFX72) || defined(CONFIG_ARCH_CHIP_MK40X128VLH72) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLK72) || defined(CONFIG_ARCH_CHIP_MK40X128VMB72) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLL72) || defined(CONFIG_ARCH_CHIP_MK40X128VML72)
#  define KINETIS_K40             1          /* Kinetics K40 family */
#  undef  KINETIS_K60                        /* Not Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (128*1024) /* 128Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)  /* 32Kb */
#  define KINETIS_SRAM_SIZE       (32*1024)  /* 32Kb */
#  undef  KINETIS_MPU                        /* No memory protection unit */
#  undef  KINETIS_EXTBUS                     /* No external bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                      /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  undef  KINETIS_NSDHC                      /* No SD host controller */
#  undef  KINETIS_NTOUCHIF                   /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2          /* Two I2C modules */
#  undef  KINETIS_NISO7816                   /* No UART with ISO-786 */
#  define KINETIS_NUART           6          /* Six UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            1          /* One I2S module */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            2          /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  define KINETIS_NDAC6           3          /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Two 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLK72) || defined(CONFIG_ARCH_CHIP_MK40X256VMB72) || \
    defined(CONFIG_ARCH_CHIP_MK40X256VLL72) || defined(CONFIG_ARCH_CHIP_MK40X256VML72)
#  define KINETIS_K40             1          /* Kinetics K40 family */
#  undef  KINETIS_K60                        /* Not Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)  /* 32Kb */
#  define KINETIS_SRAM_SIZE       (32*1024)  /* 64Kb */
#  undef  KINETIS_MPU                        /* No memory protection unit */
#  undef  KINETIS_EXTBUS                     /* No external bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                      /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  undef  KINETIS_NSDHC                      /* No SD host controller */
#  undef  KINETIS_NTOUCHIF                   /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2          /* Two I2C modules */
#  undef  KINETIS_NISO7816                   /* No UART with ISO-786 */
#  define KINETIS_NUART           6          /* Six UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            1          /* One I2S module */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            2          /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  define KINETIS_NDAC6           3          /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Two 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100)
#  define KINETIS_K40             1          /* Kinetics K40 family */
#  undef  KINETIS_K60                        /* Not Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (128*1024) /* 128Kb */
#  define KINETIS_FLEXMEM_SIZE    (128*1024) /* 128Kb */
#  define KINETIS_SRAM_SIZE       (32*1024)  /* 32Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                      /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* One SD host controller */
#  undef  KINETIS_NTOUCHIF                   /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2          /* Two I2C modules */
#  undef  KINETIS_NISO7816                   /* No UART with ISO-786 */
#  define KINETIS_NUART           6          /* Six UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            1          /* One I2S module */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 40x8/44x4)*/
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            2          /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  define KINETIS_NDAC6           3          /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Two 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100)
#  define KINETIS_K40             1          /* Kinetics K40 family */
#  undef  KINETIS_K60                        /* Not Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  define KINETIS_FLEXMEM_SIZE    (256*1024) /* 256Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 32Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                      /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* One SD host controller */
#  undef  KINETIS_NTOUCHIF                   /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2          /* Two I2C modules */
#  undef  KINETIS_NISO7816                   /* No UART with ISO-786 */
#  define KINETIS_NUART           6          /* Six UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            1          /* One I2S module */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 40x8/44x4)*/
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            2          /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  define KINETIS_NDAC6           3          /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Two 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40N512VLK100) || defined(CONFIG_ARCH_CHIP_MK40N512VMB100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLL100) || defined(CONFIG_ARCH_CHIP_MK40N512VML100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)
#  define KINETIS_K40             1          /* Kinetics K40 family */
#  undef  KINETIS_K60                        /* Not Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (512*1024) /* 512Kb */
#  undef  KINETIS_FLEXMEM_SIZE               /* No FlexMemory */
#  define KINETIS_SRAM_SIZE       (128*1024) /* 128Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                      /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* One SD host controller */
#  undef  KINETIS_NTOUCHIF                   /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2          /* Two I2C modules */
#  undef  KINETIS_NISO7816                   /* No UART with ISO-786 */
#  define KINETIS_NUART           6          /* Six UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            1          /* One I2S module */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 40x8/44x4)*/
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            2          /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  define KINETIS_NDAC6           3          /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Two 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLL100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           4          /* Four additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  define KINETIS_NADC12          1          /* One 12-channel ADC (ADC0)*/
#  define KINETIS_NADC13          1          /* No 13-channel ADC (ADC1) */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          1          /* One 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLL100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024) /* 256Kb  */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)   /* 32Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           4          /* Four additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  define KINETIS_NADC12          1          /* One 12-channel ADC (ADC0)*/
#  define KINETIS_NADC13          1          /* No 13-channel ADC (ADC1) */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          1          /* One 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLL100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (512*1024) /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024) /* 128Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           4          /* Four additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  define KINETIS_NADC12          1          /* One 12-channel ADC (ADC0)*/
#  define KINETIS_NADC13          1          /* No 13-channel ADC (ADC1) */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          1          /* One 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VML100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           4          /* Four additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  define KINETIS_NADC12          1          /* One 12-channel ADC (ADC0)*/
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC1) */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          1          /* One 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VML100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024) /* 256Kb */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)   /* 4Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           4          /* Four additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  define KINETIS_NADC12          1          /* One 12-channel ADC (ADC0)*/
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC1) */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          1          /* One 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VML100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (512*1024) /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024) /* 128Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           4          /* Four additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  define KINETIS_NADC12          1          /* One 12-channel ADC (ADC0)*/
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC1) */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          1          /* One 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           5          /* Five additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1          /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLQ100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024) /* 256Kb */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)   /* 4Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           5          /* Five additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1          /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLQ100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (512*1024) /* 512Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024) /* 128Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           5          /* Five additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1          /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VMD100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           5          /* Five additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1          /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VMD100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (256*1024) /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024) /* 256Kb */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)   /* 4Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)  /* 64Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           5          /* Five additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1          /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VMD100)
#  undef  KINETIS_K40                        /* Not Kinetics K40 family */
#  define KINETIS_K60             1          /* Kinetis K60 family */
#  define KINETIS_FLASH_SIZE      (512*1024) /* 512Kb */
#  undef  KINETIS_FLEXNVM_SIZE               /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE               /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024) /* 128Kb */
#  define KINETIS_MPU             1          /* Memory protection unit */
#  define KINETIS_EXTBUS          1          /* External bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  define KINETIS_NENET           1          /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  define KINETIS_NSDHC           1          /* SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3          /* Three I2C modules */
#  define KINETIS_NISO7816        1          /* One UART with ISO-786 */
#  define KINETIS_NUART           5          /* Five additional UARTs */
#  define KINETIS_NSPI            3          /* Three SPI modules */
#  define KINETIS_NCAN            2          /* Two CAN controllers */
#  define KINETIS_NI2S            2          /* Two I2S modules */
#  define KINETIS_NSLCD           1          /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4          /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  define KINETIS_NADC15          1          /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1          /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4          /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3          /* Three analog comparators */
#  undef  KINETIS_NDAC6                      /* No 6-bit DAC */
#  define KINETIS_NDAC12          2          /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4          /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3          /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* CRC */

#else
#  error "Unsupported Kinetis chip"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-15. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:4] of each field, bits[3:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Steps between supported priority values */

#define NVIC_SYSH_DISABLE_PRIORITY (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#define NVIC_SYSH_SVCALL_PRIORITY  NVIC_SYSH_PRIORITY_MAX

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_KINETIS_CHIP_H */
