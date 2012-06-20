/****************************************************************************
 * arch/mips/include/pic32mx/chip.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_INCLUDE_PIC32MX_CHIP_H
#define __ARCH_MIPS_INCLUDE_PIC32MX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#if defined(CONFIG_ARCH_CHIP_PIC32MX110F016B)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 16  /* 16Kb program FLASH */
#  define CHIP_DATAMEM_KB   4   /* 4Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       10  /* 10 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX110F016C)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 16  /* 16Kb program FLASH */
#  define CHIP_DATAMEM_KB   4   /* 4Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX110F016D)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 16  /* 16Kb program FLASH */
#  define CHIP_DATAMEM_KB   4   /* 4Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX120F032B)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       10  /* 10 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX120F032C)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX120F032D)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX130F064B)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       10  /* 10 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX130F064C)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX130F064D)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX150F128B)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       10  /* 10 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX150F128C)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX150F128D)
#  define CHIP_PIC32MX1     1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0   /* No dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       0   /* No USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX210F016B)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 16  /* 16Kb program FLASH */
#  define CHIP_DATAMEM_KB   4   /* 4Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       9   /* 9 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX210F016C)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 16  /* 16Kb program FLASH */
#  define CHIP_DATAMEM_KB   4   /* 4Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX210F016D)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 16  /* 16Kb program FLASH */
#  define CHIP_DATAMEM_KB   4   /* 4Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX220F032B)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       9   /* 9 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX220F032C)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX220F032D)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX230F064B)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       9   /* 9 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX230F064C)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX230F064D)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX250F128B)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        28  /* Package SOIC, SSOP, SPDIP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       9   /* 9 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX250F128C)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        36  /* Package VTLA */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       12  /* 12 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX250F128D)
#  undef  CHIP_PIC32MX1
#  define CHIP_PIC32MX2     1
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        44  /* Package VTLA, TQFP, QFN */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 3   /* 3Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  undef  CHIP_CHE              /* No pre-fetch cache controller */
#  define CHIP_NPORTS       3   /* 3 ports (A, B, C) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2   /* 2 dedicated DMA channels */
#  define CHIP_CTMU         1   /* Has CTMU */
#  define CHIP_VRFSEL       1   /* Comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI/I2S interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       13  /* 13 10-bit ADC channels */
#  define CHIP_NCM          3   /* 3 Analog comparators */
#  define CHIP_USBOTG       1   /* Has USB OTG */
#  define CHIP_RTCC         1   /* Has RTCC */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          0   /* No parallel slave port (?) */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG         1   /* Has JTAG */
#elif defined(CONFIG_ARCH_CHIP_PIC32MX320F032H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       0   /* No programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX320F064H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       0   /* No programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX320F128H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       0   /* No programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F128H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F256H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F512H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX320F128L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       0   /* No programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F128L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX360F256L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX360F512L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  0
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX420F032H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 32  /* 32Kb program FLASH */
#  define CHIP_DATAMEM_KB   8   /* 8Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       0   /* No programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         1   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F128H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          40  /* 40MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         1   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F256H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         1   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F512H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         1   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F128L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX460F256L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX460F512L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  2
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       2   /* 2 UARTS */
#  define CHIP_UARTFIFOD    8   /* 4 level deep UART FIFOs */
#  define CHIP_NSPI         2   /* 2 SPI interfaces */
#  define CHIP_NI2C         2   /* 2 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX534F064H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX564F064H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX564F128H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX575F256H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX575F512H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX534F064L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   16  /* 16Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX564F064L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX564F128L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX575F256L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX575F512L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  define CHIP_PIC32MX5     1
#  undef  CHIP_PIC32MX6
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    0   /* No Ethernet */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX664F064H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX664F128H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX675F256H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX675F512H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX695F512H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   128 /* 128Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX664F064L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 64  /* 64Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX664F128L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX675F256L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX675F512L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX695F512L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  define CHIP_PIC32MX6     1
#  undef  CHIP_PIC32MX7
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   128 /* 128Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* xx programmable DMA channels (4 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         0   /* No CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX764F128H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (6 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX775F256H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (8 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         2   /* 2 CAN interfaces */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX775F512H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (8 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         2   /* 2 CAN interfaces */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX795F512H)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        64  /* Package PT,MR */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   128 /* 128Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (8 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  undef  CHIP_TRACE            /* No trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         3   /* 3 SPI interfaces */
#  define CHIP_NI2C         4   /* 4 I2C interfaces */
#  define CHIP_NCAN         2   /* 2 CAN interfaces */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX764F128L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        100 /* Package  PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 128 /* 128Kb program FLASH */
#  define CHIP_DATAMEM_KB   32  /* 32Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       4   /* 4 programmable DMA channels (6 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         1   /* 1 CAN interface */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX775F256L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 256 /* 256Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (8 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         2   /* 2 CAN interfaces */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX775F512L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   64  /* 64Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (8 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  undef  CHIP_VRFSEL           /* No comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         2   /* 2 CAN interfaces */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#elif defined(CONFIG_ARCH_CHIP_PIC32MX795F512L)
#  undef  CHIP_PIC32MX1
#  undef  CHIP_PIC32MX2
#  undef  CHIP_PIC32MX3
#  undef  CHIP_PIC32MX4
#  undef  CHIP_PIC32MX5
#  undef  CHIP_PIC32MX6
#  define CHIP_PIC32MX7     1
#  define CHIP_NPINS        100 /* Package PT,PF,BG */
#  define CHIP_MHZ          80  /* 80MHz maximum frequency */
#  define CHIP_BOOTFLASH_KB 12  /* 12Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 512 /* 512Kb program FLASH */
#  define CHIP_DATAMEM_KB   128 /* 128Kb data memory */
#  define CHIP_CHE          1   /* Has pre-fetch cache controller */
#  define CHIP_NPORTS       7   /* 7 ports (A, B, C, D, E, F, G) */
#  define CHIP_NTIMERS      5   /* 5 timers */
#  define CHIP_NIC          5   /* 5 input capture */
#  define CHIP_NOC          5   /* 5 output compare */
#  define CHIP_NDMACH       8   /* 8 programmable DMA channels (8 dedicated) */
#  define CHIP_NUSBDMACHAN  tbd
#  define CHIP_VRFSEL       1   /* Have comparator voltage reference selection */
#  define CHIP_TRACE        1   /* Have trace capability */
#  define CHIP_NUARTS       6   /* 6 UARTS */
#  define CHIP_UARTFIFOD    8   /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         4   /* 4 SPI interfaces */
#  define CHIP_NI2C         5   /* 5 I2C interfaces */
#  define CHIP_NCAN         2   /* 2 CAN interfaces */
#  define CHIP_NADC10       16  /* 16 10-bit ADC channels */
#  define CHIP_NCM          2   /* 2 Comparators */
#  define CHIP_PMP          1   /* Have parallel master port */
#  define CHIP_PSP          1   /* Have parallel slave port */
#  define CHIP_NETHERNET    1   /* 1 Ethernet interface */
#  define CHIP_JTAG
#else
#  error "Unrecognized PIC32 device
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_PIC32MX_CHIP_H */
