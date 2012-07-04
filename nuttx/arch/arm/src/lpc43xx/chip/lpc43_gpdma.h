/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_gpdma.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GPDMA_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GPDMA_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

#define LPC43_GPDMA_INTSTAT_OFFSET        0x0000 /* DMA Interrupt Status Register */
#define LPC43_GPDMA_INTTCSTAT_OFFSET      0x0004 /* DMA Interrupt Terminal Count Request Status Register */
#define LPC43_GPDMA_INTTCCLEAR_OFFSET     0x0008 /* DMA Interrupt Terminal Count Request Clear Register */
#define LPC43_GPDMA_INTERRSTAT_OFFSET     0x000c /* DMA Interrupt Error Status Register */
#define LPC43_GPDMA_INTERRCLR_OFFSET      0x0010 /* DMA Interrupt Error Clear Register */
#define LPC43_GPDMA_RAWINTTCSTAT_OFFSET   0x0014 /* DMA Raw Interrupt Terminal Count Status Register */
#define LPC43_GPDMA_RAWINTERRSTAT_OFFSET  0x0018 /* DMA Raw Error Interrupt Status Register */
#define LPC43_GPDMA_ENBLDCHNS_OFFSET      0x001c /* DMA Enabled Channel Register */
#define LPC43_GPDMA_SOFTBREQ_OFFSET       0x0020 /* DMA Software Burst Request Register */
#define LPC43_GPDMA_SOFTSREQ_OFFSET       0x0024 /* DMA Software Single Request Register */
#define LPC43_GPDMA_SOFTLBREQ_OFFSET      0x0028 /* DMA Software Last Burst Request Register */
#define LPC43_GPDMA_SOFTLSREQ_OFFSET      0x002c /* DMA Software Last Single Request Register */
#define LPC43_GPDMA_CONFIG_OFFSET         0x0030 /* DMA Configuration Register */
#define LPC43_GPDMA_SYNC_OFFSET           0x0034 /* DMA Synchronization Register */

/* Channel registers */

#define LPC43_GPDMA_SRCADDR_CHOFFSET      0x0000 /* DMA Channel Source Address Register */
#define LPC43_GPDMA_DESTADDR_CHOFFSET     0x0004 /* DMA Channel Destination Address Register */
#define LPC43_GPDMA_LLI_CHOFFSET          0x0008 /* DMA Channel Linked List Item Register */
#define LPC43_GPDMA_CONTROL_CHOFFSET      0x000c /* DMA Channel Control Register */
#define LPC43_GPDMA_CONFIG_CHOFFSET       0x0010 /* DMA Channel Configuration Register */

#define LPC43_GPDMA_CHOFFSET(n)           (0x0100 ((n) << 5))
#define LPC43_GPDMA_SRCADDR_OFFSET(n)     (LPC43_GPDMA_CHOFFSET(n)+LPC43_GPDMA_SRCADDR_CHOFFSET)
#define LPC43_GPDMA_DESTADDR_OFFSET(n)    (LPC43_GPDMA_CHOFFSET(n)+LPC43_GPDMA_DESTADDR_CHOFFSET)
#define LPC43_GPDMA_LLI_OFFSET(n)         (LPC43_GPDMA_CHOFFSET(n)+LPC43_GPDMA_LLI_CHOFFSET)
#define LPC43_GPDMA_CONTROL_OFFSET(n)     (LPC43_GPDMA_CHOFFSET(n)+LPC43_GPDMA_CONTROL_CHOFFSET)
#define LPC43_GPDMA_CONFIG_OFFSET(n)      (LPC43_GPDMA_CHOFFSET(n)+LPC43_GPDMA_CONFIG_CHOFFSET)

#define LPC43_GPDMA_SRCADDR0_OFFSET       0x0100 /* DMA Channel 0 Source Address Register */
#define LPC43_GPDMA_DESTADDR0_OFFSET      0x0104 /* DMA Channel 0 Destination Address Register */
#define LPC43_GPDMA_LLI0_OFFSET           0x0108 /* DMA Channel 0 Linked List Item Register */
#define LPC43_GPDMA_CONTROL0_OFFSET       0x010c /* DMA Channel 0 Control Register */
#define LPC43_GPDMA_CONFIG0_OFFSET        0x0110 /* DMA Channel 0 Configuration Register */

#define LPC43_GPDMA_SRCADDR1_OFFSET       0x0120 /* DMA Channel 1 Source Address Register */
#define LPC43_GPDMA_DESTADDR1_OFFSET      0x0124 /* DMA Channel 1 Destination Address Register */
#define LPC43_GPDMA_LLI1_OFFSET           0x0128 /* DMA Channel 1 Linked List Item Register */
#define LPC43_GPDMA_CONTROL1_OFFSET       0x012c /* DMA Channel 1 Control Register */
#define LPC43_GPDMA_CONFIG1_OFFSET        0x0130 /* DMA Channel 1 Configuration Register */

#define LPC43_GPDMA_SRCADDR2_OFFSET       0x0140 /* DMA Channel 2 Source Address Register */
#define LPC43_GPDMA_DESTADDR2_OFFSET      0x0144 /* DMA Channel 2 Destination Address Register */
#define LPC43_GPDMA_LLI2_OFFSET           0x0148 /* DMA Channel 2 Linked List Item Register */
#define LPC43_GPDMA_CONTROL2_OFFSET       0x014c /* DMA Channel 2 Control Register */
#define LPC43_GPDMA_CONFIG2_OFFSET        0x0150 /* DMA Channel 2 Configuration Register */

#define LPC43_GPDMA_SRCADDR3_OFFSET       0x0160 /* DMA Channel 3 Source Address Register */
#define LPC43_GPDMA_DESTADDR3_OFFSET      0x0164 /* DMA Channel 3 Destination Address */
#define LPC43_GPDMA_LLI3_OFFSET           0x0168 /* DMA Channel 3 Linked List Item Register */
#define LPC43_GPDMA_CONTROL3_OFFSET       0x016c /* DMA Channel 3 Control Register */
#define LPC43_GPDMA_CONFIG3_OFFSET        0x0170 /* DMA Channel 3 Configuration Register */

#define LPC43_GPDMA_SRCADDR4_OFFSET       0x0180 /* DMA Channel 4 Source Address Register */
#define LPC43_GPDMA_DESTADDR4_OFFSET      0x0184 /* DMA Channel 4 Destination Address Register */
#define LPC43_GPDMA_LLI4_OFFSET           0x0188 /* DMA Channel 4 Linked List Item Register */
#define LPC43_GPDMA_CONTROL4_OFFSET       0x018c /* DMA Channel 4 Control Register */
#define LPC43_GPDMA_CONFIG4_OFFSET        0x0190 /* DMA Channel 4 Configuration Register */

#define LPC43_GPDMA_SRCADDR5_OFFSET       0x01a0 /* DMA Channel 5 Source Address Register */
#define LPC43_GPDMA_DESTADDR5_OFFSET      0x01a4 /* DMA Channel 5 Destination Address Register */
#define LPC43_GPDMA_LLI5_OFFSET           0x01a8 /* DMA Channel 5 Linked List Item Register */
#define LPC43_GPDMA_CONTROL5_OFFSET       0x01ac /* DMA Channel 5 Control Register */
#define LPC43_GPDMA_CONFIG5_OFFSET        0x01b0 /* DMA Channel 5 Configuration Register */

#define LPC43_GPDMA_SRCADDR6_OFFSET       0x01c0 /* DMA Channel 6 Source Address Register */
#define LPC43_GPDMA_DESTADDR6_OFFSET      0x01c4 /* DMA Channel 6 Destination Address Register */
#define LPC43_GPDMA_LLI6_OFFSET           0x01c8 /* DMA Channel 6 Linked List Item Register */
#define LPC43_GPDMA_CONTROL6_OFFSET       0x01cc /* DMA Channel 6 Control Register */
#define LPC43_GPDMA_CONFIG6_OFFSET        0x01d0 /* DMA Channel 6 Configuration Register */

#define LPC43_GPDMA_SRCADDR7_OFFSET       0x01e0 /* DMA Channel 7 Source Address Register */
#define LPC43_GPDMA_DESTADDR7_OFFSET      0x01e4 /* DMA Channel 7 Destination Address Register */
#define LPC43_GPDMA_LLI7_OFFSET           0x01e8 /* DMA Channel 7 Linked List Item Register */
#define LPC43_GPDMA_CONTROL7_OFFSET       0x01ec /* DMA Channel 7 Control Register */
#define LPC43_GPDMA_CONFIG7_OFFSET        0x01f0 /* DMA Channel 7 Configuration Register */

/* Register Addresses *******************************************************************************/

#define LPC43_GPDMA_INTSTAT               (LPC43_DMA_BASE+LPC43_GPDMA_INTSTAT_OFFSET)
#define LPC43_GPDMA_INTTCSTAT             (LPC43_DMA_BASE+LPC43_GPDMA_INTTCSTAT_OFFSET)
#define LPC43_GPDMA_INTTCCLEAR            (LPC43_DMA_BASE+LPC43_GPDMA_INTTCCLEAR_OFFSET)
#define LPC43_GPDMA_INTERRSTAT            (LPC43_DMA_BASE+LPC43_GPDMA_INTERRSTAT_OFFSET)
#define LPC43_GPDMA_INTERRCLR             (LPC43_DMA_BASE+LPC43_GPDMA_INTERRCLR_OFFSET)
#define LPC43_GPDMA_RAWINTTCSTAT          (LPC43_DMA_BASE+LPC43_GPDMA_RAWINTTCSTAT_OFFSET)
#define LPC43_GPDMA_RAWINTERRSTAT         (LPC43_DMA_BASE+LPC43_GPDMA_RAWINTERRSTAT_OFFSET)
#define LPC43_GPDMA_ENBLDCHNS             (LPC43_DMA_BASE+LPC43_GPDMA_ENBLDCHNS_OFFSET)
#define LPC43_GPDMA_SOFTBREQ              (LPC43_DMA_BASE+LPC43_GPDMA_SOFTBREQ_OFFSET)
#define LPC43_GPDMA_SOFTSREQ              (LPC43_DMA_BASE+LPC43_GPDMA_SOFTSREQ_OFFSET)
#define LPC43_GPDMA_SOFTLBREQ             (LPC43_DMA_BASE+LPC43_GPDMA_SOFTLBREQ_OFFSET)
#define LPC43_GPDMA_SOFTLSREQ             (LPC43_DMA_BASE+LPC43_GPDMA_SOFTLSREQ_OFFSET)
#define LPC43_GPDMA_CONFIG                (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG_OFFSET)
#define LPC43_GPDMA_SYNC                  (LPC43_DMA_BASE+LPC43_GPDMA_SYNC_OFFSET)

/* Channel registers */

#define LPC43_GPDMA_CHANNEL(n)            (LPC43_DMA_BASE+LPC43_GPDMA_CHOFFSET(n))
#define LPC43_GPDMA_SRCADDR(n)            (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR_OFFSET(n))
#define LPC43_GPDMA_DESTADDR(n)           (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR_OFFSET(n))
#define LPC43_GPDMA_LLI(n)                (LPC43_DMA_BASE+LPC43_GPDMA_LLI_OFFSET(n))
#define LPC43_GPDMA_CONTROL(n)            (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL_OFFSET(n))
#define LPC43_GPDMA_CONFIG(n)             (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG_OFFSET(n))

#define LPC43_GPDMA_SRCADDR0              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR0_OFFSET)
#define LPC43_GPDMA_DESTADDR0             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR0_OFFSET)
#define LPC43_GPDMA_LLI0                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI0_OFFSET)
#define LPC43_GPDMA_CONTROL0              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL0_OFFSET)
#define LPC43_GPDMA_CONFIG0               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG0_OFFSET)

#define LPC43_GPDMA_SRCADDR1              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR1_OFFSET)
#define LPC43_GPDMA_DESTADDR1             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR1_OFFSET)
#define LPC43_GPDMA_LLI1                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI1_OFFSET)
#define LPC43_GPDMA_CONTROL1              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL1_OFFSET)
#define LPC43_GPDMA_CONFIG1               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG1_OFFSET)

#define LPC43_GPDMA_SRCADDR2              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR2_OFFSET)
#define LPC43_GPDMA_DESTADDR2             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR2_OFFSET)
#define LPC43_GPDMA_LLI2                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI2_OFFSET)
#define LPC43_GPDMA_CONTROL2              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL2_OFFSET)
#define LPC43_GPDMA_CONFIG2               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG2_OFFSET)

#define LPC43_GPDMA_SRCADDR3              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR3_OFFSET)
#define LPC43_GPDMA_DESTADDR3             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR3_OFFSET)
#define LPC43_GPDMA_LLI3                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI3_OFFSET)
#define LPC43_GPDMA_CONTROL3              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL3_OFFSET)
#define LPC43_GPDMA_CONFIG3               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG3_OFFSET)

#define LPC43_GPDMA_SRCADDR4              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR4_OFFSET)
#define LPC43_GPDMA_DESTADDR4             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR4_OFFSET)
#define LPC43_GPDMA_LLI4                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI4_OFFSET)
#define LPC43_GPDMA_CONTROL4              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL4_OFFSET)
#define LPC43_GPDMA_CONFIG4               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG4_OFFSET)

#define LPC43_GPDMA_SRCADDR5              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR5_OFFSET)
#define LPC43_GPDMA_DESTADDR5             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR5_OFFSET)
#define LPC43_GPDMA_LLI5                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI5_OFFSET)
#define LPC43_GPDMA_CONTROL5              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL5_OFFSET)
#define LPC43_GPDMA_CONFIG5               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG5_OFFSET)

#define LPC43_GPDMA_SRCADDR6              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR6_OFFSET)
#define LPC43_GPDMA_DESTADDR6             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR6_OFFSET)
#define LPC43_GPDMA_LLI6                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI6_OFFSET)
#define LPC43_GPDMA_CONTROL6              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL6_OFFSET)
#define LPC43_GPDMA_CONFIG6               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG6_OFFSET)

#define LPC43_GPDMA_SRCADDR7              (LPC43_DMA_BASE+LPC43_GPDMA_SRCADDR7_OFFSET)
#define LPC43_GPDMA_DESTADDR7             (LPC43_DMA_BASE+LPC43_GPDMA_DESTADDR7_OFFSET)
#define LPC43_GPDMA_LLI7                  (LPC43_DMA_BASE+LPC43_GPDMA_LLI7_OFFSET)
#define LPC43_GPDMA_CONTROL7              (LPC43_DMA_BASE+LPC43_GPDMA_CONTROL7_OFFSET)
#define LPC43_GPDMA_CONFIG7               (LPC43_DMA_BASE+LPC43_GPDMA_CONFIG7_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Common macros for DMA channel and source bit settings */

#define GPDMA_CHANNEL(n)                  (1 << (n)) /* Bits 0-7 correspond to DMA channel 0-7 */
#define GPDMA_SOURCE(n)                   (1 << (n)) /* Bits 0-15 correspond to DMA source 0-15 */
#define GPDMA_REQUEST(n)                  (1 << (n)) /* Bits 0-15 correspond to DMA request 0-15 */

/* DMA Interrupt Status Register */

#define GPDMA_INTSTAT(n)                  (1 << (n)) /* Bits 0-7: Status of DMA channel n interrupts after masking */
                                                     /* Bits 8-31: Reserved */
/* DMA Interrupt Terminal Count Request Status Register */

#define GPDMA_INTTCSTAT(n)                (1 << (n)) /* Bits 0-7: Terminal count interrupt request status for DMA channel n */
                                                     /* Bits 8-31: Reserved */
/* DMA Interrupt Terminal Count Request Clear Register */

#define GPDMA_INTTCCLEAR(n)               (1 << (n)) /* Bits 0-7: Clear terminal count interrupt request for DMA channel n */
                                                     /* Bits 8-31: Reserved */
/* DMA Interrupt Error Status Register */

#define GPDMA_INTERRSTAT(n)               (1 << (n)) /* Bits 0-7: Interrupt error status for DMA channel n */
                                                     /* Bits 8-31: Reserved */
/* DMA Interrupt Error Clear Register */

#define GPDMA_INTERRCLR(n)                (1 << (n)) /* Bits 0-7: Clear nterrupt error status for DMA channel n */
                                                     /* Bits 8-31: Reserved */
/* DMA Raw Interrupt Terminal Count Status Register */

#define GPDMA_RAWINTTCSTAT(n)             (1 << (n)) /* Bits 0-7: Terminal count interrupt request status for DMA channel n */
                                                     /* Bits 8-31: Reserved */
/* DMA Raw Error Interrupt Status Register */

#define GPDMA_RAWINTERRSTAT(n)            (1 << (n)) /* Bits 0-7: Interrupt error status for DMA channel n */
                                                     /* Bits 8-31: Reserved */
/* DMA Enabled Channel Register */

#define GPDMA_ENBLDCHNS(n)                (1 << (n)) /* Bits 0-7: Enabled status for DMA channel n */
                                                     /* Bits 8-31: Reserved */
/* DMA Software Burst Request Register */

#define GPDMA_SOFTBREQ(n)                 (1 << (n)) /* Bits 0-15: Software burst request flags for source n */
                                                     /* Bits 16-31: Reserved */

/* DMA Software Single Request Register */

#define GPDMA_SOFTSREQ(n)                 (1 << (n)) /* Bits 0-15: Software single burst request flags for source n */
                                                     /* Bits 16-31: Reserved */
/* DMA Software Last Burst Request Register */

#define GPDMA_SOFTLBREQ(n)                (1 << (n)) /* Bits 0-15: Software last burst request flags for source n */
                                                     /* Bits 16-31: Reserved */
                                                     
/* DMA Software Last Single Request Register */

#define GPDMA_SOFTLSREQ(n)                (1 << (n)) /* Bits 0-15: Software last single burst request flags for source n */
                                                     /* Bits 16-31: Reserved */
/* DMA Configuration Register */

#define GPDMA_CONFIG_ENA                  (1 << 0)  /* Bit 0:  DMA Controller enable */
#define GPDMA_CONFIG_M0                   (1 << 1)  /* Bit 1:  AHB Master 0 endianness configuration */
#define GPDMA_CONFIG_M1                   (1 << 2)  /* Bit 2:  M1 AHB Master 1 endianness configuration */
                                                    /* Bits 3-31: Reserved */
/* DMA Synchronization Register */

#define GPDMA_SYNC(n)                     (1 << (n)) /* Bits 0-15: Control synchrononization for DMA request n */
                                                     /* Bits 16-31: Reserved */
/* DMA Channel Source Address Register (32-bit address) */
/* DMA Channel Destination Address Register (32-bit address) */

/* DMA Channel Linked List Item Register */

#define GPDMA_LLI_LM                      (1 << 0)   /* Bit 0: LM AHB master select for loading the next LLI */
                                                     /* Bit 1: Reserved */
#define GPDMA_LLI_MASK                    0xfffffffc /* 31:2 LLI Linked list item */

/* DMA Channel Control Register */

#define GPDMA_CONTROL_XFRSIZE_SHIFT      (0)       /* Bits 0-11: Transfer size in number of transfers */
#define GPDMA_CONTROL_XFRSIZE_MASK       (0xfff << GPDMA_CONTROL_XFRSIZE_SHIFT)
#define GPDMA_CONTROL_SBSIZE_SHIFT       (12)      /* Bits 12-14: Source burst size */
#define GPDMA_CONTROL_SBSIZE_MASK        (7 << GPDMA_CONTROL_XFRSIZE_MASK)
#  define GPDMA_CONTROL_SBSIZE_1         (0 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 1 */
#  define GPDMA_CONTROL_SBSIZE_4         (1 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 4 */
#  define GPDMA_CONTROL_SBSIZE_8         (2 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 8 */
#  define GPDMA_CONTROL_SBSIZE_16        (3 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 16 */
#  define GPDMA_CONTROL_SBSIZE_32        (4 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 32 */
#  define GPDMA_CONTROL_SBSIZE_64        (5 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 64 */
#  define GPDMA_CONTROL_SBSIZE_128       (6 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 128 */
#  define GPDMA_CONTROL_SBSIZE_256       (7 << GPDMA_CONTROL_XFRSIZE_MASK) /* Source burst size = 256 */
#define GPDMA_CONTROL_DBSIZE_SHIFT       (15)      /* Bits 15-17: Destination burst size */
#define GPDMA_CONTROL_DBSIZE_MASK        (7 << GPDMA_CONTROL_DBSIZE_SHIFT)
#  define GPDMA_CONTROL_DBSIZE_1         (0 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 1 */
#  define GPDMA_CONTROL_DBSIZE_4         (1 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 4 */
#  define GPDMA_CONTROL_DBSIZE_8         (2 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 8 */
#  define GPDMA_CONTROL_DBSIZE_16        (3 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 16 */
#  define GPDMA_CONTROL_DBSIZE_32        (4 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 32 */
#  define GPDMA_CONTROL_DBSIZE_64        (5 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 64 */
#  define GPDMA_CONTROL_DBSIZE_128       (6 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 128 */
#  define GPDMA_CONTROL_DBSIZE_256       (7 << GPDMA_CONTROL_DBSIZE_SHIFT) /* Destination burst size = 256 */
#define GPDMA_CONTROL_SWIDTH_SHIFT       (18)       /* Bits 18-20: Source transfer width */
#define GPDMA_CONTROL_SWIDTH_MASK        (7 << GPDMA_CONTROL_SWIDTH_SHIFT)
#  define GPDMA_CONTROL_SWIDTH_BYTE      (0 << GPDMA_CONTROL_SWIDTH_SHIFT) /* Byte (8-bit) */
#  define GPDMA_CONTROL_SWIDTH_HWORD     (1 << GPDMA_CONTROL_SWIDTH_SHIFT) /* Halfword (16-bit) */
#  define GPDMA_CONTROL_SWIDTH_WORD      (2 << GPDMA_CONTROL_SWIDTH_SHIFT) /* Word (32-bit) */
#define GPDMA_CONTROL_DWIDTH_SHIFT       (21)       /* Bits 21-23: Destination transfer width */
#define GPDMA_CONTROL_DWIDTH_MASK        (7 << GPDMA_CONTROL_DWIDTH_SHIFT)
#  define GPDMA_CONTROL_DWIDTH_BYTE      (0 << GPDMA_CONTROL_DWIDTH_SHIFT) /* Byte (8-bit) */
#  define GPDMA_CONTROL_DWIDTH_HWORD     (1 << GPDMA_CONTROL_DWIDTH_SHIFT) /* Halfword (16-bit) */
#  define GPDMA_CONTROL_DWIDTH_WORD      (2 << GPDMA_CONTROL_DWIDTH_SHIFT) /* Word (32-bit) */
#define GPDMA_CONTROL_SS                 (1 << 24) /* Bit 24: Source AHB master select */
#define GPDMA_CONTROL_DS                 (1 << 25) /* Bit 25: Destination AHB master select */
#define GPDMA_CONTROL_SI                 (1 << 26) /* Bit 26: Source increment */
#define GPDMA_CONTROL_DI                 (1 << 27) /* Bit 27: Destination increment */
#define GPDMA_CONTROL_PROT1              (1 << 28) /* Bit 28: Privileged mode */
#define GPDMA_CONTROL_PROT2              (1 << 29) /* Bit 29: Bufferable */
#define GPDMA_CONTROL_PROT3              (1 << 30) /* Bit 30: Cacheable */
#define GPDMA_CONTROL_IE                 (1 << 31) /* Bit 31: Terminal count interrupt enable bit */

/* DMA Channel Configuration Register */

#define GPDMA_CONFIG_ENA                 (1 << 0)  /* Bit 0:  Channel enable */
#define GPDMA_CONFIG_SRCPER_SHIFT        (1)       /* Bits 1-5: Source peripheral */
#define GPDMA_CONFIG_SRCPER_MASK         (31 << GPDMA_CONFIG_SRCPER_SHIFT)
#  define GPDMA_CONFIG_SRCPER_SPIFI      (0 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SPIFI */
#  define GPDMA_CONFIG_SRCPER_SCTM3_1    (0 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SCT match3 */
#  define GPDMA_CONFIG_SRCPER_SGPIO14_1  (0 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SGPIO14 */
#  define GPDMA_CONFIG_SRCPER_T3MAT1_1   (0 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer3 match 1 */
#  define GPDMA_CONFIG_SRCPER_T0MAT0     (1 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer0 match 0 */
#  define GPDMA_CONFIG_SRCPER_U0TX_1     (1 << GPDMA_CONFIG_SRCPER_SHIFT)  /* USART0 transmit */
#  define GPDMA_CONFIG_SRCPER_T0MAT1     (2 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer0 match 1 */
#  define GPDMA_CONFIG_SRCPER_U0RX_1     (2 << GPDMA_CONFIG_SRCPER_SHIFT)  /* USART0 receive */
#  define GPDMA_CONFIG_SRCPER_T1MAT0     (3 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer1 match 0 */
#  define GPDMA_CONFIG_SRCPER_U1TX       (3 << GPDMA_CONFIG_SRCPER_SHIFT)  /* UART1 transmit */
#  define GPDMA_CONFIG_SRCPER_I2S1D1     (3 << GPDMA_CONFIG_SRCPER_SHIFT)  /* I2S1 DMA request 1 */
#  define GPDMA_CONFIG_SRCPER_SSP1TX_1   (3 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SSP1 transmit */
#  define GPDMA_CONFIG_SRCPER_T1MAT1     (4 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer1 match 1 */
#  define GPDMA_CONFIG_SRCPER_U1RX       (4 << GPDMA_CONFIG_SRCPER_SHIFT)  /* UART1 receive */
#  define GPDMA_CONFIG_SRCPER_I2S1D2     (4 << GPDMA_CONFIG_SRCPER_SHIFT)  /* I2S1 DMA request 2 */
#  define GPDMA_CONFIG_SRCPER_SSP1RX_1   (4 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SSP1 receive */
#  define GPDMA_CONFIG_SRCPER_T2MAT0     (5 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer2 match 0 */
#  define GPDMA_CONFIG_SRCPER_U2TX       (5 << GPDMA_CONFIG_SRCPER_SHIFT)  /* USART2 transmit */
#  define GPDMA_CONFIG_SRCPER_SSP1TX_2   (5 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SSP1 transmit */
#  define GPDMA_CONFIG_SRCPER_SGPIO15_1  (5 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SGPIO15 */
#  define GPDMA_CONFIG_SRCPER_T2MAT1     (6 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer2 match 1 */
#  define GPDMA_CONFIG_SRCPER_U2RX       (6 << GPDMA_CONFIG_SRCPER_SHIFT)  /* USART2 receive */
#  define GPDMA_CONFIG_SRCPER_SSP1RX_2   (6 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SSP1 receive */
#  define GPDMA_CONFIG_SRCPER_SGPIO14_2  (6 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SGPIO14 */
#  define GPDMA_CONFIG_SRCPER_T3MAT0_1   (7 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer3 match 0 */
#  define GPDMA_CONFIG_SRCPER_U3TX_1     (7 << GPDMA_CONFIG_SRCPER_SHIFT)  /* USART3 transmit */
#  define GPDMA_CONFIG_SRCPER_SCTD0_1    (7 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SCT DMA request 0 */
#  define GPDMA_CONFIG_SRCPER_VADCWR     (7 << GPDMA_CONFIG_SRCPER_SHIFT)  /* VADC write */
#  define GPDMA_CONFIG_SRCPER_T3MAT1_2   (8 << GPDMA_CONFIG_SRCPER_SHIFT)  /* Timer3 match 1 */
#  define GPDMA_CONFIG_SRCPER_U3RX_1     (8 << GPDMA_CONFIG_SRCPER_SHIFT)  /* USART3 receive */
#  define GPDMA_CONFIG_SRCPER_SCTD1_1    (8 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SCT DMA request 1 */
#  define GPDMA_CONFIG_SRCPER_VADCRD     (8 << GPDMA_CONFIG_SRCPER_SHIFT)  /* VADC read */
#  define GPDMA_CONFIG_SRCPER_SSP0RX     (9 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SSP0 receive */
#  define GPDMA_CONFIG_SRCPER_I2S0D1     (9 << GPDMA_CONFIG_SRCPER_SHIFT)  /* I2S0 DMA request 1 */
#  define GPDMA_CONFIG_SRCPER_SCTD1_2    (9 << GPDMA_CONFIG_SRCPER_SHIFT)  /* SCT DMA request 1 */
#  define GPDMA_CONFIG_SRCPER_SSP0TX     (10 << GPDMA_CONFIG_SRCPER_SHIFT) /* SSP0 transmit */
#  define GPDMA_CONFIG_SRCPER_I2S0D2     (10 << GPDMA_CONFIG_SRCPER_SHIFT) /* I2S0 DMA request 2 */
#  define GPDMA_CONFIG_SRCPER_SCTD0_2    (10 << GPDMA_CONFIG_SRCPER_SHIFT) /* SCT DMA request 0 */
#  define GPDMA_CONFIG_SRCPER_SSP1RX_3   (11 << GPDMA_CONFIG_SRCPER_SHIFT) /* SSP1 receive */
#  define GPDMA_CONFIG_SRCPER_SGPIO14_3  (11 << GPDMA_CONFIG_SRCPER_SHIFT) /* SGPIO14 */
#  define GPDMA_CONFIG_SRCPER_U0TX_2     (11 << GPDMA_CONFIG_SRCPER_SHIFT) /* USART0 transmit */
#  define GPDMA_CONFIG_SRCPER_SSP1TX_3   (12 << GPDMA_CONFIG_SRCPER_SHIFT) /* SSP1 transmit */
#  define GPDMA_CONFIG_SRCPER_SGPIO15_2  (12 << GPDMA_CONFIG_SRCPER_SHIFT) /* SGPIO15 */
#  define GPDMA_CONFIG_SRCPER_U0RX_2     (12 << GPDMA_CONFIG_SRCPER_SHIFT) /* USART0 receive */
#  define GPDMA_CONFIG_SRCPER_ADC0       (13 << GPDMA_CONFIG_SRCPER_SHIFT) /* ADC0 */
#  define GPDMA_CONFIG_SRCPER_SSP1RX_4   (13 << GPDMA_CONFIG_SRCPER_SHIFT) /* SSP1 receive */
#  define GPDMA_CONFIG_SRCPER_U3RX_2     (13 << GPDMA_CONFIG_SRCPER_SHIFT) /* USART3 receive */
#  define GPDMA_CONFIG_SRCPER_ADC1       (14 << GPDMA_CONFIG_SRCPER_SHIFT) /* ADC1 */
#  define GPDMA_CONFIG_SRCPER_SSP1TX_4   (14 << GPDMA_CONFIG_SRCPER_SHIFT) /* SSP1 transmit */
#  define GPDMA_CONFIG_SRCPER_U3TX_2     (14 << GPDMA_CONFIG_SRCPER_SHIFT) /* USART3 transmit */
#  define GPDMA_CONFIG_SRCPER_DAC        (15 << GPDMA_CONFIG_SRCPER_SHIFT) /* DAC */
#  define GPDMA_CONFIG_SRCPER_SCTM3_2    (15 << GPDMA_CONFIG_SRCPER_SHIFT) /* SCT match 3 */
#  define GPDMA_CONFIG_SRCPER_SGPIO15_3  (15 << GPDMA_CONFIG_SRCPER_SHIFT) /* SGPIO15 */
#  define GPDMA_CONFIG_SRCPER_T3MAT0_2   (15 << GPDMA_CONFIG_SRCPER_SHIFT) /* Timer3 match 0 */
#define GPDMA_CONFIG_DESTPER_SHIFT       (6)       /* Bits 6-10: Destination peripheral */
#define GPDMA_CONFIG_DESTPER_MASK        (31 << GPDMA_CONFIG_DESTPER_SHIFT)
#  define GPDMA_CONFIG_DESTPER_SPIFI     (0 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SPIFI */
#  define GPDMA_CONFIG_DESTPER_SCTM3_1   (0 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SCT match3 */
#  define GPDMA_CONFIG_DESTPER_SGPIO14_1 (0 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SGPIO14 */
#  define GPDMA_CONFIG_DESTPER_T3MAT1_1  (0 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer3 match 1 */
#  define GPDMA_CONFIG_DESTPER_T0MAT0    (1 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer0 match 0 */
#  define GPDMA_CONFIG_DESTPER_U0TX_1    (1 << GPDMA_CONFIG_DESTPER_SHIFT)  /* USART0 transmit */
#  define GPDMA_CONFIG_DESTPER_T0MAT1    (2 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer0 match 1 */
#  define GPDMA_CONFIG_DESTPER_U0RX_1    (2 << GPDMA_CONFIG_DESTPER_SHIFT)  /* USART0 receive */
#  define GPDMA_CONFIG_DESTPER_T1MAT0    (3 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer1 match 0 */
#  define GPDMA_CONFIG_DESTPER_U1TX      (3 << GPDMA_CONFIG_DESTPER_SHIFT)  /* UART1 transmit */
#  define GPDMA_CONFIG_DESTPER_I2S1D1    (3 << GPDMA_CONFIG_DESTPER_SHIFT)  /* I2S1 DMA request 1 */
#  define GPDMA_CONFIG_DESTPER_SSP1TX_1  (3 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SSP1 transmit */
#  define GPDMA_CONFIG_DESTPER_T1MAT1    (4 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer1 match 1 */
#  define GPDMA_CONFIG_DESTPER_U1RX      (4 << GPDMA_CONFIG_DESTPER_SHIFT)  /* UART1 receive */
#  define GPDMA_CONFIG_DESTPER_I2S1D2    (4 << GPDMA_CONFIG_DESTPER_SHIFT)  /* I2S1 DMA request 2 */
#  define GPDMA_CONFIG_DESTPER_SSP1RX_1  (4 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SSP1 receive */
#  define GPDMA_CONFIG_DESTPER_T2MAT0    (5 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer2 match 0 */
#  define GPDMA_CONFIG_DESTPER_U2TX      (5 << GPDMA_CONFIG_DESTPER_SHIFT)  /* USART2 transmit */
#  define GPDMA_CONFIG_DESTPER_SSP1TX_2  (5 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SSP1 transmit */
#  define GPDMA_CONFIG_DESTPER_SGPIO15_1 (5 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SGPIO15 */
#  define GPDMA_CONFIG_DESTPER_T2MAT1    (6 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer2 match 1 */
#  define GPDMA_CONFIG_DESTPER_U2RX      (6 << GPDMA_CONFIG_DESTPER_SHIFT)  /* USART2 receive */
#  define GPDMA_CONFIG_DESTPER_SSP1RX_2  (6 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SSP1 receive */
#  define GPDMA_CONFIG_DESTPER_SGPIO14_2 (6 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SGPIO14 */
#  define GPDMA_CONFIG_DESTPER_T3MAT0_1  (7 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer3 match 0 */
#  define GPDMA_CONFIG_DESTPER_U3TX_1    (7 << GPDMA_CONFIG_DESTPER_SHIFT)  /* USART3 transmit */
#  define GPDMA_CONFIG_DESTPER_SCTD0_1   (7 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SCT DMA request 0 */
#  define GPDMA_CONFIG_DESTPER_VADCWR    (7 << GPDMA_CONFIG_DESTPER_SHIFT)  /* VADC write */
#  define GPDMA_CONFIG_DESTPER_T3MAT1_2  (8 << GPDMA_CONFIG_DESTPER_SHIFT)  /* Timer3 match 1 */
#  define GPDMA_CONFIG_DESTPER_U3RX_1    (8 << GPDMA_CONFIG_DESTPER_SHIFT)  /* USART3 receive */
#  define GPDMA_CONFIG_DESTPER_SCTD1_1   (8 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SCT DMA request 1 */
#  define GPDMA_CONFIG_DESTPER_VADCRD    (8 << GPDMA_CONFIG_DESTPER_SHIFT)  /* VADC read */
#  define GPDMA_CONFIG_DESTPER_SSP0RX    (9 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SSP0 receive */
#  define GPDMA_CONFIG_DESTPER_I2S0D1    (9 << GPDMA_CONFIG_DESTPER_SHIFT)  /* I2S0 DMA request 1 */
#  define GPDMA_CONFIG_DESTPER_SCTD1_2   (9 << GPDMA_CONFIG_DESTPER_SHIFT)  /* SCT DMA request 1 */
#  define GPDMA_CONFIG_DESTPER_SSP0TX    (10 << GPDMA_CONFIG_DESTPER_SHIFT) /* SSP0 transmit */
#  define GPDMA_CONFIG_DESTPER_I2S0D2    (10 << GPDMA_CONFIG_DESTPER_SHIFT) /* I2S0 DMA request 2 */
#  define GPDMA_CONFIG_DESTPER_SCTD0_2   (10 << GPDMA_CONFIG_DESTPER_SHIFT) /* SCT DMA request 0 */
#  define GPDMA_CONFIG_DESTPER_SSP1RX_3  (11 << GPDMA_CONFIG_DESTPER_SHIFT) /* SSP1 receive */
#  define GPDMA_CONFIG_DESTPER_SGPIO14_3 (11 << GPDMA_CONFIG_DESTPER_SHIFT) /* SGPIO14 */
#  define GPDMA_CONFIG_DESTPER_U0TX_2    (11 << GPDMA_CONFIG_DESTPER_SHIFT) /* USART0 transmit */
#  define GPDMA_CONFIG_DESTPER_SSP1TX_3  (12 << GPDMA_CONFIG_DESTPER_SHIFT) /* SSP1 transmit */
#  define GPDMA_CONFIG_DESTPER_SGPIO15_2 (12 << GPDMA_CONFIG_DESTPER_SHIFT) /* SGPIO15 */
#  define GPDMA_CONFIG_DESTPER_U0RX_2    (12 << GPDMA_CONFIG_DESTPER_SHIFT) /* USART0 receive */
#  define GPDMA_CONFIG_DESTPER_ADC0      (13 << GPDMA_CONFIG_DESTPER_SHIFT) /* ADC0 */
#  define GPDMA_CONFIG_DESTPER_SSP1RX_4  (13 << GPDMA_CONFIG_DESTPER_SHIFT) /* SSP1 receive */
#  define GPDMA_CONFIG_DESTPER_U3RX_2    (13 << GPDMA_CONFIG_DESTPER_SHIFT) /* USART3 receive */
#  define GPDMA_CONFIG_DESTPER_ADC1      (14 << GPDMA_CONFIG_DESTPER_SHIFT) /* ADC1 */
#  define GPDMA_CONFIG_DESTPER_SSP1TX_4  (14 << GPDMA_CONFIG_DESTPER_SHIFT) /* SSP1 transmit */
#  define GPDMA_CONFIG_DESTPER_U3TX_2    (14 << GPDMA_CONFIG_DESTPER_SHIFT) /* USART3 transmit */
#  define GPDMA_CONFIG_DESTPER_DAC       (15 << GPDMA_CONFIG_DESTPER_SHIFT) /* DAC */
#  define GPDMA_CONFIG_DESTPER_SCTM3_2   (15 << GPDMA_CONFIG_DESTPER_SHIFT) /* SCT match 3 */
#  define GPDMA_CONFIG_DESTPER_SGPIO15_3 (15 << GPDMA_CONFIG_DESTPER_SHIFT) /* SGPIO15 */
#  define GPDMA_CONFIG_DESTPER_T3MAT0_2  (15 << GPDMA_CONFIG_DESTPER_SHIFT) /* Timer3 match 0 */
#define GPDMA_CONFIG_FCNTRL_SHIFT        (11)       /* Bits 11-13: Flow control and transfer type */
#define GPDMA_CONFIG_FCNTRL_MASK         (7 << GPDMA_CONFIG_FCNTRL_SHIFT)
#  define GPDMA_CONFIG_FCNTRL_M2M_DMA    (0 << GPDMA_CONFIG_FCNTRL_SHIFT) /* Memory to memory (DMA control) */
#  define GPDMA_CONFIG_FCNTRL_M2P_DMA    (1 << GPDMA_CONFIG_FCNTRL_SHIFT) /* Memory to peripheral (DMA control) */
#  define GPDMA_CONFIG_FCNTRL_P2M_DMA    (2 << GPDMA_CONFIG_FCNTRL_SHIFT) /* Peripheral to memory (DMA control) */
#  define GPDMA_CONFIG_FCNTRL_P2P_DMA    (3 << GPDMA_CONFIG_FCNTRL_SHIFT) /* SRC peripheral to DEST peripheral (DMA control) */
#  define GPDMA_CONFIG_FCNTRL_P2P_DEST   (4 << GPDMA_CONFIG_FCNTRL_SHIFT) /* SRC peripheral to DEST peripheral (DEST control) */
#  define GPDMA_CONFIG_FCNTRL_M2P_PER    (5 << GPDMA_CONFIG_FCNTRL_SHIFT) /* Memory to peripheral (peripheral control) */
#  define GPDMA_CONFIG_FCNTRL_P2M_PER    (6 << GPDMA_CONFIG_FCNTRL_SHIFT) /* Peripheral to memory (peripheral control) */
#  define GPDMA_CONFIG_FCNTRL_P2P_SRC    (7 << GPDMA_CONFIG_FCNTRL_SHIFT) /* SRC peripheral to DEST peripheral (SRC control) */
#define GPDMA_CONFIG_IE                  (1 << 14) /* Bit 14: Interrupt error mask */
#define GPDMA_CONFIG_ITC                 (1 << 15) /* Bit 15: Terminal count interrupt mask */
#define GPDMA_CONFIG_LOCK                (1 << 16) /* Bit 16: Lock */
#define GPDMA_CONFIG_ACTIVE              (1 << 17) /* Bit 17: Active */
#define GPDMA_CONFIG_HALT                (1 << 18) /* Bit 18: Halt */
                                                   /* Bits 19-31: Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GPDMA_H */
