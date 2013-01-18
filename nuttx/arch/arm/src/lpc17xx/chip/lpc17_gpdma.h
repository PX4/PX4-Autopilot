/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_gpdma.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_GPDMA_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_GPDMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

/* General registers (see also LPC17_SYSCON_DMAREQSEL_OFFSET in lpc17_syscon.h) */

#define LPC17_DMA_INTST_OFFSET        0x0000 /* DMA Interrupt Status Register */
#define LPC17_DMA_INTTCST_OFFSET      0x0004 /* DMA Interrupt Terminal Count Request Status Register */
#define LPC17_DMA_INTTCCLR_OFFSET     0x0008 /* DMA Interrupt Terminal Count Request Clear Register */
#define LPC17_DMA_INTERRST_OFFSET     0x000c /* DMA Interrupt Error Status Register */
#define LPC17_DMA_INTERRCLR_OFFSET    0x0010 /* DMA Interrupt Error Clear Register */
#define LPC17_DMA_RAWINTTCST_OFFSET   0x0014 /* DMA Raw Interrupt Terminal Count Status Register */
#define LPC17_DMA_RAWINTERRST_OFFSET  0x0018 /* DMA Raw Error Interrupt Status Register */
#define LPC17_DMA_ENBLDCHNS_OFFSET    0x001c /* DMA Enabled Channel Register */
#define LPC17_DMA_SOFTBREQ_OFFSET     0x0020 /* DMA Software Burst Request Register */
#define LPC17_DMA_SOFTSREQ_OFFSET     0x0024 /* DMA Software Single Request Register */
#define LPC17_DMA_SOFTLBREQ_OFFSET    0x0028 /* DMA Software Last Burst Request Register */
#define LPC17_DMA_SOFTLSREQ_OFFSET    0x002c /* DMA Software Last Single Request Register */
#define LPC17_DMA_CONFIG_OFFSET       0x0030 /* DMA Configuration Register */
#define LPC17_DMA_SYNC_OFFSET         0x0034 /* DMA Synchronization Register */

/* Channel Registers */

#define LPC17_DMA_CHAN_OFFSET(n)      (0x0100 + ((n) << 5)) /* n=0,1,...7 */

#define LPC17_DMACH_SRCADDR_OFFSET    0x0000 /* DMA Channel Source Address Register */
#define LPC17_DMACH_DESTADDR_OFFSET   0x0004 /* DMA Channel Destination Address Register */
#define LPC17_DMACH_LLI_OFFSET        0x0008 /* DMA Channel Linked List Item Register */
#define LPC17_DMACH_CONTROL_OFFSET    0x000c /* DMA Channel Control Register */
#define LPC17_DMACH_CONFIG_OFFSET     0x0010 /* DMA Channel Configuration Register */

#define LPC17_DMACH0_SRCADDR_OFFSET   (0x100+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH0_DESTADDR_OFFSET  (0x100+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH0_LLI_OFFSET       (0x100+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH0_CONTROL_OFFSET   (0x100+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH0_CONFIG_OFFSET    (0x100+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH1_SRCADDR_OFFSET   (0x120+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH1_DESTADDR_OFFSET  (0x120+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH1_LLI_OFFSET       (0x120+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH1_CONTROL_OFFSET   (0x120+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH1_CONFIG_OFFSET    (0x120+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH2_SRCADDR_OFFSET   (0x140+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH2_DESTADDR_OFFSET  (0x140+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH2_LLI_OFFSET       (0x140+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH2_CONTROL_OFFSET   (0x140+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH2_CONFIG_OFFSET    (0x140+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH3_SRCADDR_OFFSET   (0x160+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH3_DESTADDR_OFFSET  (0x160+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH3_LLI_OFFSET       (0x160+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH3_CONTROL_OFFSET   (0x160+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH3_CONFIG_OFFSET    (0x160+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH4_SRCADDR_OFFSET   (0x180+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH4_DESTADDR_OFFSET  (0x180+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH4_LLI_OFFSET       (0x180+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH4_CONTROL_OFFSET   (0x180+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH4_CONFIG_OFFSET    (0x180+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH5_SRCADDR_OFFSET   (0x1a0+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH5_DESTADDR_OFFSET  (0x1a0+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH5_LLI_OFFSET       (0x1a0+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH5_CONTROL_OFFSET   (0x1a0+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH5_CONFIG_OFFSET    (0x1a0+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH6_SRCADDR_OFFSET   (0x1c0+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH6_DESTADDR_OFFSET  (0x1c0+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH6_LLI_OFFSET       (0x1c0+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH6_CONTROL_OFFSET   (0x1c0+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH6_CONFIG_OFFSET    (0x1c0+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH7_SRCADDR_OFFSET   (0x1e0+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH7_DESTADDR_OFFSET  (0x1e0+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH7_LLI_OFFSET       (0x1e0+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH7_CONTROL_OFFSET   (0x1e0+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH7_CONFIG_OFFSET    (0x1e0+LPC17_DMACH_CONFIG_OFFSET)

/* Register addresses ***************************************************************/
/* General registers (see also LPC17_SYSCON_DMAREQSEL in lpc17_syscon.h) */

#define LPC17_DMA_INTST               (LPC17_GPDMA_BASE+LPC17_DMA_INTST_OFFSET)
#define LPC17_DMA_INTTCST             (LPC17_GPDMA_BASE+LPC17_DMA_INTTCST_OFFSET)
#define LPC17_DMA_INTTCCLR            (LPC17_GPDMA_BASE+LPC17_DMA_INTTCCLR_OFFSET)
#define LPC17_DMA_INTERRST            (LPC17_GPDMA_BASE+LPC17_DMA_INTERRST_OFFSET)
#define LPC17_DMA_INTERRCLR           (LPC17_GPDMA_BASE+LPC17_DMA_INTERRCLR_OFFSET)
#define LPC17_DMA_RAWINTTCST          (LPC17_GPDMA_BASE+LPC17_DMA_RAWINTTCST_OFFSET)
#define LPC17_DMA_RAWINTERRST         (LPC17_GPDMA_BASE+LPC17_DMA_RAWINTERRST_OFFSET)
#define LPC17_DMA_ENBLDCHNS           (LPC17_GPDMA_BASE+LPC17_DMA_ENBLDCHNS_OFFSET)
#define LPC17_DMA_SOFTBREQ            (LPC17_GPDMA_BASE+LPC17_DMA_SOFTBREQ_OFFSET)
#define LPC17_DMA_SOFTSREQ            (LPC17_GPDMA_BASE+LPC17_DMA_SOFTSREQ_OFFSET)
#define LPC17_DMA_SOFTLBREQ           (LPC17_GPDMA_BASE+LPC17_DMA_SOFTLBREQ_OFFSET)
#define LPC17_DMA_SOFTLSREQ           (LPC17_GPDMA_BASE+LPC17_DMA_SOFTLSREQ_OFFSET)
#define LPC17_DMA_CONFIG              (LPC17_GPDMA_BASE+LPC17_DMA_CONFIG_OFFSET)
#define LPC17_DMA_SYNC                (LPC17_GPDMA_BASE+LPC17_DMA_SYNC_OFFSET)

/* Channel Registers */

#define LPC17_DMACH_BASE(n)           (LPC17_GPDMA_BASE+LPC17_DMA_CHAN_OFFSET(n))

#define LPC17_DMACH_SRCADDR(n)        (LPC17_DMACH_BASE(n)+LPC17_DMACH_SRCADDR_OFFSET)
#define LPC17_DMACH_DESTADDR(n)       (LPC17_DMACH_BASE(n)+LPC17_DMACH_DESTADDR_OFFSET)
#define LPC17_DMACH_LLI(n)            (LPC17_DMACH_BASE(n)+LPC17_DMACH_LLI_OFFSET)
#define LPC17_DMACH_CONTROL(n)        (LPC17_DMACH_BASE(n)+LPC17_DMACH_CONTROL_OFFSET)
#define LPC17_DMACH_CONFIG(n)         (LPC17_DMACH_BASE(n)+LPC17_DMACH_CONFIG_OFFSET)

#define LPC17_DMACH0_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH0_SRCADDR_OFFSET)
#define LPC17_DMACH0_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH0_DESTADDR_OFFSET)
#define LPC17_DMACH0_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH0_LLI_OFFSET)
#define LPC17_DMACH0_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH0_CONTROL_OFFSET)
#define LPC17_DMACH0_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH0_CONFIG_OFFSET)

#define LPC17_DMACH1_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH1_SRCADDR_OFFSET)
#define LPC17_DMACH1_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH1_DESTADDR_OFFSET)
#define LPC17_DMACH1_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH1_LLI_OFFSET)
#define LPC17_DMACH1_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH1_CONTROL_OFFSET)
#define LPC17_DMACH1_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH1_CONFIG_OFFSET)

#define LPC17_DMACH2_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH2_SRCADDR_OFFSET)
#define LPC17_DMACH2_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH2_DESTADDR_OFFSET)
#define LPC17_DMACH2_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH2_LLI_OFFSET)
#define LPC17_DMACH2_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH2_CONTROL_OFFSET)
#define LPC17_DMACH2_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH2_CONFIG_OFFSET)

#define LPC17_DMACH3_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH3_SRCADDR_OFFSET)
#define LPC17_DMACH3_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH3_DESTADDR_OFFSET)
#define LPC17_DMACH3_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH3_LLI_OFFSET)
#define LPC17_DMACH3_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH3_CONTROL_OFFSET)
#define LPC17_DMACH3_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH3_CONFIG_OFFSET)

#define LPC17_DMACH4_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH4_SRCADDR_OFFSET)
#define LPC17_DMACH4_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH4_DESTADDR_OFFSET)
#define LPC17_DMACH4_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH4_LLI_OFFSET)
#define LPC17_DMACH4_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH4_CONTROL_OFFSET)
#define LPC17_DMACH4_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH4_CONFIG_OFFSET)

#define LPC17_DMACH5_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH5_SRCADDR_OFFSET)
#define LPC17_DMACH5_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH5_DESTADDR_OFFSET)
#define LPC17_DMACH5_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH5_LLI_OFFSET)
#define LPC17_DMACH5_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH5_CONTROL_OFFSET)
#define LPC17_DMACH5_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH5_CONFIG_OFFSET)

#define LPC17_DMACH6_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH6_SRCADDR_OFFSET)
#define LPC17_DMACH6_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH6_DESTADDR_OFFSET)
#define LPC17_DMACH6_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH6_LLI_OFFSET)
#define LPC17_DMACH6_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH6_CONTROL_OFFSET)
#define LPC17_DMACH6_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH6_CONFIG_OFFSET)

#define LPC17_DMACH7_SRCADDR          (LPC17_GPDMA_BASE+LPC17_DMACH7_SRCADDR_OFFSET)
#define LPC17_DMACH7_DESTADDR         (LPC17_GPDMA_BASE+LPC17_DMACH7_DESTADDR_OFFSET)
#define LPC17_DMACH7_LLI              (LPC17_GPDMA_BASE+LPC17_DMACH7_LLI_OFFSET)
#define LPC17_DMACH7_CONTROL          (LPC17_GPDMA_BASE+LPC17_DMACH7_CONTROL_OFFSET)
#define LPC17_DMACH7_CONFIG           (LPC17_GPDMA_BASE+LPC17_DMACH7_CONFIG_OFFSET)

/* Register bit definitions *********************************************************/
/* DMA request connections */

#define DMA_REQ_SSP0TX                (0)
#define DMA_REQ_SSP0RX                (1)
#define DMA_REQ_SSP1TX                (2)
#define DMA_REQ_SSP1RX                (3)
#define DMA_REQ_ADC                   (4)
#define DMA_REQ_I2SCH0                (5)
#define DMA_REQ_I2SCH1                (6)
#define DMA_REQ_DAC                   (7)

#define DMA_REQ_UART0TX               (8)
#define DMA_REQ_UART0RX               (9)
#define DMA_REQ_UART1TX               (10)
#define DMA_REQ_UART1RX               (11)
#define DMA_REQ_UART2TX               (12)
#define DMA_REQ_UART2RX               (13)
#define DMA_REQ_UART3TX               (14)
#define DMA_REQ_UART3RX               (15)

#define DMA_REQ_MAT0p0                (8)
#define DMA_REQ_MAT0p1                (9)
#define DMA_REQ_MAT1p0                (10)
#define DMA_REQ_MAT1p1                (11)
#define DMA_REQ_MAT2p0                (12)
#define DMA_REQ_MAT2p1                (13)
#define DMA_REQ_MAT3p0                (14)
#define DMA_REQ_MAT3p1                (15)

/* General registers (see also LPC17_SYSCON_DMAREQSEL in lpc17_syscon.h) */
/* Fach of the following registers, bits 0-7 controls DMA channels 9-7,
 * respectively.  Bits 8-31 are reserved.
 *
 *   DMA Interrupt Status Register
 *   DMA Interrupt Terminal Count Request Status Register
 *   DMA Interrupt Terminal Count Request Clear Register
 *   DMA Interrupt Error Status Register
 *   DMA Interrupt Error Clear Register
 *   DMA Raw Interrupt Terminal Count Status Register
 *   DMA Raw Error Interrupt Status Register
 *   DMA Enabled Channel Register
 */

#define DMACH(n)                      (1 << (n)) /* n=0,1,...7 */

/* For each of the following registers, bits 0-15 represent a set of encoded
 * DMA sources. Bits 16-31 are reserved in each case.
 *
 *   DMA Software Burst Request Register
 *   DMA Software Single Request Register
 *   DMA Software Last Burst Request Register
 *   DMA Software Last Single Request Register
 *   DMA Synchronization Register
 */

#define DMA_REQ_SSP0TX_BIT            (1 << DMA_REQ_SSP0TX)
#define DMA_REQ_SSP0RX_BIT            (1 << DMA_REQ_SSP0RX)
#define DMA_REQ_SSP1TX_BIT            (1 << DMA_REQ_SSP1TX)
#define DMA_REQ_SSP1RX_BIT            (1 << DMA_REQ_SSP0RX)
#define DMA_REQ_ADC_BIT               (1 << DMA_REQ_ADC)
#define DMA_REQ_I2SCH0_BIT            (1 << DMA_REQ_I2SCH0)
#define DMA_REQ_I2SCH1_BIT            (1 << DMA_REQ_I2SCH1)
#define DMA_REQ_DAC_BIT               (1 << DMA_REQ_DAC)

#define DMA_REQ_UART0TX_BIT           (1 << DMA_REQ_UART0TX)
#define DMA_REQ_UART0RX_BIT           (1 << DMA_REQ_UART0RX)
#define DMA_REQ_UART1TX_BIT           (1 << DMA_REQ_UART1TX)
#define DMA_REQ_UART1RX_BIT           (1 << DMA_REQ_UART1RX)
#define DMA_REQ_UART2TX_BIT           (1 << DMA_REQ_UART2TX)
#define DMA_REQ_UART2RX_BIT           (1 << DMA_REQ_UART2RX)
#define DMA_REQ_UART3TX_BIT           (1 << DMA_REQ_UART3TX)
#define DMA_REQ_UART3RX_BIT           (1 << DMA_REQ_UART3RX)

#define DMA_REQ_MAT0p0_BIT            (1 << DMA_REQ_MAT0p0)
#define DMA_REQ_MAT0p1_BIT            (1 << DMA_REQ_MAT0p1)
#define DMA_REQ_MAT1p0_BIT            (1 << DMA_REQ_MAT1p0)
#define DMA_REQ_MAT1p1_BIT            (1 << DMA_REQ_MAT1p1)
#define DMA_REQ_MAT2p0_BIT            (1 << DMA_REQ_MAT2p0)
#define DMA_REQ_MAT2p1_BIT            (1 << DMA_REQ_MAT2p1)
#define DMA_REQ_MAT3p0_BIT            (1 << DMA_REQ_MAT3p0)
#define DMA_REQ_MAT3p1_BIT            (1 << DMA_REQ_MAT3p1)

/* DMA Configuration Register */

#define DMA_CONFIG_E                  (1 << 0)  /* Bit 0:  DMA Controller enable */
#define DMA_CONFIG_M                  (1 << 1)  /* Bit 1:  AHB Master endianness configuration */
                                                /* Bits 2-31: Reserved */
/* Channel Registers */

/* DMA Channel Source Address Register (Bits 0-31: Source Address) */
/* DMA Channel Destination Address Register Bits 0-31: Destination Address) */
/* DMA Channel Linked List Item Register (Bits 0-31: Address of next link list
 * item.  Bits 0-1 must be zero.
 */

/* DMA Channel Control Register */

#define DMACH_CONTROL_XFRSIZE_SHIFT   (0)       /* Bits 0-11: Transfer size */
#define DMACH_CONTROL_XFRSIZE_MASK    (0x0fff << DMACH_CONTROL_XFRSIZE_SHIFT)
#define DMACH_CONTROL_SBSIZE_SHIFT    (12)      /* Bits 12-14: Source burst size */
#define DMACH_CONTROL_SBSIZE_MASK     (7 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_1      (0 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_4      (1 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_8      (2 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_16     (3 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_32     (4 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_64     (5 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_128    (6 << DMACH_CONTROL_SBSIZE_SHIFT)
#  define DMACH_CONTROL_SBSIZE_256    (7 << DMACH_CONTROL_SBSIZE_SHIFT)
#define DMACH_CONTROL_DBSIZE_SHIFT    (15)      /* Bits 15-17: Destination burst size */
#define DMACH_CONTROL_DBSIZE_MASK     (7 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_1      (0 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_4      (1 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_8      (2 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_16     (3 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_32     (4 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_64     (5 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_128    (6 << DMACH_CONTROL_DBSIZE_SHIFT)
#  define DMACH_CONTROL_DBSIZE_256    (7 << DMACH_CONTROL_DBSIZE_SHIFT)
#define DMACH_CONTROL_SWIDTH_SHIFT    (18)      /* Bits 18-20: Source transfer width */
#define DMACH_CONTROL_SWIDTH_MASK     (7 << DMACH_CONTROL_SWIDTH_SHIFT)
#define DMACH_CONTROL_DWIDTH_SHIFT    (21)      /* Bits 21-23: Destination transfer width */
#define DMACH_CONTROL_DWIDTH_MASK     (7 << DMACH_CONTROL_DWIDTH_SHIFT)
#define DMACH_CONTROL_SI              (1 << 26) /* Bit 26: Source increment */
#define DMACH_CONTROL_DI              (1 << 27) /* Bit 27: Destination increment */
#define DMACH_CONTROL_PROT1           (1 << 28) /* Bit 28: User/priviledged mode */
#define DMACH_CONTROL_PROT2           (1 << 29) /* Bit 29: Bufferable */
#define DMACH_CONTROL_PROT3           (1 << 30) /* Bit 30: Cacheable */
#define DMACH_CONTROL_I               (1 << 31) /* Bit 31: Terminal count interrupt enable */

/* DMA Channel Configuration Register */


#define DMACH_CONFIG_E                (1 << 0) /* Bit 0: Channel enable */
#define DMACH_CONFIG_SRCPER_SHIFT     (1)      /* Bits 1-5: Source peripheral */
#define DMACH_CONFIG_SRCPER_MASK      (31 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP0TX  (DMA_REQ_SSP0TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP0RX  (DMA_REQ_SSP0RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP1TX  (DMA_REQ_SSP1TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_SSP1RX  (DMA_REQ_SSP0RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_ADC     (DMA_REQ_ADC << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_I2SCH0  (DMA_REQ_I2SCH0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_I2SCH1  (DMA_REQ_I2SCH1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_DAC     (DMA_REQ_DAC << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART0TX (DMA_REQ_UART0TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART0RX (DMA_REQ_UART0RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART1TX (DMA_REQ_UART1TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART1RX (DMA_REQ_UART1RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART2TX (DMA_REQ_UART2TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART2RX (DMA_REQ_UART2RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART3TX (DMA_REQ_UART3TX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_UART3RX (DMA_REQ_UART3RX << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT0p0  (DMA_REQ_MAT0p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT0p1  (DMA_REQ_MAT0p1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT1p0  (DMA_REQ_MAT1p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT1p1  (DMA_REQ_MAT1p1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT2p0  (DMA_REQ_MAT2p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT2p1  (DMA_REQ_MAT2p1 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT3p0  (DMA_REQ_MAT3p0 << DMACH_CONFIG_SRCPER_SHIFT)
#  define DMACH_CONFIG_SRCPER_MAT3p1  (DMA_REQ_MAT3p1 << DMACH_CONFIG_SRCPER_SHIFT)
#define DMACH_CONFIG_DSTPER_SHIFT     (6)      /* Bits 6-10: Source peripheral */
#define DMACH_CONFIG_DSTPER_MASK      (31 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP0TX  (DMA_REQ_SSP0TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP0RX  (DMA_REQ_SSP0RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP1TX  (DMA_REQ_SSP1TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_SSP1RX  (DMA_REQ_SSP0RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_ADC     (DMA_REQ_ADC << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_I2SCH0  (DMA_REQ_I2SCH0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_I2SCH1  (DMA_REQ_I2SCH1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_DAC     (DMA_REQ_DAC << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART0TX (DMA_REQ_UART0TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART0RX (DMA_REQ_UART0RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART1TX (DMA_REQ_UART1TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART1RX (DMA_REQ_UART1RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART2TX (DMA_REQ_UART2TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART2RX (DMA_REQ_UART2RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART3TX (DMA_REQ_UART3TX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_UART3RX (DMA_REQ_UART3RX << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT0p0  (DMA_REQ_MAT0p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT0p1  (DMA_REQ_MAT0p1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT1p0  (DMA_REQ_MAT1p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT1p1  (DMA_REQ_MAT1p1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT2p0  (DMA_REQ_MAT2p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT2p1  (DMA_REQ_MAT2p1 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT3p0  (DMA_REQ_MAT3p0 << DMACH_CONFIG_DSTPER_SHIFT)
#  define DMACH_CONFIG_DSTPER_MAT3p1  (DMA_REQ_MAT3p1 << DMACH_CONFIG_DSTPER_SHIFT)
#define DMACH_CONFIG_XFRTYPE_SHIFT    (11)      /* Bits 11-13: Type of transfer */
#define DMACH_CONFIG_XFRTYPE_MASK     (7 << DMACH_CONFIG_XFRTYPE_SHIFT)
#  define DMACH_CONFIG_XFRTYPE_M2M    (0 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Memory to memory DMA */
#  define DMACH_CONFIG_XFRTYPE_M2P    (1 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Memory to peripheral DMA */
#  define DMACH_CONFIG_XFRTYPE_P2M    (2 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Peripheral to memory DMA */
#  define DMACH_CONFIG_XFRTYPE_P2P    (3 << DMACH_CONFIG_XFRTYPE_SHIFT) /* Peripheral to peripheral DMA */
#define DMACH_CONFIG_IE               (1 << 14) /* Bit 14: Interrupt error mask */
#define DMACH_CONFIG_ ITC             (1 << 15) /* Bit 15: Terminal count interrupt mask */
#define DMACH_CONFIG_L                (1 << 16) /* Bit 16: Lock */
#define DMACH_CONFIG_A                (1 << 17) /* Bit 17: Active */
#define DMACH_CONFIG_H                (1 << 18) /* Bit 18: Halt */
                                                /* Bits 19-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_GPDMA_H */
