/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f20xxx_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32F20XXX_DMA_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32F20XXX_DMA_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* 2 DMA controllers */

#define DMA1                      (0)
#define DMA2                      (1)

/* 8 DMA streams */

#define DMA_STREAM0               (0)
#define DMA_STREAM1               (1)
#define DMA_STREAM2               (2)
#define DMA_STREAM3               (3)
#define DMA_STREAM4               (4)
#define DMA_STREAM5               (5)
#define DMA_STREAM6               (6)
#define DMA_STREAM7               (7)

/* 8 DMA channels */

#define DMA_CHAN0                 (0)
#define DMA_CHAN1                 (1)
#define DMA_CHAN2                 (2)
#define DMA_CHAN3                 (3)
#define DMA_CHAN4                 (4)
#define DMA_CHAN5                 (5)
#define DMA_CHAN6                 (6)
#define DMA_CHAN7                 (7)

/* Register Offsets *****************************************************************/

#define STM32_DMA_LISR_OFFSET     0x0000 /* DMA low interrupt status register */
#define STM32_DMA_HISR_OFFSET     0x0004 /* DMA high interrupt status register */
#define STM32_DMA_LIFCR_OFFSET    0x0008 /* DMA low interrupt flag clear register */
#define STM32_DMA_HIFCR_OFFSET    0x000c /* DMA high interrupt flag clear register */

#define STM32_DMA_OFFSET(n)       (0x0010+0x0018*(n))
#define STM32_DMA_SCR_OFFSET      0x0000 /* DMA stream n configuration register */
#define STM32_DMA_SNDTR_OFFSET    0x0004 /* DMA stream n number of data register */
#define STM32_DMA_SPAR_OFFSET     0x0008 /* DMA stream n peripheral address register */
#define STM32_DMA_SM0AR_OFFSET    0x000c /* DMA stream n memory 0 address register */
#define STM32_DMA_SM1AR_OFFSET    0x0010 /* DMA stream n memory 1 address register */
#define STM32_DMA_SFCR_OFFSET     0x0014 /* DMA stream n FIFO control register */

#define STM32_DMA_S0CR_OFFSET     0x0010 /* DMA stream 0 configuration register */
#define STM32_DMA_S1CR_OFFSET     0x0028 /* DMA stream 1 configuration register */
#define STM32_DMA_S2CR_OFFSET     0x0040 /* DMA stream 2 configuration register */
#define STM32_DMA_S3CR_OFFSET     0x0058 /* DMA stream 3 configuration register */
#define STM32_DMA_S4CR_OFFSET     0x0070 /* DMA stream 4 configuration register */
#define STM32_DMA_S5CR_OFFSET     0x0088 /* DMA stream 5 configuration register */
#define STM32_DMA_S6CR_OFFSET     0x00a0 /* DMA stream 6 configuration register */
#define STM32_DMA_S7CR_OFFSET     0x00b8 /* DMA stream 7 configuration register */ 

#define STM32_DMA_S0NDTR_OFFSET   0x0014 /* DMA stream 0 number of data register */
#define STM32_DMA_S1NDTR_OFFSET   0x002c /* DMA stream 1 number of data register */
#define STM32_DMA_S2NDTR_OFFSET   0x0044 /* DMA stream 2 number of data register */
#define STM32_DMA_S3NDTR_OFFSET   0x005c /* DMA stream 3 number of data register */
#define STM32_DMA_S4NDTR_OFFSET   0x0074 /* DMA stream 4 number of data register */
#define STM32_DMA_S5NDTR_OFFSET   0x008c /* DMA stream 5 number of data register */
#define STM32_DMA_S6NDTR_OFFSET   0x00a4 /* DMA stream 6 number of data register */
#define STM32_DMA_S7NDTR_OFFSET   0x00bc /* DMA stream 7 number of data register */

#define STM32_DMA_S0PAR_OFFSET    0x0018 /* DMA stream 0 peripheral address register */
#define STM32_DMA_S1PAR_OFFSET    0x0030 /* DMA stream 1 peripheral address register */
#define STM32_DMA_S2PAR_OFFSET    0x0048 /* DMA stream 2 peripheral address register */
#define STM32_DMA_S3PAR_OFFSET    0x0060 /* DMA stream 3 peripheral address register */
#define STM32_DMA_S4PAR_OFFSET    0x0078 /* DMA stream 4 peripheral address register */
#define STM32_DMA_S5PAR_OFFSET    0x0090 /* DMA stream 5 peripheral address register */
#define STM32_DMA_S6PAR_OFFSET    0x00a8 /* DMA stream 6 peripheral address register */
#define STM32_DMA_S7PAR_OFFSET    0x00c0 /* DMA stream 7 peripheral address register */

#define STM32_DMA_S0M0AR_OFFSET   0x001c /* DMA stream 0 memory 0 address register */
#define STM32_DMA_S1M0AR_OFFSET   0x0034 /* DMA stream 1 memory 0 address register */
#define STM32_DMA_S2M0AR_OFFSET   0x004c /* DMA stream 2 memory 0 address register */
#define STM32_DMA_S3M0AR_OFFSET   0x0064 /* DMA stream 3 memory 0 address register */
#define STM32_DMA_S4M0AR_OFFSET   0x007c /* DMA stream 4 memory 0 address register */
#define STM32_DMA_S5M0AR_OFFSET   0x0094 /* DMA stream 5 memory 0 address register */
#define STM32_DMA_S6M0AR_OFFSET   0x00ac /* DMA stream 6 memory 0 address register */
#define STM32_DMA_S7M0AR_OFFSET   0x00c4 /* DMA stream 7 memory 0 address register */

#define STM32_DMA_S0M1AR_OFFSET   0x0020 /* DMA stream 0 memory 1 address register */
#define STM32_DMA_S1M1AR_OFFSET   0x0038 /* DMA stream 1 memory 1 address register */
#define STM32_DMA_S2M1AR_OFFSET   0x0050 /* DMA stream 2 memory 1 address register */
#define STM32_DMA_S3M1AR_OFFSET   0x0068 /* DMA stream 3 memory 1 address register */
#define STM32_DMA_S4M1AR_OFFSET   0x0080 /* DMA stream 4 memory 1 address register */
#define STM32_DMA_S5M1AR_OFFSET   0x0098 /* DMA stream 5 memory 1 address register */
#define STM32_DMA_S6M1AR_OFFSET   0x00b0 /* DMA stream 6 memory 1 address register */
#define STM32_DMA_S7M1AR_OFFSET   0x00c8 /* DMA stream 7 memory 1 address register */

#define STM32_DMA_S0FCR_OFFSET    0x0024 /* DMA stream 0 FIFO control register */
#define STM32_DMA_S1FCR_OFFSET    0x003c /* DMA stream 1 FIFO control register */
#define STM32_DMA_S2FCR_OFFSET    0x0054 /* DMA stream 2 FIFO control register */
#define STM32_DMA_S3FCR_OFFSET    0x006c /* DMA stream 3 FIFO control register */
#define STM32_DMA_S4FCR_OFFSET    0x0084 /* DMA stream 4 FIFO control register */
#define STM32_DMA_S5FCR_OFFSET    0x009c /* DMA stream 5 FIFO control register */
#define STM32_DMA_S6FCR_OFFSET    0x00b4 /* DMA stream 6 FIFO control register */
#define STM32_DMA_S7FCR_OFFSET    0x00cc /* DMA stream 7 FIFO control register */

/* Register Addresses ***************************************************************/

#define STM32_DMA1_LISRC          (STM32_DMA1_BASE+STM32_DMA_LISR_OFFSET)
#define STM32_DMA1_HISRC          (STM32_DMA1_BASE+STM32_DMA_HISR_OFFSET)
#define STM32_DMA1_LIFCR          (STM32_DMA1_BASE+STM32_DMA_LIFCR_OFFSET)
#define STM32_DMA1_HIFCR          (STM32_DMA1_BASE+STM32_DMA_HIFCR_OFFSET)

#define STM32_DMA1_SCR(n)         (STM32_DMA1_BASE+STM32_DMA_SCR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA1_S0CR           (STM32_DMA1_BASE+STM32_DMA_S0CR_OFFSET)
#define STM32_DMA1_S1CR           (STM32_DMA1_BASE+STM32_DMA_S1CR_OFFSET)
#define STM32_DMA1_S2CR           (STM32_DMA1_BASE+STM32_DMA_S2CR_OFFSET)
#define STM32_DMA1_S3CR           (STM32_DMA1_BASE+STM32_DMA_S3CR_OFFSET)
#define STM32_DMA1_S4CR           (STM32_DMA1_BASE+STM32_DMA_S4CR_OFFSET)
#define STM32_DMA1_S5CR           (STM32_DMA1_BASE+STM32_DMA_S5CR_OFFSET)
#define STM32_DMA1_S6CR           (STM32_DMA1_BASE+STM32_DMA_S6CR_OFFSET)
#define STM32_DMA1_S7CR           (STM32_DMA1_BASE+STM32_DMA_S7CR_OFFSET)

#define STM32_DMA1_SNDTR(n)       (STM32_DMA1_BASE+STM32_DMA_SNDTR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA1_S0NDTR         (STM32_DMA1_BASE+STM32_DMA_S0NDTR_OFFSET)
#define STM32_DMA1_S1NDTR         (STM32_DMA1_BASE+STM32_DMA_S1NDTR_OFFSET)
#define STM32_DMA1_S2NDTR         (STM32_DMA1_BASE+STM32_DMA_S2NDTR_OFFSET)
#define STM32_DMA1_S3NDTR         (STM32_DMA1_BASE+STM32_DMA_S3NDTR_OFFSET)
#define STM32_DMA1_S4NDTR         (STM32_DMA1_BASE+STM32_DMA_S4NDTR_OFFSET)
#define STM32_DMA1_S5NDTR         (STM32_DMA1_BASE+STM32_DMA_S5NDTR_OFFSET)
#define STM32_DMA1_S6NDTR         (STM32_DMA1_BASE+STM32_DMA_S6NDTR_OFFSET)
#define STM32_DMA1_S7NDTR         (STM32_DMA1_BASE+STM32_DMA_S7NDTR_OFFSET)

#define STM32_DMA1_SPAR(n)        (STM32_DMA1_BASE+STM32_DMA_SPAR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA1_S0PAR          (STM32_DMA1_BASE+STM32_DMA_S0PAR_OFFSET)
#define STM32_DMA1_S1PAR          (STM32_DMA1_BASE+STM32_DMA_S1PAR_OFFSET)
#define STM32_DMA1_S2PAR          (STM32_DMA1_BASE+STM32_DMA_S2PAR_OFFSET)
#define STM32_DMA1_S3PAR          (STM32_DMA1_BASE+STM32_DMA_S3PAR_OFFSET)
#define STM32_DMA1_S4PAR          (STM32_DMA1_BASE+STM32_DMA_S4PAR_OFFSET)
#define STM32_DMA1_S5PAR          (STM32_DMA1_BASE+STM32_DMA_S5PAR_OFFSET)
#define STM32_DMA1_S6PAR          (STM32_DMA1_BASE+STM32_DMA_S6PAR_OFFSET)
#define STM32_DMA1_S7PAR          (STM32_DMA1_BASE+STM32_DMA_S7PAR_OFFSET)

#define STM32_DMA1_SM0AR(n)       (STM32_DMA1_BASE+STM32_DMA_SM0AR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA1_S0M0AR         (STM32_DMA1_BASE+STM32_DMA_S0M0AR_OFFSET)
#define STM32_DMA1_S1M0AR         (STM32_DMA1_BASE+STM32_DMA_S1M0AR_OFFSET)
#define STM32_DMA1_S2M0AR         (STM32_DMA1_BASE+STM32_DMA_S2M0AR_OFFSET)
#define STM32_DMA1_S3M0AR         (STM32_DMA1_BASE+STM32_DMA_S3M0AR_OFFSET)
#define STM32_DMA1_S4M0AR         (STM32_DMA1_BASE+STM32_DMA_S4M0AR_OFFSET)
#define STM32_DMA1_S5M0AR         (STM32_DMA1_BASE+STM32_DMA_S5M0AR_OFFSET)
#define STM32_DMA1_S6M0AR         (STM32_DMA1_BASE+STM32_DMA_S6M0AR_OFFSET)
#define STM32_DMA1_S7M0AR         (STM32_DMA1_BASE+STM32_DMA_S7M0AR_OFFSET)

#define STM32_DMA1_SM1AR(n)       (STM32_DMA1_BASE+STM32_DMA_SM1AR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA1_S0M1AR         (STM32_DMA1_BASE+STM32_DMA_S0M1AR_OFFSET)
#define STM32_DMA1_S1M1AR         (STM32_DMA1_BASE+STM32_DMA_S1M1AR_OFFSET)
#define STM32_DMA1_S2M1AR         (STM32_DMA1_BASE+STM32_DMA_S2M1AR_OFFSET)
#define STM32_DMA1_S3M1AR         (STM32_DMA1_BASE+STM32_DMA_S3M1AR_OFFSET)
#define STM32_DMA1_S4M1AR         (STM32_DMA1_BASE+STM32_DMA_S4M1AR_OFFSET)
#define STM32_DMA1_S5M1AR         (STM32_DMA1_BASE+STM32_DMA_S5M1AR_OFFSET)
#define STM32_DMA1_S6M1AR         (STM32_DMA1_BASE+STM32_DMA_S6M1AR_OFFSET)
#define STM32_DMA1_S7M1AR         (STM32_DMA1_BASE+STM32_DMA_S7M1AR_OFFSET)

#define STM32_DMA1_SFCR(n)        (STM32_DMA1_BASE+STM32_DMA_SFCR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA1_S0FCR          (STM32_DMA1_BASE+STM32_DMA_S0FCR_OFFSET)
#define STM32_DMA1_S1FCR          (STM32_DMA1_BASE+STM32_DMA_S1FCR_OFFSET)
#define STM32_DMA1_S2FCR          (STM32_DMA1_BASE+STM32_DMA_S2FCR_OFFSET)
#define STM32_DMA1_S3FCR          (STM32_DMA1_BASE+STM32_DMA_S3FCR_OFFSET)
#define STM32_DMA1_S4FCR          (STM32_DMA1_BASE+STM32_DMA_S4FCR_OFFSET)
#define STM32_DMA1_S5FCR          (STM32_DMA1_BASE+STM32_DMA_S5FCR_OFFSET)
#define STM32_DMA1_S6FCR          (STM32_DMA1_BASE+STM32_DMA_S6FCR_OFFSET)
#define STM32_DMA1_S7FCR          (STM32_DMA1_BASE+STM32_DMA_S7FCR_OFFSET)

#define STM32_DMA2_LISRC          (STM32_DMA2_BASE+STM32_DMA_LISR_OFFSET)
#define STM32_DMA2_HISRC          (STM32_DMA2_BASE+STM32_DMA_HISR_OFFSET)
#define STM32_DMA2_LIFCR          (STM32_DMA2_BASE+STM32_DMA_LIFCR_OFFSET)
#define STM32_DMA2_HIFCR          (STM32_DMA2_BASE+STM32_DMA_HIFCR_OFFSET)

#define STM32_DMA2_SCR(n)         (STM32_DMA2_BASE+STM32_DMA_SCR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA2_S0CR           (STM32_DMA2_BASE+STM32_DMA_S0CR_OFFSET)
#define STM32_DMA2_S1CR           (STM32_DMA2_BASE+STM32_DMA_S1CR_OFFSET)
#define STM32_DMA2_S2CR           (STM32_DMA2_BASE+STM32_DMA_S2CR_OFFSET)
#define STM32_DMA2_S3CR           (STM32_DMA2_BASE+STM32_DMA_S3CR_OFFSET)
#define STM32_DMA2_S4CR           (STM32_DMA2_BASE+STM32_DMA_S4CR_OFFSET)
#define STM32_DMA2_S5CR           (STM32_DMA2_BASE+STM32_DMA_S5CR_OFFSET)
#define STM32_DMA2_S6CR           (STM32_DMA2_BASE+STM32_DMA_S6CR_OFFSET)
#define STM32_DMA2_S7CR           (STM32_DMA2_BASE+STM32_DMA_S7CR_OFFSET)

#define STM32_DMA2_SNDTR(n)       (STM32_DMA2_BASE+STM32_DMA_SNDTR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA2_S0NDTR         (STM32_DMA2_BASE+STM32_DMA_S0NDTR_OFFSET)
#define STM32_DMA2_S1NDTR         (STM32_DMA2_BASE+STM32_DMA_S1NDTR_OFFSET)
#define STM32_DMA2_S2NDTR         (STM32_DMA2_BASE+STM32_DMA_S2NDTR_OFFSET)
#define STM32_DMA2_S3NDTR         (STM32_DMA2_BASE+STM32_DMA_S3NDTR_OFFSET)
#define STM32_DMA2_S4NDTR         (STM32_DMA2_BASE+STM32_DMA_S4NDTR_OFFSET)
#define STM32_DMA2_S5NDTR         (STM32_DMA2_BASE+STM32_DMA_S5NDTR_OFFSET)
#define STM32_DMA2_S6NDTR         (STM32_DMA2_BASE+STM32_DMA_S6NDTR_OFFSET)
#define STM32_DMA2_S7NDTR         (STM32_DMA2_BASE+STM32_DMA_S7NDTR_OFFSET)

#define STM32_DMA2_SPAR(n)        (STM32_DMA2_BASE+STM32_DMA_SPAR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA2_S0PAR          (STM32_DMA2_BASE+STM32_DMA_S0PAR_OFFSET)
#define STM32_DMA2_S1PAR          (STM32_DMA2_BASE+STM32_DMA_S1PAR_OFFSET)
#define STM32_DMA2_S2PAR          (STM32_DMA2_BASE+STM32_DMA_S2PAR_OFFSET)
#define STM32_DMA2_S3PAR          (STM32_DMA2_BASE+STM32_DMA_S3PAR_OFFSET)
#define STM32_DMA2_S4PAR          (STM32_DMA2_BASE+STM32_DMA_S4PAR_OFFSET)
#define STM32_DMA2_S5PAR          (STM32_DMA2_BASE+STM32_DMA_S5PAR_OFFSET)
#define STM32_DMA2_S6PAR          (STM32_DMA2_BASE+STM32_DMA_S6PAR_OFFSET)
#define STM32_DMA2_S7PAR          (STM32_DMA2_BASE+STM32_DMA_S7PAR_OFFSET)

#define STM32_DMA2_SM0AR(n)       (STM32_DMA2_BASE+STM32_DMA_SM0AR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA2_S0M0AR         (STM32_DMA2_BASE+STM32_DMA_S0M0AR_OFFSET)
#define STM32_DMA2_S1M0AR         (STM32_DMA2_BASE+STM32_DMA_S1M0AR_OFFSET)
#define STM32_DMA2_S2M0AR         (STM32_DMA2_BASE+STM32_DMA_S2M0AR_OFFSET)
#define STM32_DMA2_S3M0AR         (STM32_DMA2_BASE+STM32_DMA_S3M0AR_OFFSET)
#define STM32_DMA2_S4M0AR         (STM32_DMA2_BASE+STM32_DMA_S4M0AR_OFFSET)
#define STM32_DMA2_S5M0AR         (STM32_DMA2_BASE+STM32_DMA_S5M0AR_OFFSET)
#define STM32_DMA2_S6M0AR         (STM32_DMA2_BASE+STM32_DMA_S6M0AR_OFFSET)
#define STM32_DMA2_S7M0AR         (STM32_DMA2_BASE+STM32_DMA_S7M0AR_OFFSET)

#define STM32_DMA2_SM1AR(n)       (STM32_DMA2_BASE+STM32_DMA_SM1AR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA2_S0M1AR         (STM32_DMA2_BASE+STM32_DMA_S0M1AR_OFFSET)
#define STM32_DMA2_S1M1AR         (STM32_DMA2_BASE+STM32_DMA_S1M1AR_OFFSET)
#define STM32_DMA2_S2M1AR         (STM32_DMA2_BASE+STM32_DMA_S2M1AR_OFFSET)
#define STM32_DMA2_S3M1AR         (STM32_DMA2_BASE+STM32_DMA_S3M1AR_OFFSET)
#define STM32_DMA2_S4M1AR         (STM32_DMA2_BASE+STM32_DMA_S4M1AR_OFFSET)
#define STM32_DMA2_S5M1AR         (STM32_DMA2_BASE+STM32_DMA_S5M1AR_OFFSET)
#define STM32_DMA2_S6M1AR         (STM32_DMA2_BASE+STM32_DMA_S6M1AR_OFFSET)
#define STM32_DMA2_S7M1AR         (STM32_DMA2_BASE+STM32_DMA_S7M1AR_OFFSET)

#define STM32_DMA2_SFCR(n)        (STM32_DMA2_BASE+STM32_DMA_SFCR_OFFSET+STM32_DMA_OFFSET(n))
#define STM32_DMA2_S0FCR          (STM32_DMA2_BASE+STM32_DMA_S0FCR_OFFSET)
#define STM32_DMA2_S1FCR          (STM32_DMA2_BASE+STM32_DMA_S1FCR_OFFSET)
#define STM32_DMA2_S2FCR          (STM32_DMA2_BASE+STM32_DMA_S2FCR_OFFSET)
#define STM32_DMA2_S3FCR          (STM32_DMA2_BASE+STM32_DMA_S3FCR_OFFSET)
#define STM32_DMA2_S4FCR          (STM32_DMA2_BASE+STM32_DMA_S4FCR_OFFSET)
#define STM32_DMA2_S5FCR          (STM32_DMA2_BASE+STM32_DMA_S5FCR_OFFSET)
#define STM32_DMA2_S6FCR          (STM32_DMA2_BASE+STM32_DMA_S6FCR_OFFSET)
#define STM32_DMA2_S7FCR          (STM32_DMA2_BASE+STM32_DMA_S7FCR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

#define DMA_STREAM_MASK           0x3f
#define DMA_STREAM_FEIF_BIT       (1 << 0)  /* Bit 0: Stream FIFO error interrupt flag */
#define DMA_STREAM_DMEIF_BIT      (1 << 2)  /* Bit 2: Stream direct mode error interrupt flag */
#define DMA_STREAM_TEIF_BIT       (1 << 3)  /* Bit 3: Stream Transfer Error flag */
#define DMA_STREAM_HTIF_BIT       (1 << 4)  /* Bit 4: Stream Half Transfer flag */
#define DMA_STREAM_TCIF_BIT       (1 << 5)  /* Bit 5: Stream Transfer Complete flag */

/* DMA interrupt status register and interrupt flag clear register field defintions */

#define DMA_INT_STREAM0_SHIFT     (0)       /* Bits 0-5:   DMA Stream 0 interrupt */
#define DMA_INT_STREAM0_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM0_SHIFT)
#define DMA_INT_STREAM1_SHIFT     (6)       /* Bits 6-11:  DMA Stream 1 interrupt */
#define DMA_INT_STREAM1_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM1_SHIFT)
#define DMA_INT_STREAM2_SHIFT     (16)      /* Bits 16-21: DMA Stream 2 interrupt */
#define DMA_INT_STREAM2_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM2_SHIFT)
#define DMA_INT_STREAM3_SHIFT     (22)      /* Bits 22-27: DMA Stream 3 interrupt */
#define DMA_INT_STREAM3_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM3_SHIFT)

#define DMA_INT_STREAM4_SHIFT     (0)       /* Bits 0-5:   DMA Stream 4 interrupt */
#define DMA_INT_STREAM4_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM4_SHIFT)
#define DMA_INT_STREAM5_SHIFT     (6)       /* Bits 6-11:  DMA Stream 5 interrupt */
#define DMA_INT_STREAM5_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM5_SHIFT)
#define DMA_INT_STREAM6_SHIFT     (16)      /* Bits 16-21: DMA Stream 6 interrupt */
#define DMA_INT_STREAM6_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM6_SHIFT)
#define DMA_INT_STREAM7_SHIFT     (22)      /* Bits 22-27: DMA Stream 7 interrupt */
#define DMA_INT_STREAM7_MASK      (DMA_STREAM_MASK <<  DMA_INT_STREAM7_SHIFT)

/* DMA stream configuration register */

#define DMA_SCR_EN                (1 << 0)  /* Bit 0:  Stream enable */
#define DMA_SCR_DMEIE             (1 << 1)  /* Bit 1:  Direct mode error interrupt enable */
#define DMA_SCR_TEIE              (1 << 2)  /* Bit 2:  Transfer error interrupt enable */
#define DMA_SCR_HTIE              (1 << 3)  /* Bit 3:  Half Transfer interrupt enable */
#define DMA_SCR_TCIE              (1 << 4)  /* Bit 4:  Transfer complete interrupt enable */
#define DMA_SCR_PFCTRL            (1 << 5)  /* Bit 5:  Peripheral flow controller */
#define DMA_SCR_DIR_SHIFT         (6)       /* Bits 6-7: Data transfer direction */
#define DMA_SCR_DIR_MASK          (3 << DMA_SCR_DIR_SHIFT)
#  define DMA_SCR_DIR_P2M         (0 << DMA_SCR_DIR_SHIFT) /* 00: Peripheral-to-memory */
#  define DMA_SCR_DIR_M2P         (1 << DMA_SCR_DIR_SHIFT) /* 01: Memory-to-peripheral */
#  define DMA_SCR_DIR_M2M         (2 << DMA_SCR_DIR_SHIFT) /* 10: Memory-to-memory */
#define DMA_SCR_CIRC              (1 << 8)  /* Bit 8:  Circular mode */
#define DMA_SCR_PINC              (1 << 9)  /* Bit 9:  Peripheral increment mode */
#define DMA_SCR_MINC              (1 << 10) /* Bit 10: Memory increment mode */
#define DMA_SCR_PSIZE_SHIFT       (11)      /* Bits 11-12: Peripheral size */
#define DMA_SCR_PSIZE_MASK        (3 << DMA_SCR_PSIZE_SHIFT)
#  define DMA_SCR_PSIZE_8BITS     (0 << DMA_SCR_PSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_SCR_PSIZE_16BITS    (1 << DMA_SCR_PSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_SCR_PSIZE_32BITS    (2 << DMA_SCR_PSIZE_SHIFT) /* 10: 32-bits */
#define DMA_SCR_MSIZE_SHIFT       (13)      /* Bits 13-14: Memory size */
#define DMA_SCR_MSIZE_MASK        (3 << DMA_SCR_MSIZE_SHIFT)
#  define DMA_SCR_MSIZE_8BITS     (0 << DMA_SCR_MSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_SCR_MSIZE_16BITS    (1 << DMA_SCR_MSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_SCR_MSIZE_32BITS    (2 << DMA_SCR_MSIZE_SHIFT) /* 10: 32-bits */
#define DMA_SCR_PINCOS            (1 << 15) /* Bit 15: Peripheral increment offset size */
#define DMA_SCR_PL_SHIFT          (16)      /* Bits 16-17: Stream Priority level */
#define DMA_SCR_PL_MASK           (3 << DMA_SCR_PL_SHIFT)
#  define DMA_SCR_PRILO           (0 << DMA_SCR_PL_SHIFT) /* 00: Low */
#  define DMA_SCR_PRIMED          (1 << DMA_SCR_PL_SHIFT) /* 01: Medium */
#  define DMA_SCR_PRIHI           (2 << DMA_SCR_PL_SHIFT) /* 10: High */
#  define DMA_SCR_PRIVERYHI       (3 << DMA_SCR_PL_SHIFT) /* 11: Very high */
#define DMA_SCR_DBM               (1 << 18) /* Bit 15: Double buffer mode */
#define DMA_SCR_CT                (1 << 19) /* Bit 19: Current target */
#define DMA_SCR_PBURST_SHIFT      (21)      /* Bits 21-22: Peripheral burst transfer configuration */
#define DMA_SCR_PBURST_MASK       (3 << DMA_SCR_PBURST_SHIFT)
#  define DMA_SCR_PBURST_SINGLE   (0 << DMA_SCR_PBURST_SHIFT) /* 00: Single transfer */
#  define DMA_SCR_PBURST_INCR4    (1 << DMA_SCR_PBURST_SHIFT) /* 01: Incremental burst of 4 beats */
#  define DMA_SCR_PBURST_INCR8    (2 << DMA_SCR_PBURST_SHIFT) /* 10: Incremental burst of 8 beats */
#  define DMA_SCR_PBURST_INCR16   (3 << DMA_SCR_PBURST_SHIFT) /* 11: Incremental burst of 16 beats */
#define DMA_SCR_MBURST_SHIFT      (23)      /* Bits 23-24: Memory burst transfer configuration */
#define DMA_SCR_MBURST_MASK       (3 << DMA_SCR_MBURST_SHIFT)
#  define DMA_SCR_MBURST_SINGLE   (0 << DMA_SCR_MBURST_SHIFT) /* 00: Single transfer */
#  define DMA_SCR_MBURST_INCR4    (1 << DMA_SCR_MBURST_SHIFT) /* 01: Incremental burst of 4 beats */
#  define DMA_SCR_MBURST_INCR8    (2 << DMA_SCR_MBURST_SHIFT) /* 10: Incremental burst of 8 beats */
#  define DMA_SCR_MBURST_INCR16   (3 << DMA_SCR_MBURST_SHIFT) /* 11: Incremental burst of 16 beats */
#define DMA_SCR_CHSEL_SHIFT       (25)      /* Bits 25-27:  Channel selection */
#define DMA_SCR_CHSEL_MASK        (7 << DMA_SCR_CHSEL_SHIFT)
#  define DMA_SCR_CHSEL(n)        ((n) << DMA_SCR_CHSEL_SHIFT)

#define DMA_SCR_ALLINTS           (DMA_SCR_DMEIE|DMA_SCR_TEIE|DMA_SCR_HTIE|DMA_SCR_TCIE)

/* DMA stream number of data register */

#define DMA_SNDTR_NDT_SHIFT       (0)       /* Bits 15-0: Number of data to Transfer */
#define DMA_SNDTR_NDT_MASK        (0xffff << DMA_SNDTR_NDT_SHIFT)

/* DMA stream n FIFO control register */

#define DMA_SFCR_FTH_SHIFT        (0)       /* Bits 0-1: FIFO threshold selection */
#define DMA_SFCR_FTH_MASK         (3 << DMA_SFCR_FTH_SHIFT)
#  define DMA_SFCR_FTH_QUARTER    (0 << DMA_SFCR_FTH_SHIFT) /* 1/4 full FIFO */
#  define DMA_SFCR_FTH_HALF       (1 << DMA_SFCR_FTH_SHIFT) /* 1/2 full FIFO */
#  define DMA_SFCR_FTH_3QUARTER   (2 << DMA_SFCR_FTH_SHIFT) /* 3/4 full FIFO */
#  define DMA_SFCR_FTH_FULL       (3 << DMA_SFCR_FTH_SHIFT) /* full FIFO */
#define DMA_SFCR_DMDIS            (1 << 2)  /* Bit 2:  Direct mode disable */
#define DMA_SFCR_FS_SHIFT         (3)       /* Bits 3-5: FIFO status */
#define DMA_SFCR_FS_MASK          (7 << DMA_SFCR_FS_SHIFT)
#  define DMA_SFCR_FS_QUARTER     (0 << DMA_SFCR_FS_SHIFT) /* 0 < fifo_level < 1/4 */
#  define DMA_SFCR_FS_HALF        (1 << DMA_SFCR_FS_SHIFT) /* 1/4 = fifo_level < 1/2 */
#  define DMA_SFCR_FS_3QUARTER    (2 << DMA_SFCR_FS_SHIFT) /* 1/2 = fifo_level < 3/4 */
#  define DMA_SFCR_FS_ALMOSTFULL  (3 << DMA_SFCR_FS_SHIFT) /* 3/4 = fifo_level < full */
#  define DMA_SFCR_FS_EMPTY       (4 << DMA_SFCR_FS_SHIFT) /* FIFO is empty */
#  define DMA_SFCR_FS_FULL        (5 << DMA_SFCR_FS_SHIFT) /* FIFO is full */
                                            /* Bit 6:  Reserved */
#define DMA_SFCR_FEIE             (1 << 7)  /* Bit 7:  FIFO error interrupt enable */
                                            /* Bits 8-31: Reserved */

/* DMA Stream mapping.  Each DMA stream has a mapping to several possible
 * sources/sinks of data.  The requests from peripherals assigned to a stream
 * are simply OR'ed together before entering the DMA block.  This means that only
 * one request on a given stream can be enabled at once.
 *
 * Alternative stream selections are provided with a numeric suffix like _1, _2, etc.
 * The DMA driver, however, will use the pin selection without the numeric suffix.
 * Additional definitions are required in the board.h file.  For example, if
 * SPI3_RX connects via DMA STREAM0, then following should be application-specific
 * mapping should be used:
 *
 * #define DMAMAP_SPI3_RX DMAMAP_SPI3_RX_1
 */

#define STM32_DMA_MAP(d,s,c)       ((d) << 6 | (s) << 3 | (c))
#define STM32_DMA_CONTROLLER(m)    (((m) >> 6) & 1)
#define STM32_DMA_STREAM(m)        (((m) >> 3) & 7)
#define STM32_DMA_CHANNEL(m)       ((m) & 7)

#define DMAMAP_SPI3_RX_1           STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN0)
#define DMAMAP_SPI3_RX_2           STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN0)
#define DMAMAP_SPI2_RX             STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN0)
#define DMAMAP_SPI2_TX             STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN0)
#define DMAMAP_SPI3_TX_1           STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN0)
#define DMAMAP_SPI3_TX_2           STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN0)

#define DMAMAP_I2C1_RX_1           STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN1)
#define DMAMAP_TIM7_UP_1           STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN1)
#define DMAMAP_TIM7_UP_2           STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN1)
#define DMAMAP_I2C1_RX_2           STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN1)
#define DMAMAP_I2C1_TX_1           STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN1)
#define DMAMAP_I2C1_TX_2           STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN1)

#define DMAMAP_TIM4_CH1            STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN2)
#define DMAMAP_I2S2_EXT_RX_1       STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN2)
#define DMAMAP_TIM4_CH2            STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN2)
#define DMAMAP_I2S2_EXT_TX         STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN2)
#define DMAMAP_I2S3_EXT_TX         STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN2)
#define DMAMAP_TIM4_UP             STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN2)
#define DMAMAP_TIM4_CH3            STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN2)

#define DMAMAP_I2S3_EXT_RX         STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN3)
#define DMAMAP_TIM2_UP_1           STM32_DMA_MAP(DMA1,DMA_STREAM1,DMA_CHAN3)
#define DMAMAP_TIM2_CH3            STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN3)
#define DMAMAP_I2C3_RX             STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN3)
#define DMAMAP_I2S2_EXT_RX_2       STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN3)
#define DMAMAP_I2C3_TX             STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN3)
#define DMAMAP_TIM2_CH1            STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN3)
#define DMAMAP_TIM2_CH2            STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN3)
#define DMAMAP_TIM2_CH4_1          STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN3)
#define DMAMAP_TIM2_UP_2           STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN3)
#define DMAMAP_TIM2_CH4_2          STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN3)

#define DMAMAP_UART5_RX            STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN4)
#define DMAMAP_USART3_RX           STM32_DMA_MAP(DMA1,DMA_STREAM1,DMA_CHAN4)
#define DMAMAP_UART4_RX            STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN4)
#define DMAMAP_USART3_TX_1         STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN4)
#define DMAMAP_UART4_TX            STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN4)
#define DMAMAP_USART2_RX           STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN4)
#define DMAMAP_USART2_TX           STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN4)
#define DMAMAP_UART5_TX            STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN4)

#define DMAMAP_TIM3_CH4            STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN5)
#define DMAMAP_TIM3_UP             STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN5)
#define DMAMAP_TIM3_CH1            STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN5)
#define DMAMAP_TIM3_TRIG           STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN5)
#define DMAMAP_TIM3_CH2            STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN5)
#define DMAMAP_TIM3_CH3            STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN5)

#define DMAMAP_TIM5_CH3            STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN6)
#define DMAMAP_TIM5_UP_1           STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN6)
#define DMAMAP_TIM5_CH4_1          STM32_DMA_MAP(DMA1,DMA_STREAM1,DMA_CHAN6)
#define DMAMAP_TIM5_TRIG_1         STM32_DMA_MAP(DMA1,DMA_STREAM1,DMA_CHAN6)
#define DMAMAP_TIM5_CH1            STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN6)
#define DMAMAP_TIM5_CH4_2          STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN6)
#define DMAMAP_TIM5_TRIG_2         STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN6)
#define DMAMAP_TIM5_CH2            STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN6)
#define DMAMAP_TIM5_UP_2           STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN6)

#define DMAMAP_TIM6_UP             STM32_DMA_MAP(DMA1,DMA_STREAM1,DMA_CHAN7)
#define DMAMAP_I2C2_RX_1           STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN7)
#define DMAMAP_I2C2_RX_2           STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN7)
#define DMAMAP_USART3_TX_2         STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN7)
#define DMAMAP_DAC1                STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN7)
#define DMAMAP_DAC2                STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN7)
#define DMAMAP_I2C2_TX             STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN7)

#define DMAMAP_ADC1_1              STM32_DMA_MAP(DMA2,DMA_STREAM0,DMA_CHAN0)
#define DMAMAP_TIM8_CH1_1          STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN0)
#define DMAMAP_TIM8_CH2_1          STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN0)
#define DMAMAP_TIM8_CH3_1          STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN0)
#define DMAMAP_ADC1_2              STM32_DMA_MAP(DMA2,DMA_STREAM4,DMA_CHAN0)
#define DMAMAP_TIM1_CH1_1          STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN0)
#define DMAMAP_TIM1_CH2_1          STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN0)
#define DMAMAP_TIM1_CH3_1          STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN0)

#define DMAMAP_DCMI_1              STM32_DMA_MAP(DMA2,DMA_STREAM1,DMA_CHAN1)
#define DMAMAP_ADC2_1              STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN1)
#define DMAMAP_ADC2_2              STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN1)
#define DMAMAP_DCMI_2              STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN1)

#define DMAMAP_ADC3_1              STM32_DMA_MAP(DMA2,DMA_STREAM0,DMA_CHAN2)
#define DMAMAP_ADC3_2              STM32_DMA_MAP(DMA2,DMA_STREAM1,DMA_CHAN2)
#define DMAMAP_CRYP_OUT            STM32_DMA_MAP(DMA2,DMA_STREAM5,DMA_CHAN2)
#define DMAMAP_CRYP_IN             STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN2)
#define DMAMAP_HASH_IN             STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN2)

#define DMAMAP_SPI1_RX_1           STM32_DMA_MAP(DMA2,DMA_STREAM0,DMA_CHAN3)
#define DMAMAP_SPI1_RX_2           STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN3)
#define DMAMAP_SPI1_TX_1           STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN3)
#define DMAMAP_SPI1_TX_2           STM32_DMA_MAP(DMA2,DMA_STREAM5,DMA_CHAN3)

#define DMAMAP_USART1_RX_1         STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN4)
#define DMAMAP_SDIO_1              STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN4)
#define DMAMAP_USART1_RX_2         STM32_DMA_MAP(DMA2,DMA_STREAM5,DMA_CHAN4)
#define DMAMAP_SDIO_2              STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN4)
#define DMAMAP_USART1_TX           STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN4)

#define DMAMAP_USART6_RX_1         STM32_DMA_MAP(DMA2,DMA_STREAM1,DMA_CHAN5)
#define DMAMAP_USART6_RX_2         STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN5)
#define DMAMAP_USART6_TX_1         STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN5)
#define DMAMAP_USART6_TX_2         STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN5)

#define DMAMAP_TIM1_TRIG_1         STM32_DMA_MAP(DMA2,DMA_STREAM0,DMA_CHAN6)
#define DMAMAP_TIM1_CH1_2          STM32_DMA_MAP(DMA2,DMA_STREAM1,DMA_CHAN6)
#define DMAMAP_TIM1_CH2_2          STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN6)
#define DMAMAP_TIM1_CH1            STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN6)
#define DMAMAP_TIM1_CH4            STM32_DMA_MAP(DMA2,DMA_STREAM4,DMA_CHAN6)
#define DMAMAP_TIM1_TRIG_2         STM32_DMA_MAP(DMA2,DMA_STREAM4,DMA_CHAN6)
#define DMAMAP_TIM1_COM            STM32_DMA_MAP(DMA2,DMA_STREAM4,DMA_CHAN6)
#define DMAMAP_TIM1_UP             STM32_DMA_MAP(DMA2,DMA_STREAM5,DMA_CHAN6)
#define DMAMAP_TIM1_CH3_2          STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN6)

#define DMAMAP_TIM8_UP             STM32_DMA_MAP(DMA2,DMA_STREAM1,DMA_CHAN7)
#define DMAMAP_TIM8_CH1_2          STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN7)
#define DMAMAP_TIM8_CH2_2          STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN7)
#define DMAMAP_TIM8_CH3_2          STM32_DMA_MAP(DMA2,DMA_STREAM5,DMA_CHAN7)
#define DMAMAP_TIM8_CH4            STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN7)
#define DMAMAP_TIM8_TRIG           STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN7)
#define DMAMAP_TIM8_COM            STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN7)

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32F20XXX_DMA_H */
