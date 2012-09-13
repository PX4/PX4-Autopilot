/************************************************************************************
 * arch/arm/src/imx/imx_dma.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_IMX_DMA_H
#define __ARCH_ARM_IMX_DMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* DMA Register Offsets *************************************************************/

#define DMA_SYS_OFFSET            0x0000
#define DMA_M2D_OFFSET            0x0040
#define DMA_CH0_OFFSET            0x0080
#define DMA_CH1_OFFSET            0x00c0
#define DMA_CH2_OFFSET            0x0100
#define DMA_CH3_OFFSET            0x0140
#define DMA_CH4_OFFSET            0x0180
#define DMA_CH5_OFFSET            0x01C0
#define DMA_CH6_OFFSET            0x0200
#define DMA_CH7_OFFSET            0x0240
#define DMA_CH8_OFFSET            0x0280
#define DMA_CH9_OFFSET            0x02c0
#define DMA_CH10_OFFSET           0x0300
#define DMA_CH_OFFSET(n)          (DMA_CH0_OFFSET + (n)*0x0040)
#define DMA_TST_OFFSET            0x0340

#define DMA_DCR_OFFSET            0x0000
#define DMA_ISR_OFFSET            0x0004
#define DMA_IMR_OFFSET            0x0008
#define DMA_BTOSR_OFFSET          0x000c
#define DMA_RTOSR_OFFSET          0x0010
#define DMA_TESR_OFFSET           0x0014
#define DMA_BOSR_OFFSET           0x0018
#define DMA_BTOCR_OFFSET          0x001c

#define DMA_WSRA_OFFSET           0x0000
#define DMA_XSRA_OFFSET           0x0004
#define DMA_YSRA_OFFSET           0x0008
#define DMA_WSRB_OFFSET           0x000c
#define DMA_XSRB_OFFSET           0x0010
#define DMA_YSRB_OFFSET           0x0014

#define DMA_SAR_OFFSET            0x0000
#define DMA_DAR_OFFSET            0x0004
#define DMA_CNTR_OFFSET           0x0008
#define DMA_CCR_OFFSET            0x000c
#define DMA_RSSR_OFFSET           0x0010
#define DMA_BLR_OFFSET            0x0014
#define DMA_RTOR_OFFSET           0x0018
#define DMA_BUCR_OFFSET           0x0018

#define DMA_TCR_OFFSET            0x0000
#define DMA_TFIFOA_OFFSET         0x0004
#define DMA_TDRR_OFFSET           0x0008
#define DMA_TDIPR_OFFSET          0x000c
#define DMA_TFIFOB_OFFSET         0x0010

/* DMA Register Addresses ***********************************************************/

#define IMX_DMA_SYS_BASE         (IMX_DMA_VBASE + DMA_SYS_OFFSET)
#define IMX_DMA_M2D_BASE         (IMX_DMA_VBASE + DMA_M2D_OFFSET)
#define IMX_DMA_CH0_BASE         (IMX_DMA_VBASE + DMA_CH0_OFFSET)
#define IMX_DMA_CH1_BASE         (IMX_DMA_VBASE + DMA_CH1_OFFSET)
#define IMX_DMA_CH2_BASE         (IMX_DMA_VBASE + DMA_CH2_OFFSET)
#define IMX_DMA_CH3_BASE         (IMX_DMA_VBASE + DMA_CH3_OFFSET)
#define IMX_DMA_CH4_BASE         (IMX_DMA_VBASE + DMA_CH4_OFFSET)
#define IMX_DMA_CH5_BASE         (IMX_DMA_VBASE + DMA_CH5_OFFSET)
#define IMX_DMA_CH6_BASE         (IMX_DMA_VBASE + DMA_CH6_OFFSET)
#define IMX_DMA_CH7_BASE         (IMX_DMA_VBASE + DMA_CH7_OFFSET)
#define IMX_DMA_CH8_BASE         (IMX_DMA_VBASE + DMA_CH8_OFFSET)
#define IMX_DMA_CH9_BASE         (IMX_DMA_VBASE + DMA_CH9_OFFSET)
#define IMX_DMA_CH10_BASE        (IMX_DMA_VBASE + DMA_CH10_OFFSET)
#define IMX_DMA_CH_BASE(n)       (IMX_DMA_VBASE + DMA_CH_OFFSET(n))
#define IMX_DMA_TST_BASE         (IMX_DMA_VBASE + DMA_TST_OFFSET)

#define IMX_DMA_DCR              (DMA_SYS_BASE + DMA_DCR_OFFSET)
#define IMX_DMA_ISR              (DMA_SYS_BASE + DMA_ISR_OFFSET)
#define IMX_DMA_IMR              (DMA_SYS_BASE + DMA_IMR_OFFSET)
#define IMX_DMA_BTOSR            (DMA_SYS_BASE + DMA_BTOSR_OFFSET)
#define IMX_DMA_RTOSR            (DMA_SYS_BASE + DMA_RTOSR_OFFSET)
#define IMX_DMA_TESR             (DMA_SYS_BASE + DMA_TESR_OFFSET)
#define IMX_DMA_BOSR             (DMA_SYS_BASE + DMA_BOSR_OFFSET)
#define IMX_DMA_BTOCR            (DMA_SYS_BASE + DMA_BTOCR_OFFSET)

#define IMX_DMA_WSRA             (DMA_M2D_BASE + DMA_WSRA_OFFSET)
#define IMX_DMA_XSRA             (DMA_M2D_BASE + DMA_XSRA_OFFSET)
#define IMX_DMA_YSRA             (DMA_M2D_BASE + DMA_YSRA_OFFSET)
#define IMX_DMA_WSRB             (DMA_M2D_BASE + DMA_WSRB_OFFSET)
#define IMX_DMA_XSRB             (DMA_M2D_BASE + DMA_XSRB_OFFSET)
#define IMX_DMA_YSRB             (DMA_M2D_BASE + DMA_YSRB_OFFSET)

#define IMX_DMA_SAR0             (DMA_CH0_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR0             (DMA_CH0_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR0            (DMA_CH0_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR0             (DMA_CH0_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR0            (DMA_CH0_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR0             (DMA_CH0_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR0            (DMA_CH0_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR0            (DMA_CH0_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR1             (DMA_CH1_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR1             (DMA_CH1_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR1            (DMA_CH1_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR1             (DMA_CH1_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR1            (DMA_CH1_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR1             (DMA_CH1_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR1            (DMA_CH1_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR1            (DMA_CH1_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR2             (DMA_CH2_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR2             (DMA_CH2_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR2            (DMA_CH2_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR2             (DMA_CH2_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR2            (DMA_CH2_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR2             (DMA_CH2_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR2            (DMA_CH2_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR2            (DMA_CH2_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR3             (DMA_CH3_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR3             (DMA_CH3_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR3            (DMA_CH3_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR3             (DMA_CH3_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR3            (DMA_CH3_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR3             (DMA_CH3_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR3            (DMA_CH3_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR3            (DMA_CH3_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR4             (DMA_CH4_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR4             (DMA_CH4_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR4            (DMA_CH4_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR4             (DMA_CH4_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR4            (DMA_CH4_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR4             (DMA_CH4_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR4            (DMA_CH4_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR4            (DMA_CH4_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR5             (DMA_CH5_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR5             (DMA_CH5_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR5            (DMA_CH5_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR5             (DMA_CH5_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR5            (DMA_CH5_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR5             (DMA_CH5_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR5            (DMA_CH5_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR5            (DMA_CH5_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR6             (DMA_CH6_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR6             (DMA_CH6_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR6            (DMA_CH6_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR6             (DMA_CH6_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR6            (DMA_CH6_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR6             (DMA_CH6_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR6            (DMA_CH6_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR6            (DMA_CH6_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR7             (DMA_CH7_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR7             (DMA_CH7_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR7            (DMA_CH7_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR7             (DMA_CH7_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR7            (DMA_CH7_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR7             (DMA_CH7_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR7            (DMA_CH7_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR7            (DMA_CH7_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR8             (DMA_CH8_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR8             (DMA_CH8_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR8            (DMA_CH8_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR8             (DMA_CH8_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR8            (DMA_CH8_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR8             (DMA_CH8_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR8            (DMA_CH8_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR8            (DMA_CH8_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR9             (DMA_CH9_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR9             (DMA_CH9_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR9            (DMA_CH9_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR9             (DMA_CH9_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR9            (DMA_CH9_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR9             (DMA_CH9_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR9            (DMA_CH9_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR9            (DMA_CH9_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR10            (DMA_CH10_BASE + DMA_SAR_OFFSET)
#define IMX_DMA_DAR10            (DMA_CH10_BASE + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR10           (DMA_CH10_BASE + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR10            (DMA_CH10_BASE + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR10           (DMA_CH10_BASE + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR10            (DMA_CH10_BASE + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR10           (DMA_CH10_BASE + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR10           (DMA_CH10_BASE + DMA_BUCR_OFFSET)

#define IMX_DMA_SAR(n)           (DMA_CH_BASE(n) + DMA_SAR_OFFSET)
#define IMX_DMA_DAR(n)           (DMA_CH_BASE(n) + DMA_DAR_OFFSET)
#define IMX_DMA_CNTR(n)          (DMA_CH_BASE(n) + DMA_CNTR_OFFSET)
#define IMX_DMA_CCR(n)           (DMA_CH_BASE(n) + DMA_CCR_OFFSET)
#define IMX_DMA_RSSR(n)          (DMA_CH_BASE(n) + DMA_RSSR_OFFSET)
#define IMX_DMA_BLR(n)           (DMA_CH_BASE(n) + DMA_BLR_OFFSET)
#define IMX_DMA_RTOR(n)          (DMA_CH_BASE(n) + DMA_RTOR_OFFSET)
#define IMX_DMA_BUCR(n)          (DMA_CH_BASE(n) + DMA_BUCR_OFFSET)

#define IMX_DMA_TCR              (DMA_TST_BASE + DMA_TCR_OFFSET)
#define IMX_DMA_TFIFOA           (DMA_TST_BASE + DMA_TFIFOA_OFFSET)
#define IMX_DMA_TDRR             (DMA_TST_BASE + DMA_TDRR_OFFSET)
#define IMX_DMA_TDIPR            (DMA_TST_BASE + DMA_TDIPR_OFFSET)
#define IMX_DMA_TFIFOB           (DMA_TST_BASE + DMA_TFIFOB_OFFSET)

/* DMA Register Bit Definitions *****************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_DMA_H */
