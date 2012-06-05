/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-adc.h
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
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_ADC_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_ADC_CON1_OFFSET    0x0000 /* ADC control register 1 */
#define PIC32MX_ADC_CON1CLR_OFFSET 0x0004 /* ADC control clear register 1 */
#define PIC32MX_ADC_CON1SET_OFFSET 0x0008 /* ADC control set register 1 */
#define PIC32MX_ADC_CON1INV_OFFSET 0x000c /* ADC control invert register 1 */
#define PIC32MX_ADC_CON2_OFFSET    0x0010 /* ADC control register 2 */
#define PIC32MX_ADC_CON2CLR_OFFSET 0x0014 /* ADC control clear register 2 */
#define PIC32MX_ADC_CON2SET_OFFSET 0x0018 /* ADC control set register 2 */
#define PIC32MX_ADC_CON2INV_OFFSET 0x001c /* ADC control invert register 2 */
#define PIC32MX_ADC_CON3_OFFSET    0x0020 /* ADC control register 3 */
#define PIC32MX_ADC_CON3CLR_OFFSET 0x0024 /* ADC control clear register 3 */
#define PIC32MX_ADC_CON3SET_OFFSET 0x0028 /* ADC control set register 3 */
#define PIC32MX_ADC_CON3INV_OFFSET 0x002c /* ADC control invert register 3 */
#define PIC32MX_ADC_CHS_OFFSET     0x0040 /* ADC input pin selection register */
#define PIC32MX_ADC_CHSCLR_OFFSET  0x0044 /* ADC input pin selection clear register */
#define PIC32MX_ADC_CHSSET_OFFSET  0x0048 /* ADC input pin selection set register */
#define PIC32MX_ADC_CHSINV_OFFSET  0x004c /* ADC input pin selection invert register */
#define PIC32MX_ADC_CSSL_OFFSET    0x0050 /* ADC sequentially scanned input register */
#define PIC32MX_ADC_CSSLCLR_OFFSET 0x0054 /* ADC sequentially scanned input clear register */
#define PIC32MX_ADC_CSSLSET_OFFSET 0x0058 /* ADC sequentially scanned input set register */
#define PIC32MX_ADC_CSSLINV_OFFSET 0x005c /* ADC sequentially scanned input invert register */
#define PIC32MX_ADC_CFG_OFFSET     0x0060 /* ADC input pin configuration register */
#define PIC32MX_ADC_CFGCLR_OFFSET  0x0064 /* ADC input pin configuration clear register */
#define PIC32MX_ADC_CFGSET_OFFSET  0x0068 /* ADC input pin configuration set register */
#define PIC32MX_ADC_CFGINV_OFFSET  0x006c /* ADC input pin configuration invert register */

#define PIC32MX_ADC_BUF_OFFSET(n)  (0x0070 + ((n) << 4))
#define PIC32MX_ADC_BUF0_OFFSET    0x0070 /* ADC result word 0 */
#define PIC32MX_ADC_BUF1_OFFSET    0x0080 /* ADC result word 1 */
#define PIC32MX_ADC_BUF2_OFFSET    0x0090 /* ADC result word 2 */
#define PIC32MX_ADC_BUF3_OFFSET    0x00a0 /* ADC result word 3 */
#define PIC32MX_ADC_BUF4_OFFSET    0x00b0 /* ADC result word 4 */
#define PIC32MX_ADC_BUF5_OFFSET    0x00c0 /* ADC result word 5 */
#define PIC32MX_ADC_BUF6_OFFSET    0x00d0 /* ADC result word 6 */
#define PIC32MX_ADC_BUF7_OFFSET    0x00e0 /* ADC result word 7 */
#define PIC32MX_ADC_BUF8_OFFSET    0x00f0 /* ADC result word 8 */
#define PIC32MX_ADC_BUF9_OFFSET    0x0100 /* ADC result word 9 */
#define PIC32MX_ADC_BUF10_OFFSET   0x0110 /* ADC result word 10 */
#define PIC32MX_ADC_BUF11_OFFSET   0x0120 /* ADC result word 11 */
#define PIC32MX_ADC_BUF12_OFFSET   0x0130 /* ADC result word 12 */
#define PIC32MX_ADC_BUF13_OFFSET   0x0140 /* ADC result word 13 */
#define PIC32MX_ADC_BUF14_OFFSET   0x0150 /* ADC result word 14 */
#define PIC32MX_ADC_BUF15_OFFSET   0x0160 /* ADC result word 15 */

/* Register Addresses ***************************************************************/

#define PIC32MX_ADC_CON1           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON1_OFFSET)
#define PIC32MX_ADC_CON1CLR        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON1CLR_OFFSET)
#define PIC32MX_ADC_CON1SET        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON1SET_OFFSET)
#define PIC32MX_ADC_CON1INV        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON1INV_OFFSET)
#define PIC32MX_ADC_CON2           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON2_OFFSET)
#define PIC32MX_ADC_CON2CLR        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON2CLR_OFFSET)
#define PIC32MX_ADC_CON2SET        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON2SET_OFFSET)
#define PIC32MX_ADC_CON2INV        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON2INV_OFFSET)
#define PIC32MX_ADC_CON3           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON3_OFFSET)
#define PIC32MX_ADC_CON3CLR        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON3CLR_OFFSET)
#define PIC32MX_ADC_CON3SET        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON3SET_OFFSET)
#define PIC32MX_ADC_CON3INV        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CON3INV_OFFSET)
#define PIC32MX_ADC_CHS            (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CHS_OFFSET)
#define PIC32MX_ADC_CHSCLR         (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CHSCLR_OFFSET)
#define PIC32MX_ADC_CHSSET         (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CHSSET_OFFSET)
#define PIC32MX_ADC_CHSINV         (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CHSINV_OFFSET)
#define PIC32MX_ADC_CSSL           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CSSL_OFFSET)
#define PIC32MX_ADC_CSSLCLR        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CSSLCLR_OFFSET)
#define PIC32MX_ADC_CSSLSET        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CSSLSET_OFFSET)
#define PIC32MX_ADC_CSSLINV        (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CSSLINV_OFFSET)
#define PIC32MX_ADC_CFG            (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CFG_OFFSET)
#define PIC32MX_ADC_CFGCLR         (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CFGCLR_OFFSET)
#define PIC32MX_ADC_CFGSET         (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CFGSET_OFFSET)
#define PIC32MX_ADC_CFGINV         (PIC32MX_ADC_K1BASE+PIC32MX_ADC_CFGINV_OFFSET)

#define PIC32MX_ADC_BUF(n)         (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF_OFFSET(n))
#define PIC32MX_ADC_BUF0           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF0_OFFSET)
#define PIC32MX_ADC_BUF1           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF1_OFFSET)
#define PIC32MX_ADC_BUF2           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF2_OFFSET)
#define PIC32MX_ADC_BUF3           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF3_OFFSET)
#define PIC32MX_ADC_BUF4           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF4_OFFSET)
#define PIC32MX_ADC_BUF5           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF5_OFFSET)
#define PIC32MX_ADC_BUF6           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF6_OFFSET)
#define PIC32MX_ADC_BUF7           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF7_OFFSET)
#define PIC32MX_ADC_BUF8           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF8_OFFSET)
#define PIC32MX_ADC_BUF9           (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF9_OFFSET)
#define PIC32MX_ADC_BUF10          (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF10_OFFSET)
#define PIC32MX_ADC_BUF11          (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF11_OFFSET)
#define PIC32MX_ADC_BUF12          (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF12_OFFSET)
#define PIC32MX_ADC_BUF13          (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF13_OFFSET)
#define PIC32MX_ADC_BUF14          (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF14_OFFSET)
#define PIC32MX_ADC_BUF15          (PIC32MX_ADC_K1BASE+PIC32MX_ADC_BUF15_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* ADC control register 1 */

#define ADC_CON1_DONE              (1 << 0)  /* Bit 0:  A/D conversion status */
#define ADC_CON1_SAMP              (1 << 1)  /* Bit 1:  ADC sample enable */
#define ADC_CON1_ASAM              (1 << 2)  /* Bit 2:  ADC sample auto start */
#define ADC_CON1_CLRASAM           (1 << 4)  /* Bit 4:  Stop conversion sequence */
#define ADC_CON1_SSRC_SHIFT        (5)       /* Bits 5-7: Conversion trigger source select */
#define ADC_CON1_SSRC_MASK         (7 << ADC_CON1_SSRC_SHIFT)
#  define ADC_CON1_SSRC_SAMP       (0 << ADC_CON1_SSRC_SHIFT) /* Clearing SAMP starts */
#  define ADC_CON1_SSRC_INT0       (1 << ADC_CON1_SSRC_SHIFT) /* INT0 transition starts */
#  define ADC_CON1_SSRC_TIMER3     (2 << ADC_CON1_SSRC_SHIFT) /* Timer3 match starts */
#  define ADC_CON1_SSRC_COUNT      (7 << ADC_CON1_SSRC_SHIFT) /* Internal counter starts */
#define ADC_CON1_FORM_SHIFT        (8)       /* Bits 8-10: Data output format */
#define ADC_CON1_FORM_MASK         (7 << ADC_CON1_FORM_SHIFT)
#  define ADC_CON1_FORM_UINT16     (0 << ADC_CON1_FORM_SHIFT) /* Integer 16-bit */
#  define ADC_CON1_FORM_SINT16     (1 << ADC_CON1_FORM_SHIFT) /* Signed integer 16-bit */
#  define ADC_CON1_FORM_FRAC16     (2 << ADC_CON1_FORM_SHIFT) /* Fractional 16-bit */
#  define ADC_CON1_FORM_SFRAC16    (3 << ADC_CON1_FORM_SHIFT) /* Signed fractional 16-bit */
#  define ADC_CON1_FORM_UINT32     (4 << ADC_CON1_FORM_SHIFT) /* Integer 32-bit */
#  define ADC_CON1_FORM_SINT32     (5 << ADC_CON1_FORM_SHIFT) /* Signed integer 32-bit */
#  define ADC_CON1_FORM_FRAC32     (6 << ADC_CON1_FORM_SHIFT) /* Fractional 32-bit */
#  define ADC_CON1_FORM_SFRAC32    (7 << ADC_CON1_FORM_SHIFT) /* Signed fractional 32-bit */
#define ADC_CON1_SIDL              (1 << 13) /* Bit 13: Stop in idle mode */
#define ADC_CON1_FRZ               (1 << 14) /* Bit 14: Freeze in debug exception mode */
#define ADC_CON1_ON                (1 << 15) /* Bit 14: ADC operating mode */

/* ADC control register 2 */

#define ADC_CON2_ALTS              (1 << 0)  /* Bit 0:  Alternate input sample mode select */
#define ADC_CON2_BUFM              (1 << 1)  /* Bit 1:  ADC result buffer mode select */
#define ADC_CON2_SMPI_SHIFT        (2)       /* Bits 2-5: Sample/sequences per interrupt */
#define ADC_CON2_SMPI_MASK         (15 << ADC_CON2_SMPI_SHIFT)
#  define ADC_CON2_SMPI(n)         ((n-1) << ADC_CON2_SMPI_SHIFT) /* Interrupt after nth conversion */
#define ADC_CON2_BUFS              (1 << 7)  /* Bit 7:  Buffer fill status */
#define ADC_CON2_CSCNA             (1 << 10) /* Bit 10: Scan input selections */
#define ADC_CON2_OFFCAL            (1 << 12) /* Bit 12: Input offset calibration mode select */
#define ADC_CON2_VCFG_SHIFT        (13)      /* Bits 13-15: Voltage reference configuation */
#define ADC_CON2_VCFG_MASK         (15 << ADC_CON2_VCFG_SHIFT)
#  define ADC_CON2_VCFG_AVDDAVSS   (0 << ADC_CON2_VCFG_SHIFT) /* VR+=AVdd  VR-=AVss */
#  define ADC_CON2_VCFG_VREFAVSS   (1 << ADC_CON2_VCFG_SHIFT) /* VR+=VREF+ VR-=AVss */
#  define ADC_CON2_VCFG_AVDDVREF   (2 << ADC_CON2_VCFG_SHIFT) /* VR+=AVdd  VR-=VREF- */
#  define ADC_CON2_VCFG_VREFVREF   (3 << ADC_CON2_VCFG_SHIFT) /* VR+=VREF+ VR-=VREF- */

/* ADC control register 3 */

#define ADC_CON3_ADCS_SHIFT        (0)       /* Bits 0-7: ADC conversion clock select */
#define ADC_CON3_ADCS_MASK         (0xff << ADC_CON3_ADCS_SHIFT)
#  define ADC_CON3_ADCS(n)         ((((n)>>1)-1) << ADC_CON3_ADCS_SHIFT) /* n*Tpb = Tad, n=2,4,..,512 */
#define ADC_CON3_SAMC_SHIFT        (8)       /* Bits 8-12: Auto-sample time bits */
#define ADC_CON3_SAMC_MASK         (31 << ADC_CON3_SAMC_SHIFT)
#  define ADC_CON3_SAMC(n)         ((n) << ADC_CON3_SAMC_SHIFT) /* Tad = n, n=1..15 */
#define ADC_CON3_ADRC              (1 << 15) /* Bit 15: ADC conversion clock source */

/* ADC input pin selection register */

#define ADC_CHS_CH0SA_SHIFT        (16)      /* Bits 16-19: MUX A positive input select */
#define ADC_CHS_CH0SA_MASK         (15 << ADC_CHS_CH0SA_SHIFT)
#  define ADC_CHS_CH0SA(n)         ((n) << ADC_CHS_CH0SA_SHIFT) /* Channel 0 positive input ANn, n=0..15 */
#define ADC_CHS_CH0NA              (1 << 23) /* Bit 23: MUX A negative input select */
#define ADC_CHS_CH0SB_SHIFT        (24)      /* Bits 24-27: MUX B positive input select */
#define ADC_CHS_CH0SB_MASK         (15 << ADC_CHS_CH0SB_SHIFT)
#  define ADC_CHS_CH0SB(n)         ((n) << ADC_CHS_CH0SB_SHIFT) /* Channel 0 positive input ANn, n=0..15 */
#define ADC_CHS_CH0NB              (1 << 31) /* Bit 31: MUX B negative input select */

/* ADC sequentially scanned input register */

#define ADC_CSSL(n)                (1 << (n)) /* Bit n: xx n, n=0..15 */

/* ADC input pin configuration register */

#define ADC_CFG(n)                 (1 << (n)) /* Bit n: xx n, n=0..15 */

/* ADC result word 0-15 -- 32-bits of ADC result data */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

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
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_ADC_H */
