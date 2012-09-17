/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_adc.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_ADC_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_ADC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_ADC_SC1A_OFFSET  0x0000 /* ADC status and control registers 1 */
#define KINETIS_ADC_SC1B_OFFSET  0x0004 /* ADC status and control registers 1 */
#define KINETIS_ADC_CFG1_OFFSET  0x0008 /* ADC configuration register 1 */
#define KINETIS_ADC_CFG2_OFFSET  0x000c /* Configuration register 2 */
#define KINETIS_ADC_RA_OFFSET    0x0010 /* ADC data result register */
#define KINETIS_ADC_RB_OFFSET    0x0014 /* ADC data result register */
#define KINETIS_ADC_CV1_OFFSET   0x0018 /* Compare value registers */
#define KINETIS_ADC_CV2_OFFSET   0x001c /* Compare value registers */
#define KINETIS_ADC_SC2_OFFSET   0x0020 /* Status and control register 2 */
#define KINETIS_ADC_SC3_OFFSET   0x0024 /* Status and control register 3 */
#define KINETIS_ADC_OFS_OFFSET   0x0028 /* ADC offset correction register */
#define KINETIS_ADC_PG_OFFSET    0x002c /* ADC plus-side gain register */
#define KINETIS_ADC_MG_OFFSET    0x0030 /* ADC minus-side gain register */
#define KINETIS_ADC_CLPD_OFFSET  0x0034 /* ADC plus-side general calibration value register */
#define KINETIS_ADC_CLPS_OFFSET  0x0038 /* ADC plus-side general calibration value register */
#define KINETIS_ADC_CLP4_OFFSET  0x003c /* ADC plus-side general calibration value register */
#define KINETIS_ADC_CLP3_OFFSET  0x0040 /* ADC plus-side general calibration value register */
#define KINETIS_ADC_CLP2_OFFSET  0x0044 /* ADC plus-side general calibration value register */
#define KINETIS_ADC_CLP1_OFFSET  0x0048 /* ADC plus-side general calibration value register */
#define KINETIS_ADC_CLP0_OFFSET  0x004c /* ADC plus-side general calibration value register */
#define KINETIS_ADC_PGA_OFFSET   0x0050 /* ADC PGA register */
#define KINETIS_ADC_CLMD_OFFSET  0x0054 /* ADC minus-side general calibration value register */
#define KINETIS_ADC_CLMS_OFFSET  0x0058 /* ADC minus-side general calibration value register */
#define KINETIS_ADC_CLM4_OFFSET  0x005c /* ADC minus-side general calibration value register */
#define KINETIS_ADC_CLM3_OFFSET  0x0060 /* ADC minus-side general calibration value register */
#define KINETIS_ADC_CLM2_OFFSET  0x0064 /* ADC minus-side general calibration value register */
#define KINETIS_ADC_CLM1_OFFSET  0x0068 /* ADC minus-side general calibration value register */
#define KINETIS_ADC_CLM0_OFFSET  0x006c /* ADC minus-side general calibration value register */

/* Register Addresses ***********************************************************************/
# define KINETIS_ADC1_BASE      0x400bb000 /* Analog-to-digital converter (ADC) 1 */

#define KINETIS_ADC0_SC1A        (KINETIS_ADC0_BASE+KINETIS_ADC_SC1A_OFFSET)
#define KINETIS_ADC0_SC1B        (KINETIS_ADC0_BASE+KINETIS_ADC_SC1B_OFFSET)
#define KINETIS_ADC0_CFG1        (KINETIS_ADC0_BASE+KINETIS_ADC_CFG1_OFFSET)
#define KINETIS_ADC0_CFG2        (KINETIS_ADC0_BASE+KINETIS_ADC_CFG2_OFFSET)
#define KINETIS_ADC0_RA          (KINETIS_ADC0_BASE+KINETIS_ADC_RA_OFFSET)
#define KINETIS_ADC0_RB          (KINETIS_ADC0_BASE+KINETIS_ADC_RB_OFFSET)
#define KINETIS_ADC0_CV1         (KINETIS_ADC0_BASE+KINETIS_ADC_CV1_OFFSET)
#define KINETIS_ADC0_CV2         (KINETIS_ADC0_BASE+KINETIS_ADC_CV2_OFFSET)
#define KINETIS_ADC0_SC2         (KINETIS_ADC0_BASE+KINETIS_ADC_SC2_OFFSET)
#define KINETIS_ADC0_SC3         (KINETIS_ADC0_BASE+KINETIS_ADC_SC3_OFFSET)
#define KINETIS_ADC0_OFS         (KINETIS_ADC0_BASE+KINETIS_ADC_OFS_OFFSET)
#define KINETIS_ADC0_PG          (KINETIS_ADC0_BASE+KINETIS_ADC_PG_OFFSET)
#define KINETIS_ADC0_MG          (KINETIS_ADC0_BASE+KINETIS_ADC_MG_OFFSET)
#define KINETIS_ADC0_CLPD        (KINETIS_ADC0_BASE+KINETIS_ADC_CLPD_OFFSET)
#define KINETIS_ADC0_CLPS        (KINETIS_ADC0_BASE+KINETIS_ADC_CLPS_OFFSET)
#define KINETIS_ADC0_CLP4        (KINETIS_ADC0_BASE+KINETIS_ADC_CLP4_OFFSET)
#define KINETIS_ADC0_CLP3        (KINETIS_ADC0_BASE+KINETIS_ADC_CLP3_OFFSET)
#define KINETIS_ADC0_CLP2        (KINETIS_ADC0_BASE+KINETIS_ADC_CLP2_OFFSET)
#define KINETIS_ADC0_CLP1        (KINETIS_ADC0_BASE+KINETIS_ADC_CLP1_OFFSET)
#define KINETIS_ADC0_CLP0        (KINETIS_ADC0_BASE+KINETIS_ADC_CLP0_OFFSET)
#define KINETIS_ADC0_PGA         (KINETIS_ADC0_BASE+KINETIS_ADC_PGA_OFFSET)
#define KINETIS_ADC0_CLMD        (KINETIS_ADC0_BASE+KINETIS_ADC_CLMD_OFFSET)
#define KINETIS_ADC0_CLMS        (KINETIS_ADC0_BASE+KINETIS_ADC_CLMS_OFFSET)
#define KINETIS_ADC0_CLM4        (KINETIS_ADC0_BASE+KINETIS_ADC_CLM4_OFFSET)
#define KINETIS_ADC0_CLM3        (KINETIS_ADC0_BASE+KINETIS_ADC_CLM3_OFFSET)
#define KINETIS_ADC0_CLM2        (KINETIS_ADC0_BASE+KINETIS_ADC_CLM2_OFFSET)
#define KINETIS_ADC0_CLM1        (KINETIS_ADC0_BASE+KINETIS_ADC_CLM1_OFFSET)
#define KINETIS_ADC0_CLM0        (KINETIS_ADC0_BASE+KINETIS_ADC_CLM0_OFFSET)

#define KINETIS_ADC1_SC1A        (KINETIS_ADC1_BASE+KINETIS_ADC_SC1A_OFFSET)
#define KINETIS_ADC1_SC1B        (KINETIS_ADC1_BASE+KINETIS_ADC_SC1B_OFFSET)
#define KINETIS_ADC1_CFG1        (KINETIS_ADC1_BASE+KINETIS_ADC_CFG1_OFFSET)
#define KINETIS_ADC1_CFG2        (KINETIS_ADC1_BASE+KINETIS_ADC_CFG2_OFFSET)
#define KINETIS_ADC1_RA          (KINETIS_ADC1_BASE+KINETIS_ADC_RA_OFFSET)
#define KINETIS_ADC1_RB          (KINETIS_ADC1_BASE+KINETIS_ADC_RB_OFFSET)
#define KINETIS_ADC1_CV1         (KINETIS_ADC1_BASE+KINETIS_ADC_CV1_OFFSET)
#define KINETIS_ADC1_CV2         (KINETIS_ADC1_BASE+KINETIS_ADC_CV2_OFFSET)
#define KINETIS_ADC1_SC2         (KINETIS_ADC1_BASE+KINETIS_ADC_SC2_OFFSET)
#define KINETIS_ADC1_SC3         (KINETIS_ADC1_BASE+KINETIS_ADC_SC3_OFFSET)
#define KINETIS_ADC1_OFS         (KINETIS_ADC1_BASE+KINETIS_ADC_OFS_OFFSET)
#define KINETIS_ADC1_PG          (KINETIS_ADC1_BASE+KINETIS_ADC_PG_OFFSET)
#define KINETIS_ADC1_MG          (KINETIS_ADC1_BASE+KINETIS_ADC_MG_OFFSET)
#define KINETIS_ADC1_CLPD        (KINETIS_ADC1_BASE+KINETIS_ADC_CLPD_OFFSET)
#define KINETIS_ADC1_CLPS        (KINETIS_ADC1_BASE+KINETIS_ADC_CLPS_OFFSET)
#define KINETIS_ADC1_CLP4        (KINETIS_ADC1_BASE+KINETIS_ADC_CLP4_OFFSET)
#define KINETIS_ADC1_CLP3        (KINETIS_ADC1_BASE+KINETIS_ADC_CLP3_OFFSET)
#define KINETIS_ADC1_CLP2        (KINETIS_ADC1_BASE+KINETIS_ADC_CLP2_OFFSET)
#define KINETIS_ADC1_CLP1        (KINETIS_ADC1_BASE+KINETIS_ADC_CLP1_OFFSET)
#define KINETIS_ADC1_CLP0        (KINETIS_ADC1_BASE+KINETIS_ADC_CLP0_OFFSET)
#define KINETIS_ADC1_PGA         (KINETIS_ADC1_BASE+KINETIS_ADC_PGA_OFFSET)
#define KINETIS_ADC1_CLMD        (KINETIS_ADC1_BASE+KINETIS_ADC_CLMD_OFFSET)
#define KINETIS_ADC1_CLMS        (KINETIS_ADC1_BASE+KINETIS_ADC_CLMS_OFFSET)
#define KINETIS_ADC1_CLM4        (KINETIS_ADC1_BASE+KINETIS_ADC_CLM4_OFFSET)
#define KINETIS_ADC1_CLM3        (KINETIS_ADC1_BASE+KINETIS_ADC_CLM3_OFFSET)
#define KINETIS_ADC1_CLM2        (KINETIS_ADC1_BASE+KINETIS_ADC_CLM2_OFFSET)
#define KINETIS_ADC1_CLM1        (KINETIS_ADC1_BASE+KINETIS_ADC_CLM1_OFFSET)
#define KINETIS_ADC1_CLM0        (KINETIS_ADC1_BASE+KINETIS_ADC_CLM0_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* ADC status and control registers 1 */

#define ADC_SC1_ADCH_SHIFT        (0)       /* Bits 0-4: Input channel select */
#define ADC_SC1_ADCH_MASK         (31 << ADC_SC1_ADCH_SHIFT)
#  define ADC_SC1_ADCH_DADP0      (0 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 DADP0; DIFF=1, DAD0 */
#  define ADC_SC1_ADCH_DADP1      (1 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 DADP1; DIFF=1, DAD1 */
#  define ADC_SC1_ADCH_DADP2      (2 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 DADP2; DIFF=1, DAD2 */
#  define ADC_SC1_ADCH_DADP3      (3 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 DADP3; DIFF=1, DAD3 */
#  define ADC_SC1_ADCH_AD4        (4 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 AD4; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD5        (5 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 AD5; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD6        (6 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 AD6; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD7        (7 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 AD7; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD8        (8 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 AD8; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD9        (9 << ADC_SC1_ADCH_SHIFT)  /* DIFF=0 AD9; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD10       (10 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD10; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD11       (11 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD11; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD12       (12 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD12; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD13       (13 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD13; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD14       (14 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD14; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD15       (15 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD15; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD16       (16 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD16; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD17       (17 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD17; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD18       (18 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD18; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD19       (19 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD19; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD20       (20 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD20; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD21       (21 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD21; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD22       (22 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD22; DIFF=1 reserved */
#  define ADC_SC1_ADCH_AD23       (23 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 AD23; DIFF=1 reserved */
#  define ADC_SC1_ADCH_TEMP       (26 << ADC_SC1_ADCH_SHIFT) /* Temp sensor */
#  define ADC_SC1_ADCH_BANDGAP    (27 << ADC_SC1_ADCH_SHIFT) /* Bandgap */
#  define ADC_SC1_ADCH_VREFSH     (29 << ADC_SC1_ADCH_SHIFT) /* VREFSH */
#  define ADC_SC1_ADCH_VREFSL     (30 << ADC_SC1_ADCH_SHIFT) /* DIFF=0 VREFSL; DIFF=1 reserved */
#  define ADC_SC1_ADCH_DISABLED   (31 << ADC_SC1_ADCH_SHIFT) /* Module disabled */
#define ADC_SC1_DIFF              (1 << 5)  /* Bit 5:  Differential mode enable */
#define ADC_SC1_AIEN              (1 << 6)  /* Bit 6:  Interrupt enable */
#define ADC_SC1_COCO              (1 << 7)  /* Bit 7:  Conversion complete flag */
                                            /* Bits 8-31: Reserved */
/* ADC configuration register 1 */

#define ADC_CFG1_ADICLK_SHIFT     (0)       /* Bits 0-1: Input clock select */
#define ADC_CFG1_ADICLK_MASK      (3 << ADC_CFG1_ADICLK_SHIFT)
#  define ADC_CFG1_ADICLK_BUSCLK  (0 << ADC_CFG1_ADICLK_SHIFT) /* Bus clock */
#  define ADC_CFG1_ADICLK_BUSDIV2 (1 << ADC_CFG1_ADICLK_SHIFT) /* Bus clock/ 2 */
#  define ADC_CFG1_ADICLK_ALTCLK  (2 << ADC_CFG1_ADICLK_SHIFT) /* Alternate clock */
#  define ADC_CFG1_ADICLK_ADACK   (3 << ADC_CFG1_ADICLK_SHIFT) /* Asynchronous clock */
#define ADC_CFG1_MODE_SHIFT       (2)       /* Bits 2-3: Conversion mode selection */
#define ADC_CFG1_MODE_MASK        (3 << ADC_CFG1_MODE_SHIFT)
# define ADC_CFG1_MODE_89BIT      (0 << ADC_CFG1_MODE_SHIFT) /* DIFF=0 8-bit; DIFF=1 9-bit */
# define ADC_CFG1_MODE_1213BIT    (1 << ADC_CFG1_MODE_SHIFT) /* DIFF=0 12-bit; DIFF=1 13-bit */
# define ADC_CFG1_MODE_1011BIT    (2 << ADC_CFG1_MODE_SHIFT) /* DIFF=0 10-bit; DIFF=1 11-bit */
# define ADC_CFG1_MODE_1616BIT    (3 << ADC_CFG1_MODE_SHIFT) /*  DIFF=0 16-bit; DIFF=1 16-bit */
#define ADC_CFG1_ADLSMP           (1 << 4)  /* Bit 4:  Sample time configuration */
#define ADC_CFG1_ADIV_SHIFT       (5)       /* Bits 5-6: Clock divide select */
#define ADC_CFG1_ADIV_MASK        (3 << ADC_CFG1_ADIV_SHIFT)
#  define ADC_CFG1_ADIV_DIV1      (0 << ADC_CFG1_ADIV_SHIFT) /* Divider=1 rate=input clock */
#  define ADC_CFG1_ADIV_DIV2      (1 << ADC_CFG1_ADIV_SHIFT) /* Divider=2 rate=input clock/2 */
#  define ADC_CFG1_ADIV_DIV4      (2 << ADC_CFG1_ADIV_SHIFT) /* Divider=4 rate=input clock/4 */
#  define ADC_CFG1_ADIV_DIV5      (3 << ADC_CFG1_ADIV_SHIFT) /* Divider=8 rate=input clock/8 */
#define ADC_CFG1_ADLPC            (1 << 7)  /* Bit 7:  Low-power configuration */
                                            /* Bits 8-31: Reserved */
/* Configuration register 2 */

#define ADC_CFG2_ADLSTS_SHIFT     (0)       /* Bits 0-1: Long sample time select */
#define ADC_CFG2_ADLSTS_MASK      (3 << ADC_CFG2_ADLSTS_SHIFT)
#  define ADC_CFG2_ADLSTS_LONGEST (0 << ADC_CFG2_ADLSTS_SHIFT) /* Default longest sample time */
#  define ADC_CFG2_ADLSTS_PLUS12  (1 << ADC_CFG2_ADLSTS_SHIFT) /* 12 extra ADCK cycles */
#  define ADC_CFG2_ADLSTS_PLUS6   (2 << ADC_CFG2_ADLSTS_SHIFT) /* 6 extra ADCK cycles */
#  define ADC_CFG2_ADLSTS_PLUS2   (3 << ADC_CFG2_ADLSTS_SHIFT) /* 2 extra ADCK cycles */
#define ADC_CFG2_ADHSC            (1 << 2)  /* Bit 2:  High speed configuration */
#define ADC_CFG2_ADACKEN          (1 << 3)  /* Bit 3:  Asynchronous clock output enable */
#define ADC_CFG2_MUXSEL           (1 << 4)  /* Bit 4:  ADC Mux select */
                                            /* Bits 5-31: Reserved */
/* ADC data result register */

#define ADC_R_MASK                (0xffff)  /* 16-bit signed or unsigned data */

/* Compare value registers */

#define ADC_CV_MASK               (0xffff)  /* Compare value */

/* Status and control register 2 */

#define ADC_SC2_REFSEL_SHIFT      (0)       /* Bits 0-1: Voltage reference selection */
#define ADC_SC2_REFSEL_MASK       (3 << ADC_SC2_REFSEL_SHIFT)
#  define ADC_SC2_REFSEL_DEFAULT  (0 << ADC_SC2_REFSEL_SHIFT) /* Default reference: V REFH and V REFL */
#  define ADC_SC2_REFSEL_ALT      (1 << ADC_SC2_REFSEL_SHIFT) /* Alternate reference: V ALTH and V ALTL */
#define ADC_SC2_DMAEN             (1 << 2)  /* Bit 2:  DMA enable */
#define ADC_SC2_ACREN             (1 << 3)  /* Bit 3:  Compare function range enable */
#define ADC_SC2_ACFGT             (1 << 4)  /* Bit 4:  Compare function greater than enable */
#define ADC_SC2_ACFE              (1 << 5)  /* Bit 5:  Compare function enable */
#define ADC_SC2_ADTRG             (1 << 6)  /* Bit 6:  Conversion trigger select */
#define ADC_SC2_ADACT             (1 << 7)  /* Bit 7:  Conversion active */
                                            /* Bits 8-31: Reserved */
/* Status and control register 3 */

#define ADC_SC3_AVGS_SHIFT        (0)       /* Bits 0-1: Hardware average select */
#define ADC_SC3_AVGS_MASK         (3 << ADC_SC3_AVGS_SHIFT)
#  define ADC_SC3_AVGS_4SMPLS     (0 << ADC_SC3_AVGS_SHIFT) /* 4 samples averaged */
#  define ADC_SC3_AVGS_8SMPLS     (1 << ADC_SC3_AVGS_SHIFT) /* 8 samples averaged */
#  define ADC_SC3_AVGS_16SMPLS    (2 << ADC_SC3_AVGS_SHIFT) /* 18 samples averaged */
#  define ADC_SC3_AVGS_32SMPLS    (3 << ADC_SC3_AVGS_SHIFT) /* 32 samples averaged */
#define ADC_SC3_AVGE              (1 << 2)  /* Bit 2:  Hardware average enable */
#define ADC_SC3_ADCO              (1 << 3)  /* Bit 3:  Continuous conversion enable */
                                            /* Bits 4-5: Reserved */
#define ADC_SC3_CALF              (1 << 6)  /* Bit 6:  Calibration failed flag */
#define ADC_SC3_CAL               (1 << 7)  /* Bit 7:  Calibration */
                                            /* Bits 8-31: Reserved */
/* ADC offset correction register */

#define ADC_OFS_MASK              (0xffff)  /* Bits 0-15: Offset error correction value */

/* ADC plus-side gain register */

#define ADC_PG_MASK               (0xffff)  /* Bits 0-15: Plus-side gain */

/* ADC minus-side gain register */

#define ADC_MG_MASK               (0xffff)  /* Bits 0-15: Minus-side gain */

/* ADC plus-side general calibration value registers */

#define ADC_CLPD_MASK             (0x3f)    /* Bits 0-5: Calibration value */
#define ADC_CLPS_MASK             (0x3f)    /* Bits 0-5: Calibration value */
#define ADC_CLP4_MASK             (0x3ff)   /* Bits 0-9: Calibration value */
#define ADC_CLP3_MASK             (0x1ff)   /* Bits 0-8: Calibration value */
#define ADC_CLP2_MASK             (0xff)    /* Bits 0-7: Calibration value */
#define ADC_CLP1_MASK             (0x7f)    /* Bits 0-6: Calibration value */
#define ADC_CLP0_MASK             (0x3f)    /* Bits 0-5: Calibration value */

/* ADC PGA register */
                                            /* Bits 0-15: Reserved */
#define ADC_PGA_PGAG_SHIFT        (16)      /* Bits 16-19: PGA gain setting*/
#define ADC_PGA_PGAG_MASK         (15 << ADC_PGA_PGAG_SHIFT)
#  define ADC_PGA_PGAG_1          (0 << ADC_PGA_PGAG_SHIFT)
#  define ADC_PGA_PGAG_2          (1 << ADC_PGA_PGAG_SHIFT)
#  define ADC_PGA_PGAG_4          (2 << ADC_PGA_PGAG_SHIFT)
#  define ADC_PGA_PGAG_8          (3 << ADC_PGA_PGAG_SHIFT)
#  define ADC_PGA_PGAG_16         (4 << ADC_PGA_PGAG_SHIFT)
#  define ADC_PGA_PGAG_32         (5 << ADC_PGA_PGAG_SHIFT)
#  define ADC_PGA_PGAG_64         (6 << ADC_PGA_PGAG_SHIFT)
#ifdef KINETIS_K40
#  define ADC_PGA_PGALP           (1 << 20) /* Bit 20:  PGA low-power mode control */
#endif
                                            /* Bits 21-22: Reserved */
#define ADC_PGA_PGAEN             (1 << 23) /* Bit 23:  PGA enable*/
                                            /* Bits 24-31: Reserved */
/* ADC minus-side general calibration value registers */

#define ADC_CLMD_MASK             (0x3f)    /* Bits 0-5: Calibration value */
#define ADC_CLMS_MASK             (0x3f)    /* Bits 0-5: Calibration value */
#define ADC_CLM4_MASK             (0x3ff)   /* Bits 0-9: Calibration value */
#define ADC_CLM3_MASK             (0x1ff)   /* Bits 0-8: Calibration value */
#define ADC_CLM2_MASK             (0xff)    /* Bits 0-7: Calibration value */
#define ADC_CLM1_MASK             (0x7f)    /* Bits 0-6: Calibration value */
#define ADC_CLM0_MASK             (0x3f)    /* Bits 0-5: Calibration value */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_ADC_H */
