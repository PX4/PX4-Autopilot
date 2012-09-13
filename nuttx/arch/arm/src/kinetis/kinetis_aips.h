/************************************************************************************
 * arch/arm/src/kinetis/kinetis_aips.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_AIPS_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_AIPS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define KINETIS_AIPS_MPRA_OFFSET    0x0000 /* Master Privilege Register A */

#define KINETIS_AIPS_PACRA_OFFSET   0x0020 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRB_OFFSET   0x0024 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRC_OFFSET   0x0028 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRD_OFFSET   0x002c /* Peripheral Access Control Register */

#define KINETIS_AIPS_PACRE_OFFSET   0x0040 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRF_OFFSET   0x0044 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRG_OFFSET   0x0048 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRH_OFFSET   0x004c /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRI_OFFSET   0x0050 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRJ_OFFSET   0x0054 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRK_OFFSET   0x0058 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRL_OFFSET   0x005c /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRM_OFFSET   0x0060 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRN_OFFSET   0x0064 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRO_OFFSET   0x0068 /* Peripheral Access Control Register */
#define KINETIS_AIPS_PACRP_OFFSET   0x006c /* Peripheral Access Control Register */

/* Register Addresses ***************************************************************/

#define KINETIS_AIPS0_MPRA          (KINETIS_AIPS0_BASE+KINETIS_AIPS_MPRA_OFFSET)
#define KINETIS_AIPS0_PACRA         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRA_OFFSET)
#define KINETIS_AIPS0_PACRB         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRB_OFFSET)
#define KINETIS_AIPS0_PACRC         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRC_OFFSET)
#define KINETIS_AIPS0_PACRD         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRD_OFFSET)
#define KINETIS_AIPS0_PACRE         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRE_OFFSET)
#define KINETIS_AIPS0_PACRF         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRF_OFFSET)
#define KINETIS_AIPS0_PACRG         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRG_OFFSET)
#define KINETIS_AIPS0_PACRH         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRH_OFFSET)
#define KINETIS_AIPS0_PACRI         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRI_OFFSET)
#define KINETIS_AIPS0_PACRJ         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRJ_OFFSET)
#define KINETIS_AIPS0_PACRK         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRK_OFFSET)
#define KINETIS_AIPS0_PACRL         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRL_OFFSET)
#define KINETIS_AIPS0_PACRM         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRM_OFFSET)
#define KINETIS_AIPS0_PACRN         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRN_OFFSET)
#define KINETIS_AIPS0_PACRO         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRO_OFFSET)
#define KINETIS_AIPS0_PACRP         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRP_OFFSET)

#define KINETIS_AIPS1_MPRA          (KINETIS_AIPS0_BASE+KINETIS_AIPS_MPRA_OFFSET)
#define KINETIS_AIPS1_PACRA         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRA_OFFSET)
#define KINETIS_AIPS1_PACRB         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRB_OFFSET)
#define KINETIS_AIPS1_PACRC         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRC_OFFSET)
#define KINETIS_AIPS1_PACRD         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRD_OFFSET)
#define KINETIS_AIPS1_PACRE         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRE_OFFSET)
#define KINETIS_AIPS1_PACRF         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRF_OFFSET)
#define KINETIS_AIPS1_PACRG         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRG_OFFSET)
#define KINETIS_AIPS1_PACRH         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRH_OFFSET)
#define KINETIS_AIPS1_PACRI         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRI_OFFSET)
#define KINETIS_AIPS1_PACRJ         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRJ_OFFSET)
#define KINETIS_AIPS1_PACRK         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRK_OFFSET)
#define KINETIS_AIPS1_PACRL         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRL_OFFSET)
#define KINETIS_AIPS1_PACRM         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRM_OFFSET)
#define KINETIS_AIPS1_PACRN         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRN_OFFSET)
#define KINETIS_AIPS1_PACRO         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRO_OFFSET)
#define KINETIS_AIPS1_PACRP         (KINETIS_AIPS0_BASE+KINETIS_AIPS_PACRP_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Master Privilege Register A */

                                              /* Bits 0-7: Reserved */
#define AIPS_MPRA_MPL5              (1 << 8)  /* Bit 8:  Master privilege level */
#define AIPS_MPRA_MTW5              (1 << 9)  /* Bit 9:  Master trusted for writes */
#define AIPS_MPRA_MTR5              (1 << 10) /* Bit 10:  Master trusted for read */
                                              /* Bit 11 Reserved */
#define AIPS_MPRA_MPL4              (1 << 12) /* Bit 12: Master privilege level */
#define AIPS_MPRA_MTW4              (1 << 13) /* Bit 13: Master trusted for writes */
#define AIPS_MPRA_MTR4              (1 << 14) /* Bit 14: Master trusted for read */
                                              /* Bit 15: Reserved */
#define AIPS_MPRA_MPL3              (1 << 16) /* Bit 16: Master privilege level */
#define AIPS_MPRA_MTW3              (1 << 17) /* Bit 17: Master trusted for writes */
#define AIPS_MPRA_MTR3              (1 << 18) /* Bit 18: Master trusted for read */
                                              /* Bit 19: Reserved */
#define AIPS_MPRA_MPL2              (1 << 20) /* Bit 20: Master privilege level */
#define AIPS_MPRA_MTW2              (1 << 21) /* Bit 21: Master trusted for writes */
#define AIPS_MPRA_MTR2              (1 << 22) /* Bit 22: Master trusted for read */
                                              /* Bit 23: Reserved */
#define AIPS_MPRA_MPL1              (1 << 24) /* Bit 24: Master privilege level */
#define AIPS_MPRA_MTW1              (1 << 25) /* Bit 25: Master trusted for writes */
#define AIPS_MPRA_MTR1              (1 << 26) /* Bit 26: Master trusted for read */
                                              /* Bit 27: Reserved */
#define AIPS_MPRA_MPL0              (1 << 28) /* Bit 28: Master privilege level */
#define AIPS_MPRA_MTW0              (1 << 29) /* Bit 29: Master trusted for writes */
#define AIPS_MPRA_MTR0              (1 << 30) /* Bit 30: Master trusted for read */
                                              /* Bit 31: Reserved */

/* Peripheral Access Control Register.  Naming here is only accurate for PACRA.
 * PACRA: PACR0   PACR1   PACR2   PACR3   PACR4   PACR5   PACR6   PACR7
 * PACRB: PACR8   PACR9   PACR10  PACR11  PACR12  PACR13  PACR14  PACR15
 * PACRC: PACR16  PACR17  PACR18  PACR19  PACR20  PACR21  PACR22  PACR23
 * PACRD: PACR24  PACR25  PACR26  PACR27  PACR28  PACR29  PACR30  PACR31
 * PACRE: PACR32  PACR33  PACR34  PACR35  PACR36  PACR37  PACR38  PACR39
 * PACRF: PACR40  PACR41  PACR42  PACR43  PACR44  PACR45  PACR46  PACR47
 * PACRG: PACR48  PACR49  PACR50  PACR51  PACR52  PACR53  PACR54  PACR55
 * PACRH: PACR56  PACR57  PACR58  PACR59  PACR60  PACR61  PACR62  PACR63
 * PACRI: PACR64  PACR65  PACR66  PACR67  PACR68  PACR69  PACR70  PACR71
 * PACRJ: PACR72  PACR73  PACR74  PACR75  PACR76  PACR77  PACR78  PACR79
 * PACRK: PACR80  PACR81  PACR82  PACR83  PACR84  PACR85  PACR86  PACR87
 * PACRL: PACR88  PACR89  PACR90  PACR91  PACR92  PACR93  PACR94  PACR95
 * PACRM: PACR96  PACR97  PACR98  PACR99  PACR100 PACR101 PACR102 PACR103
 * PACRN: PACR104 PACR105 PACR106 PACR107 PACR108 PACR109 PACR110 PACR111
 * PACRO: PACR112 PACR113 PACR114 PACR115 PACR116 PACR117 PACR118 PACR119
 * PACRP: PACR120 PACR121 PACR122 PACR123 PACR124 PACR125 PACR126 PACR127
 */

#define AIPS_PACR_TP(n)            (1 << ((7 - ((n) & 7)) << 2))
#define AIPS_PACR_WP(n)            (2 << ((7 - ((n) & 7)) << 2))
#define AIPS_PACR_SP(n)            (4 << ((7 - ((n) & 7)) << 2))

#define AIPS_PACR_TP7               (1 << 0)  /* Bit 0:  Trusted protect */
#define AIPS_PACR_WP7               (1 << 1)  /* Bit 1:  Write protect */
#define AIPS_PACR_SP7               (1 << 2)  /* Bit 2:  Supervisor protect */
                                              /* Bit 3:  Reserved */
#define AIPS_PACR_TP6               (1 << 4)  /* Bit 4:  Trusted protect */
#define AIPS_PACR_WP6               (1 << 5)  /* Bit 5:  Write protect */
#define AIPS_PACR_SP6               (1 << 6)  /* Bit 6:  Supervisor protect */
                                              /* Bit 7:  Reserved */
#define AIPS_PACR_TP5               (1 << 8)  /* Bit 8:  Trusted protect */
#define AIPS_PACR_WP5               (1 << 9)  /* Bit 9:  Write protect */
#define AIPS_PACR_SP5               (1 << 10) /* Bit 10: Supervisor protect */
                                              /* Bit 11: Reserved */
#define AIPS_PACR_TP4               (1 << 12) /* Bit 12: Trusted protect */
#define AIPS_PACR_WP4               (1 << 13) /* Bit 13: Write protect */
#define AIPS_PACR_SP4               (1 << 14) /* Bit 14: Supervisor protect */
                                              /* Bit 15: Reserved */
#define AIPS_PACR_TP3               (1 << 16) /* Bit 16: Trusted protect */
#define AIPS_PACR_WP3               (1 << 17) /* Bit 17: Write protect */
#define AIPS_PACR_SP3               (1 << 18) /* Bit 18: Supervisor protect */
                                              /* Bit 19: Reserved */
#define AIPS_PACR_TP2               (1 << 20) /* Bit 20: Trusted protect */
#define AIPS_PACR_WP2               (1 << 21) /* Bit 21: Write protect */
#define AIPS_PACR_SP2               (1 << 22) /* Bit 22: Supervisor protect */
                                              /* Bit 23: Reserved */
#define AIPS_PACR_TP1               (1 << 24) /* Bit 24: Trusted protect */
#define AIPS_PACR_WP1               (1 << 25) /* Bit 25: Write protect */
#define AIPS_PACR_SP1               (1 << 26) /* Bit 26: Supervisor protect */
                                              /* Bit 27: Reserved */
#define AIPS_PACR_TP0               (1 << 28) /* Bit 28: Trusted protect */
#define AIPS_PACR_WP0               (1 << 29) /* Bit 29: Write protect */
#define AIPS_PACR_SP0               (1 << 30) /* Bit 30: Supervisor protect */
                                              /* Bit 31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_AIPS_H */
