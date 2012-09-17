/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-cm.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CM_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx-memorymap.h"

#if CHIP_NCM > 0

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_CM_CON_OFFSET     0x0000 /* Comparator control register */
#define PIC32MX_CM_CONCLR_OFFSET  0x0004 /* Comparator control clear register */
#define PIC32MX_CM_CONSET_OFFSET  0x0008 /* Comparator control set register */
#define PIC32MX_CM_CONINV_OFFSET  0x000c /* Comparator control invert register */
#define PIC32MX_CM_STAT_OFFSET    0x0060 /* Comparator status register */
#define PIC32MX_CM_STATCLR_OFFSET 0x0064 /* Comparator status clear register */
#define PIC32MX_CM_STATSET_OFFSET 0x0068 /* Comparator status set register */
#define PIC32MX_CM_STATINV_OFFSET 0x006c /* Comparator status invert register */

/* Register Addresses ***************************************************************/

#define PIC32MX_CM1_CON             (PIC32MX_CM1_K1BASE+PIC32MX_CM_CON_OFFSET)
#define PIC32MX_CM1_CONCLR          (PIC32MX_CM1_K1BASE+PIC32MX_CM_CONCLR_OFFSET)
#define PIC32MX_CM1_CONSET          (PIC32MX_CM1_K1BASE+PIC32MX_CM_CONSET_OFFSET)
#define PIC32MX_CM1_CONINV          (PIC32MX_CM1_K1BASE+PIC32MX_CM_CONINV_OFFSET)

#if CHIP_NCM > 0
#  define PIC32MX_CM2_CON           (PIC32MX_CM2_K1BASE+PIC32MX_CM_CON_OFFSET)
#  define PIC32MX_CM2_CONCLR        (PIC32MX_CM2_K1BASE+PIC32MX_CM_CONCLR_OFFSET)
#  define PIC32MX_CM2_CONSET        (PIC32MX_CM2_K1BASE+PIC32MX_CM_CONSET_OFFSET)
#  define PIC32MX_CM2_CONINV        (PIC32MX_CM2_K1BASE+PIC32MX_CM_CONINV_OFFSET)
#endif

#define PIC32MX_CM_STAT             (PIC32MX_CM_K1BASE+PIC32MX_CM_STAT_OFFSET)
#define PIC32MX_CM_STATCLR          (PIC32MX_CM1_K1BASE+PIC32MX_CM_STATCLR_OFFSET)
#define PIC32MX_CM_STATSET          (PIC32MX_CM1_K1BASE+PIC32MX_CM_STATSET_OFFSET)
#define PIC32MX_CM_STATINV          (PIC32MX_CM1_K1BASE+PIC32MX_CM_STATINV_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Comparator control register */

#define CM_CON_CCH_SHIFT            (0)       /* Bits 0-1: Comparator negative input select */
#define CM_CON_CCH_MASK             (3 << CM_CON_CCH_SHIFT)
#  define CM_CON_CCH_CXINM          (0 << CM_CON_CCH_SHIFT) /* Inverting input connected to CxIN- */
#  define CM_CON_CCH_CXINP          (1 << CM_CON_CCH_SHIFT) /* Inverting input connected to CxIN+ */
#  define CM_CON_CCH_CYINP          (2 << CM_CON_CCH_SHIFT) /* Inverting input connected to CyIN+ */
#  define CM_CON_CCH_IVREF          (3 << CM_CON_CCH_SHIFT) /* Inverting input connected to IVREF */
#define CM_CON_CREF                 (1 << 4)  /* Bit 4:  Comparator positive input configure */
#define CM_CON_EVPOL_SHIFT          (6)       /* Bits 6-7: Interrupt event polarity select */
#define CM_CON_EVPOL_MASK           (3 << CM_CON_EVPOL_SHIFT)
#  define CM_CON_EVPOL_DISABLED     (0 << CM_CON_EVPOL_SHIFT) /* Interrupt disabled */
#  define CM_CON_EVPOL_RISING       (1 << CM_CON_EVPOL_SHIFT) /* Interrupt on low-to-high transition */
#  define CM_CON_EVPOL_FALLING      (2 << CM_CON_EVPOL_SHIFT) /* Interrupt on high-to-low transition */
#  define CM_CON_EVPOL_BOTH         (3 << CM_CON_EVPOL_SHIFT) /* Interrupt on a both transitions */
#define CM_CON_COUT                 (1 << 8)  /* Bit 8:  Comparator output */
#define CM_CON_CPOL                 (1 << 13) /* Bit 13: Comparator output inversion */
#define CM_CON_COE                  (1 << 14) /* Bit 14: Comparator output enable */
#define CM_CON_ON                   (1 << 15) /* Bit 15: Comparator ON */

/* Comparator status register */

#define CM_STAT_C1OUT               (1 << 0)  /* Bit 0:  Comparator 1 output */
#define CM_STAT_C2OUT               (1 << 1)  /* Bit 1:  Comparator 2 output */
#define CM_STAT_SIDL                (1 << 13) /* Bit 13: Stop in idle control */
#define CM_STAT_FRZ                 (1 << 14) /* Bit 14: Freeze control */

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
#endif /* CHIP_NCM > 0 */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CM_H */
