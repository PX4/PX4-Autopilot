/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-oc.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OC_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx-memorymap.h"

#if CHIP_NOC > 0

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_OC_CON_OFFSET    0x0000 /* Output compare control register */
#define PIC32MX_OC_CONCLR_OFFSET 0x0004 /* Output compare control clear register */
#define PIC32MX_OC_CONSET_OFFSET 0x0008 /* Output compare control set register */
#define PIC32MX_OC_CONINV_OFFSET 0x000c /* Output compare control invert register */
#define PIC32MX_OC_R_OFFSET      0x0010 /* Output compare data register */
#define PIC32MX_OC_RCLR_OFFSET   0x0014 /* Output compare data clear register */
#define PIC32MX_OC_RSET_OFFSET   0x0018 /* Output compare data set register */
#define PIC32MX_OC_RINV_OFFSET   0x001c /* Output compare data invert register */
#define PIC32MX_OC_RS_OFFSET     0x0020 /* Output compare secondary data register */
#define PIC32MX_OC_RSCLR_OFFSET  0x0024 /* Output compare secondary data clear register */
#define PIC32MX_OC_RSSET_OFFSET  0x0028 /* Output compare secondary data set register */
#define PIC32MX_OC_RSINV_OFFSET  0x002c /* Output compare secondary data invert register */

/* See also TIMER2 and TIMER3 registers */

/* Register Addresses ***************************************************************/

#define PIC32MX_OC_CON(n)          (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_CON_OFFSET)
#define PIC32MX_OC_CONCLR(n)       (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_CONCLR_OFFSET)
#define PIC32MX_OC_CONSET(n)       (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_CONSET_OFFSET)
#define PIC32MX_OC_CONINV(n)       (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_CONINV_OFFSET)
#define PIC32MX_OC_R(n)            (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_R_OFFSET)
#define PIC32MX_OC_RCLR(n)         (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_RCLR_OFFSET)
#define PIC32MX_OC_RSET(n)         (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_RSET_OFFSET)
#define PIC32MX_OC_RINV(n)         (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_RINV_OFFSET)
#define PIC32MX_OC_RS(n)           (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_RS_OFFSET)
#define PIC32MX_OC_RSCLR(n)        (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_RSCLR_OFFSET)
#define PIC32MX_OC_RSSET(n)        (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_RSSET_OFFSET)
#define PIC32MX_OC_RSINV(n)        (PIC32MX_OC_K1BASE(n)+PIC32MX_OC_RSINV_OFFSET)

#define PIC32MX_OC1_CON            (PIC32MX_OC1_K1BASE+PIC32MX_OC_CON_OFFSET)
#define PIC32MX_OC1_CONCLR         (PIC32MX_OC1_K1BASE+PIC32MX_OC_CONCLR_OFFSET)
#define PIC32MX_OC1_CONSET         (PIC32MX_OC1_K1BASE+PIC32MX_OC_CONSET_OFFSET)
#define PIC32MX_OC1_CONINV         (PIC32MX_OC1_K1BASE+PIC32MX_OC_CONINV_OFFSET)
#define PIC32MX_OC1_R              (PIC32MX_OC1_K1BASE+PIC32MX_OC_R_OFFSET)
#define PIC32MX_OC1_RCLR           (PIC32MX_OC1_K1BASE+PIC32MX_OC_RCLR_OFFSET)
#define PIC32MX_OC1_RSET           (PIC32MX_OC1_K1BASE+PIC32MX_OC_RSET_OFFSET)
#define PIC32MX_OC1_RINV           (PIC32MX_OC1_K1BASE+PIC32MX_OC_RINV_OFFSET)
#define PIC32MX_OC1_RS             (PIC32MX_OC1_K1BASE+PIC32MX_OC_RS_OFFSET)
#define PIC32MX_OC1_RSCLR          (PIC32MX_OC1_K1BASE+PIC32MX_OC_RSCLR_OFFSET)
#define PIC32MX_OC1_RSSET          (PIC32MX_OC1_K1BASE+PIC32MX_OC_RSSET_OFFSET)
#define PIC32MX_OC1_RSINV          (PIC32MX_OC1_K1BASE+PIC32MX_OC_RSINV_OFFSET)

#if CHIP_NOC > 1
#  define PIC32MX_OC2_CON          (PIC32MX_OC2_K1BASE+PIC32MX_OC_CON_OFFSET)
#  define PIC32MX_OC2_CONCLR       (PIC32MX_OC2_K1BASE+PIC32MX_OC_CONCLR_OFFSET)
#  define PIC32MX_OC2_CONSET       (PIC32MX_OC2_K1BASE+PIC32MX_OC_CONSET_OFFSET)
#  define PIC32MX_OC2_CONINV       (PIC32MX_OC2_K1BASE+PIC32MX_OC_CONINV_OFFSET)
#  define PIC32MX_OC2_R            (PIC32MX_OC2_K1BASE+PIC32MX_OC_R_OFFSET)
#  define PIC32MX_OC2_RCLR         (PIC32MX_OC2_K1BASE+PIC32MX_OC_RCLR_OFFSET)
#  define PIC32MX_OC2_RSET         (PIC32MX_OC2_K1BASE+PIC32MX_OC_RSET_OFFSET)
#  define PIC32MX_OC2_RINV         (PIC32MX_OC2_K1BASE+PIC32MX_OC_RINV_OFFSET)
#  define PIC32MX_OC2_RS           (PIC32MX_OC2_K1BASE+PIC32MX_OC_RS_OFFSET)
#  define PIC32MX_OC2_RSCLR        (PIC32MX_OC2_K1BASE+PIC32MX_OC_RSCLR_OFFSET)
#  define PIC32MX_OC2_RSSET        (PIC32MX_OC2_K1BASE+PIC32MX_OC_RSSET_OFFSET)
#  define PIC32MX_OC2_RSINV        (PIC32MX_OC2_K1BASE+PIC32MX_OC_RSINV_OFFSET)
#endif

#if CHIP_NOC > 2
#  define PIC32MX_OC3_CON          (PIC32MX_OC3_K1BASE+PIC32MX_OC_CON_OFFSET)
#  define PIC32MX_OC3_CONCLR       (PIC32MX_OC3_K1BASE+PIC32MX_OC_CONCLR_OFFSET)
#  define PIC32MX_OC3_CONSET       (PIC32MX_OC3_K1BASE+PIC32MX_OC_CONSET_OFFSET)
#  define PIC32MX_OC3_CONINV       (PIC32MX_OC3_K1BASE+PIC32MX_OC_CONINV_OFFSET)
#  define PIC32MX_OC3_R            (PIC32MX_OC3_K1BASE+PIC32MX_OC_R_OFFSET)
#  define PIC32MX_OC3_RCLR         (PIC32MX_OC3_K1BASE+PIC32MX_OC_RCLR_OFFSET)
#  define PIC32MX_OC3_RSET         (PIC32MX_OC3_K1BASE+PIC32MX_OC_RSET_OFFSET)
#  define PIC32MX_OC3_RINV         (PIC32MX_OC3_K1BASE+PIC32MX_OC_RINV_OFFSET)
#  define PIC32MX_OC3_RS           (PIC32MX_OC3_K1BASE+PIC32MX_OC_RS_OFFSET)
#  define PIC32MX_OC3_RSCLR        (PIC32MX_OC3_K1BASE+PIC32MX_OC_RSCLR_OFFSET)
#  define PIC32MX_OC3_RSSET        (PIC32MX_OC3_K1BASE+PIC32MX_OC_RSSET_OFFSET)
#  define PIC32MX_OC3_RSINV        (PIC32MX_OC3_K1BASE+PIC32MX_OC_RSINV_OFFSET)
#endif

#if CHIP_NOC > 3
#  define PIC32MX_OC4_CON          (PIC32MX_OC4_K1BASE+PIC32MX_OC_CON_OFFSET)
#  define PIC32MX_OC4_CONCLR       (PIC32MX_OC4_K1BASE+PIC32MX_OC_CONCLR_OFFSET)
#  define PIC32MX_OC4_CONSET       (PIC32MX_OC4_K1BASE+PIC32MX_OC_CONSET_OFFSET)
#  define PIC32MX_OC4_CONINV       (PIC32MX_OC4_K1BASE+PIC32MX_OC_CONINV_OFFSET)
#  define PIC32MX_OC4_R            (PIC32MX_OC4_K1BASE+PIC32MX_OC_R_OFFSET)
#  define PIC32MX_OC4_RCLR         (PIC32MX_OC4_K1BASE+PIC32MX_OC_RCLR_OFFSET)
#  define PIC32MX_OC4_RSET         (PIC32MX_OC4_K1BASE+PIC32MX_OC_RSET_OFFSET)
#  define PIC32MX_OC4_RINV         (PIC32MX_OC4_K1BASE+PIC32MX_OC_RINV_OFFSET)
#  define PIC32MX_OC4_RS           (PIC32MX_OC4_K1BASE+PIC32MX_OC_RS_OFFSET)
#  define PIC32MX_OC4_RSCLR        (PIC32MX_OC4_K1BASE+PIC32MX_OC_RSCLR_OFFSET)
#  define PIC32MX_OC4_RSSET        (PIC32MX_OC4_K1BASE+PIC32MX_OC_RSSET_OFFSET)
#  define PIC32MX_OC4_RSINV        (PIC32MX_OC4_K1BASE+PIC32MX_OC_RSINV_OFFSET)
#endif

#if CHIP_NOC > 4
#  define PIC32MX_OC5_CON          (PIC32MX_OC5_K1BASE+PIC32MX_OC_CON_OFFSET)
#  define PIC32MX_OC5_CONCLR       (PIC32MX_OC5_K1BASE+PIC32MX_OC_CONCLR_OFFSET)
#  define PIC32MX_OC5_CONSET       (PIC32MX_OC5_K1BASE+PIC32MX_OC_CONSET_OFFSET)
#  define PIC32MX_OC5_CONINV       (PIC32MX_OC5_K1BASE+PIC32MX_OC_CONINV_OFFSET)
#  define PIC32MX_OC5_R            (PIC32MX_OC5_K1BASE+PIC32MX_OC_R_OFFSET)
#  define PIC32MX_OC5_RCLR         (PIC32MX_OC5_K1BASE+PIC32MX_OC_RCLR_OFFSET)
#  define PIC32MX_OC5_RSET         (PIC32MX_OC5_K1BASE+PIC32MX_OC_RSET_OFFSET)
#  define PIC32MX_OC5_RINV         (PIC32MX_OC5_K1BASE+PIC32MX_OC_RINV_OFFSET)
#  define PIC32MX_OC5_RS           (PIC32MX_OC5_K1BASE+PIC32MX_OC_RS_OFFSET)
#  define PIC32MX_OC5_RSCLR        (PIC32MX_OC5_K1BASE+PIC32MX_OC_RSCLR_OFFSET)
#  define PIC32MX_OC5_RSSET        (PIC32MX_OC5_K1BASE+PIC32MX_OC_RSSET_OFFSET)
#  define PIC32MX_OC5_RSINV        (PIC32MX_OC5_K1BASE+PIC32MX_OC_RSINV_OFFSET)
#endif

/* Register Bit-Field Definitions ***************************************************/

/* Output compare control register */

#define OC_CON_OCM_SHIFT           (0)       /* Bits 0-2: Output compare mode select */
#define OC_CON_OCM_MASK            (7 << OC_CON_OCM_SHIFT)
#  define OC_CON_OCM_DISABLE       (0 << OC_CON_OCM_SHIFT) /* Output compare peripheral disabled */
#  define OC_CON_OCM_LOW2HI        (1 << OC_CON_OCM_SHIFT) /* OCx low; compare forces high */
#  define OC_CON_OCM_HITOLOW       (2 << OC_CON_OCM_SHIFT) /* OCx high; compare forces low */
#  define OC_CON_OCM_TOGGLE        (3 << OC_CON_OCM_SHIFT) /* Compare event toggles OCx */
#  define OC_CON_OCM_LOWPULSE      (4 << OC_CON_OCM_SHIFT) /* OCx low; output pulse on OCx*/
#  define OC_CON_OCM_HIPULSE       (5 << OC_CON_OCM_SHIFT) /* OCx high; output pulse on OCx */
#  define OC_CON_OCM_PWM           (6 << OC_CON_OCM_SHIFT) /* PWM mode on OCx; fault disabled */
#  define OC_CON_OCM_PWMFAULT      (7 << OC_CON_OCM_SHIFT) /* PWM mode on OCx; fault enabled */
#define OC_CON_OCTSEL              (1 << 3)  /* Bit 3:  Output compare timer select */
#define OC_CON_OCFLT               (1 << 4)  /* Bit 4:  PWM fault condition status */
#define OC_CON_OC32                (1 << 5)  /* Bit 5:  32-bit compare more */
#define OC_CON_SIDL                (1 << 13) /* Bit 13: Stop in idle mode */
#define OC_CON_FRZ                 (1 << 14) /* Bit 14: Freeze in debug exception mode */
#define OC_CON_ON                  (1 << 15) /* Bit 15: Output compare periperal on */

/* Output compare data register -- 32-bit data register */

/* Output compare secondary data register -- 32-bit data register */

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
#endif /* CHIP_NOC > 0 */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OC_H */
