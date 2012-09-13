/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_hmatrix.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_HMATRIX_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_HMATRIX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_HMATRIX_MCFG_OFFSET(n)    (0x0000+((n)<<2))
#define AVR32_HMATRIX_MCFG0_OFFSET      0x0000 /* Master Configuration Register 0 */
#define AVR32_HMATRIX_MCFG1_OFFSET      0x0004 /* Master Configuration Register 1 */
#define AVR32_HMATRIX_MCFG2_OFFSET      0x0008 /* Master Configuration Register 2 */
#define AVR32_HMATRIX_MCFG3_OFFSET      0x000c /* Master Configuration Register 3 */
#define AVR32_HMATRIX_MCFG4_OFFSET      0x0010 /* Master Configuration Register 4 */
#define AVR32_HMATRIX_MCFG5_OFFSET      0x0014 /* Master Configuration Register 5 */
#define AVR32_HMATRIX_MCFG6_OFFSET      0x0018 /* Master Configuration Register 6 */
#define AVR32_HMATRIX_MCFG7_OFFSET      0x001c /* Master Configuration Register 7 */
#define AVR32_HMATRIX_MCFG8_OFFSET      0x0020 /* Master Configuration Register 8 */
#define AVR32_HMATRIX_MCFG9_OFFSET      0x0024 /* Master Configuration Register 9 */
#define AVR32_HMATRIX_MCFG10_OFFSET     0x0028 /* Master Configuration Register 10 */
#define AVR32_HMATRIX_MCFG11_OFFSET     0x002c /* Master Configuration Register 11 */
#define AVR32_HMATRIX_MCFG12_OFFSET     0x0030 /* Master Configuration Register 12 */
#define AVR32_HMATRIX_MCFG13_OFFSET     0x0034 /* Master Configuration Register 13 */
#define AVR32_HMATRIX_MCFG14_OFFSET     0x0038 /* Master Configuration Register 14 */
#define AVR32_HMATRIX_MCFG15_OFFSET     0x003c /* Master Configuration Register 15 */

#define AVR32_HMATRIX_SCFG_OFFSET(n)    (0x0040+((n)<<2))
#define AVR32_HMATRIX_SCFG0_OFFSET      0x0040 /* Slave Configuration Register 0 */
#define AVR32_HMATRIX_SCFG1_OFFSET      0x0044 /* Slave Configuration Register 1 */
#define AVR32_HMATRIX_SCFG2_OFFSET      0x0048 /* Slave Configuration Register 2 */
#define AVR32_HMATRIX_SCFG3_OFFSET      0x004c /* Slave Configuration Register 3 */
#define AVR32_HMATRIX_SCFG4_OFFSET      0x0050 /* Slave Configuration Register 4 */
#define AVR32_HMATRIX_SCFG5_OFFSET      0x0054 /* Slave Configuration Register 5 */
#define AVR32_HMATRIX_SCFG6_OFFSET      0x0058 /* Slave Configuration Register 6 */
#define AVR32_HMATRIX_SCFG7_OFFSET      0x005c /* Slave Configuration Register 7 */
#define AVR32_HMATRIX_SCFG8_OFFSET      0x0060 /* Slave Configuration Register 8 */
#define AVR32_HMATRIX_SCFG9_OFFSET      0x0064 /* Slave Configuration Register 9 */
#define AVR32_HMATRIX_SCFG10_OFFSET     0x0068 /* Slave Configuration Register 10 */
#define AVR32_HMATRIX_SCFG11_OFFSET     0x006c /* Slave Configuration Register 11 */
#define AVR32_HMATRIX_SCFG12_OFFSET     0x0070 /* Slave Configuration Register 12 */
#define AVR32_HMATRIX_SCFG13_OFFSET     0x0074 /* Slave Configuration Register 13 */
#define AVR32_HMATRIX_SCFG14_OFFSET     0x0078 /* Slave Configuration Register 14 */
#define AVR32_HMATRIX_SCFG15_OFFSET     0x007c /* Slave Configuration Register 15 */

#define AVR32_HMATRIX_PRAS_OFFSET(n)    (0x0080+((n)<<3))
#define AVR32_HMATRIX_PRBS_OFFSET(n)    (0x0084+((n)<<3))
#define AVR32_HMATRIX_PRAS0_OFFSET      0x0080 /* Priority Register A for Slave 0 */
#define AVR32_HMATRIX_PRBS0_OFFSET      0x0084 /* Priority Register B for Slave 0 */
#define AVR32_HMATRIX_PRAS1_OFFSET      0x0088 /* Priority Register A for Slave 1 */
#define AVR32_HMATRIX_PRBS1_OFFSET      0x008c /* Priority Register B for Slave 1 */
#define AVR32_HMATRIX_PRAS2_OFFSET      0x0090 /* Priority Register A for Slave 2 */
#define AVR32_HMATRIX_PRBS2_OFFSET      0x0094 /* Priority Register B for Slave 2 */
#define AVR32_HMATRIX_PRAS3_OFFSET      0x0098 /* Priority Register A for Slave 3 */
#define AVR32_HMATRIX_PRBS3_OFFSET      0x009c /* Priority Register B for Slave 3 */
#define AVR32_HMATRIX_PRAS4_OFFSET      0x00a0 /* Priority Register A for Slave 4 */
#define AVR32_HMATRIX_PRBS4_OFFSET      0x00a4 /* Priority Register B for Slave 4 */
#define AVR32_HMATRIX_PRAS5_OFFSET      0x00a8 /* Priority Register A for Slave 5 */
#define AVR32_HMATRIX_PRBS5_OFFSET      0x00ac /* Priority Register B for Slave 5 */
#define AVR32_HMATRIX_PRAS6_OFFSET      0x00b0 /* Priority Register A for Slave 6 */
#define AVR32_HMATRIX_PRBS6_OFFSET      0x00b4 /* Priority Register B for Slave 6 */
#define AVR32_HMATRIX_PRAS7_OFFSET      0x00b8 /* Priority Register A for Slave 7 */
#define AVR32_HMATRIX_PRBS7_OFFSET      0x00bc /* Priority Register B for Slave 7 */
#define AVR32_HMATRIX_PRAS8_OFFSET      0x00c0 /* Priority Register A for Slave 8 */
#define AVR32_HMATRIX_PRBS8_OFFSET      0x00c4 /* Priority Register B for Slave 8 */
#define AVR32_HMATRIX_PRAS9_OFFSET      0x00c8 /* Priority Register A for Slave 9 */
#define AVR32_HMATRIX_PRBS9_OFFSET      0x00cc /* Priority Register B for Slave 9 */
#define AVR32_HMATRIX_PRAS10_OFFSET     0x00d0 /* Priority Register A for Slave 10 */
#define AVR32_HMATRIX_PRBS10_OFFSET     0x00d4 /* Priority Register B for Slave 10 */
#define AVR32_HMATRIX_PRAS11_OFFSET     0x00d8 /* Priority Register A for Slave 11 */
#define AVR32_HMATRIX_PRBS11_OFFSET     0x00dc /* Priority Register B for Slave 11 */
#define AVR32_HMATRIX_PRAS12_OFFSET     0x00e0 /* Priority Register A for Slave 12 */
#define AVR32_HMATRIX_PRBS12_OFFSET     0x00e4 /* Priority Register B for Slave 12 */
#define AVR32_HMATRIX_PRAS13_OFFSET     0x00e8 /* Priority Register A for Slave 13 */
#define AVR32_HMATRIX_PRBS13_OFFSET     0x00ec /* Priority Register B for Slave 13 */
#define AVR32_HMATRIX_PRAS14_OFFSET     0x00f0 /* Priority Register A for Slave 14 */
#define AVR32_HMATRIX_PRBS14_OFFSET     0x00f4 /* Priority Register B for Slave 14 */
#define AVR32_HMATRIX_PRAS15_OFFSET     0x00f8 /* Priority Register A for Slave 15 */
#define AVR32_HMATRIX_PRBS15_OFFSET     0x00fc /* Priority Register B for Slave 15 */

#define AVR32_HMATRIX_SFR_OFFSET(n)     (0x0110+((n)<<2))
#define AVR32_HMATRIX_SFR0_OFFSET       0x0110 /* Special Function Register 0 */
#define AVR32_HMATRIX_SFR1_OFFSET       0x0114 /* Special Function Register 1 */
#define AVR32_HMATRIX_SFR2_OFFSET       0x0118 /* Special Function Register 2 */
#define AVR32_HMATRIX_SFR3_OFFSET       0x011c /* Special Function Register 3 */
#define AVR32_HMATRIX_SFR4_OFFSET       0x0120 /* Special Function Register 4 */
#define AVR32_HMATRIX_SFR5_OFFSET       0x0124 /* Special Function Register 5 */
#define AVR32_HMATRIX_SFR6_OFFSET       0x0128 /* Special Function Register 6 */
#define AVR32_HMATRIX_SFR7_OFFSET       0x012c /* Special Function Register 7 */
#define AVR32_HMATRIX_SFR8_OFFSET       0x0130 /* Special Function Register 8 */
#define AVR32_HMATRIX_SFR9_OFFSET       0x0134 /* Special Function Register 9 */
#define AVR32_HMATRIX_SFR10_OFFSET      0x0138 /* Special Function Register 10 */
#define AVR32_HMATRIX_SFR11_OFFSET      0x013c /* Special Function Register 11 */
#define AVR32_HMATRIX_SFR12_OFFSET      0x0140 /* Special Function Register 12 */
#define AVR32_HMATRIX_SFR13_OFFSET      0x0144 /* Special Function Register 13 */
#define AVR32_HMATRIX_SFR14_OFFSET      0x0148 /* Special Function Register 14 */
#define AVR32_HMATRIX_SFR15_OFFSET      0x014c /* Special Function Register 15 */

/* Register Addresses ***************************************************************/

#define AVR32_HMATRIX_MCFG(n)           (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG_OFFSET(n))
#define AVR32_HMATRIX_MCFG0             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG0_OFFSET)
#define AVR32_HMATRIX_MCFG1             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG1_OFFSET)
#define AVR32_HMATRIX_MCFG2             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG2_OFFSET)
#define AVR32_HMATRIX_MCFG3             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG3_OFFSET)
#define AVR32_HMATRIX_MCFG4             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG4_OFFSET)
#define AVR32_HMATRIX_MCFG5             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG5_OFFSET)
#define AVR32_HMATRIX_MCFG6             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG6_OFFSET)
#define AVR32_HMATRIX_MCFG7             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG7_OFFSET)
#define AVR32_HMATRIX_MCFG8             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG8_OFFSET)
#define AVR32_HMATRIX_MCFG9             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG9_OFFSET)
#define AVR32_HMATRIX_MCFG10            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG10_OFFSET)
#define AVR32_HMATRIX_MCFG11            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG11_OFFSET)
#define AVR32_HMATRIX_MCFG12            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG12_OFFSET)
#define AVR32_HMATRIX_MCFG13            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG13_OFFSET)
#define AVR32_HMATRIX_MCFG14            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG14_OFFSET)
#define AVR32_HMATRIX_MCFG15            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_MCFG15_OFFSET)

#define AVR32_HMATRIX_SCFG(n)           (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG_OFFSET(n))
#define AVR32_HMATRIX_SCFG0             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG0_OFFSET)
#define AVR32_HMATRIX_SCFG1             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG1_OFFSET)
#define AVR32_HMATRIX_SCFG2             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG2_OFFSET)
#define AVR32_HMATRIX_SCFG3             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG3_OFFSET)
#define AVR32_HMATRIX_SCFG4             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG4_OFFSET)
#define AVR32_HMATRIX_SCFG5             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG5_OFFSET)
#define AVR32_HMATRIX_SCFG6             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG6_OFFSET)
#define AVR32_HMATRIX_SCFG7             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG7_OFFSET)
#define AVR32_HMATRIX_SCFG8             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG8_OFFSET)
#define AVR32_HMATRIX_SCFG9             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG9_OFFSET)
#define AVR32_HMATRIX_SCFG10            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG10_OFFSET)
#define AVR32_HMATRIX_SCFG11            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG11_OFFSET)
#define AVR32_HMATRIX_SCFG12            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG12_OFFSET)
#define AVR32_HMATRIX_SCFG13            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG13_OFFSET)
#define AVR32_HMATRIX_SCFG14            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG14_OFFSET)
#define AVR32_HMATRIX_SCFG15            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SCFG15_OFFSET)

#define AVR32_HMATRIX_PRAS(n)           (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS_OFFSET(n))
#define AVR32_HMATRIX_PRBS(n)           (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS_OFFSET(n))
#define AVR32_HMATRIX_PRAS0             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS0_OFFSET)
#define AVR32_HMATRIX_PRBS0             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS0_OFFSET)
#define AVR32_HMATRIX_PRAS1             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS1_OFFSET)
#define AVR32_HMATRIX_PRBS1             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS1_OFFSET)
#define AVR32_HMATRIX_PRAS2             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS2_OFFSET)
#define AVR32_HMATRIX_PRBS2             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS2_OFFSET)
#define AVR32_HMATRIX_PRAS3             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS3_OFFSET)
#define AVR32_HMATRIX_PRBS3             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS3_OFFSET)
#define AVR32_HMATRIX_PRAS4             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS4_OFFSET)
#define AVR32_HMATRIX_PRBS4             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS4_OFFSET)
#define AVR32_HMATRIX_PRAS5             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS5_OFFSET)
#define AVR32_HMATRIX_PRBS5             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS5_OFFSET)
#define AVR32_HMATRIX_PRAS6             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS6_OFFSET)
#define AVR32_HMATRIX_PRBS6             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS6_OFFSET)
#define AVR32_HMATRIX_PRAS7             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS7_OFFSET)
#define AVR32_HMATRIX_PRBS7             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS7_OFFSET)
#define AVR32_HMATRIX_PRAS8             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS8_OFFSET)
#define AVR32_HMATRIX_PRBS8             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS8_OFFSET)
#define AVR32_HMATRIX_PRAS9             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS9_OFFSET)
#define AVR32_HMATRIX_PRBS9             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS9_OFFSET)
#define AVR32_HMATRIX_PRAS10            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS10_OFFSET)
#define AVR32_HMATRIX_PRBS10            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS10_OFFSET)
#define AVR32_HMATRIX_PRAS11            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS11_OFFSET)
#define AVR32_HMATRIX_PRBS11            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS11_OFFSET)
#define AVR32_HMATRIX_PRAS12            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS12_OFFSET)
#define AVR32_HMATRIX_PRBS12            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS12_OFFSET)
#define AVR32_HMATRIX_PRAS13            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS13_OFFSET)
#define AVR32_HMATRIX_PRBS13            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS13_OFFSET)
#define AVR32_HMATRIX_PRAS14            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS14_OFFSET)
#define AVR32_HMATRIX_PRBS14            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS14_OFFSET)
#define AVR32_HMATRIX_PRAS15            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRAS15_OFFSET)
#define AVR32_HMATRIX_PRBS15            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_PRBS15_OFFSET)

#define AVR32_HMATRIX_SFR(n)            (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR_OFFSET(n))
#define AVR32_HMATRIX_SFR0              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR0_OFFSET)
#define AVR32_HMATRIX_SFR1              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR1_OFFSET)
#define AVR32_HMATRIX_SFR2              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR2_OFFSET)
#define AVR32_HMATRIX_SFR3              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR3_OFFSET)
#define AVR32_HMATRIX_SFR4              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR4_OFFSET)
#define AVR32_HMATRIX_SFR5              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR5_OFFSET)
#define AVR32_HMATRIX_SFR6              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR6_OFFSET)
#define AVR32_HMATRIX_SFR7              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR7_OFFSET)
#define AVR32_HMATRIX_SFR8              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR8_OFFSET)
#define AVR32_HMATRIX_SFR9              (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR9_OFFSET)
#define AVR32_HMATRIX_SFR10             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR10_OFFSET)
#define AVR32_HMATRIX_SFR11             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR11_OFFSET)
#define AVR32_HMATRIX_SFR12             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR12_OFFSET)
#define AVR32_HMATRIX_SFR13             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR13_OFFSET)
#define AVR32_HMATRIX_SFR14             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR14_OFFSET)
#define AVR32_HMATRIX_SFR15             (AVR32_HMATRIX_BASE+AVR32_HMATRIX_SFR15_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Master Configuration Register Bit-field Definitions */

#define HMATRIX_MCFG_UBLT_SHIFT         (0)       /* Bits 0-2: Undefined Length Burst Type */
#define HMATRIX_MCFG_UBLT_MASK          (7 << HMATRIX_MCFG_UBLT_SHIFT)
#  define HMATRIX_MCFG_UBLT_INF         (0 << HMATRIX_MCFG_UBLT_SHIFT) /* Infinite Length Burst */
#  define HMATRIX_MCFG_UBLT_SINGLE      (1 << HMATRIX_MCFG_UBLT_SHIFT) /* Single Access */
#  define HMATRIX_MCFG_UBLT_4BEAT       (2 << HMATRIX_MCFG_UBLT_SHIFT) /* Four Beat Burst */
#  define HMATRIX_MCFG_UBLT_8BEAT       (3 << HMATRIX_MCFG_UBLT_SHIFT) /* Eight Beat Burst */
#  define HMATRIX_MCFG_UBLT_16BEAT      (4 << HMATRIX_MCFG_UBLT_SHIFT) /* Sixteen Beat Burst */

/* Slave Configuration Register Bit-field Definitions */

#define HMATRIX_SCFG_SLOTCYCLE_SHIFT    (0)       /* Bits 0-7: Maximum Number of Allowed Cycles for a Burst
#define HMATRIX_SCFG_SLOTCYCLE_MASK     (0xff << HMATRIX_SCFG_SLOTCYCLE_SHIFT)
#define HMATRIX_SCFG_DEFMSTRTYPE_SHIFT  (16)      /* Bits 16-17: Default Master Type
#define HMATRIX_SCFG_DEFMSTRTYPE_MASK   (3 << HMATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define HMATRIX_SCFG_DEFMSTRTYPE_NONE    (0 << HMATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define HMATRIX_SCFG_DEFMSTRTYPE_LAST    (1 << HMATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define HMATRIX_SCFG_DEFMSTRTYPE_FIXED   (2 << HMATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#define HMATRIX_SCFG_FIXEDDEFMSTR_SHIFT (18)      /* Bits 18-21: Fixed Default Master
#define HMATRIX_SCFG_FIXEDDEFMSTR_MASK  (15 << HMATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#define HMATRIX_SCFG_ARBT               (1 << 24) /* Bit 24: Arbitration Type */

/* Priority Register A for Slave Bit-field Definitions */

#define HMATRIX_PRAS_MPR_SHIFT(n)       ((n) << 2)
#define HMATRIX_PRAS_MPR_MASK(n)        (3 << HMATRIX_PRAS_MPR_SHIFT(n))
#define HMATRIX_PRAS_M0PR_SHIFT         (0)
#define HMATRIX_PRAS_M0PR_MASK          (3 << HMATRIX_PRAS_M0PR_SHIFT)
#define HMATRIX_PRAS_M1PR_SHIFT         (4)
#define HMATRIX_PRAS_M1PR_MASK          (3 << HMATRIX_PRAS_M1PR_SHIFT)
#define HMATRIX_PRAS_M2PR_SHIFT         (8)
#define HMATRIX_PRAS_M2PR_MASK          (3 << HMATRIX_PRAS_M2PR_SHIFT)
#define HMATRIX_PRAS_M3PR_SHIFT         (12)
#define HMATRIX_PRAS_M3PR_MASK          (3 << HMATRIX_PRAS_M3PR_SHIFT)
#define HMATRIX_PRAS_M4PR_SHIFT         (16)
#define HMATRIX_PRAS_M4PR_MASK          (3 << HMATRIX_PRAS_M4PR_SHIFT)
#define HMATRIX_PRAS_M5PR_SHIFT         (20)
#define HMATRIX_PRAS_M5PR_MASK          (3 << HMATRIX_PRAS_M5PR_SHIFT)
#define HMATRIX_PRAS_M6PR_SHIFT         (24)
#define HMATRIX_PRAS_M6PR_MASK          (3 << HMATRIX_PRAS_M6PR_SHIFT)
#define HMATRIX_PRAS_M7PR_SHIFT         (28)
#define HMATRIX_PRAS_M7PR_MASK          (3 << HMATRIX_PRAS_M7PR_SHIFT)

/* Priority Register B for Slave Bit-field Definitions */

#define HMATRIX_PRBS_MPR_SHIFT(n)       (((n)-8) << 2)
#define HMATRIX_PRBS_MPR_MASK(n)        (3 << HMATRIX_PRBS_MPR_SHIFT(n))
#define HMATRIX_PRBS_M8PR_SHIFT         (0)
#define HMATRIX_PRBS_M8PR_MASK          (3 << HMATRIX_PRBS_M8PR_SHIFT)
#define HMATRIX_PRBS_M9PR_SHIFT         (4)
#define HMATRIX_PRBS_M9PR_MASK          (3 << HMATRIX_PRBS_M9PR_SHIFT)
#define HMATRIX_PRBS_M10PR_SHIFT        (8)
#define HMATRIX_PRBS_M10PR_MASK         (3 << HMATRIX_PRBS_M10PR_SHIFT)
#define HMATRIX_PRBS_M11PR_SHIFT        (12)
#define HMATRIX_PRBS_M11PR_MASK         (3 << HMATRIX_PRBS_M11PR_SHIFT)
#define HMATRIX_PRBS_M12PR_SHIFT        (16)
#define HMATRIX_PRBS_M12PR_MASK         (3 << HMATRIX_PRBS_M12PR_SHIFT)
#define HMATRIX_PRBS_M13PR_SHIFT        (20)
#define HMATRIX_PRBS_M13PR_MASK         (3 << HMATRIX_PRBS_M13PR_SHIFT)
#define HMATRIX_PRBS_M14PR_SHIFT        (24)
#define HMATRIX_PRBS_M14PR_MASK         (3 << HMATRIX_PRBS_M14PR_SHIFT)
#define HMATRIX_PRBS_M15PR_SHIFT        (28)
#define HMATRIX_PRBS_M15PR_MASK         (3 << HMATRIX_PRBS_M15PR_SHIFT)

/* Special Function Register Bit-field Definitions */
/* This register contains only the 32-bit SFR value and has no bit-fields */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_HMATRIX_H */

