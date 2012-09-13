/************************************************************************************
 * arch/avr/include/avr32/avr32.h
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

#ifndef __ARCH_AVR_INCLUDE_AVR32_AVR32_H
#define __ARCH_AVR_INCLUDE_AVR32_AVR32_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* AVR32 System Registers */

#define AVR32_SR               0x000 /* Status Register */
#define AVR32_EVBA             0x004 /* Exception Vector Base Address */
#define AVR32_ACBA             0x008 /* Application Call Base Address */
#define AVR32_CPUCR            0x00c /* CPU Control Register */
#define AVR32_ECR              0x010 /* Exception Cause Register */
#define AVR32_RSR_SUP          0x014 /* Return Status Register for Supervisor context */
#define AVR32_RSR_INT0         0x018 /* Return Status Register for INT 0 context */
#define AVR32_RSR_INT1         0x01c /* Return Status Register for INT 1 context */
#define AVR32_RSR_INT2         0x020 /* Return Status Register for INT 2 context */
#define AVR32_RSR_INT3         0x024 /* Return Status Register for INT 3 context */
#define AVR32_RSR_EX           0x028 /* Return Status Register for Exception context */
#define AVR32_RSR_NMI          0x02c /* Return Status Register for NMI context */
#define AVR32_RSR_DBG          0x030 /* Return Status Register for Debug Mode */
#define AVR32_RAR_SUP          0x034 /* Return Address Register for Supervisor context */
#define AVR32_RAR_INT0         0x038 /* Return Address Register for INT 0 context */
#define AVR32_RAR_INT1         0x03c /* Return Address Register for INT 1 context */
#define AVR32_RAR_INT2         0x040 /* Return Address Register for INT 2 context */
#define AVR32_RAR_INT3         0x044 /* Return Address Register for INT 3 context */
#define AVR32_RAR_EX           0x048 /* Return Address Register for Exception context */
#define AVR32_RAR_NMI          0x04c /* Return Status Register for NMI context */
#define AVR32_RAR_DBG          0x050 /* Return Status Register for Debug Mode */
#define AVR32_JECR             0x054 /* Java Exception Cause Register */
#define AVR32_JOSP             0x058 /* Java Operand Stack Pointer */
#define AVR32_JAVA_LV0         0x05c /* Java Local Variable 0 */
#define AVR32_JAVA_LV1         0x060 /* Java Local Variable 1 */
#define AVR32_JAVA_LV2         0x064 /* Java Local Variable 2 */
#define AVR32_JAVA_LV3         0x068 /* Java Local Variable 3 */
#define AVR32_JAVA_LV4         0x06c /* Java Local Variable 4 */
#define AVR32_JAVA_LV5         0x070 /* Java Local Variable 5 */
#define AVR32_JAVA_LV6         0x074 /* Java Local Variable 6 */
#define AVR32_JAVA_LV7         0x078 /* Java Local Variable 7 */
#define AVR32_JTBA             0x07c /* Java Trap Base Address */
#define AVR32_JBCR             0x080 /* Java Trap Base Address */
                                     /* 0x084-0x0fc: Reserved for future use */
#define AVR32_CONFIG0          0x100 /* Configuration Register 0 */
#define AVR32_CONFIG1          0x104 /* Configuration Register 1 */
#define AVR32_COUNT            0x108 /* Cycle Counter Register */
#define AVR32_COMPARE          0x10c /* Compare register */
#define AVR32_TLBEHI           0x110 /* MMU TLB Entry High */
#define AVR32_TLBELO           0x114 /* MMU TLB Entry Low */
#define AVR32_PTBR             0x118 /* MMU Page Table Base Register */
#define AVR32_TLBEAR           0x11c /* MMU TLB Exception Address Register */
#define AVR32_MMUCR            0x120 /* MMU Control Register */
#define AVR32_TLBARLO          0x124 /* MMU TLB Accessed Register Low */
#define AVR32_TLBARHI          0x128 /* MMU TLB Accessed Register High */
#define AVR32_PCCNT            0x12c /* Performance Clock Counter */
#define AVR32_PCNT0            0x130 /* Performance Counter 0 */
#define AVR32_PCNT1            0x134 /* Performance Counter 1 */
#define AVR32_PCCR             0x138 /* Performance Counter Control Register */
#define AVR32_BEAR             0x13c /* Bus Error Address */
#define AVR32_MPUARI0          0x140 /* MPU Address Register Instruction region 0 */
#define AVR32_MPUARI1          0x144 /* MPU Address Register Instruction region 1 */
#define AVR32_MPUARI2          0x148 /* MPU Address Register Instruction region 2 */
#define AVR32_MPUARI3          0x14c /* MPU Address Register Instruction region 3 */
#define AVR32_MPUARI4          0x150 /* MPU Address Register Instruction region 4 */
#define AVR32_MPUARI5          0x154 /* MPU Address Register Instruction region 5 */
#define AVR32_MPUARI6          0x158 /* MPU Address Register Instruction region 6 */
#define AVR32_MPUARI7          0x15c /* MPU Address Register Instruction region 7 */
#define AVR32_MPUARD0          0x160 /* MPU Address Register Data region 0 */
#define AVR32_MPUARD1          0x164 /* MPU Address Register Data region 1 */
#define AVR32_MPUARD2          0x168 /* MPU Address Register Data region 2 */
#define AVR32_MPUARD3          0x16c /* MPU Address Register Data region 3 */
#define AVR32_MPUARD4          0x170 /* MPU Address Register Data region 4 */
#define AVR32_MPUARD5          0x174 /* MPU Address Register Data region 5 */
#define AVR32_MPUARD6          0x178 /* MPU Address Register Data region 6 */
#define AVR32_MPUARD7          0x17c /* MPU Address Register Data region 7 */
#define AVR32_MPUCRI           0x180 /* MPU Cacheable Register Instruction regions */
#define AVR32_MPUCRD           0x184 /* MPU Cacheable Register Data regions */
#define AVR32_MPUBRD           0x188 /* MPU Bufferable Register Data regions */
#define AVR32_MPUAPRI          0x18c /* MPU Access Permission Register Instruction regions */
#define AVR32_MPUAPRD          0x190 /* MPU Access Permission Register Data regions */
#define AVR32_MPUCR            0x194 /* MPU Control Register */
                                     /* 0x198-0x2fc: Reserved for future use */
#define AVR32_IMPL             0x300 /* 0x300-0x3fc: Implementation defined */

/* Status register bit definitions */

#define AVR32_SR_C_SHIFT       0
#define AVR32_SR_C_MASK        (1 << AVR32_SR_C_SHIFT)

#define AVR32_SR_Z_SHIFT       1
#define AVR32_SR_Z_MASK        (1 << AVR32_SR_Z_SHIFT)

#define AVR32_SR_N_SHIFT       2
#define AVR32_SR_N_MASK        (1 << AVR32_SR_N_SHIFT)

#define AVR32_SR_V_SHIFT       3
#define AVR32_SR_V_MASK        (1 << AVR32_SR_V_SHIFT)

#define AVR32_SR_Q_SHIFT       4
#define AVR32_SR_Q_MASK        (1 << AVR32_SR_Q_SHIFT)

#define AVR32_SR_L_SHIFT       5
#define AVR32_SR_L_MASK        (1 << AVR32_SR_L_SHIFT)

#define AVR32_SR_T_SHIFT       14
#define AVR32_SR_T_MASK        (1 << AVR32_SR_T_SHIFT)

#define AVR32_SR_R_SHIFT       15
#define AVR32_SR_R_MASK        (1 << AVR32_SR_R_SHIFT)

#define AVR32_SR_GM_SHIFT      16
#define AVR32_SR_GM_MASK       (1 << AVR32_SR_GM_SHIFT)

#define AVR32_SR_I0M_SHIFT     17
#define AVR32_SR_I0M_MASK      (1 << AVR32_SR_I0M_SHIFT)

#define AVR32_SR_I1M_SHIFT     18
#define AVR32_SR_I1M_MASK      (1 << AVR32_SR_I1M_SHIFT)

#define AVR32_SR_I2M_SHIFT     19
#define AVR32_SR_I2M_MASK      (1 << AVR32_SR_I2M_SHIFT)

#define AVR32_SR_I3M_SHIFT     20
#define AVR32_SR_I3M_MASK      (1 << AVR32_SR_I3M_SHIFT)

#define AVR32_SR_EM_SHIFT      21
#define AVR32_SR_EM_MASK       (1 << AVR32_SR_EM_SHIFT)

#define AVR32_SR_M_SHIFT       22
#define AVR32_SR_M_MASK        (7 << AVR32_SR_M_SHIFT)
#  define AVR32_SR_M_APP       (0 << AVR32_SR_M_SHIFT) /* Application */
#  define AVR32_SR_M_SUPER     (1 << AVR32_SR_M_SHIFT) /* Supervisor */
#  define AVR32_SR_M_INT0      (2 << AVR32_SR_M_SHIFT) /* Interrupt level 0 */
#  define AVR32_SR_M_INT1      (3 << AVR32_SR_M_SHIFT) /* Interrupt level 1 */
#  define AVR32_SR_M_INT2      (4 << AVR32_SR_M_SHIFT) /* Interrupt level 2 */
#  define AVR32_SR_M_INT3      (5 << AVR32_SR_M_SHIFT) /* Interrupt level 3 */
#  define AVR32_SR_M_EX        (6 << AVR32_SR_M_SHIFT) /* Exception */
#  define AVR32_SR_M_NMI       (7 << AVR32_SR_M_SHIFT) /* Non Maskable Interrupt */

#define AVR32_SR_D_SHIFT       26
#define AVR32_SR_D_MASK        (1 << AVR32_SR_D_SHIFT)

#define AVR32_SR_DM_SHIFT      27
#define AVR32_SR_DM_MASK       (1 << AVR32_SR_DM_SHIFT)

#define AVR32_SR_J_SHIFT       28
#define AVR32_SR_J_MASK        (1 << AVR32_SR_J_SHIFT)

#define AVR32_SR_H_SHIFT       29
#define AVR32_SR_H_MASK        (1 << AVR32_SR_H_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_INCLUDE_AVR32_AVR32_H */

