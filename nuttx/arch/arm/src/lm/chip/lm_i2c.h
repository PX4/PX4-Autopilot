/************************************************************************************
 * arch/arm/src/lm/chip/lm_i2c.h
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM_I2C_H
#define __ARCH_ARM_SRC_LM_CHIP_LM_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* I2C Register Offsets *************************************************************/

/* I2C Master */

#define LM_I2CM_SA_OFFSET    0x000 /* I2C Master Slave Address */
#define LM_I2CM_CS_OFFSET    0x004 /* I2C Master Control/Status */
#define LM_I2CM_DR_OFFSET    0x008 /* I2C Master Data */
#define LM_I2CM_TPR_OFFSET   0x00c /* I2C Master Timer Period */
#define LM_I2CM_IMR_OFFSET   0x010 /* I2C Master Interrupt Mask */
#define LM_I2CM_RIS_OFFSET   0x014 /* I2C Master Raw Interrupt Status */
#define LM_I2CM_MIS_OFFSET   0x018 /* I2C Master Masked Interrupt Status */
#define LM_I2CM_ICR_OFFSET   0x01c /* I2C Master Interrupt Clear */
#define LM_I2CM_CR_OFFSET    0x020 /* I2C Master Configuration */

/* I2C Slave */

#define LM_I2CS_OAR_OFFSET   0x000 /* I2C Slave Own Address */
#define LM_I2CS_CSR_OFFSET   0x004 /* I2C Slave Control/Status */
#define LM_I2CS_DR_OFFSET    0x008 /* I2C Slave Data */
#define LM_I2CS_IMR_OFFSET   0x00c /* I2C Slave Interrupt Mask */
#define LM_I2CS_RIS_OFFSET   0x010 /* I2C Slave Raw Interrupt Status */
#define LM_I2CS_MIS_OFFSET   0x014 /* I2C Slave Masked Interrupt Status */
#define LM_I2CS_ICR_OFFSET   0x018 /* I2C Slave Interrupt Clear */

/* I2C Register Addresses ***********************************************************/

#if LM_NI2C > 0

/* I2C Master */

#define LM_I2CM_BASE(n)      (LM_I2CM0_BASE + (n)*0x1000)
#define LM_I2CM_SA(n)        (LM_I2CM_BASE(n) + LM_I2CM_SA_OFFSET)
#define LM_I2CM_CS(n)        (LM_I2CM_BASE(n) + LM_I2CM_CS_OFFSET)
#define LM_I2CM_DR(n)        (LM_I2CM_BASE(n) + LM_I2CM_DR_OFFSET)
#define LM_I2CM_TPR(n)       (LM_I2CM_BASE(n) + LM_I2CM_TPR_OFFSET)
#define LM_I2CM_IMR(n)       (LM_I2CM_BASE(n) + LM_I2CM_IMR_OFFSET)
#define LM_I2CM_RIS(n)       (LM_I2CM_BASE(n) + LM_I2CM_RIS_OFFSET)
#define LM_I2CM_MIS(n)       (LM_I2CM_BASE(n) + LM_I2CM_MIS_OFFSET)
#define LM_I2CM_ICR(n)       (LM_I2CM_BASE(n) + LM_I2CM_ICR_OFFSET)
#define LM_I2CM_CR(n)        (LM_I2CM_BASE(n) + LM_I2CM_CR_OFFSET)

/* I2C Slave */

#define LM_I2CS_BASE(n)      (LM_I2CS0_BASE + (n)*0x1000)
#define LM_I2CS_OAR(n)       (LM_I2CS_BASE(n) + LM_I2CS_OAR_OFFSET)
#define LM_I2CS_CSR(n)       (LM_I2CS_BASE(n) + LM_I2CS_CSR_OFFSET)
#define LM_I2CS_DR(n)        (LM_I2CS_BASE(n) + LM_I2CS_DR_OFFSET)
#define LM_I2CS_IMR(n)       (LM_I2CS_BASE(n) + LM_I2CS_IMR_OFFSET)
#define LM_I2CS_RIS(n)       (LM_I2CS_BASE(n) + LM_I2CS_RIS_OFFSET)
#define LM_I2CS_MIS(n)       (LM_I2CS_BASE(n) + LM_I2CS_MIS_OFFSET)
#define LM_I2CS_ICR(n)       (LM_I2CS_BASE(n) + LM_I2CS_ICR_OFFSET)

/* I2C0 Master */

#define LM_I2CM0_SA          (LM_I2CM0_BASE + LM_I2CM_SA_OFFSET)
#define LM_I2CM0_CS          (LM_I2CM0_BASE + LM_I2CM_CS_OFFSET)
#define LM_I2CM0_DR          (LM_I2CM0_BASE + LM_I2CM_DR_OFFSET)
#define LM_I2CM0_TPR         (LM_I2CM0_BASE + LM_I2CM_TPR_OFFSET)
#define LM_I2CM0_IMR         (LM_I2CM0_BASE + LM_I2CM_IMR_OFFSET)
#define LM_I2CM0_RIS         (LM_I2CM0_BASE + LM_I2CM_RIS_OFFSET)
#define LM_I2CM0_MIS         (LM_I2CM0_BASE + LM_I2CM_MIS_OFFSET)
#define LM_I2CM0_ICR         (LM_I2CM0_BASE + LM_I2CM_ICR_OFFSET)
#define LM_I2CM0_CR          (LM_I2CM0_BASE + LM_I2CM_CR_OFFSET)

/* I2C0 Slave */

#define LM_I2CS0_OAR         (LM_I2CS0_BASE + LM_I2CS_OAR_OFFSET)
#define LM_I2CS0_CSR         (LM_I2CS0_BASE + LM_I2CS_CSR_OFFSET)
#define LM_I2CS0_DR          (LM_I2CS0_BASE + LM_I2CS_DR_OFFSET)
#define LM_I2CS0_IMR         (LM_I2CS0_BASE + LM_I2CS_IMR_OFFSET)
#define LM_I2CS0_RIS         (LM_I2CS0_BASE + LM_I2CS_RIS_OFFSET)
#define LM_I2CS0_MIS         (LM_I2CS0_BASE + LM_I2CS_MIS_OFFSET)
#define LM_I2CS0_ICR         (LM_I2CS0_BASE + LM_I2CS_ICR_OFFSET)

#if LM_NI2C > 1

/* I2C1 Master */

#define LM_I2CM1_SA          (LM_I2CM1_BASE + LM_I2CM_SA_OFFSET)
#define LM_I2CM1_CS          (LM_I2CM1_BASE + LM_I2CM_CS_OFFSET)
#define LM_I2CM1_DR          (LM_I2CM1_BASE + LM_I2CM_DR_OFFSET)
#define LM_I2CM1_TPR         (LM_I2CM1_BASE + LM_I2CM_TPR_OFFSET)
#define LM_I2CM1_IMR         (LM_I2CM1_BASE + LM_I2CM_IMR_OFFSET)
#define LM_I2CM1_RIS         (LM_I2CM1_BASE + LM_I2CM_RIS_OFFSET)
#define LM_I2CM1_MIS         (LM_I2CM1_BASE + LM_I2CM_MIS_OFFSET)
#define LM_I2CM1_ICR         (LM_I2CM1_BASE + LM_I2CM_ICR_OFFSET)
#define LM_I2CM1_CR          (LM_I2CM1_BASE + LM_I2CM_CR_OFFSET)

/* I2C1 Slave */

#define LM_I2CS1_OAR         (LM_I2CS1_BASE + LM_I2CS_OAR_OFFSET)
#define LM_I2CS1_CSR         (LM_I2CS1_BASE + LM_I2CS_CSR_OFFSET)
#define LM_I2CS1_DR          (LM_I2CS1_BASE + LM_I2CS_DR_OFFSET)
#define LM_I2CS1_IMR         (LM_I2CS1_BASE + LM_I2CS_IMR_OFFSET)
#define LM_I2CS1_RIS         (LM_I2CS1_BASE + LM_I2CS_RIS_OFFSET)
#define LM_I2CS1_MIS         (LM_I2CS1_BASE + LM_I2CS_MIS_OFFSET)
#define LM_I2CS1_ICR         (LM_I2CS1_BASE + LM_I2CS_ICR_OFFSET)

#endif
#endif

/* I2C_Register Bit Definitions *****************************************************/

/* I2C Master Slave Address (I2CM_SA), offset 0x000 */

#define I2CM_SA_RS           (1 << 0)  /* Bit 0:  Receive/Send */
#define I2CM_SA_SA_SHIFT     1         /* Bits 7-1: I2C Slave Address */
#define I2CM_SA_SA_MASK      (0x7f << I2CM_SA_SA_SHIFT)

/* I2C Master Control/Status (I2CM_CS), offset 0x004 */

#define I2CM_CS_BUSY         (1 << 0)  /* Bit 0:  I2C Busy (read) */
#define I2CM_CS_ERROR        (1 << 1)  /* Bit 1:  Error in last bus operation (read) */
#define I2CM_CS_ADRACK       (1 << 2)  /* Bit 2:  Acknowledge Address (read) */
#define I2CM_CS_DATACK       (1 << 3)  /* Bit 3:  Acknowledge Data (read) */
#define I2CM_CS_ARBLST       (1 << 4)  /* Bit 4:  Arbitration Lost (read) */
#define I2CM_CS_IDLE         (1 << 5)  /* Bit 5:  I2C Idle (read) */
#define I2CM_CS_BUSBSY       (1 << 6)  /* Bit 6:  Bus Busy (read) */

#define I2CM_CS_RUN          (1 << 0)  /* Bit 0:  I2C Master Enable (write) */
#define I2CM_CS_START        (1 << 1)  /* Bit 1:  Generate START (write) */
#define I2CM_CS_STOP         (1 << 2)  /* Bit 2:  Generate STOP (write) */
#define I2CM_CS_ACK          (1 << 3)  /* Bit 3:  Data Acknowledge Enable (write) */

/* I2C Master Data (I2CM_DR), offset 0x008 */

#define I2CM_DR_MASK         0xff      /* Bits 7-0: Data transferred */

/* I2C Master Timer Period (I2CM_TPR), offset 0x00c */

#define I2CM_TPR_MASK        0xff      /* Bits 7-0: SCL Clock Period */

/* I2C Master Interrupt Mask (I2CM_IMR), offset 0x010 */

#define I2CM_IMR_IM          (1 << 0)  /* Bit 0:  Interrupt Mask */

/* I2C Master Raw Interrupt Status (I2CM_RIS), offset 0x014 */

#define I2CM_RIS_RIS         (1 << 0)  /* Bit 0:  Raw Interrupt Status */

/* I2C Master Masked Interrupt Status (I2CM_MIS), offset 0x018 */

#define I2CM_MIS_MIS         (1 << 0)  /* Bit 0:  Masked Interrupt Status */

/* I2C Master Masked Interrupt Status (I2CM_ICR), offset 0x01c */

#define I2CM_ICR_IC          (1 << 0)  /* Bit 0:  Masked Interrupt Status */

/* I2C Master Configuration (I2CM_CR), offset 0x020 */

#define I2CM_CR_LPBK         (1 << 0)  /* Bit 0:: I2C Loopback */
#define I2CM_CR_MFE          (1 << 4 ) /* Bit 4:  I2C Master Function Enable */
#define I2CM_CR_SFE          (1 << 5)  /* Bit 5:  I2C Slave Function Enable */

/* I2C Slave Own Address (I2CS_OAR), offset 0x000 */

#define I2CS_OAR_MASK        0xff      /* Bits 7-0: I2C Slave Own Address */

/* I2C Slave Control/Status (I2CS_CSR), offset 0x004 */

#define I2CS_CSR_RREQ        (1 << 0)  /* Bit 0:  Receive Request (read) */
#define I2CS_CSR_TREQ        (1 << 1)  /* Bit 1:  Transmit Request (read) */
#define I2CS_CSR_FBR         (1 << 2)  /* Bit 2:  First Byte Received (read) */

#define I2CS_CSR_DA          (1 << 0)  /* Bit 0:  Device Active (write) */

/* I2C Slave Data (I2CS_DR), offset 0x008 */

#define I2CS_DR_MASK         0xff      /* Bits 7-0: Data for Transfer */

/* I2C Slave Interrupt Mask (I2CS_IMR), offset 0x00c */

#define I2CM_IMR_DATAIM      (1 << 0)  /* Bit 0:  Data Interrupt Mask */

/* I2C Slave Raw Interrupt Status (I2CS_RIS), offset 0x010 */

#define I2CM_RIS_DATARIS     (1 << 0)  /* Bit 0:  Data Raw Interrupt Status */

/* I2C Slave Masked Interrupt Status (I2CS_MIS), offset 0x014 */

#define I2CM_MIS_DATAMIS     (1 << 0)  /* Bit 0:  Data Masked Interrupt Status */

/* I2C Slave Interrupt Clear (I2CS_ICR), offset 0x018 */

#define I2CM_ICR_DATAIC      (1 << 0)  /* Bit 0:  Data Interrupt Clear */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM_I2C_H */
