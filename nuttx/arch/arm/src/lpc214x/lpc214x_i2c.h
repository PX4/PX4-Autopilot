/************************************************************************************
 * arch/arm/src/lpc214x/lpc214x_i2c.h
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

#ifndef _ARCH_ARM_SRC_LPC214X_I2C_H
#define _ARCH_ARM_SRC_LPC214X_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register address Offsets *********************************************************/

/* Defined in chip.h */

/* Register Address Definitions *****************************************************/

#define LPC214X_I2C0_CONSET      (LPC214X_I2C0_BASE + LPC214X_I2C_ONSET_OFFSET)
#define LPC214X_I2C0_STAT        (LPC214X_I2C0_BASE + LPC214X_I2C_STAT_OFFSET)
#define LPC214X_I2C0_DAT         (LPC214X_I2C0_BASE + LPC214X_I2C_DAT_OFFSET)
#define LPC214X_I2C0_ADR         (LPC214X_I2C0_BASE + LPC214X_I2C_ADR_OFFSET)
#define LPC214X_I2C0_SCLH        (LPC214X_I2C0_BASE + LPC214X_I2C_SCLH_OFFSET)
#define LPC214X_I2C0_SCLL        (LPC214X_I2C0_BASE + LPC214X_I2C_SCLL_OFFSET)
#define LPC214X_I2C0_CONCLR      (LPC214X_I2C0_BASE + LPC214X_I2C_ONCLR_OFFSET)

#define LPC214X_I2C1_CONSET      (LPC214X_I2C1_BASE + LPC214X_I2C_ONSET_OFFSET)
#define LPC214X_I2C1_STAT        (LPC214X_I2C1_BASE + LPC214X_I2C_STAT_OFFSET)
#define LPC214X_I2C1_DAT         (LPC214X_I2C1_BASE + LPC214X_I2C_DAT_OFFSET)
#define LPC214X_I2C1_ADR         (LPC214X_I2C1_BASE + LPC214X_I2C_ADR_OFFSET)
#define LPC214X_I2C1_SCLH        (LPC214X_I2C1_BASE + LPC214X_I2C_SCLH_OFFSET)
#define LPC214X_I2C1_SCLL        (LPC214X_I2C1_BASE + LPC214X_I2C_SCLL_OFFSET)
#define LPC214X_I2C1_CONCLR      (LPC214X_I2C1_BASE + LPC214X_I2C_ONCLR_OFFSET)

/* I2C register bit definitions *****************************************************/

/* Control Set Register (CONSET) */

#define I2C_CONSET_AA            (1 << 2)  /* Bit 2: Assert acknowledge flag */
#define I2C_CONSET_SI            (1 << 3)  /* Bit 3: I2C interrrupt flag */
#define I2C_CONSET_STO           (1 << 4)  /* Bit 4: STOP flag */
#define I2C_CONSET_STA           (1 << 5)  /* Bit 5: START flag */
#define I2C_CONSET_I2EN          (1 << 6)  /* Bit 6: I2C interface enable */

/* Control Clear Register (CONCLR) */

#define I2C_CONCLR_AAC           (1 << 2)  /* Bit 2: Assert acknowledge Clear bit */
#define I2C_CONCLR_SIC           (1 << 3)  /* Bit 3: I2C interrupt Clear bit */
#define I2C_CONCLR_STAC          (1 << 5)  /* Bit 5: START flag Clear bit */
#define I2C_CONCLR_I2ENC         (1 << 6)  /* Bit 6: I2C interface Disable bit */

/* Status Register (STAT) */

#define I2C_STAT_SHIFT           (1 << 3)  /* Bits 3-7: Status bits */
#define I2C_STAT_MASK            (0xff << I2C_STAT_SHIFT)

/* Master transmit mode */

#  define I2C_STAT_MXSTART       (0 << I2C_STAT_SHIFT)  /* Start transmitted */
#  define I2C_STAT_MXRSTART      (2 << I2C_STAT_SHIFT)  /* Repeated start transmitted */
#  define I2C_STAT_MXSLAWACK     (3 << I2C_STAT_SHIFT)  /* SLA+W tranmitted + ACK received */
#  define I2C_STAT_MXSLAWNAK     (4 << I2C_STAT_SHIFT)  /* SLA+W tranmitted + NAK received */
#  define I2C_STAT_MXDATAACK     (5 << I2C_STAT_SHIFT)  /* Data tranmitted + ACK received */
#  define I2C_STAT_MXDATANAK     (6 << I2C_STAT_SHIFT)  /* Data tranmitted + NAK received */
#  define I2C_STAT_MXARBLOST     (7 << I2C_STAT_SHIFT)  /* Abritration lost in SLA+W or data */

/* Master receive mode */

#  define I2C_STAT_MRSTART       (0 << I2C_STAT_SHIFT)  /* Start transmitted */
#  define I2C_STAT_MRRSTART      (2 << I2C_STAT_SHIFT)  /* Repeated start transmitted */
#  define I2C_STAT_MRARBLOST     (7 << I2C_STAT_SHIFT)  /* Abritration lost in NAK bit */
#  define I2C_STAT_MRSLARACK     (8 << I2C_STAT_SHIFT)  /* SLA+R tranmitted + ACK received */
#  define I2C_STAT_MRSLARNAK     (9 << I2C_STAT_SHIFT)  /* SLA+R tranmitted + NAK received */
#  define I2C_STAT_MRDATAACK     (10 << I2C_STAT_SHIFT) /* Data received + send ACK */
#  define I2C_STAT_MRDATANAK     (11 << I2C_STAT_SHIFT) /* Data received + send NAK */

/* Slave receive mode -- to be provided */

/* Slave receive mode -- to be provided */

/* Data Register (DAT) -- 8-bits of data */

/* Slave Address Register (ADR) */

#define I2C_ADR_GCA              (1 << 0)  /* Bit 0: General call enable */
#define I2C_ADR_SHIFT            1         /* Bits 7-1: address */
#define I2C_ADR_MASK             (0x7f << I2C_ADR_SHIFT)

/* SCL Duty Cycle Register (high half word - SCLH) - 16-bits of data */

/* SCL Duty Cycle Register (low half word - SCLL) - 16-bits of data */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif  /* _ARCH_ARM_SRC_LPC214X_I2C_H */
