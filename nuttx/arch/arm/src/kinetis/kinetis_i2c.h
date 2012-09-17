/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_i2c.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_I2CE_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_I2CE_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_I2C_A1_OFFSET    0x0000 /* I2C Address Register 1 */
#define KINETIS_I2C_F_OFFSET     0x0001 /* I2C Frequency Divider register */
#define KINETIS_I2C_C1_OFFSET    0x0002 /* I2C Control Register 1 */
#define KINETIS_I2C_S_OFFSET     0x0003 /* I2C Status Register */
#define KINETIS_I2C_D_OFFSET     0x0004 /* I2C Data I/O register */
#define KINETIS_I2C_C2_OFFSET    0x0005 /* I2C Control Register 2 */
#define KINETIS_I2C_FLT_OFFSET   0x0006 /* I2C Programmable Input Glitch Filter register */
#define KINETIS_I2C_RA_OFFSET    0x0007 /* I2C Range Address register */
#define KINETIS_I2C_SMB_OFFSET   0x0008 /* I2C SMBus Control and Status register */
#define KINETIS_I2C_A2_OFFSET    0x0009 /* I2C Address Register 2 */
#define KINETIS_I2C_SLTH_OFFSET  0x000a /* I2C SCL Low Timeout Register High */
#define KINETIS_I2C_SLTL_OFFSET  0x000b /* I2C SCL Low Timeout Register Low */

/* Register Addresses ***********************************************************************/

#define KINETIS_I2C0_A1          (KINETIS_I2C0_BASE+KINETIS_I2C_A1_OFFSET)
#define KINETIS_I2C0_F           (KINETIS_I2C0_BASE+KINETIS_I2C_F_OFFSET)
#define KINETIS_I2C0_C1          (KINETIS_I2C0_BASE+KINETIS_I2C_C1_OFFSET)
#define KINETIS_I2C0_S           (KINETIS_I2C0_BASE+KINETIS_I2C_S_OFFSET)
#define KINETIS_I2C0_D           (KINETIS_I2C0_BASE+KINETIS_I2C_D_OFFSET)
#define KINETIS_I2C0_C2          (KINETIS_I2C0_BASE+KINETIS_I2C_C2_OFFSET)
#define KINETIS_I2C0_FLT         (KINETIS_I2C0_BASE+KINETIS_I2C_FLT_OFFSET)
#define KINETIS_I2C0_RA          (KINETIS_I2C0_BASE+KINETIS_I2C_RA_OFFSET)
#define KINETIS_I2C0_SMB         (KINETIS_I2C0_BASE+KINETIS_I2C_SMB_OFFSET)
#define KINETIS_I2C0_A2          (KINETIS_I2C0_BASE+KINETIS_I2C_A2_OFFSET)
#define KINETIS_I2C0_SLTH        (KINETIS_I2C0_BASE+KINETIS_I2C_SLTH_OFFSET)
#define KINETIS_I2C0_SLTL        (KINETIS_I2C0_BASE+KINETIS_I2C_SLTL_OFFSET)

#define KINETIS_I2C1_A1          (KINETIS_I2C1_BASE+KINETIS_I2C_A1_OFFSET)
#define KINETIS_I2C1_F           (KINETIS_I2C1_BASE+KINETIS_I2C_F_OFFSET)
#define KINETIS_I2C1_C1          (KINETIS_I2C1_BASE+KINETIS_I2C_C1_OFFSET)
#define KINETIS_I2C1_S           (KINETIS_I2C1_BASE+KINETIS_I2C_S_OFFSET)
#define KINETIS_I2C1_D           (KINETIS_I2C1_BASE+KINETIS_I2C_D_OFFSET)
#define KINETIS_I2C1_C2          (KINETIS_I2C1_BASE+KINETIS_I2C_C2_OFFSET)
#define KINETIS_I2C1_FLT         (KINETIS_I2C1_BASE+KINETIS_I2C_FLT_OFFSET)
#define KINETIS_I2C1_RA          (KINETIS_I2C1_BASE+KINETIS_I2C_RA_OFFSET)
#define KINETIS_I2C1_SMB         (KINETIS_I2C1_BASE+KINETIS_I2C_SMB_OFFSET)
#define KINETIS_I2C1_A2          (KINETIS_I2C1_BASE+KINETIS_I2C_A2_OFFSET)
#define KINETIS_I2C1_SLTH        (KINETIS_I2C1_BASE+KINETIS_I2C_SLTH_OFFSET)
#define KINETIS_I2C1_SLTL        (KINETIS_I2C1_BASE+KINETIS_I2C_SLTL_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* I2C Address Register 1 (8-bit) */
                                           /* Bit 0: Reserved */
#define I2C_A1_SHIFT             (1)       /* Bits 1-7: Address */
#define I2C_A1_MASK              (0x7f << I2C_A1_SHIFT)

/* I2C Frequency Divider register (8-bit) */

#define I2C_F_ICR_SHIFT          (0)       /* Bits 0-5: Clock rate */
#define I2C_F_ICR_MASK           (0x3f << I2C_F_ICR_SHIFT)
#define I2C_F_MULT_SHIFT         (6)       /* Bits 6-7: Multiplier factor */
#define I2C_F_MULT_MASK          (3 << I2C_F_MULT_SHIFT)
#  define I2C_F_MULT_1           (0 << I2C_F_MULT_SHIFT)
#  define I2C_F_MULT_2           (1 << I2C_F_MULT_SHIFT)
#  define I2C_F_MULT_4           (2 << I2C_F_MULT_SHIFT)

/* I2C Control Register 1 (8-bit) */

#define I2C_C1_DMAEN             (1 << 0)  /* Bit 0:  DMA enable */
#define I2C_C1_WUEN              (1 << 1)  /* Bit 1:  Wakeup enable */
#define I2C_C1_RSTA              (1 << 2)  /* Bit 2:  Repeat START */
#define I2C_C1_TXAK              (1 << 3)  /* Bit 3:  Transmit acknowledge enable */
#define I2C_C1_TX                (1 << 4)  /* Bit 4:  Transmit mode select */
#define I2C_C1_MST               (1 << 5)  /* Bit 5:  Master mode select */
#define I2C_C1_IICIE             (1 << 6)  /* Bit 6:  I2C interrupt enable */
#define I2C_C1_IICEN             (1 << 7)  /* Bit 7:  I2C enable */

/* I2C Status Register (8-bit) */

#define I2C_S_RXAK               (1 << 0)  /* Bit 0:  Receive acknowledge */
#define I2C_S_IICIF              (1 << 1)  /* Bit 1:  Interrupt flag */
#define I2C_S_SRW                (1 << 2)  /* Bit 2:  Slave read/write */
#define I2C_S_RAM                (1 << 3)  /* Bit 3:  Range address match */
#define I2C_S_ARBL               (1 << 4)  /* Bit 4:  Arbitration lost */
#define I2C_S_BUSY               (1 << 5)  /* Bit 5:  Bus busy */
#define I2C_S_IAAS               (1 << 6)  /* Bit 6:  Addressed as a slave */
#define I2C_S_TCF                (1 << 7)  /* Bit 7:  Transfer complete flag */

/* I2C Data I/O register (8-bit data register) */

/* I2C Control Register 2 (8-bit) */

#define I2C_C2_AD_SHIFT          (0)       /* Bits 0-2: Slave address */
#define I2C_C2_AD_MASK           (7 << I2C_C2_AD_SHIFT)
#define I2C_C2_RMEN              (1 << 3)  /* Bit 3:  Range address matching enable */
#define I2C_C2_SBRC              (1 << 4)  /* Bit 4:  Slave baud rate control */
#define I2C_C2_HDRS              (1 << 5)  /* Bit 5:  High drive select */
#define I2C_C2_ADEXT             (1 << 6)  /* Bit 6:  Address extension */
#define I2C_C2_GCAEN             (1 << 7)  /* Bit 7:  General call address enable */

/* I2C Programmable Input Glitch Filter register (8-bit) */
                                           /* Bits 5-7: Reserved */
#define I2C_FLT_SHIFT            (0)       /* Bits 0-4: I2C programmable filter factor */
#define I2C_FLT_MASK             (31 << I2C_FLT_SHIFT)

/* I2C Range Address register (8-bit) */
                                           /* Bit 0: Reserved */
#define I2C_RA_SHIFT             (1)       /* Bits 1-7: Range slave address */
#define I2C_RA_MASK              (0x7f << I2C_RA_SHIFT)

/* I2C SMBus Control and Status register (8-bit) */

#define I2C_SMB_SHTF2IE          (1 << 0)  /* Bit 0:  SHTF2 interrupt enable */
#define I2C_SMB_SHTF2            (1 << 1)  /* Bit 1:  SCL high timeout flag 2 */
#define I2C_SMB_SHTF1            (1 << 2)  /* Bit 2:  SCL high timeout flag 1 */
#define I2C_SMB_SLTF             (1 << 3)  /* Bit 3:  SCL low timeout flag */
#define I2C_SMB_TCKSEL           (1 << 4)  /* Bit 4:  Timeout counter clock select */
#define I2C_SMB_SIICAEN          (1 << 5)  /* Bit 5:  Second I2C address enable */
#define I2C_SMB_ALERTEN          (1 << 6)  /* Bit 6:  SMBus alert response address enable */
#define I2C_SMB_FACK             (1 << 7)  /* Bit 7:  Fast NACK/ACK enable */

/* I2C Address Register 2 (8-bit) */
                                           /* Bit 0: Reserved */
#define I2C_A2_SHIFT             (1)       /* Bits 1-7: SMBus address */
#define I2C_A2_MASK              (0x7f << I2C_A2_SHIFT)

/* I2C SCL Low Timeout Register High/Low (16-bit data in two 8-bit registers) */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_I2CE_H */
