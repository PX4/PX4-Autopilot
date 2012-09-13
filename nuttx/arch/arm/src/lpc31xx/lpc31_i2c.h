/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_i2c.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_I2C_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_I2C_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* I2C register base address offset into the APB1 domain ****************************************/

#define LPC31_I2C0_VBASE               (LPC31_APB1_VADDR+LPC31_APB1_I2C0_OFFSET)
#define LPC31_I2C0_PBASE               (LPC31_APB1_PADDR+LPC31_APB1_I2C0_OFFSET)

#define LPC31_I2C1_VBASE               (LPC31_APB1_VADDR+LPC31_APB1_I2C1_OFFSET)
#define LPC31_I2C1_PBASE               (LPC31_APB1_PADDR+LPC31_APB1_I2C1_OFFSET)

/* I2C register offsets (with respect to the I2C base) ******************************************/

#define LPC31_I2C_RX_OFFSET            0x00 /* I2C RX Data FIFO */
#define LPC31_I2C_TX_OFFSET            0x00 /* I2C TX Data FIFO */
#define LPC31_I2C_STAT_OFFSET          0x04 /* I2C Status Register */
#define LPC31_I2C_CTRL_OFFSET          0x08 /* I2C Control Register */
#define LPC31_I2C_CLKHI_OFFSET         0x0c /* I2C Clock Divider high */
#define LPC31_I2C_CLKLO_OFFSET         0x10 /* I2C Clock Divider low */
#define LPC31_I2C_ADR_OFFSET           0x14 /* I2C Slave Address */
#define LPC31_I2C_RXFL_OFFSET          0x18 /* I2C Rx FIFO level */
#define LPC31_I2C_TXFL_OFFSET          0x1c /* I2C Tx FIFO level */
#define LPC31_I2C_RXB_OFFSET           0x20 /* I2C Number of bytes received */
#define LPC31_I2C_TXB_OFFSET           0x24 /* I2C Number of bytes transmitted */
#define LPC31_I2C_STX_OFFSET           0x28 /* Slave Transmit FIFO */
#define LPC31_I2C_STXFL_OFFSET         0x2c /* Slave Transmit FIFO level */

/* I2C register (virtual) addresses *************************************************************/

#define LPC31_I2C0_RX                  (LPC31_I2C0_VBASE+LPC31_I2C_RX_OFFSET)
#define LPC31_I2C0_TX                  (LPC31_I2C0_VBASE+LPC31_I2C_TX_OFFSET)
#define LPC31_I2C0_STAT                (LPC31_I2C0_VBASE+LPC31_I2C_STAT_OFFSET)
#define LPC31_I2C0_CTRL                (LPC31_I2C0_VBASE+LPC31_I2C_CTRL_OFFSET)
#define LPC31_I2C0_CLKHI               (LPC31_I2C0_VBASE+LPC31_I2C_CLKHI_OFFSET)
#define LPC31_I2C0_CLKLO               (LPC31_I2C0_VBASE+LPC31_I2C_CLKLO_OFFSET)
#define LPC31_I2C0_ADR                 (LPC31_I2C0_VBASE+LPC31_I2C_ADR_OFFSET)
#define LPC31_I2C0_RXFL                (LPC31_I2C0_VBASE+LPC31_I2C_RXFL_OFFSET)
#define LPC31_I2C0_TXFL                (LPC31_I2C0_VBASE+LPC31_I2C_TXFL_OFFSET)
#define LPC31_I2C0_RXB                 (LPC31_I2C0_VBASE+LPC31_I2C_RXB_OFFSET)
#define LPC31_I2C0_TXB                 (LPC31_I2C0_VBASE+LPC31_I2C_TXB_OFFSET)
#define LPC31_I2C0_STX                 (LPC31_I2C0_VBASE+LPC31_I2C_STX_OFFSET)
#define LPC31_I2C0_STXFL               (LPC31_I2C0_VBASE+LPC31_I2C_STXFL_OFFSET)

#define LPC31_I2C1_RX                  (LPC31_I2C1_VBASE+LPC31_I2C_RX_OFFSET)
#define LPC31_I2C1_TX                  (LPC31_I2C1_VBASE+LPC31_I2C_TX_OFFSET)
#define LPC31_I2C1_STAT                (LPC31_I2C1_VBASE+LPC31_I2C_STAT_OFFSET)
#define LPC31_I2C1_CTRL                (LPC31_I2C1_VBASE+LPC31_I2C_CTRL_OFFSET)
#define LPC31_I2C1_CLKHI               (LPC31_I2C1_VBASE+LPC31_I2C_CLKHI_OFFSET)
#define LPC31_I2C1_CLKLO               (LPC31_I2C1_VBASE+LPC31_I2C_CLKLO_OFFSET)
#define LPC31_I2C1_ADR                 (LPC31_I2C1_VBASE+LPC31_I2C_ADR_OFFSET)
#define LPC31_I2C1_RXFL                (LPC31_I2C1_VBASE+LPC31_I2C_RXFL_OFFSET)
#define LPC31_I2C1_TXFL                (LPC31_I2C1_VBASE+LPC31_I2C_TXFL_OFFSET)
#define LPC31_I2C1_RXB                 (LPC31_I2C1_VBASE+LPC31_I2C_RXB_OFFSET)
#define LPC31_I2C1_TXB                 (LPC31_I2C1_VBASE+LPC31_I2C_TXB_OFFSET)
#define LPC31_I2C1_STX                 (LPC31_I2C1_VBASE+LPC31_I2C_STX_OFFSET)
#define LPC31_I2C1_STXFL               (LPC31_I2C1_VBASE+LPC31_I2C_STXFL_OFFSET)

/* I2C register bit definitions *****************************************************************/

/* I2Cn RX Data FIFO I2C0_RX, address 0x1300a000, I2C1_RX, address 0x1300a400 */

#define I2C_RX_DATA_SHIFT                (0)       /* Bits 0-7: RxData Receive FIFO data bits */
#define I2C_RX_DATA_MASK                 (0xff << I2C_RX_DATA_HIFT)

/* I2Cn TX Data FIFO I2C0_TX, 0x1300a000, I2C1_TX, address 0x1300a400 */

#define I2C_TX_STOP                      (1 << 9)  /* Bit 9:  Issue STOP condition after transmitting byte */
#define I2C_TX_START                     (1 << 8)  /* Bit 8:  Issue START condition before transmitting byte */
#define I2C_TX_DATA_SHIFT                (0)       /* Bits 0-7: TxData Transmit FIFO data bits */
#define I2C_TX_DATA_MASK                 (0xff << I2C_TX_DATA_SHIFT)

/* I2Cn Status register I2C0_STAT, address 0x1300a004, I2C1_STAT, address 0x1300a404 */

#define I2C_STAT_TFES                    (1 << 13) /* Bit 13: Slave Transmit FIFO Empty */
#define I2C_STAT_TFFS                    (1 << 12) /* Bit 12: Slave Transmit FIFO Full */
#define I2C_STAT_TFE                     (1 << 11) /* Bit 11: Transmit FIFO Empty */
#define I2C_STAT_TFF                     (1 << 10) /* Bit 10: Transmit FIFO Full */
#define I2C_STAT_RFE                     (1 << 9)  /* Bit 9:  Receive FIFO Empty */
#define I2C_STAT_RFF                     (1 << 8)  /* Bit 8:  Receive FIFO Full */
#define I2C_STAT_SDA                     (1 << 7)  /* Bit 7:  The current value of the SDA signal */
#define I2C_STAT_SCL                     (1 << 6)  /* Bit 6:  The current value of the SCL signal */
#define I2C_STAT_ACTIVE                  (1 << 5)  /* Bit 5:  Indicates whether the bus is busy */
#define I2C_STAT_DRSI                    (1 << 4)  /* Bit 4:  Slave Data Request Interrupt */
#define I2C_STAT_DRMI                    (1 << 3)  /* Bit 3:  Master Data Request Interrupt */
#define I2C_STAT_NAI                     (1 << 2)  /* Bit 2:  No Acknowledge Interrupt */
#define I2C_STAT_AFI                     (1 << 1)  /* Bit 1:  Arbitration Failure Interrupt */
#define I2C_STAT_TDI                     (1 << 0)  /* Bit 0:  Transaction Done Interrupt */

/* I2Cn Control register I2C0_CTRL, address 0x1300a008, I2C1_CTRL, address 0x1300a408 */

#define I2C_CTRL_TFFSIE                  (1 << 10) /* Bit 10: Slave Transmit FIFO Not Full Interrupt Enable */
#define I2C_CTRL_SEVEN                   (1 << 9)  /* Bit 9:  Seven-bit slave address */
#define I2C_CTRL_RESET                   (1 << 8)  /* Bit 8:  Soft Reset */
#define I2C_CTRL_TFFIE                   (1 << 7)  /* Bit 7:  Transmit FIFO Not Full Interrupt Enable */
#define I2C_CTRL_RFDAIE                  (1 << 6)  /* Bit 6:  Receive FIFO Data Available Interrupt Enable */
#define I2C_CTRL_RFFIE                   (1 << 5)  /* Bit 5:  Receive FIFO Full Interrupt Enable */
#define I2C_CTRL_DRSIE                   (1 << 4)  /* Bit 4:  Data Request Slave Transmitter Interrupt Enable */
#define I2C_CTRL_DRMIE                   (1 << 3)  /* Bit 3:  Data Request Master Transmitter Interrupt Enable */
#define I2C_CTRL_NAIE                    (1 << 2)  /* Bit 2:  Transmitter No Acknowledge Interrupt Enable */
#define I2C_CTRL_AFIE                    (1 << 1)  /* Bit 1:  Transmitter Arbitration Failure Interrupt Enable */
#define I2C_CTRL_TDIE                    (1 << 0)  /* Bit 0:  Transmit Done Interrupt Enable */

/* I2Cn Clock Divider High I2C0_CLKHI, address 0x1300a00c, I2C1_CLKHI, address 0x1300a40c */

#define I2C_CLKHI_SHIFT                  (0)       /* Bits 0-9: Clock Divisor High */
#define I2C_CLKHI_MASK                   (0x3ff << I2C_CLKHI_SHIFT)

/* I2Cn Clock Divider Low I2C0_CLKLO, address 0x1300a010, I2C1_CLKLO, address 0x1300a410 */

#define I2C_CLKLO_SHIFT                  (0)       /* Bits 0-9: Clock Divisor Low */
#define I2C_CLKLO_MASK                   (0x3ff << I2C_CLKLO_SHIFT)


/* I2Cn Slave Addres I2C0_ADDR, address 0x1300a014, I2C1_ADDR, address 0x1300a414 */

#define I2C_ADR_SHIFT                    (0)       /* Bits 0-9: I2C bus slave address */
#define I2C_ADR_MASK                     (0x3ff << I2C_ADR_SHIFT)

/* I2Cn RX FIFO level I2C0_RXFL, address 0x1300a018, I2C1_RXFL, address 0x1300a018 */

#define I2C_RXFL_SHIFT                   (0)       /* Bits 0-1: Receive FIFO level */
#define I2C_RXFL_MASK                    (3 << I2C_RXFL_SHIFT)

/* I2Cn TX FIFO level I2C0_TXFL, address 0x1300a01c, I2C1_TXFL, address 0x1300a01c */

#define I2C_TXFL_SHIFT                   (0)       /* Bits 0-1: Receive FIFO level */
#define I2C_TXFL_MASK                    (3 << I2C_TXFL_SHIFT)

/* I2Cn RX byte count I2C0_RXB, address 0x1300a020, I2C1_RXB, address 0x1300a420 */

#define I2C_RXB_SHIFT                    (0)       /* Bits 0-15: Number of bytes received */
#define I2C_RXB_MASK                     (0xffff << I2C_RXB_SHIFT)

/* I2Cn TX byte count I2C0_TXB, address 0x1300a024, I2C1_TXB, address 0x1300a424 */

#define I2C_TXB_SHIFT                    (0)       /* Bits 0-15: Number of bytes sent */
#define I2C_TXB_MASK                     (0xffff << I2C_TXB_SHIFT)

/* I2Cn Slave TX Data FIFO I2C0_STX, address 0x1300a028, I2C1_STX, 0x1300a428 */

#define I2C_STX_SHIFT                    (0)       /* Bits 0-7: Slave Transmit FIFO data */
#define I2C_STX_MASK                     (0xff << I2C_STX_SHIFT)

/* I2Cn Slave TX FIFO level I2C0_STXFL, address 0x1300a02c, I2C1_STXFL, address 0x1300a42c */

#define I2C_STXFL_SHIFT                  (0)       /* Bits 0-1: Slave Transmit FIFO level */
#define I2C_STXFL_MASK                   (3 << I2C_STXFL_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_I2C_H */
