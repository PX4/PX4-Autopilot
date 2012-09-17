/************************************************************************************
 * arch/arm/src/str71x/str71x_bspi.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_BSPI_H
#define __ARCH_ARM_SRC_STR71X_STR71X_BSPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STR71X_BSPI_RXR_OFFSET  (0x0000) /* 16-bits wide */
#define STR71X_BSPI_TXR_OFFSET  (0x0004) /* 16-bits wide */
#define STR71X_BSPI_CSR1_OFFSET (0x0008) /* 16-bits wide */
#define STR71X_BSPI_CSR2_OFFSET (0x000c) /* 16-bits wide */
#define STR71X_BSPI_CLK_OFFSET  (0x0010) /* 16-bits wide */

/* Registers ************************************************************************/

#define STR71X_BSPI_RXR(b)      ((b) + STR71X_BSPI_RXR_OFFSET)
#define STR71X_BSPI_TXR(b)      ((b) + STR71X_BSPI_TXR_OFFSET)
#define STR71X_BSPI_CSR1(b)     ((b) + STR71X_BSPI_CSR1_OFFSET)
#define STR71X_BSPI_CSR2(b)     ((b) + STR71X_BSPI_CSR2_OFFSET)
#define STR71X_BSPI_CLK(b)      ((b) + STR71X_BSPI_CLK_OFFSET)

#define STR71X_BSPI0_RXR        (STR71X_BSPI0_BASE + STR71X_BSPI_RXR_OFFSET)
#define STR71X_BSPI0_TXR        (STR71X_BSPI0_BASE + STR71X_BSPI_TXR_OFFSET)
#define STR71X_BSPI0_CSR1       (STR71X_BSPI0_BASE + STR71X_BSPI_CSR1_OFFSET)
#define STR71X_BSPI0_CSR2       (STR71X_BSPI0_BASE + STR71X_BSPI_CSR2_OFFSET)
#define STR71X_BSPI0_CLK        (STR71X_BSPI0_BASE + STR71X_BSPI_CLK_OFFSET)

#define STR71X_BSPI1_RXR        (STR71X_BSPI1_BASE + STR71X_BSPI_RXR_OFFSET)
#define STR71X_BSPI1_TXR        (STR71X_BSPI1_BASE + STR71X_BSPI_TXR_OFFSET)
#define STR71X_BSPI1_CSR1       (STR71X_BSPI1_BASE + STR71X_BSPI_CSR1_OFFSET)
#define STR71X_BSPI1_CSR2       (STR71X_BSPI1_BASE + STR71X_BSPI_CSR2_OFFSET)
#define STR71X_BSPI1_CLK        (STR71X_BSPI1_BASE + STR71X_BSPI_CLK_OFFSET)

/* Register bit settings ***********************************************************/

/* BSPI control/status register 1 */

#define STR71X_BSPICSR1_BSPE        (1 << 0) /* Bit 0: BSPI enable */
#define STR71X_BSPICSR1_MSTR        (1 << 1) /* Bit 1: Master/Slave select */
#define STR71X_BSPICSR1_RIESHIFT    2        /* Bit 2-3: BSPI receive interrupt enable */
#define STR71X_BSPICSR1_RIEMASK     (3 << STR71X_BSPICSR1_RIESHIFT)
#define STR71X_BSPICSR1_RIEDISABLED (0 << STR71X_BSPICSR1_RIESHIFT) /* Disabled */
#define STR71X_BSPICSR1_RIERFNE     (1 << STR71X_BSPICSR1_RIESHIFT) /* Receive FIFO not empty */
#define STR71X_BSPICSR1_RIERFF      (3 << STR71X_BSPICSR1_RIESHIFT) /* Receive FIFO full */
#define STR71X_BSPICSR1_REIE        (1 << 4) /* Bit 4: Receive error interrupt enable */
#define STR71X_BSPICSR1_BEIE        (1 << 7) /* Bit 7: Bus error interrupt enable */
#define STR71X_BSPICSR1_CPOL        (1 << 8) /* Bit 8: Clock polarity select */
#define STR71X_BSPICSR1_CPHA        (1 << 9) /* Bit 9: Clock phase select */
#define STR71X_BSPICSR1_WLSHIFT     10       /* Bits 10-11: Word length */
#define STR71X_BSPICSR1_WLMASK      (3 << STR71X_BSPICSR1_WLSHIFT)
#define STR71X_BSPICSR1_WL8BIT      (0 << STR71X_BSPICSR1_WLSHIFT) /*   8-bits */
#define STR71X_BSPICSR1_WL16BIT     (1 << STR71X_BSPICSR1_WLSHIFT) /*   16-bits */
#define STR71X_BSPICSR1_RFESHIFT    12       /* Bits 12-15: Receive FIFO enable */
#define STR71X_BSPICSR1_RFEMASK     (15 << STR71X_BSPICSR1_RFESHIFT)
#define STR71X_BSPICSR1_RFE1        (0 << STR71X_BSPICSR1_RFESHIFT) /* Word 1 enabled */
#define STR71X_BSPICSR1_RFE12       (1 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-2 enabled */
#define STR71X_BSPICSR1_RFE13       (2 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-3 enabled */
#define STR71X_BSPICSR1_RFE14       (3 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-4 enabled */
#define STR71X_BSPICSR1_RFE15       (4 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-5 enabled */
#define STR71X_BSPICSR1_RFE16       (5 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-6 enabled */
#define STR71X_BSPICSR1_RFE17       (6 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-7 enabled */
#define STR71X_BSPICSR1_RFE18       (7 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-8 enabled */
#define STR71X_BSPICSR1_RFE19       (8 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-9 enabled */
#define STR71X_BSPICSR1_RFE110      (9 << STR71X_BSPICSR1_RFESHIFT) /* Word 1-10 enabled */

/* BSPI control/status register 2 */

#define STR71X_BSPICSR2_DFIFO       (1 << 0) /* Bit 0: FIFO disable */
#define STR71X_BSPICSR2_BERR        (1 << 2) /* Bit 2: Bus error */
#define STR71X_BSPICSR2_RFNE        (1 << 3) /* Bit 3: Receiver FIFO not empty */
#define STR71X_BSPICSR2_RFF         (1 << 4) /* Bit 4: Receiver FIFO full */
#define STR71X_BSPICSR2_ROFL        (1 << 5) /* Bit 5: Receiver overflow */
#define STR71X_BSPICSR2_TFE         (1 << 6) /* Bit 6: Transmit FIFO empty */
#define STR71X_BSPICSR2_TUFL        (1 << 7) /* Bit 7: Transmit FIFO underflow */
#define STR71X_BSPICSR2_TFF         (1 << 8) /* Bit 8: Transmit FIFO full */
#define STR71X_BSPICSR2_TFNE        (1 << 9) /* Bit 9: Transmit FIFO not empty */
#define STR71X_BSPICSR2_TFESHIFT    10       /* Bits 10-13:  Transmit FIFO enable*/
#define STR71X_BSPICSR2_TFEMASK     (15 << STR71X_BSPICSR2_TFESHIFT)
#define STR71X_BSPICSR2_TFE1        (0 << STR71X_BSPICSR2_TFESHIFT) /* Word 1 enabled  */
#define STR71X_BSPICSR2_TFE12       (1 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-2 enabled  */
#define STR71X_BSPICSR2_TFE13       (2 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-3 enabled  */
#define STR71X_BSPICSR2_TFE14       (3 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-4 enabled  */
#define STR71X_BSPICSR2_TFE15       (4 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-5 enabled  */
#define STR71X_BSPICSR2_TFE16       (5 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-6 enabled  */
#define STR71X_BSPICSR2_TFE17       (6 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-7 enabled  */
#define STR71X_BSPICSR2_TFE18       (7 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-8 enabled  */
#define STR71X_BSPICSR2_TFE19       (8 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-9 enabled  */
#define STR71X_BSPICSR2_TFE110      (9 << STR71X_BSPICSR2_TFESHIFT) /* Word 1-10 enabled  */
#define STR71X_BSPICSR2_TIESHIFT    14     /* Bit 14-15:  BSPI transmit interrupt enable */
#define STR71X_BSPICSR2_TIEMASK     (3 << STR71X_BSPICSR2_TIESHIFT)
#define STR71X_BSPICSR2_TIEDISABLED (0 << STR71X_BSPICSR2_TIESHIFT) /* Disabled  */
#define STR71X_BSPICSR2_TIETFE      (1 << STR71X_BSPICSR2_TIESHIFT) /* Interrupt on transmit FIFO empty  */
#define STR71X_BSPICSR2_TIETUFL     (2 << STR71X_BSPICSR2_TIESHIFT) /* Interrupt on transmit underlow */
#define STR71X_BSPICSR2_TIETFF      (3 << STR71X_BSPICSR2_TIESHIFT) /* Interrupt on transmit FIFO full  */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_BSPI_H */
