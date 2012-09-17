/****************************************************************************************************
 * arch/arm/src/kinetis/kinetis_i2s.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_I2S_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_I2S_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define KINETIS_I2S_TX0_OFFSET     0x000 /* I2S Transmit Data Registers 0 */
#define KINETIS_I2S_TX1_OFFSET     0x004 /* I2S Transmit Data Registers 1 */
#define KINETIS_I2S_RX0_OFFSET     0x008 /* I2S Receive Data Registers 0 */
#define KINETIS_I2S_RX1_OFFSET     0x00c /* I2S Receive Data Registers 1 */
#define KINETIS_I2S_CR_OFFSET      0x010 /* I2S Control Register */
#define KINETIS_I2S_ISR_OFFSET     0x014 /* I2S Interrupt Status Register */
#define KINETIS_I2S_IER_OFFSET     0x018 /* I2S Interrupt Enable Register */
#define KINETIS_I2S_TCR_OFFSET     0x01c /* I2S Transmit Configuration Register */
#define KINETIS_I2S_RCR_OFFSET     0x020 /* I2S Receive Configuration Register */
#define KINETIS_I2S_TCCR_OFFSET    0x024 /* I2S Transmit Clock Control Registers */
#define KINETIS_I2S_RCCR_OFFSET    0x028 /* I2S Receive Clock Control Registers */
#define KINETIS_I2S_FCSR_OFFSET    0x02c /* I2S FIFO Control/Status Register */
#define KINETIS_I2S_ACNT_OFFSET    0x038 /* I2S AC97 Control Register */
#define KINETIS_I2S_ACADD_OFFSET   0x03c /* I2S AC97 Command Address Register */
#define KINETIS_I2S_ACDAT_OFFSET   0x040 /* I2S AC97 Command Data Register */
#define KINETIS_I2S_ATAG_OFFSET    0x044 /* I2S AC97 Tag Register */
#define KINETIS_I2S_TMSK_OFFSET    0x048 /* I2S Transmit Time Slot Mask Register */
#define KINETIS_I2S_RMSK_OFFSET    0x04c /* I2S Receive Time Slot Mask Register */
#define KINETIS_I2S_ACCST_OFFSET   0x050 /* I2S AC97 Channel Status Register */
#define KINETIS_I2S_ACCEN_OFFSET   0x054 /* I2S AC97 Channel Enable Register */
#define KINETIS_I2S_ACCDIS_OFFSET  0x058 /* I2S AC97 Channel Disable Register */

/* Register Addresses *******************************************************************************/

#define KINETIS_I2S0_TX0           (KINETIS_I2S0_BASE+KINETIS_I2S_TX0_OFFSET)
#define KINETIS_I2S0_TX1           (KINETIS_I2S0_BASE+KINETIS_I2S_TX1_OFFSET)
#define KINETIS_I2S0_RX0           (KINETIS_I2S0_BASE+KINETIS_I2S_RX0_OFFSET)
#define KINETIS_I2S0_RX1           (KINETIS_I2S0_BASE+KINETIS_I2S_RX1_OFFSET)
#define KINETIS_I2S0_CR            (KINETIS_I2S0_BASE+KINETIS_I2S_CR_OFFSET)
#define KINETIS_I2S0_ISR           (KINETIS_I2S0_BASE+KINETIS_I2S_ISR_OFFSET)
#define KINETIS_I2S0_IER           (KINETIS_I2S0_BASE+KINETIS_I2S_IER_OFFSET)
#define KINETIS_I2S0_TCR           (KINETIS_I2S0_BASE+KINETIS_I2S_TCR_OFFSET)
#define KINETIS_I2S0_RCR           (KINETIS_I2S0_BASE+KINETIS_I2S_RCR_OFFSET)
#define KINETIS_I2S0_TCCR          (KINETIS_I2S0_BASE+KINETIS_I2S_TCCR_OFFSET)
#define KINETIS_I2S0_RCCR          (KINETIS_I2S0_BASE+KINETIS_I2S_RCCR_OFFSET)
#define KINETIS_I2S0_FCSR          (KINETIS_I2S0_BASE+KINETIS_I2S_FCSR_OFFSET)
#define KINETIS_I2S0_ACNT          (KINETIS_I2S0_BASE+KINETIS_I2S_ACNT_OFFSET)
#define KINETIS_I2S0_ACADD         (KINETIS_I2S0_BASE+KINETIS_I2S_ACADD_OFFSET)
#define KINETIS_I2S0_ACDAT         (KINETIS_I2S0_BASE+KINETIS_I2S_ACDAT_OFFSET)
#define KINETIS_I2S0_ATAG          (KINETIS_I2S0_BASE+KINETIS_I2S_ATAG_OFFSET)
#define KINETIS_I2S0_TMSK          (KINETIS_I2S0_BASE+KINETIS_I2S_TMSK_OFFSET)
#define KINETIS_I2S0_RMSK          (KINETIS_I2S0_BASE+KINETIS_I2S_RMSK_OFFSET)
#define KINETIS_I2S0_ACCST         (KINETIS_I2S0_BASE+KINETIS_I2S_ACCST_OFFSET)
#define KINETIS_I2S0_ACCEN         (KINETIS_I2S0_BASE+KINETIS_I2S_ACCEN_OFFSET)
#define KINETIS_I2S0_ACCDIS        (KINETIS_I2S0_BASE+KINETIS_I2S_ACCDIS_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* I2S Transmit Data Registers 0/1 and I2S Receive Data Registers 0/1: 32-bit I2S data */

/* I2S Control Register */

#define I2S_CR_I2SEN               (1 << 0)  /* Bit 0:  I2S Enable */
#define I2S_CR_TE                  (1 << 1)  /* Bit 1:  Transmit Enable */
#define I2S_CR_RE                  (1 << 2)  /* Bit 2:  Receive Enable */
#define I2S_CR_NET                 (1 << 3)  /* Bit 3:  Network Mode */
#define I2S_CR_SYN                 (1 << 4)  /* Bit 4:  Synchronous Mode */
#define I2S_CR_I2SMODE_SHIFT       (5)       /* Bits 5-6: I2S Mode Select */
#define I2S_CR_I2SMODE_MASK        (3 << I2S_CR_I2SMODE_SHIFT)
#  define I2S_CR_I2SMODE_NORMAL    (0 << I2S_CR_I2SMODE_SHIFT) /* Normal mode */
#  define I2S_CR_I2SMODE_MASTER    (1 << I2S_CR_I2SMODE_SHIFT) /* I2S master mode */
#  define I2S_CR_I2SMODE_SLAVE     (2 << I2S_CR_I2SMODE_SHIFT) /* I2S slave mode */
#define I2S_CR_SYSCLKEN            (1 << 7)  /* Bit 7:  System Clock (Oversampling Clock) Enable */
#define I2S_CR_TCHEN               (1 << 8)  /* Bit 8:  Two-Channel Operation Enable */
#define I2S_CR_CLKIST              (1 << 9)  /* Bit 9:  Clock Idle */
#define I2S_CR_TFRCLKDIS           (1 << 10) /* Bit 10: Transmit Frame Clock Disable */
#define I2S_CR_RFRCLKDIS           (1 << 11) /* Bit 11: Receive Frame Clock Disable */
#define I2S_CR_SYNCTXFS            (1 << 12) /* Bit 12: CR[TE] latched with FS occurrence */
                                             /* Bits 13-31: Reserved */
/* I2S Interrupt Status Register and I2S Interrupt Enable Register common bit definitions */

#define I2S_INT_TFE0               (1 << 0)  /* Bit 0:  Transmit FIFO Empty 0 */
#define I2S_INT_TFE1               (1 << 1)  /* Bit 1:  Transmit FIFO Empty 1 */
#define I2S_INT_RFF0               (1 << 2)  /* Bit 2:  Receive FIFO Full 0 */
#define I2S_INT_RFF1               (1 << 3)  /* Bit 3:  Receive FIFO Full 1 */
#define I2S_INT_RLS                (1 << 4)  /* Bit 4:  Receive Last Time Slot */
#define I2S_INT_TLS                (1 << 5)  /* Bit 5:  Transmit Last Time Slot */
#define I2S_INT_RFS                (1 << 6)  /* Bit 6:  Receive Frame Sync */
#define I2S_INT_TFS                (1 << 7)  /* Bit 7:  Transmit Frame Sync */
#define I2S_INT_TUE0               (1 << 8)  /* Bit 8:  Transmitter Underrun Error 1 */
#define I2S_INT_TUE1               (1 << 9)  /* Bit 9:  Transmitter Underrun Error 1 */
#define I2S_INT_ROE0               (1 << 10) /* Bit 10: Receiver Overrun Error 0 */
#define I2S_INT_ROE1               (1 << 11) /* Bit 11: Receiver Overrun Error 1 */
#define I2S_INT_TDE0               (1 << 12) /* Bit 12: Transmit Data Register Empty 0 */
#define I2S_INT_TDE1               (1 << 13) /* Bit 13: Transmit Data Register Empty 1 */
#define I2S_INT_RDR0               (1 << 14) /* Bit 14: Receive Data Ready 0 */
#define I2S_INT_RDR1               (1 << 15) /* Bit 15: Receive Data Ready 1 */
#define I2S_INT_RXT                (1 << 16) /* Bit 16: Receive Tag Updated */
#define I2S_INT_CMDDU              (1 << 17) /* Bit 17: Command Data Register Updated */
#define I2S_INT_CMDAU              (1 << 18) /* Bit 18: Command Address Register Updated */
                                             /* Bits 19-22: Reserved */
#define I2S_INT_TRFC               (1 << 23) /* Bit 23: Transmit Frame Complete */
#define I2S_INT_RFRC               (1 << 24) /* Bit 24: Receive Frame Complete */
                                             /* Bits 25-31: Reserved */
/* I2S Interrupt Status Register (see common definitions above) */
/* I2S Interrupt Enable Register (see common definitions above and unique definitions below)*/
                                             /* Bits 0-18: See common definitions above */
#define I2S_IER_TIE                (1 << 19) /* Bit 19: Transmit Interrupt Enable */
#define I2S_IER_TDMAE              (1 << 20) /* Bit 20: Transmit DMA Enable */
#define I2S_IER_RIE                (1 << 21) /* Bit 21: Receive Interrupt Enable */
#define I2S_IER_RDMAE              (1 << 22) /* Bit 22: Receive DMA Enable */
                                             /* Bits 23-24: See common definitions above */
                                             /* Bits 25-31: Reserved */
/* I2S Transmit Configuration Register */

#define I2S_TCR_TEFS               (1 << 0)  /* Bit 0:  Transmit Early Frame Sync */
#define I2S_TCR_TFSL               (1 << 1)  /* Bit 1:  Transmit Frame Sync Length */
#define I2S_TCR_TFSI               (1 << 2)  /* Bit 2:  Transmit Frame Sync Invert */
#define I2S_TCR_TSCKP              (1 << 3)  /* Bit 3:  Transmit Clock Polarity */
#define I2S_TCR_TSHFD              (1 << 4)  /* Bit 4:  Transmit Shift Direction */
#define I2S_TCR_TXDIR              (1 << 5)  /* Bit 5:  Transmit clock direction */
#define I2S_TCR_TFDIR              (1 << 6)  /* Bit 6:  Transmit Frame Direction */
#define I2S_TCR_TFEN0              (1 << 7)  /* Bit 7:  Transmit FIFO Enable 0 */
#define I2S_TCR_TFEN1              (1 << 8)  /* Bit 8:  Transmit FIFO Enable 1 */
#define I2S_TCR_TXBIT0             (1 << 9)  /* Bit 9:  Transmit Bit 0 */
                                             /* Bits 10-31: Reserved */
/* I2S Receive Configuration Register */

#define I2S_RCR_REFS               (1 << 0)  /* Bit 0:  Receive Early Frame Sync */
#define I2S_RCR_RFSL               (1 << 1)  /* Bit 1:  Receive Frame Sync Length */
#define I2S_RCR_RFSI               (1 << 2)  /* Bit 2:  Receive Frame Sync Invert */
#define I2S_RCR_RSCKP              (1 << 3)  /* Bit 3:  Receive Clock Polarity */
#define I2S_RCR_RSHFD              (1 << 4)  /* Bit 4:  Receive Shift Direction */
#define I2S_RCR_RXDIR              (1 << 5)  /* Bit 5:  Receive Clock Direction */
#define I2S_RCR_RFDIR              (1 << 6)  /* Bit 6:  Receive Frame Direction */
#define I2S_RCR_RFEN0              (1 << 7)  /* Bit 7:  Receive FIFO Enable 0 */
#define I2S_RCR_RFEN1              (1 << 8)  /* Bit 8:  Receive FIFO Enable 1 */
#define I2S_RCR_RXBIT0             (1 << 9)  /* Bit 9:  Receive Bit 0 */
#define I2S_RCR_RXEXT              (1 << 10) /* Bit 10: Receive Data Extension */
                                             /* Bits 11-31: Reserved */
/* I2S Transmit Clock Control Registers */

#define I2S_TCCR_PM_SHIFT          (0)       /* Bits 0-7: Prescaler Modulus Select */
#define I2S_TCCR_PM_MASK           (0xff << I2S_TCCR_PM_SHIFT)
#define I2S_TCCR_DC_SHIFT          (8)       /* Bits 8-12: Frame Rate Divider Control */
#define I2S_TCCR_DC_MASK           (31 << I2S_TCCR_DC_SHIFT)
#define I2S_TCCR_WL_SHIFT          (13)      /* Bits 13-16: Word Length Control */
#define I2S_TCCR_WL_MASK           (15 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_8            (3 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_10           (4 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_12           (5 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_16           (7 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_18           (8 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_20           (9 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_22           (10 << I2S_TCCR_WL_SHIFT)
#  define I2S_TCCR_WL_24           (11 << I2S_TCCR_WL_SHIFT)
#define I2S_TCCR_PSR               (1 << 17) /* Bit 17: Prescaler Range */
#define I2S_TCCR_DIV2              (1 << 18) /* Bit 18: Divide By 2 */
                                             /* Bits 19-31: Reserved */
/* I2S Receive Clock Control Registers */

#define I2S_RCCR_PM_SHIFT          (0)       /* Bits 0-7: Prescaler Modulus Select */
#define I2S_RCCR_PM_MASK           (0xff << I2S_RCCR_PM_SHIFT)
#define I2S_RCCR_DC_SHIFT          (8)       /* Bits 8-12: Frame Rate Divider Control */
#define I2S_RCCR_DC_MASK           (31 << I2S_RCCR_DC_SHIFT)
#define I2S_RCCR_WL_SHIFT          (13)      /* Bits 13-16: Word Length Control */
#define I2S_RCCR_WL_MASK           (15 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_8            (3 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_10           (4 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_12           (5 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_16           (7 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_18           (8 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_20           (9 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_22           (10 << I2S_RCCR_WL_SHIFT)
#  define I2S_RCCR_WL_24           (11 << I2S_RCCR_WL_SHIFT)
#define I2S_RCCR_PSR               (1 << 17) /* Bit 17: Prescaler Range */
#define I2S_RCCR_DIV2              (1 << 18) /* Bit 18: Divide By 2 */
                                             /* Bits 19-31: Reserved */
/* I2S FIFO Control/Status Register */

#define I2S_FCSR_TFWM0_SHIFT       (0)       /* Bits 0-3: Transmit FIFO Empty WaterMark 0 */
#define I2S_FCSR_TFWM0_MASK        (15 << I2S_FCSR_TFWM0_SHIFT)
#define I2S_FCSR_RFWM0_SHIFT       (4)       /* Bits 4-7: Receive FIFO Full WaterMark 0 */
#define I2S_FCSR_RFWM0_MASK        (15 << I2S_FCSR_RFWM0_SHIFT)
#define I2S_FCSR_TFCNT0_SHIFT      (8)       /* Bits 8-11: Transmit FIFO Counter 0 */
#define I2S_FCSR_TFCNT0_MASK       (15 << I2S_FCSR_TFCNT0_SHIFT)
#define I2S_FCSR_RFCNT0_SHIFT      (12)      /* Bits 12-15: Receive FIFO Counter 0 */
#define I2S_FCSR_RFCNT0_MASK       (15 << I2S_FCSR_RFCNT0_SHIFT)
#define I2S_FCSR_TFWM1_SHIFT       (16)      /* Bits 16-19: Transmit FIFO Empty WaterMark 1 */
#define I2S_FCSR_TFWM1_MASK        (15 << I2S_FCSR_TFWM1_SHIFT)
#define I2S_FCSR_RFWM1_SHIFT       (20)      /* Bits 20-23: Receive FIFO Full WaterMark 1 */
#define I2S_FCSR_RFWM1_MASK        (15 << I2S_FCSR_RFWM1_SHIFT)
#define I2S_FCSR_TFCNT1_SHIFT      (24)      /* Bits 24-27: Transmit FIFO Counter 1 */
#define I2S_FCSR_TFCNT1_MASK       (15 << I2S_FCSR_TFCNT1_SHIFT)
#define I2S_FCSR_RFCNT1_SHIFT      (28)      /* Bits 28-31: Receive FIFO Counter 1 */
#define I2S_FCSR_RFCNT1_MASK       (15 << I2S_FCSR_RFCNT1_SHIFT)

/* I2S AC97 Control Register */

#define I2S_ACNT_AC97EN            (1 << 0)  /* Bit 0:  AC97 Mode Enable */
#define I2S_ACNT_FV                (1 << 1)  /* Bit 1:  Fixed/Variable Operation */
#define I2S_ACNT_TIF               (1 << 2)  /* Bit 2:  Tag in FIFO */
#define I2S_ACNT_RD                (1 << 3)  /* Bit 3:  Read Command */
#define I2S_ACNT_WR                (1 << 4)  /* Bit 4:  Write Command */
#define I2S_ACNT_FRDIV_SHIFT       (5)       /* Bits 5-10: Frame Rate Divider */
#define I2S_ACNT_FRDIV_MASK        (63 << I2S_ACNT_FRDIV_SHIFT)
                                             /* Bits 11-31: Reserved */
/* I2S AC97 Command Address Register */

#define I2S_ACADD_ACADD_SHIFT      (0)       /* Bits 0-18: AC97 Command Address */
#define I2S_ACADD_ACADD_MASK       (0x7ffff << I2S_ACADD_ACADD_SHIFT)
                                             /* Bits 19-31: Reserved */
/* I2S AC97 Command Data Register */

#define I2S_ACDAT_ACADD_SHIFT      (0)       /* Bits 0-18: AC97 Command Data */
#define I2S_ACDAT_ACADD_MASK       (0x7ffff << I2S_ACDAT_ACADD_SHIFT)
                                             /* Bits 19-31: Reserved */
/* I2S AC97 Tag Register */

#define I2S_ATAG_ACADD_SHIFT       (0)       /* Bits 0-15: AC97 Tag Value */
#define I2S_ATAG_ACADD_MASK        (0xffff << I2S_ACDAT_ACADD_SHIFT)
                                             /* Bits 16-31: Reserved */
/* I2S Transmit Time Slot Mask Register (32-bit Transmit Mask) */
/* I2S Receive Time Slot Mask Register (32-bit Receive Mask) */

/* I2S AC97 Channel Status Register */

#define I2S_ACCST_ACCST_SHIFT      (0)       /* Bits 0-9: AC97 Channel Status */
#define I2S_ACCST_ACCST_MASK       (0x3ff << I2S_ACCST_ACCST_SHIFT)
                                             /* Bits 10-31: Reserved */
/* I2S AC97 Channel Enable Register */

#define I2S_ACCEN_ACCST_SHIFT      (0)       /* Bits 0-9: AC97 Channel Enable */
#define I2S_ACCEN_ACCST_MASK       (0x3ff << I2S_ACCEN_ACCST_SHIFT)
                                             /* Bits 10-31: Reserved */
/* I2S AC97 Channel Disable Register */
#define I2S__

#define I2S_ACCDIS_ACCST_SHIFT     (0)       /* Bits 0-9: AC97 AC97 Channel Disable */
#define I2S_ACCDIS_ACCST_MASK      (0x3ff << I2S_ACCEN_ACCST_SHIFT)
                                             /* Bits 10-31: Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_I2S_H */
