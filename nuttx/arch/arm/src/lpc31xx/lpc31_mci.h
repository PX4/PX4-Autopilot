/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_mci.h
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_MCI_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_MCI_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* MCI register base address offset into the MCI domain *****************************************/

#define LPC31_MCI_VBASE                (LPC31_MCI_VSECTION)
#define LPC31_MCI_PBASE                (LPC31_MCI_PSECTION)

/* MCI register offsets (with respect to the MCI base) ******************************************/

#define LPC31_MCI_CTRL_OFFSET          0x000 /* Control register */
#define LPC31_MCI_PWREN_OFFSET         0x004 /* Reserved */
#define LPC31_MCI_CLKDIV_OFFSET        0x008 /* Clock-divider register */
#define LPC31_MCI_CLKSRC_OFFSET        0x00c /* Clock-source register */
#define LPC31_MCI_CLKENA_OFFSET        0x010 /* Clock-enable register */
#define LPC31_MCI_TMOUT_OFFSET         0x014 /* Time-out register */
#define LPC31_MCI_CTYPE_OFFSET         0x018 /* Card-type register */
#define LPC31_MCI_BLKSIZ_OFFSET        0x01c /* Block-size register */
#define LPC31_MCI_BYTCNT_OFFSET        0x020 /* Byte-count register */
#define LPC31_MCI_INTMASK_OFFSET       0x024 /* Interrupt-mask register */
#define LPC31_MCI_CMDARG_OFFSET        0x028 /* Command-argument register */
#define LPC31_MCI_CMD_OFFSET           0x02c /* Command register */
#define LPC31_MCI_RESP0_OFFSET         0x030 /* Response-0 register */
#define LPC31_MCI_RESP1_OFFSET         0x034 /* Response-1register */
#define LPC31_MCI_RESP2_OFFSET         0x038 /* Response-2 register */
#define LPC31_MCI_RESP3_OFFSET         0x03c /* Response-3 register */
#define LPC31_MCI_MINTSTS_OFFSET       0x040 /* Masked interrupt-status register */
#define LPC31_MCI_RINTSTS_OFFSET       0x044 /* Raw interrupt-status register */
#define LPC31_MCI_STATUS_OFFSET        0x048 /* Status register */
#define LPC31_MCI_FIFOTH_OFFSET        0x04c /* FIFO threshold register */
#define LPC31_MCI_CDETECT_OFFSET       0x050 /* Card-detect register value */
#define LPC31_MCI_WRTPRT_OFFSET        0x054 /* Write-protect register */
                                               /* 0x58: Reserved */
#define LPC31_MCI_TCBCNT_OFFSET        0x05c /* Transferred CIU card byte count */
#define LPC31_MCI_TBBCNT_OFFSET        0x060 /* Transferred cpu/DMA to/from BIU-FIFO byte count */
                                               /* 0x064-0x0ff: Reserved */
#define LPC31_MCI_DATA_OFFSET          0x100 /* Data FIFO read/write (>=) */

/* MCI register (virtual) addresses *************************************************************/

#define LPC31_MCI_CTRL                 (LPC31_MCI_VBASE+LPC31_MCI_CTRL_OFFSET)
#define LPC31_MCI_PWREN                (LPC31_MCI_VBASE+LPC31_MCI_PWREN_OFFSET)
#define LPC31_MCI_CLKDIV               (LPC31_MCI_VBASE+LPC31_MCI_CLKDIV_OFFSET)
#define LPC31_MCI_CLKSRC               (LPC31_MCI_VBASE+LPC31_MCI_CLKSRC_OFFSET)
#define LPC31_MCI_CLKENA               (LPC31_MCI_VBASE+LPC31_MCI_CLKENA_OFFSET)
#define LPC31_MCI_TMOUT                (LPC31_MCI_VBASE+LPC31_MCI_TMOUT_OFFSET)
#define LPC31_MCI_CTYPE                (LPC31_MCI_VBASE+LPC31_MCI_CTYPE_OFFSET)
#define LPC31_MCI_BLKSIZ               (LPC31_MCI_VBASE+LPC31_MCI_BLKSIZ_OFFSET)
#define LPC31_MCI_BYTCNT               (LPC31_MCI_VBASE+LPC31_MCI_BYTCNT_OFFSET)
#define LPC31_MCI_INTMASK              (LPC31_MCI_VBASE+LPC31_MCI_INTMASK_OFFSET)
#define LPC31_MCI_CMDARG               (LPC31_MCI_VBASE+LPC31_MCI_CMDARG_OFFSET)
#define LPC31_MCI_CMD                  (LPC31_MCI_VBASE+LPC31_MCI_CMD_OFFSET)
#define LPC31_MCI_RESP0                (LPC31_MCI_VBASE+LPC31_MCI_RESP0_OFFSET)
#define LPC31_MCI_RESP1                (LPC31_MCI_VBASE+LPC31_MCI_RESP1_OFFSET)
#define LPC31_MCI_RESP2                (LPC31_MCI_VBASE+LPC31_MCI_RESP2_OFFSET)
#define LPC31_MCI_RESP3                (LPC31_MCI_VBASE+LPC31_MCI_RESP3_OFFSET)
#define LPC31_MCI_MINTSTS              (LPC31_MCI_VBASE+LPC31_MCI_MINTSTS_OFFSET)
#define LPC31_MCI_RINTSTS              (LPC31_MCI_VBASE+LPC31_MCI_RINTSTS_OFFSET)
#define LPC31_MCI_STATUS               (LPC31_MCI_VBASE+LPC31_MCI_STATUS_OFFSET)
#define LPC31_MCI_FIFOTH               (LPC31_MCI_VBASE+LPC31_MCI_FIFOTH_OFFSET)
#define LPC31_MCI_CDETECT              (LPC31_MCI_VBASE+LPC31_MCI_CDETECT_OFFSET)
#define LPC31_MCI_WRTPRT               (LPC31_MCI_VBASE+LPC31_MCI_WRTPRT_OFFSET)
#define LPC31_MCI_TCBCNT               (LPC31_MCI_VBASE+LPC31_MCI_TCBCNT_OFFSET)
#define LPC31_MCI_TBBCNT               (LPC31_MCI_VBASE+LPC31_MCI_TBBCNT_OFFSET)
#define LPC31_MCI_DATA                 (LPC31_MCI_VBASE+LPC31_MCI_DATA_OFFSET)

/* MCI register bit definitions *****************************************************************/

/* Control register CTRL, address 0x18000000 */

#define MCI_CTRL_CEATAINT                (1 << 11) /* Bit 11: CE-ATA device interrupts enabled */
#define MCI_CTRL_AUTOSTOP                (1 << 10) /* Bit 10: Send STOP after CCSD to CE-ATA device */
#define MCI_CTRL_SENDCCSD                (1 << 9)  /* Bit 9:  Send CCSD to CE-ATA device */
#define MCI_CTRL_ABORTREAD               (1 << 8)  /* Bit 8:  Reset data state-machine (suspend sequence) */
#define MCI_CTRL_SENDIRQRESP             (1 << 7)  /* Bit 7:  Send auto IRQ response */
#define MCI_CTRL_READWAIT                (1 << 6)  /* Bit 6:  Assert read wait */
#define MCI_CTRL_DMAENABLE               (1 << 5)  /* Bit 5:  Enable DMA transfer mode */
#define MCI_CTRL_INTENABLE               (1 << 4)  /* Bit 4:  Enable interrupts */
#define MCI_CTRL_DMARESET                (1 << 2)  /* Bit 2:  Reset internal DMA interface control logic */
#define MCI_CTRL_FIFORESET               (1 << 1)  /* Bit 1:  Reset to data FIFO To reset FIFO pointers */
#define MCI_CTRL_CNTLRRESET              (1 << 0)  /* Bit 0:  Reset Module controller */

/* Clock divider register CLKDIV, address 0x18000008 */

#define MCI_CLKDIV_SHIFT                 (0)       /* Bits 0-7: Clock divider */
#define MCI_CLKDIV_MASK                  (255 << MCI_CLKDIV_SHIFT)

/* Clock source register CLKSRC, address 0x1800000c */

#define MCI_CLKSRC_SHIFT                 (0)       /* Bits 0-1: Must be zero */
#define MCI_CLKSRC_MASK                  (3 << MCI_CLKSRC_SHIFT)

/* Clock enable register CLKENA, address 0x18000010 */

#define MCI_CLKENA_LOWPOWER              (1 << 16) /* Bit 16: Low-power mode */
#define MCI_CLKENA_EMABLE                (1 << 0)  /* Bit 0:  Clock enable */

/*Timeout register TMOUT, address 0x18000014 */

#define MCI_TMOUT_DATA_SHIFT             (8)       /* Bits 8-31: Data Read Timeout value */
#define MCI_TMOUT_DATA_MASK              (0x00ffffff << MCI_TMOUT_DATA_SHIFT)
#define MCI_TMOUT_RESPONSE_SHIFT         (0)       /* Bits 0-7: Response timeout value */
#define MCI_TMOUT_RESPONSE_MASK          (255 << MCI_TMOUT_RESPONSE_SHIFT)

/* Card type register CTYPE, address 0x18000018 */

#define MCI_CTYPE_WIDTH8                 (1 << 16) /* Bit 16: 8-bit mode */
#define MCI_CTYPE_WIDTH4                 (1 << 0)  /* Bit 0:  4-bit mode */

/* Blocksize register BLKSIZ, address 0x1800001c */

#define MCI_BLKSIZ_SHIFT                 (0)       /* Bits 0-15: Block size */
#define MCI_BLKSIZ_MASK                  (0xffff << MCI_BLKSIZ_SHIFT)

/* Interrupt mask register INTMASK, address 0x18000024
 * Masked interrupt status register MINTSTS, address 0x18000040
 * Raw interrupt status register RINTSTS, address 0x18000044
 */

#define MCI_INT_SDIO                     (1 << 16) /* Bit 16: Mask SDIO interrupt */
#define MCI_INT_EBE                      (1 << 15) /* Bit 15: End-bit error (read)/Write no CRC */
#define MCI_INT_ACD                      (1 << 14) /* Bit 14: Auto command done */
#define MCI_INT_SBE                      (1 << 13) /* Bit 13: Start-bit error */
#define MCI_INT_HLE                      (1 << 12) /* Bit 12: Hardware locked write error */
#define MCI_INT_FRUN                     (1 << 11) /* Bit 11: FIFO underrun/overrun error */
#define MCI_INT_HTO                      (1 << 10) /* Bit 10: Data starvation-by-cpu timeout */
#define MCI_INT_DRTO                     (1 << 9)  /* Bit 9:  Data read timeout */
#define MCI_INT_RTO                      (1 << 8)  /* Bit 8:  Response timeout */
#define MCI_INT_DCRC                     (1 << 7)  /* Bit 7:  Data CRC error */
#define MCI_INT_RCRC                     (1 << 6)  /* Bit 6:  Response CRC error */
#define MCI_INT_RXDR                     (1 << 5)  /* Bit 5:  Receive FIFO data request */
#define MCI_INT_TXDR                     (1 << 4)  /* Bit 4:  Transmit FIFO data request */
#define MCI_INT_DTO                      (1 << 3)  /* Bit 3:  Data transfer over */
#define MCI_INT_CD                       (1 << 2)  /* Bit 2:  Command done */
#define MCI_INT_RE                       (1 << 1)  /* Bit 1:  Response error */
#define MCI_INT_CD                       (1 << 0)  /* Bit 0:  Card detect */
#define MCI_INT_ALL                      (0x1ffff)

/* Command register CMD, address 0x1800002c */

#define MCI_CMD_STARTCMD                 (1 << 31) /* Bit 31: Start command */
#define MCI_CMD_CCSEXPTD                 (1 << 23) /* Bit 23: Expect command completion from CE-ATA device */
#define MCI_CMD_READCEATA                (1 << 22) /* Bit 22: Performing read access on CE-ATA device */
#define MCI_CMD_UPDCLOCK                 (1 << 21) /* Bit 21: Update clock register value (no command) */
#define MCI_CMD_SENDINIT                 (1 << 15) /* Bit 15: Send initialization sequence before command */
#define MCI_CMD_STOPABORT                (1 << 14) /* Bit 14: Stop current data transfer */
#define MCI_CMD_WAITPREV                 (1 << 13) /* Bit 13: Wait previous transfer complete before sending */
#define MCI_CMD_AUTOSTOP                 (1 << 12) /* Bit 12: Send stop command at end of data transfer */
#define MCI_CMD_XFRMODE                  (1 << 11) /* Bit 11: Stream data transfer command */
#define MCI_CMD_WRITE                    (1 << 10) /* Bit 10: Write to card */
#define MCI_CMD_DATAXFREXPTD             (1 << 9)  /* Bit 9:  Data transfer expected (read/write) */
#define MCI_CMD_RESPCRC                  (1 << 8)  /* Bit 8:  Check response CRC */
#define MCI_CMD_LONGRESP                 (1 << 7)  /* Bit 7:  Long response expected from card */
#define MCI_CMD_RESPONSE                 (1 << 6)  /* Bit 6:  Response expected from card */
#define MCI_CMD_CMDINDEX_SHIFT           (0)       /* Bits 0-5: 5:0 Command index */
#define MCI_CMD_CMDINDEX_MASK            (63 << MCI_CMD_CMDINDEX_SHIFT)

/* Status register STATUS, address 0x18000048 */

#define MCI_STATUS_DMAREQ                (1 << 31) /* Bit 31: DMA request signal state */
#define MCI_STATUS_DMAACK                (1 << 30) /* Bit 30: DMA acknowledge signal state */
#define MCI_STATUS_FIFOCOUNT_SHIFT       (17)      /* Bits 17-29: FIFO count */
#define MCI_STATUS_FIFOCOUNT_MASK        (0x1fff << MCI_STATUS_FIFOCOUNT_SHIFT)
#define MCI_STATUS_RESPINDEX_SHIFT       (11)      /* Bits 11-16: Index of previous response */
#define MCI_STATUS_RESPINDEX_MASK        (63 << MCI_STATUS_RESPINDEX_SHIFT)
#define MCI_STATUS_MCBUSY                (1 << 10) /* Bit 10: Data transmit/receive state machine busy */
#define MCI_STATUS_DATABUSY              (1 << 9)  /* Bit 9:  Card data busy */
#define MCI_STATUS_DAT3                  (1 << 8)  /* Bit 8:  DAT3=1: Card present */
#define MCI_STATUS_FSMSTATE_SHIFT        (4)       /* Bits 4-7: 7:4 Command FSM states */
#define MCI_STATUS_FSMSTATE_MASK         (15 << MCI_STATUS_FSMSTATE_SHIFT)
#  define MCI_STATUS_FSMSTATE_IDLE       (0  << MCI_STATUS_FSMSTATE_SHIFT) /* Idle */
#  define MCI_STATUS_FSMSTATE_INIT       (1  << MCI_STATUS_FSMSTATE_SHIFT) /* Send init sequence */
#  define MCI_STATUS_FSMSTATE_TXSTART    (2  << MCI_STATUS_FSMSTATE_SHIFT) /* Tx cmd start bit */
#  define MCI_STATUS_FSMSTATE_TXTXBIT    (3  << MCI_STATUS_FSMSTATE_SHIFT) /* Tx cmd tx bit */
#  define MCI_STATUS_FSMSTATE_TXCMDARG   (4  << MCI_STATUS_FSMSTATE_SHIFT) /* Tx cmd index + arg */
#  define MCI_STATUS_FSMSTATE_TXCMDCRC   (5  << MCI_STATUS_FSMSTATE_SHIFT) /* Tx cmd crc7 */
#  define MCI_STATUS_FSMSTATE_TXEND      (6  << MCI_STATUS_FSMSTATE_SHIFT) /* Tx cmd end bit */
#  define MCI_STATUS_FSMSTATE_RXSTART    (7  << MCI_STATUS_FSMSTATE_SHIFT) /* Rx resp start bit */
#  define MCI_STATUS_FSMSTATE_RXIRQ      (8  << MCI_STATUS_FSMSTATE_SHIFT) /* Rx resp IRQ response */
#  define MCI_STATUS_FSMSTATE_RXTXBIT    (9  << MCI_STATUS_FSMSTATE_SHIFT) /* Rx resp tx bit */
#  define MCI_STATUS_FSMSTATE_RXCMD      (10 << MCI_STATUS_FSMSTATE_SHIFT) /* Rx resp cmd idx */
#  define MCI_STATUS_FSMSTATE_RXRESP     (11 << MCI_STATUS_FSMSTATE_SHIFT) /* Rx resp data */
#  define MCI_STATUS_FSMSTATE_RXRESPCRC  (12 << MCI_STATUS_FSMSTATE_SHIFT) /* Rx resp crc7 */
#  define MCI_STATUS_FSMSTATE_RXEND      (13 << MCI_STATUS_FSMSTATE_SHIFT) /* Rx resp end bit */
#  define MCI_STATUS_FSMSTATE_WAITNCC    (14 << MCI_STATUS_FSMSTATE_SHIFT) /* Cmd path wait NCC */
#  define MCI_STATUS_FSMSTATE_WAITTURN   (15 << MCI_STATUS_FSMSTATE_SHIFT) /* Wait; CMD-to-response turnaround */
#define MCI_STATUS_FIFOFULL              (1 << 3)  /* Bit 3:  FIFO is full */
#define MCI_STATUS_FIFOEMPTY             (1 << 2)  /* Bit 2:  FIFO is empty */
#define MCI_STATUS_TXWMARK               (1 << 1)  /* Bit 1:  FIFO reached Transmit watermark level */
#define MCI_STATUS_RXWMARK               (1 << 0)  /* Bit 0:  FIFO reached Receive watermark level */

/* FIFO threshold register FIFOTH, address 0x1800004c */

#define MCI_FIFOTH_DMABURST_SHIFT        (28)      /* Bits 28-30: Burst size for multiple transaction */
#define MCI_FIFOTH_DMABURST_MASK         (7 << MCI_FIFOTH_DMABURST_SHIFT)
  define MCI_FIFOTH_DMABURST_1XFR        (0 << MCI_FIFOTH_DMABURST_SHIFT) /* 1 transfer */
#  define MCI_FIFOTH_DMABURST_4XFRS      (1 << MCI_FIFOTH_DMABURST_SHIFT) /* 4 transfers */
#  define MCI_FIFOTH_DMABURST_8XFRS      (2 << MCI_FIFOTH_DMABURST_SHIFT) /* 8 transfers */
#define MCI_FIFOTH_RXWMARK_SHIFT         (16)      /* Bits 16-27: FIFO threshold level when receiving */
#define MCI_FIFOTH_RXWMARK_MASK          (0xfff << MCI_FIFOTH_RXWMARK_SHIFT)
#define MCI_FIFOTH_TXWMARK_SHIFT         (0)       /* Bits 0-11: FIFO threshold level when transmitting */
#define MCI_FIFOTH_TXWMARK_MASK          (0xfff << MCI_FIFOTH_TXWMARK_SHIFT)

/* Card detect register CDETECT, address 0x18000050 */

#define MCI_CDETECT_NOTPRESENT           (1 << 0)

/* Write protect register WRTPRT, address 0x18000054 */

#define MCI_WRTPRT_PROTECTED             (1 << 0)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_MCI_H */
