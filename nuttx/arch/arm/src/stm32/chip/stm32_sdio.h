/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_sdio.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_SDIO_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_SDIO_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_SDIO_POWER_OFFSET   0x0000 /* SDIO power control register */
#define STM32_SDIO_CLKCR_OFFSET   0x0004 /* SDI clock control register */
#define STM32_SDIO_ARG_OFFSET     0x0008 /* SDIO argument register */
#define STM32_SDIO_CMD_OFFSET     0x000c /* SDIO command register */
#define STM32_SDIO_RESPCMD_OFFSET 0x0010 /* SDIO command response register */
#define STM32_SDIO_RESP_OFFSET(n) (0x0010+4*(n))
#define STM32_SDIO_RESP1_OFFSET   0x0014 /* SDIO response 1 register */
#define STM32_SDIO_RESP2_OFFSET   0x0018 /* SDIO response 2 register */
#define STM32_SDIO_RESP3_OFFSET   0x001c /* SDIO response 3 register */
#define STM32_SDIO_RESP4_OFFSET   0x0020 /* SDIO response 4 register */
#define STM32_SDIO_DTIMER_OFFSET  0x0024 /* SDIO data timer register */
#define STM32_SDIO_DLEN_OFFSET    0x0028 /* SDIO data length register */
#define STM32_SDIO_DCTRL_OFFSET   0x002c /* SDIO data control register */
#define STM32_SDIO_DCOUNT_OFFSET  0x0030 /* SDIO data counter register */
#define STM32_SDIO_STA_OFFSET     0x0034 /* SDIO status register */
#define STM32_SDIO_ICR_OFFSET     0x0038 /* SDIO interrupt clear register */
#define STM32_SDIO_MASK_OFFSET    0x003c /* SDIO mask register */
#define STM32_SDIO_FIFOCNT_OFFSET 0x0048 /* SDIO FIFO counter register */
#define STM32_SDIO_FIFO_OFFSET    0x0080 /* SDIO data FIFO register */

/* Register Addresses ***************************************************************/

#define STM32_SDIO_POWER          (STM32_SDIO_BASE+STM32_SDIO_POWER_OFFSET)
#define STM32_SDIO_CLKCR          (STM32_SDIO_BASE+STM32_SDIO_CLKCR_OFFSET)
#define STM32_SDIO_ARG            (STM32_SDIO_BASE+STM32_SDIO_ARG_OFFSET)
#define STM32_SDIO_CMD            (STM32_SDIO_BASE+STM32_SDIO_CMD_OFFSET)
#define STM32_SDIO_RESPCMD        (STM32_SDIO_BASE+STM32_SDIO_RESPCMD_OFFSET)
#define STM32_SDIO_RESP(n)        (STM32_SDIO_BASE+STM32_SDIO_RESP_OFFSET(n))
#define STM32_SDIO_RESP1          (STM32_SDIO_BASE+STM32_SDIO_RESP1_OFFSET)
#define STM32_SDIO_RESP2          (STM32_SDIO_BASE+STM32_SDIO_RESP2_OFFSET)
#define STM32_SDIO_RESP3          (STM32_SDIO_BASE+STM32_SDIO_RESP3_OFFSET)
#define STM32_SDIO_RESP4          (STM32_SDIO_BASE+STM32_SDIO_RESP4_OFFSET)
#define STM32_SDIO_DTIMER         (STM32_SDIO_BASE+STM32_SDIO_DTIMER_OFFSET)
#define STM32_SDIO_DLEN           (STM32_SDIO_BASE+STM32_SDIO_DLEN_OFFSET)
#define STM32_SDIO_DCTRL          (STM32_SDIO_BASE+STM32_SDIO_DCTRL_OFFSET)
#define STM32_SDIO_DCOUNT         (STM32_SDIO_BASE+STM32_SDIO_DCOUNT_OFFSET)
#define STM32_SDIO_STA            (STM32_SDIO_BASE+STM32_SDIO_STA_OFFSET)
#define STM32_SDIO_ICR            (STM32_SDIO_BASE+STM32_SDIO_ICR_OFFSET)
#define STM32_SDIO_MASK           (STM32_SDIO_BASE+STM32_SDIO_MASK_OFFSET)
#define STM32_SDIO_FIFOCNT        (STM32_SDIO_BASE+STM32_SDIO_FIFOCNT_OFFSET)
#define STM32_SDIO_FIFO           (STM32_SDIO_BASE+STM32_SDIO_FIFO_OFFSET)

/* Bit-band (BB) base addresses ****************************************************/

#define STM32_SDIO_OFFSET         (STM32_SDIO_BASE-STM32_PERIPH_BASE)

#define STM32_SDIO_POWER_BB       (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_POWER_OFFSET)<<5))
#define STM32_SDIO_CLKCR_BB       (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_CLKCR_OFFSET)<<5))
#define STM32_SDIO_ARG_BB         (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_ARG_OFFSET)<<5))
#define STM32_SDIO_CMD_BB         (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_CMD_OFFSET)<<5))
#define STM32_SDIO_RESPCMD_BB     (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_RESPCMD_OFFSET)<<5))
#define STM32_SDIO_RESP_BB(n)     (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_RESP_OFFSET(n))<<5))
#define STM32_SDIO_RESP1_BB       (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_RESP1_OFFSET)<<5))
#define STM32_SDIO_RESP2_BB       (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_RESP2_OFFSET)<<5))
#define STM32_SDIO_RESP3_BB       (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_RESP3_OFFSET)<<5))
#define STM32_SDIO_RESP4_BB       (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_RESP4_OFFSET)<<5))
#define STM32_SDIO_DTIMER_BB      (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_DTIMER_OFFSET)<<5))
#define STM32_SDIO_DLEN_BB        (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_DLEN_OFFSET)<<5))
#define STM32_SDIO_DCTRL_BB       (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_DCTRL_OFFSET)<<5))
#define STM32_SDIO_DCOUNT_BB      (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_DCOUNT_OFFSET)<<5))
#define STM32_SDIO_STA_BB         (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_STA_OFFSET)<<5))
#define STM32_SDIO_ICR_BB         (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_ICR_OFFSET)<<5))
#define STM32_SDIO_MASK_BB        (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_MASK_OFFSET)<<5))
#define STM32_SDIO_FIFOCNT_BB     (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_FIFOCNT_OFFSET)<<5))
#define STM32_SDIO_FIFO_BB        (STM32_PERIPHBB_BASE+((STM32_SDIO_OFFSET+STM32_SDIO_FIFO_OFFSET)<<5))

/* Register Bitfield Definitions ****************************************************/

#define SDIO_POWER_PWRCTRL_SHIFT       (0)       /* Bits 0-1: Power supply control bits */
#define SDIO_POWER_PWRCTRL_MASK        (3 << SDIO_POWER_PWRCTRL_SHIFT)
#  define SDIO_POWER_PWRCTRL_OFF       (0 << SDIO_POWER_PWRCTRL_SHIFT) /* 00: Power-off: card clock stopped */
#  define SDIO_POWER_PWRCTRL_PWRUP     (2 << SDIO_POWER_PWRCTRL_SHIFT) /* 10: Reserved power-up */
#  define SDIO_POWER_PWRCTRL_ON        (3 << SDIO_POWER_PWRCTRL_SHIFT) /* 11: Power-on: card is clocked */

#define SDIO_POWER_RESET               (0)       /* Reset value */

#define SDIO_CLKCR_CLKDIV_SHIFT        (0)       /* Bits 7-0: Clock divide factor */
#define SDIO_CLKCR_CLKDIV_MASK         (0xff << SDIO_CLKCR_CLKDIV_SHIFT)
#define SDIO_CLKCR_CLKEN               (1 << 8)  /* Bit 8: Clock enable bit */
#define SDIO_CLKCR_PWRSAV              (1 << 9)  /* Bit 9: Power saving configuration bit */
#define SDIO_CLKCR_BYPASS              (1 << 10) /* Bit 10: Clock divider bypass enable bit */
#define SDIO_CLKCR_WIDBUS_SHIFT        (11)      /* Bits 12-11: Wide bus mode enable bits */
#define SDIO_CLKCR_WIDBUS_MASK         (3 << SDIO_CLKCR_WIDBUS_SHIFT)
#  define SDIO_CLKCR_WIDBUS_D1         (0 << SDIO_CLKCR_WIDBUS_SHIFT) /* 00: Default (SDIO_D0) */
#  define SDIO_CLKCR_WIDBUS_D4         (1 << SDIO_CLKCR_WIDBUS_SHIFT) /* 01: 4-wide (SDIO_D[3:0]) */
#  define SDIO_CLKCR_WIDBUS_D8         (2 << SDIO_CLKCR_WIDBUS_SHIFT) /* 10: 8-wide (SDIO_D[7:0]) */
#define SDIO_CLKCR_NEGEDGE             (1 << 13) /* Bit 13: SDIO_CK dephasing selection bit */
#define SDIO_CLKCR_HWFC_EN             (1 << 14) /* Bit 14: HW Flow Control enable */

#define SDIO_CLKCR_RESET               (0)       /* Reset value */
#define SDIO_ARG_RESET                 (0)       /* Reset value */

#define SDIO_CLKCR_CLKEN_BB            (STM32_SDIO_CLKCR_BB + (8 * 4))
#define SDIO_CLKCR_PWRSAV_BB           (STM32_SDIO_CLKCR_BB + (9 * 4))
#define SDIO_CLKCR_BYPASS_BB           (STM32_SDIO_CLKCR_BB + (10 * 4))
#define SDIO_CLKCR_NEGEDGE_BB          (STM32_SDIO_CLKCR_BB + (13 * 4))
#define SDIO_CLKCR_HWFC_EN_BB          (STM32_SDIO_CLKCR_BB + (14 * 4))

#define SDIO_CMD_CMDINDEX_SHIFT        (0)
#define SDIO_CMD_CMDINDEX_MASK         (0x3f << SDIO_CMD_CMDINDEX_SHIFT)
#define SDIO_CMD_WAITRESP_SHIFT        (6)       /* Bits 7-6: Wait for response bits */
#define SDIO_CMD_WAITRESP_MASK         (3 << SDIO_CMD_WAITRESP_SHIFT)
#  define SDIO_CMD_NORESPONSE          (0 << SDIO_CMD_WAITRESP_SHIFT) /* 00/10: No response */
#  define SDIO_CMD_SHORTRESPONSE       (1 << SDIO_CMD_WAITRESP_SHIFT) /* 01: Short response */
#  define SDIO_CMD_LONGRESPONSE        (3 << SDIO_CMD_WAITRESP_SHIFT) /* 11: Long response */
#define SDIO_CMD_WAITINT               (1 << 8)  /* Bit 8: CPSM waits for interrupt request */
#define SDIO_CMD_WAITPEND              (1 << 9)  /* Bit 9: CPSM Waits for ends of data transfer */
#define SDIO_CMD_CPSMEN                (1 << 10) /* Bit 10: Command path state machine enable */
#define SDIO_CMD_SUSPEND               (1 << 11) /* Bit 11: SD I/O suspend command */
#define SDIO_CMD_ENDCMD                (1 << 12) /* Bit 12: Enable CMD completion */
#define SDIO_CMD_NIEN                  (1 << 13) /* Bit 13: not Interrupt Enable */
#define SDIO_CMD_ATACMD                (1 << 14) /* Bit 14: CE-ATA command */

#define SDIO_CMD_RESET                 (0)       /* Reset value */

#define SDIO_CMD_WAITINT_BB            (STM32_SDIO_CMD_BB + (8 * 4))
#define SDIO_CMD_WAITPEND_BB           (STM32_SDIO_CMD_BB + (9 * 4))
#define SDIO_CMD_CPSMEN_BB             (STM32_SDIO_CMD_BB + (10 * 4))
#define SDIO_CMD_SUSPEND_BB            (STM32_SDIO_CMD_BB + (11 * 4))
#define SDIO_CMD_ENCMD_BB              (STM32_SDIO_CMD_BB + (12 * 4))
#define SDIO_CMD_NIEN_BB               (STM32_SDIO_CMD_BB + (13 * 4))
#define SDIO_CMD_ATACMD_BB             (STM32_SDIO_CMD_BB + (14 * 4))

#define SDIO_RESPCMD_SHIFT             (0)
#define SDIO_RESPCMD_MASK              (0x3f << SDIO_RESPCMD_SHIFT)

#define SDIO_DTIMER_RESET              (0)       /* Reset value */

#define SDIO_DLEN_SHIFT                (0)
#define SDIO_DLEN_MASK                 (0x01ffffff << SDIO_DLEN_SHIFT)

#define SDIO_DLEN_RESET                (0)       /* Reset value */

#define SDIO_DCTRL_DTEN                (1 << 0)  /* Bit 0: Data transfer enabled bit */
#define SDIO_DCTRL_DTDIR               (1 << 1)  /* Bit 1: Data transfer direction */
#define SDIO_DCTRL_DTMODE              (1 << 2)  /* Bit 2: Data transfer mode */
#define SDIO_DCTRL_DMAEN               (1 << 3)  /* Bit 3: DMA enable bit */
#define SDIO_DCTRL_DBLOCKSIZE_SHIFT    (4)       /* Bits 7-4: Data block size */
#define SDIO_DCTRL_DBLOCKSIZE_MASK     (15 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_1BYTE             (0 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_2BYTES            (1 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_4BYTES            (2 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_8BYTES            (3 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_16BYTES           (4 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_32BYTES           (5 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_64BYTES           (6 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_128BYTES          (7 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_256BYTES          (8 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_512BYTES          (9 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_1KBYTE            (10 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_2KBYTES           (11 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_4KBYTES           (12 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_8KBYTES           (13 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDIO_DCTRL_16KBYTES          (14 << SDIO_DCTRL_DBLOCKSIZE_SHIFT)
#define SDIO_DCTRL_RWSTART             (1 << 8)  /* Bit 8: Read wait start */
#define SDIO_DCTRL_RWSTOP              (1 << 9)  /* Bit 9: Read wait stop */
#define SDIO_DCTRL_RWMOD               (1 << 10) /* Bit 10: Read wait mode */
#define SDIO_DCTRL_SDIOEN              (1 << 11) /* Bit 11: SD I/O enable functions */

#define SDIO_DCTRL_RESET               (0)       /* Reset value */

#define SDIO_DCTRL_DTEN_BB             (STM32_SDIO_DCTRL_BB + (0 * 4))
#define SDIO_DCTRL_DTDIR_BB            (STM32_SDIO_DCTRL_BB + (1 * 4))
#define SDIO_DCTRL_DTMODE_BB           (STM32_SDIO_DCTRL_BB + (2 * 4))
#define SDIO_DCTRL_DMAEN_BB            (STM32_SDIO_DCTRL_BB + (3 * 4))
#define SDIO_DCTRL_RWSTART_BB          (STM32_SDIO_DCTRL_BB + (8 * 4))
#define SDIO_DCTRL_RWSTOP_BB           (STM32_SDIO_DCTRL_BB + (9 * 4))
#define SDIO_DCTRL_RWMOD_BB            (STM32_SDIO_DCTRL_BB + (10 * 4))
#define SDIO_DCTRL_SDIOEN_BB           (STM32_SDIO_DCTRL_BB + (11 * 4))

#define SDIO_DATACOUNT_SHIFT           (0)
#define SDIO_DATACOUNT_MASK            (0x01ffffff << SDIO_DATACOUNT_SHIFT)

#define SDIO_STA_CCRCFAIL              (1 << 0)  /* Bit 0: Command response CRC fail */
#define SDIO_STA_DCRCFAIL              (1 << 1)  /* Bit 1: Data block CRC fail */
#define SDIO_STA_CTIMEOUT              (1 << 2)  /* Bit 2: Command response timeout */
#define SDIO_STA_DTIMEOUT              (1 << 3)  /* Bit 3: Data timeout */
#define SDIO_STA_TXUNDERR              (1 << 4)  /* Bit 4: Transmit FIFO underrun error */
#define SDIO_STA_RXOVERR               (1 << 5)  /* Bit 5: Received FIFO overrun error */
#define SDIO_STA_CMDREND               (1 << 6)  /* Bit 6: Command response received  */
#define SDIO_STA_CMDSENT               (1 << 7)  /* Bit 7: Command sent  */
#define SDIO_STA_DATAEND               (1 << 8)  /* Bit 8: Data end */
#define SDIO_STA_STBITERR              (1 << 9)  /* Bit 9: Start bit not detected  */
#define SDIO_STA_DBCKEND               (1 << 10) /* Bit 10: Data block sent/received  */
#define SDIO_STA_CMDACT                (1 << 11) /* Bit 11: Command transfer in progress */
#define SDIO_STA_TXACT                 (1 << 12) /* Bit 12: Data transmit in progress */
#define SDIO_STA_RXACT                 (1 << 13) /* Bit 13: Data receive in progress */
#define SDIO_STA_TXFIFOHE              (1 << 14) /* Bit 14: Transmit FIFO half empty */
#define SDIO_STA_RXFIFOHF              (1 << 15) /* Bit 15: Receive FIFO half full */
#define SDIO_STA_TXFIFOF               (1 << 16) /* Bit 16: Transmit FIFO full */
#define SDIO_STA_RXFIFOF               (1 << 17) /* Bit 17: Receive FIFO full */
#define SDIO_STA_TXFIFOE               (1 << 18) /* Bit 18: Transmit FIFO empty */
#define SDIO_STA_RXFIFOE               (1 << 19) /* Bit 19: Receive FIFO empty */
#define SDIO_STA_TXDAVL                (1 << 20) /* Bit 20: Data available in transmit FIFO */
#define SDIO_STA_RXDAVL                (1 << 21) /* Bit 21: Data available in receive FIFO */
#define SDIO_STA_SDIOIT                (1 << 22) /* Bit 22: SDIO interrupt received */
#define SDIO_STA_CEATAEND              (1 << 23) /* Bit 23: CMD6 CE-ATA command completion */

#define SDIO_ICR_CCRCFAILC             (1 << 0)  /* Bit 0: CCRCFAIL flag clear bit */
#define SDIO_ICR_DCRCFAILC             (1 << 1)  /* Bit 1: DCRCFAIL flag clear bit */
#define SDIO_ICR_CTIMEOUTC             (1 << 2)  /* Bit 2: CTIMEOUT flag clear bit */
#define SDIO_ICR_DTIMEOUTC             (1 << 3)  /* Bit 3: DTIMEOUT flag clear bit */
#define SDIO_ICR_TXUNDERRC             (1 << 4)  /* Bit 4: TXUNDERR flag clear bit */
#define SDIO_ICR_RXOVERRC              (1 << 5)  /* Bit 5: RXOVERR flag clear bit */
#define SDIO_ICR_CMDRENDC              (1 << 6)  /* Bit 6: CMDREND flag clear bit */
#define SDIO_ICR_CMDSENTC              (1 << 7)  /* Bit 7: CMDSENT flag clear bit */
#define SDIO_ICR_DATAENDC              (1 << 8)  /* Bit 8: DATAEND flag clear bit */
#define SDIO_ICR_STBITERRC             (1 << 9)  /* Bit 9: STBITERR flag clear bit */
#define SDIO_ICR_DBCKENDC              (1 << 10) /* Bit 10: DBCKEND flag clear bit */
#define SDIO_ICR_SDIOITC               (1 << 22) /* Bit 22: SDIOIT flag clear bit */
#define SDIO_ICR_CEATAENDC             (1 << 23) /* Bit 23: CEATAEND flag clear bit */

#define SDIO_ICR_RESET                 0x00c007ff
#define SDIO_ICR_STATICFLAGS           0x000005ff

#define SDIO_MASK_CCRCFAILIE           (1 << 0)  /* Bit 0: Command CRC fail interrupt enable */
#define SDIO_MASK_DCRCFAILIE           (1 << 1)  /* Bit 1: Data CRC fail interrupt enable */
#define SDIO_MASK_CTIMEOUTIE           (1 << 2)  /* Bit 2: Command timeout interrupt enable */
#define SDIO_MASK_DTIMEOUTIE           (1 << 3)  /* Bit 3: Data timeout interrupt enable */
#define SDIO_MASK_TXUNDERRIE           (1 << 4)  /* Bit 4: Tx FIFO underrun error interrupt enable */
#define SDIO_MASK_RXOVERRIE            (1 << 5)  /* Bit 5: Rx FIFO overrun error interrupt enable */
#define SDIO_MASK_CMDRENDIE            (1 << 6)  /* Bit 6: Command response received interrupt enable */
#define SDIO_MASK_CMDSENTIE            (1 << 7)  /* Bit 7: Command sent interrupt enable */
#define SDIO_MASK_DATAENDIE            (1 << 8)  /* Bit 8: Data end interrupt enable */
#define SDIO_MASK_STBITERRIE           (1 << 9)  /* Bit 9: Start bit error interrupt enable */
#define SDIO_MASK_DBCKENDIE            (1 << 10) /* Bit 10: Data block end interrupt enable */
#define SDIO_MASK_CMDACTIE             (1 << 11) /* Bit 11: Command acting interrupt enable */
#define SDIO_MASK_TXACTIE              (1 << 12) /* Bit 12: Data transmit acting interrupt enable */
#define SDIO_MASK_RXACTIE              (1 << 13) /* Bit 13: Data receive acting interrupt enable */
#define SDIO_MASK_TXFIFOHEIE           (1 << 14) /* Bit 14: Tx FIFO half empty interrupt enable */
#define SDIO_MASK_RXFIFOHFIE           (1 << 15) /* Bit 15: Rx FIFO half full interrupt enable */
#define SDIO_MASK_TXFIFOFIE            (1 << 16) /* Bit 16: Tx FIFO full interrupt enable */
#define SDIO_MASK_RXFIFOFIE            (1 << 17) /* Bit 17: Rx FIFO full interrupt enable */
#define SDIO_MASK_TXFIFOEIE            (1 << 18) /* Bit 18: Tx FIFO empty interrupt enable */
#define SDIO_MASK_RXFIFOEIE            (1 << 19) /* Bit 19: Rx FIFO empty interrupt enable */
#define SDIO_MASK_TXDAVLIE             (1 << 20) /* Bit 20: Data available in Tx FIFO interrupt enable */
#define SDIO_MASK_RXDAVLIE             (1 << 21) /* Bit 21: Data available in Rx FIFO interrupt enable */
#define SDIO_MASK_SDIOITIE             (1 << 22) /* Bit 22: SDIO mode interrupt received interrupt enable */
#define SDIO_MASK_CEATAENDIE           (1 << 23) /* Bit 23: CE-ATA command completion interrupt enable */

#define SDIO_MASK_RESET                (0)

#define SDIO_FIFOCNT_SHIFT             (0)
#define SDIO_FIFOCNT_MASK              (0x01ffffff << SDIO_FIFOCNT_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_SDIO_H */

