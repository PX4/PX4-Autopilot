/****************************************************************************
 * arch/arm/src/stm32/stm32_eth.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_STM32_ETHMAC)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <queue.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/net/mii.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include "up_internal.h"

#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "stm32_syscfg.h"
#include "stm32_eth.h"

#include <arch/board/board.h>

/* STM32_NETHERNET determines the number of physical interfaces
 * that will be supported.
 */

#if STM32_NETHERNET > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* See configs/stm3240g-eval/README.txt for an explanation of the configuration
 * settings.
 */

#if STM32_NETHERNET > 1
#  error "Logic to support multiple Ethernet interfaces is incomplete"
#endif

#if !defined(CONFIG_STM32_SYSCFG) && !defined(CONFIG_STM32_CONNECTIVITYLINE)
#  error "CONFIG_STM32_SYSCFG must be defined in the NuttX configuration"
#endif

#ifndef CONFIG_STM32_PHYADDR
#  error "CONFIG_STM32_PHYADDR must be defined in the NuttX configuration"
#endif

#if !defined(CONFIG_STM32_MII) && !defined(CONFIG_STM32_RMII)
#  warning "Neither CONFIG_STM32_MII nor CONFIG_STM32_RMII defined"
#endif

#if defined(CONFIG_STM32_MII) && defined(CONFIG_STM32_RMII)
#  error "Both CONFIG_STM32_MII and CONFIG_STM32_RMII defined"
#endif

#ifdef CONFIG_STM32_MII
#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    if !defined(CONFIG_STM32_MII_MCO1) && !defined(CONFIG_STM32_MII_MCO2) && !defined(CONFIG_STM32_MII_EXTCLK)
#      warning "Neither CONFIG_STM32_MII_MCO1, CONFIG_STM32_MII_MCO2, nor CONFIG_STM32_MII_EXTCLK defined"
#    endif
#    if defined(CONFIG_STM32_MII_MCO1) && defined(CONFIG_STM32_MII_MCO2)
#      error "Both CONFIG_STM32_MII_MCO1 and CONFIG_STM32_MII_MCO2 defined"
#    endif
#  elif defined(CONFIG_STM32_CONNECTIVITYLINE)
#    if !defined(CONFIG_STM32_MII_MCO) && !defined(CONFIG_STM32_MII_EXTCLK)
#      warning "Neither CONFIG_STM32_MII_MCO nor CONFIG_STM32_MII_EXTCLK defined"
#    endif
#  endif
#endif

#ifdef CONFIG_STM32_RMII
#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    if !defined(CONFIG_STM32_RMII_MCO1) && !defined(CONFIG_STM32_RMII_MCO2) && !defined(CONFIG_STM32_RMII_EXTCLK)
#      warning "Neither CONFIG_STM32_RMII_MCO1, CONFIG_STM32_RMII_MCO2, nor CONFIG_STM32_RMII_EXTCLK defined"
#    endif
#    if defined(CONFIG_STM32_RMII_MCO1) && defined(CONFIG_STM32_RMII_MCO2)
#      error "Both CONFIG_STM32_RMII_MCO1 and CONFIG_STM32_RMII_MCO2 defined"
#    endif
#  elif defined(CONFIG_STM32_CONNECTIVITYLINE)
#    if !defined(CONFIG_STM32_RMII_MCO) && !defined(CONFIG_STM32_RMII_EXTCLK)
#      warning "Neither CONFIG_STM32_RMII_MCO nor CONFIG_STM32_RMII_EXTCLK defined"
#    endif
#  endif
#endif

#ifdef CONFIG_STM32_AUTONEG
#  ifndef CONFIG_STM32_PHYSR
#    error "CONFIG_STM32_PHYSR must be defined in the NuttX configuration"
#  endif
#  ifdef CONFIG_STM32_PHYSR_ALTCONFIG
#    ifndef CONFIG_STM32_PHYSR_ALTMODE
#      error "CONFIG_STM32_PHYSR_ALTMODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_10HD
#      error "CONFIG_STM32_PHYSR_10HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_100HD
#      error "CONFIG_STM32_PHYSR_100HD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_10FD
#      error "CONFIG_STM32_PHYSR_10FD must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_100FD
#      error "CONFIG_STM32_PHYSR_100FD must be defined in the NuttX configuration"
#    endif
#  else
#    ifndef CONFIG_STM32_PHYSR_SPEED
#      error "CONFIG_STM32_PHYSR_SPEED must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_100MBPS
#      error "CONFIG_STM32_PHYSR_100MBPS must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_MODE
#      error "CONFIG_STM32_PHYSR_MODE must be defined in the NuttX configuration"
#    endif
#    ifndef CONFIG_STM32_PHYSR_FULLDUPLEX
#      error "CONFIG_STM32_PHYSR_FULLDUPLEX must be defined in the NuttX configuration"
#    endif
#  endif
#endif

#ifdef CONFIG_STM32_ETH_PTP
#  warning "CONFIG_STM32_ETH_PTP is not yet supported"
#endif

/* This driver does not use enhanced descriptors.  Enhanced descriptors must
 * be used, however, if time stamping or and/or IPv4 checksum offload is
 * supported.
 */

#undef CONFIG_STM32_ETH_ENHANCEDDESC
#undef CONFIG_STM32_ETH_HWCHECKSUM

/* Ethernet buffer sizes, number of buffers, and number of descriptors */

#ifndef CONFIG_NET_MULTIBUFFER
#  error "CONFIG_NET_MULTIBUFFER is required"
#endif

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_ETH_BUFSIZE ((CONFIG_NET_BUFSIZE + 4 + 15) & ~15)

#ifndef CONFIG_STM32_ETH_BUFSIZE
#  define CONFIG_STM32_ETH_BUFSIZE OPTIMAL_ETH_BUFSIZE
#endif

#if CONFIG_STM32_ETH_BUFSIZE > ETH_TDES1_TBS1_MASK
#  error "CONFIG_STM32_ETH_BUFSIZE is too large"
#endif

#if (CONFIG_STM32_ETH_BUFSIZE & 15) != 0
#  error "CONFIG_STM32_ETH_BUFSIZE must be aligned"
#endif

#if CONFIG_STM32_ETH_BUFSIZE != OPTIMAL_ETH_BUFSIZE
#  warning "You using an incomplete/untested configuration"
#endif

#ifndef CONFIG_STM32_ETH_NRXDESC
#  define CONFIG_STM32_ETH_NRXDESC 8
#endif
#ifndef CONFIG_STM32_ETH_NTXDESC
#  define CONFIG_STM32_ETH_NTXDESC 4
#endif

/* We need at least one more free buffer than transmit buffers */

#define STM32_ETH_NFREEBUFFERS (CONFIG_STM32_ETH_NTXDESC+1)

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_STM32_ETHMAC_REGDEBUG
#endif

/* Clocking *****************************************************************/
/* Set MACMIIAR CR bits depending on HCLK setting */

#if STM32_HCLK_FREQUENCY >= 20000000 && STM32_HCLK_FREQUENCY < 35000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_20_35
#elif STM32_HCLK_FREQUENCY >= 35000000 && STM32_HCLK_FREQUENCY < 60000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_35_60
#elif STM32_HCLK_FREQUENCY >= 60000000 && STM32_HCLK_FREQUENCY < 100000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_60_100
#elif STM32_HCLK_FREQUENCY >= 100000000 && STM32_HCLK_FREQUENCY < 150000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_100_150
#elif STM32_HCLK_FREQUENCY >= 150000000 && STM32_HCLK_FREQUENCY <= 168000000
#  define ETH_MACMIIAR_CR ETH_MACMIIAR_CR_150_168
#else
#  error "STM32_HCLK_FREQUENCY not supportable"
#endif

/* Timing *******************************************************************/
/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second
 */

#define STM32_WDDELAY     (1*CLK_TCK)
#define STM32_POLLHSEC    (1*2)

/* TX timeout = 1 minute */

#define STM32_TXTIMEOUT   (60*CLK_TCK)

/* PHY reset/configuration delays in milliseconds */ 

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
#define PHY_RETRY_TIMEOUT (0x0004ffff)

/* Register values **********************************************************/

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_MACCR_RE    Bit 2:  Receiver enable
 * ETH_MACCR_TE    Bit 3:  Transmitter enable
 * ETH_MACCR_DC    Bit 4:  Deferral check
 * ETH_MACCR_BL    Bits 5-6: Back-off limit
 * ETH_MACCR_APCS  Bit 7:  Automatic pad/CRC stripping
 * ETH_MACCR_RD    Bit 9:  Retry disable
 * ETH_MACCR_IPCO  Bit 10: IPv4 checksum offload
 * ETH_MACCR_DM    Bit 11: Duplex mode
 * ETH_MACCR_LM    Bit 12: Loopback mode
 * ETH_MACCR_ROD   Bit 13: Receive own disable
 * ETH_MACCR_FES   Bit 14: Fast Ethernet speed
 * ETH_MACCR_CSD   Bit 16: Carrier sense disable
 * ETH_MACCR_IFG   Bits 17-19: Interframe gap
 * ETH_MACCR_JD    Bit 22: Jabber disable
 * ETH_MACCR_WD    Bit 23: Watchdog disable
 * ETH_MACCR_CSTF  Bits 25: CRC stripping for Type frames (F2/F4 only)
 */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#define MACCR_CLEAR_BITS \
  (ETH_MACCR_RE | ETH_MACCR_TE | ETH_MACCR_DC | ETH_MACCR_BL_MASK | \
   ETH_MACCR_APCS | ETH_MACCR_RD | ETH_MACCR_IPCO | ETH_MACCR_DM | \
   ETH_MACCR_LM | ETH_MACCR_ROD | ETH_MACCR_FES | ETH_MACCR_CSD | \
   ETH_MACCR_IFG_MASK | ETH_MACCR_JD | ETH_MACCR_WD | ETH_MACCR_CSTF)
#else
#define MACCR_CLEAR_BITS \
  (ETH_MACCR_RE | ETH_MACCR_TE | ETH_MACCR_DC | ETH_MACCR_BL_MASK | \
   ETH_MACCR_APCS | ETH_MACCR_RD | ETH_MACCR_IPCO | ETH_MACCR_DM | \
   ETH_MACCR_LM | ETH_MACCR_ROD | ETH_MACCR_FES | ETH_MACCR_CSD | \
   ETH_MACCR_IFG_MASK | ETH_MACCR_JD | ETH_MACCR_WD)
#endif

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACCR_RE    Receiver enable                0 (disabled)
 * ETH_MACCR_TE    Transmitter enable             0 (disabled)
 * ETH_MACCR_DC    Deferral check                 0 (disabled)
 * ETH_MACCR_BL    Back-off limit                 0 (10)
 * ETH_MACCR_APCS  Automatic pad/CRC stripping    0 (disabled)
 * ETH_MACCR_RD    Retry disable                  1 (disabled)
 * ETH_MACCR_IPCO  IPv4 checksum offload          Depends on CONFIG_STM32_ETH_HWCHECKSUM
 * ETH_MACCR_LM    Loopback mode                  0 (disabled)
 * ETH_MACCR_ROD   Receive own disable            0 (enabled)
 * ETH_MACCR_CSD   Carrier sense disable          0 (enabled)
 * ETH_MACCR_IFG   Interframe gap                 0 (96 bits)
 * ETH_MACCR_JD    Jabber disable                 0 (enabled)
 * ETH_MACCR_WD    Watchdog disable               0 (enabled)
 * ETH_MACCR_CSTF  CRC stripping for Type frames  0 (disabled, F2/F4 only)
 *
 * The following are set conditioinally based on mode and speed.
 *
 * ETH_MACCR_DM       Duplex mode                    Depends on priv->fduplex
 * ETH_MACCR_FES      Fast Ethernet speed            Depends on priv->mbps100
 */

#ifdef CONFIG_STM32_ETH_HWCHECKSUM
#  define MACCR_SET_BITS \
     (ETH_MACCR_BL_10 | ETH_MACCR_RD | ETH_MACCR_IPCO | ETH_MACCR_IFG(96))
#else
#  define MACCR_SET_BITS \
     (ETH_MACCR_BL_10 | ETH_MACCR_RD | ETH_MACCR_IFG(96))
#endif

/* Clear the MACCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_MACFFR_PM    Bit 0: Promiscuous mode
 * ETH_MACFFR_HU    Bit 1: Hash unicast
 * ETH_MACFFR_HM    Bit 2: Hash multicast
 * ETH_MACFFR_DAIF  Bit 3: Destination address inverse filtering
 * ETH_MACFFR_PAM   Bit 4: Pass all multicast
 * ETH_MACFFR_BFD   Bit 5: Broadcast frames disable
 * ETH_MACFFR_PCF   Bits 6-7: Pass control frames
 * ETH_MACFFR_SAIF  Bit 8: Source address inverse filtering
 * ETH_MACFFR_SAF   Bit 9: Source address filter
 * ETH_MACFFR_HPF   Bit 10: Hash or perfect filter
 * ETH_MACFFR_RA    Bit 31: Receive all
 */

#define MACFFR_CLEAR_BITS \
  (ETH_MACFFR_PM | ETH_MACFFR_HU | ETH_MACFFR_HM | ETH_MACFFR_DAIF | \
   ETH_MACFFR_PAM | ETH_MACFFR_BFD | ETH_MACFFR_PCF_MASK | ETH_MACFFR_SAIF | \
   ETH_MACFFR_SAF | ETH_MACFFR_HPF | ETH_MACFFR_RA)
  
/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACFFR_PM    Promiscuous mode                       0 (disabled)
 * ETH_MACFFR_HU    Hash unicast                           0 (perfect dest filtering)
 * ETH_MACFFR_HM    Hash multicast                         0 (perfect dest filtering)
 * ETH_MACFFR_DAIF  Destination address inverse filtering  0 (normal)
 * ETH_MACFFR_PAM   Pass all multicast                     0 (Depends on HM bit)
 * ETH_MACFFR_BFD   Broadcast frames disable               0 (enabled)
 * ETH_MACFFR_PCF   Pass control frames                    1 (block all but PAUSE)
 * ETH_MACFFR_SAIF  Source address inverse filtering       0 (not used)
 * ETH_MACFFR_SAF   Source address filter                  0 (disabled)
 * ETH_MACFFR_HPF   Hash or perfect filter                 0 (Only matching frames passed)
 * ETH_MACFFR_RA    Receive all                            0 (disabled)
 */

#define MACFFR_SET_BITS (ETH_MACFFR_PCF_PAUSE)

/* Clear the MACFCR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_MACFCR_FCB_BPA Bit 0: Flow control busy/back pressure activate
 * ETH_MACFCR_TFCE    Bit 1: Transmit flow control enable
 * ETH_MACFCR_RFCE    Bit 2: Receive flow control enable
 * ETH_MACFCR_UPFD    Bit 3: Unicast pause frame detect
 * ETH_MACFCR_PLT     Bits 4-5: Pause low threshold
 * ETH_MACFCR_ZQPD    Bit 7: Zero-quanta pause disable
 * ETH_MACFCR_PT      Bits 16-31: Pause time
 */

#define MACFCR_CLEAR_MASK \
  (ETH_MACFCR_FCB_BPA | ETH_MACFCR_TFCE | ETH_MACFCR_RFCE | ETH_MACFCR_UPFD | \
   ETH_MACFCR_PLT_MASK | ETH_MACFCR_ZQPD | ETH_MACFCR_PT_MASK)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_MACFCR_FCB_BPA Flow control busy/back pressure activate   0 (no pause control frame)
 * ETH_MACFCR_TFCE    Transmit flow control enable               0 (disabled)
 * ETH_MACFCR_RFCE    Receive flow control enable                0 (disabled)
 * ETH_MACFCR_UPFD    Unicast pause frame detect                 0 (disabled)
 * ETH_MACFCR_PLT     Pause low threshold                        0 (pause time - 4)
 * ETH_MACFCR_ZQPD    Zero-quanta pause disable                  1 (disabled)
 * ETH_MACFCR_PT      Pause time                                 0
 */

#define MACFCR_SET_MASK (ETH_MACFCR_PLT_M4 | ETH_MACFCR_ZQPD)

/* Clear the DMAOMR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_DMAOMR_SR     Bit 1:  Start/stop receive
 * TH_DMAOMR_OSF     Bit 2:  Operate on second frame
 * ETH_DMAOMR_RTC    Bits 3-4: Receive threshold control
 * ETH_DMAOMR_FUGF   Bit 6:  Forward undersized good frames
 * ETH_DMAOMR_FEF    Bit 7:  Forward error frames
 * ETH_DMAOMR_ST     Bit 13: Start/stop transmission
 * ETH_DMAOMR_TTC    Bits 14-16: Transmit threshold control
 * ETH_DMAOMR_FTF    Bit 20: Flush transmit FIFO
 * ETH_DMAOMR_TSF    Bit 21: Transmit store and forward
 * ETH_DMAOMR_DFRF   Bit 24: Disable flushing of received frames
 * ETH_DMAOMR_RSF    Bit 25: Receive store and forward
 * TH_DMAOMR_DTCEFD  Bit 26: Dropping of TCP/IP checksum error frames disable
 */

#define DMAOMR_CLEAR_MASK \
  (ETH_DMAOMR_SR | ETH_DMAOMR_OSF | ETH_DMAOMR_RTC_MASK | ETH_DMAOMR_FUGF | \
   ETH_DMAOMR_FEF | ETH_DMAOMR_ST | ETH_DMAOMR_TTC_MASK | ETH_DMAOMR_FTF | \
   ETH_DMAOMR_TSF | ETH_DMAOMR_DFRF | ETH_DMAOMR_RSF | ETH_DMAOMR_DTCEFD)

/* The following bits are set or left zero unconditionally in all modes.
 *
 * ETH_DMAOMR_SR     Start/stop receive                   0 (not running)
 * TH_DMAOMR_OSF     Operate on second frame              1 (enabled)
 * ETH_DMAOMR_RTC    Receive threshold control            0 (64 bytes)
 * ETH_DMAOMR_FUGF   Forward undersized good frames       0 (disabled)
 * ETH_DMAOMR_FEF    Forward error frames                 0 (disabled)
 * ETH_DMAOMR_ST     Start/stop transmission              0 (not running)
 * ETH_DMAOMR_TTC    Transmit threshold control           0 (64 bytes)
 * ETH_DMAOMR_FTF    Flush transmit FIFO                  0 (no flush)
 * ETH_DMAOMR_TSF    Transmit store and forward           Depends on CONFIG_STM32_ETH_HWCHECKSUM
 * ETH_DMAOMR_DFRF   Disable flushing of received frames  0 (enabled)
 * ETH_DMAOMR_RSF    Receive store and forward            Depends on CONFIG_STM32_ETH_HWCHECKSUM
 * TH_DMAOMR_DTCEFD  Dropping of TCP/IP checksum error    Depends on CONFIG_STM32_ETH_HWCHECKSUM
 *                   frames disable
 *
 * When the checksum offload feature is enabled, we need to enable the Store
 * and Forward mode: the store and forward guarantee that a whole frame is
 * stored in the FIFO, so the MAC can insert/verify the checksum, if the
 * checksum is OK the DMA can handle the frame otherwise the frame is dropped
 */
 
#if CONFIG_STM32_ETH_HWCHECKSUM
#  define DMAOMR_SET_MASK \
    (ETH_DMAOMR_OSF | ETH_DMAOMR_RTC_64 | ETH_DMAOMR_TTC_64 | \
     ETH_DMAOMR_TSF | ETH_DMAOMR_RSF)
#else
#  define DMAOMR_SET_MASK \
    (ETH_DMAOMR_OSF | ETH_DMAOMR_RTC_64 | ETH_DMAOMR_TTC_64 | \
     ETH_DMAOMR_DTCEFD)
#endif

/* Clear the DMABMR bits that will be setup during MAC initialization (or that
 * are cleared unconditionally).  Per the reference manual, all reserved bits
 * must be retained at their reset value.
 *
 * ETH_DMABMR_SR    Bit 0: Software reset
 * ETH_DMABMR_DA    Bit 1: DMA Arbitration
 * ETH_DMABMR_DSL   Bits 2-6: Descriptor skip length
 * ETH_DMABMR_EDFE  Bit 7: Enhanced descriptor format enable
 * ETH_DMABMR_PBL   Bits 8-13: Programmable burst length
 * ETH_DMABMR_RTPR  Bits 14-15: RX TX priority ratio
 * ETH_DMABMR_FB    Bit 16: Fixed burst
 * ETH_DMABMR_RDP   Bits 17-22: RX DMA PBL
 * ETH_DMABMR_USP   Bit 23: Use separate PBL
 * ETH_DMABMR_FPM   Bit 24: 4xPBL mode
 * ETH_DMABMR_AAB   Bit 25: Address-aligned beats
 * ETH_DMABMR_MB    Bit 26: Mixed burst (F2/F4 only)
 */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#define DMABMR_CLEAR_MASK \
  (ETH_DMABMR_SR | ETH_DMABMR_DA | ETH_DMABMR_DSL_MASK | ETH_DMABMR_EDFE | \
   ETH_DMABMR_PBL_MASK | ETH_DMABMR_RTPR_MASK | ETH_DMABMR_FB | ETH_DMABMR_RDP_MASK | \
   ETH_DMABMR_USP | ETH_DMABMR_FPM | ETH_DMABMR_AAB | ETH_DMABMR_MB)
#else
#define DMABMR_CLEAR_MASK \
  (ETH_DMABMR_SR | ETH_DMABMR_DA | ETH_DMABMR_DSL_MASK | ETH_DMABMR_EDFE | \
   ETH_DMABMR_PBL_MASK | ETH_DMABMR_RTPR_MASK | ETH_DMABMR_FB | ETH_DMABMR_RDP_MASK | \
   ETH_DMABMR_USP | ETH_DMABMR_FPM | ETH_DMABMR_AAB)
#endif

/* The following bits are set or left zero unconditionally in all modes.
 *
 *
 * ETH_DMABMR_SR    Software reset                     0 (no reset)
 * ETH_DMABMR_DA    DMA Arbitration                    0 (round robin)
 * ETH_DMABMR_DSL   Descriptor skip length             0
 * ETH_DMABMR_EDFE  Enhanced descriptor format enable  Depends on CONFIG_STM32_ETH_ENHANCEDDESC
 * ETH_DMABMR_PBL   Programmable burst length          32 beats
 * ETH_DMABMR_RTPR  RX TX priority ratio               2:1
 * ETH_DMABMR_FB    Fixed burst                        1 (enabled)
 * ETH_DMABMR_RDP   RX DMA PBL                         32 beats
 * ETH_DMABMR_USP   Use separate PBL                   1 (enabled)
 * ETH_DMABMR_FPM   4xPBL mode                         0 (disabled)
 * ETH_DMABMR_AAB   Address-aligned beats              1 (enabled)
 * ETH_DMABMR_MB    Mixed burst                        0 (disabled, F2/F4 only)
 */

#ifdef CONFIG_STM32_ETH_ENHANCEDDESC
#  define DMABMR_SET_MASK \
     (ETH_DMABMR_DSL(0) | ETH_DMABMR_PBL(32) | ETH_DMABMR_EDFE | ETH_DMABMR_RTPR_2TO1 | \
      ETH_DMABMR_FB | ETH_DMABMR_RDP(32) | ETH_DMABMR_USP | ETH_DMABMR_AAB)
#else
#  define DMABMR_SET_MASK \
     (ETH_DMABMR_DSL(0) | ETH_DMABMR_PBL(32) | ETH_DMABMR_RTPR_2TO1 | ETH_DMABMR_FB | \
      ETH_DMABMR_RDP(32) | ETH_DMABMR_USP | ETH_DMABMR_AAB)
#endif

/* Interrupt bit sets *******************************************************/
/* All interrupts in the normal and abnormal interrupt summary.  Early transmit
 * interrupt (ETI) is excluded from the abnormal set because it causes too
 * many interrupts and is not interesting.
 */

#define ETH_DMAINT_NORMAL \
  (ETH_DMAINT_TI | ETH_DMAINT_TBUI |ETH_DMAINT_RI | ETH_DMAINT_ERI)

#define ETH_DMAINT_ABNORMAL \
  (ETH_DMAINT_TPSI | ETH_DMAINT_TJTI | ETH_DMAINT_ROI | ETH_DMAINT_TUI | \
   ETH_DMAINT_RBUI | ETH_DMAINT_RPSI | ETH_DMAINT_RWTI | /* ETH_DMAINT_ETI | */ \
   ETH_DMAINT_FBEI)

/* Normal receive, transmit, error interrupt enable bit sets */

#define ETH_DMAINT_RECV_ENABLE    (ETH_DMAINT_NIS | ETH_DMAINT_RI)
#define ETH_DMAINT_XMIT_ENABLE    (ETH_DMAINT_NIS | ETH_DMAINT_TI)
#define ETH_DMAINT_XMIT_DISABLE   (ETH_DMAINT_TI)

#ifdef CONFIG_DEBUG_NET
#  define ETH_DMAINT_ERROR_ENABLE (ETH_DMAINT_AIS | ETH_DMAINT_ABNORMAL)
#else
#  define ETH_DMAINT_ERROR_ENABLE (0)
#endif

/* Helpers ******************************************************************/
/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((struct uip_eth_hdr *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The stm32_ethmac_s encapsulates all state information for a single hardware
 * interface
 */

struct stm32_ethmac_s
{
  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  uint8_t              mbps100 : 1; /* 100MBps operation (vs 10 MBps) */
  uint8_t              fduplex : 1; /* Full (vs. half) duplex */
  WDOG_ID              txpoll;      /* TX poll timer */
  WDOG_ID              txtimeout;   /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s  dev;         /* Interface understood by uIP */

  /* Used to track transmit and receive descriptors */

  struct eth_txdesc_s *txhead;      /* Next available TX descriptor */
  struct eth_rxdesc_s *rxhead;      /* Next available RX descriptor */

  struct eth_txdesc_s *txtail;      /* First "in_flight" TX descriptor */
  struct eth_rxdesc_s *rxcurr;      /* First RX descriptor of the segment */
  uint16_t             segments;    /* RX segment count */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */
  sq_queue_t           freeb;       /* The free buffer list */

  /* Descriptor allocations */

  struct eth_rxdesc_s rxtable[CONFIG_STM32_ETH_NRXDESC];
  struct eth_txdesc_s txtable[CONFIG_STM32_ETH_NTXDESC];

  /* Buffer allocations */

  uint8_t rxbuffer[CONFIG_STM32_ETH_NRXDESC*CONFIG_STM32_ETH_BUFSIZE];
  uint8_t alloc[STM32_ETH_NFREEBUFFERS*CONFIG_STM32_ETH_BUFSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_ethmac_s g_stm32ethmac[STM32_NETHERNET];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_STM32_ETHMAC_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint32_t val, uint32_t addr);
static void stm32_checksetup(void);
#else
# define stm32_getreg(addr)      getreg32(addr)
# define stm32_putreg(val,addr)  putreg32(val,addr)
# define stm32_checksetup()
#endif

/* Free buffer management */

static void stm32_initbuffer(FAR struct stm32_ethmac_s *priv);
static inline uint8_t *stm32_allocbuffer(FAR struct stm32_ethmac_s *priv);
static inline void stm32_freebuffer(FAR struct stm32_ethmac_s *priv, uint8_t *buffer);
static inline bool stm32_isfreebuffer(FAR struct stm32_ethmac_s *priv);

/* Common TX logic */

static int  stm32_transmit(FAR struct stm32_ethmac_s *priv);
static int  stm32_uiptxpoll(struct uip_driver_s *dev);
static void stm32_dopoll(FAR struct stm32_ethmac_s *priv);

/* Interrupt handling */

static void stm32_enableint(FAR struct stm32_ethmac_s *priv, uint32_t ierbit);
static void stm32_disableint(FAR struct stm32_ethmac_s *priv, uint32_t ierbit);

static void stm32_freesegment(FAR struct stm32_ethmac_s *priv,
                              FAR struct eth_rxdesc_s *rxfirst, int segments);
static int  stm32_recvframe(FAR struct stm32_ethmac_s *priv);
static void stm32_receive(FAR struct stm32_ethmac_s *priv);
static void stm32_freeframe(FAR struct stm32_ethmac_s *priv);
static void stm32_txdone(FAR struct stm32_ethmac_s *priv);
static int  stm32_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void stm32_polltimer(int argc, uint32_t arg, ...);
static void stm32_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int  stm32_ifup(struct uip_driver_s *dev);
static int  stm32_ifdown(struct uip_driver_s *dev);
static int  stm32_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int  stm32_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int  stm32_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/* Descriptor Initialization */

static void stm32_txdescinit(FAR struct stm32_ethmac_s *priv);
static void stm32_rxdescinit(FAR struct stm32_ethmac_s *priv);

/* PHY Initialization */

static int  stm32_phyread(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t *value);
static int  stm32_phywrite(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t value);
#ifdef CONFIG_PHY_DM9161
static inline int stm32_dm9161(FAR struct stm32_ethmac_s *priv);
#endif
static int  stm32_phyinit(FAR struct stm32_ethmac_s *priv);

/* MAC/DMA Initialization */

#ifdef CONFIG_STM32_MII
static inline void stm32_selectmii(void);
#endif
#ifdef CONFIG_STM32_RMII
static inline void stm32_selectrmii(void);
#endif
static inline void stm32_ethgpioconfig(FAR struct stm32_ethmac_s *priv);
static void stm32_ethreset(FAR struct stm32_ethmac_s *priv);
static int  stm32_macconfig(FAR struct stm32_ethmac_s *priv);
static void stm32_macaddress(FAR struct stm32_ethmac_s *priv);
static int  stm32_macenable(FAR struct stm32_ethmac_s *priv);
static int  stm32_ethconfig(FAR struct stm32_ethmac_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_ETHMAC_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t stm32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       preval   = val;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_ETHMAC_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: stm32_checksetup
 *
 * Description:
 *   Show the state of critical configuration registers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_ETHMAC_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_checksetup(void)
{
}
#endif

/****************************************************************************
 * Function: stm32_initbuffer
 *
 * Description:
 *   Initialize the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during early driver initialization before Ethernet interrupts
 *   are enabled.
 *
 ****************************************************************************/

static void stm32_initbuffer(FAR struct stm32_ethmac_s *priv)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = priv->alloc;
       i < STM32_ETH_NFREEBUFFERS;
       i++, buffer += CONFIG_STM32_ETH_BUFSIZE)
    {
      sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: stm32_allocbuffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline uint8_t *stm32_allocbuffer(FAR struct stm32_ethmac_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: stm32_freebuffer
 *
 * Description:
 *   Return a buffer to the free buffer list.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static inline void stm32_freebuffer(FAR struct stm32_ethmac_s *priv, uint8_t *buffer)
{
  /* Free the buffer by adding it to to the end of the free buffer list */

  sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: stm32_isfreebuffer
 *
 * Description:
 *   Return TRUE if the free buffer list is not empty.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   True if there are one or more buffers in the free buffer list;
 *   false if the free buffer list is empty
 *
 * Assumptions:
 *   None.
 *
 ****************************************************************************/

static inline bool stm32_isfreebuffer(FAR struct stm32_ethmac_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Function: stm32_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_transmit(FAR struct stm32_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  struct eth_txdesc_s *txfirst;

  /* The internal (optimal) uIP buffer size may be configured to be larger
   * than the Ethernet buffer size.
   */

#if OPTIMAL_ETH_BUFSIZE > CONFIG_STM32_ETH_BUFSIZE
  uint8_t *buffer;
  int bufcount;
  int lastsize;
  int i;
#endif

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  txdesc  = priv->txhead;
  txfirst = txdesc;

  nllvdbg("d_len: %d d_buf: %p txhead: %p tdes0: %08x\n",
          priv->dev.d_len, priv->dev.d_buf, txdesc, txdesc->tdes0);

  DEBUGASSERT(txdesc && (txdesc->tdes0 & ETH_TDES0_OWN) == 0);

  /* Is the size to be sent greater than the size of the Ethernet buffer? */

  DEBUGASSERT(priv->dev.d_len > 0 && priv->dev.d_buf != NULL);

#if OPTIMAL_ETH_BUFSIZE > CONFIG_STM32_ETH_BUFSIZE
  if (priv->dev.d_len > CONFIG_STM32_ETH_BUFSIZE)
    {
      /* Yes... how many buffers will be need to send the packet? */

      bufcount = (priv->dev.d_len + (CONFIG_STM32_ETH_BUFSIZE-1)) / CONFIG_STM32_ETH_BUFSIZE;
      lastsize = priv->dev.d_len - (bufcount - 1) * CONFIG_STM32_ETH_BUFSIZE;

      nllvdbg("bufcount: %d lastsize: %d\n", bufcount, lastsize);

      /* Set the first segment bit in the first TX descriptor */

      txdesc->tdes0 |= ETH_TDES0_FS;

      /* Set up all but the last TX descriptor */

      buffer = priv->dev.d_buf;

      for (i = 0; i < bufcount; i++)
        {
          /* This could be a normal event but the design does not handle it */

          DEBUGASSERT((txdesc->tdes0 & ETH_TDES0_OWN) == 0);

          /* Set the Buffer1 address pointer */

          txdesc->tdes2 = (uint32_t)buffer;

          /* Set the buffer size in all TX descriptors */

          if (i == (bufcount-1))
            {
              /* This is the last segment.  Set the last segment bit in the
               * last TX descriptor and ask for an interrupt when this
               * segment transfer completes.
               */

              txdesc->tdes0 |= (ETH_TDES0_LS | ETH_TDES0_IC);

              /* This segement is, most likely, of fractional buffersize */

              txdesc->tdes1  = lastsize;
              buffer        += lastsize;
            }
          else
            {
              /* This is not the last segment.  We don't want an interrupt
               * when this segment transfer completes.
               */

              txdesc->tdes0 &= ~ETH_TDES0_IC;

              /* The size of the transfer is the whole buffer */

              txdesc->tdes1  = CONFIG_STM32_ETH_BUFSIZE;
              buffer        += CONFIG_STM32_ETH_BUFSIZE;
            }

          /* Give the descriptor to DMA */

          txdesc->tdes0 |= ETH_TDES0_OWN;
          txdesc         = (struct eth_txdesc_s *)txdesc->tdes3;
        }
    }
  else
#endif
    {
      /* The single descriptor is both the first and last segment.  And we do
       * want an interrupt when the transfer completes.
       */

      txdesc->tdes0 |= (ETH_TDES0_FS | ETH_TDES0_LS | ETH_TDES0_IC);

      /* Set frame size */

      DEBUGASSERT(priv->dev.d_len <= CONFIG_NET_BUFSIZE);
      txdesc->tdes1 = priv->dev.d_len;

      /* Set the Buffer1 address pointer */

      txdesc->tdes2 = (uint32_t)priv->dev.d_buf;

      /* Set OWN bit of the TX descriptor tdes0.  This gives the buffer to
       * Ethernet DMA
       */

      txdesc->tdes0 |= ETH_TDES0_OWN;

      /* Point to the next available TX descriptor */

      txdesc = (struct eth_txdesc_s *)txdesc->tdes3;
    }

  /* Remember where we left off in the TX descriptor chain */

  priv->txhead = txdesc;

  /* Detach the buffer from priv->dev structure.  That buffer is now
   * "in-flight".
   */

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  /* If there is no other TX buffer, in flight, then remember the location
   * of the TX descriptor.  This is the location to check for TX done events.
   */

  if (!priv->txtail)
    {
      DEBUGASSERT(priv->inflight == 0);
      priv->txtail = txfirst;
    }

  /* Increment the number of TX transfer in-flight */

  priv->inflight++;

  nllvdbg("txhead: %p txtail: %p inflight: %d\n",
          priv->txhead, priv->txtail, priv->inflight);

  /* If all TX descriptors are in-flight, then we have to disable receive interrupts
   * too.  This is because receive events can trigger more un-stoppable transmit
   * events.
   */

  if (priv->inflight >= CONFIG_STM32_ETH_NTXDESC)
    {
      stm32_disableint(priv, ETH_DMAINT_RI);
    }

  /* Check if the TX Buffer unavailable flag is set */

  if ((stm32_getreg(STM32_ETH_DMASR) & ETH_DMAINT_TBUI) != 0)
    {
      /* Clear TX Buffer unavailable flag */

      stm32_putreg(ETH_DMAINT_TBUI, STM32_ETH_DMASR);

      /* Resume DMA transmission */

      stm32_putreg(0, STM32_ETH_DMATPDR);
    }

  /* Enable TX interrupts */

  stm32_enableint(priv, ETH_DMAINT_TI);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, STM32_TXTIMEOUT, stm32_txtimeout, 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: stm32_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      uip_arp_out(&priv->dev);
      stm32_transmit(priv);
      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

      /* Check if the next TX descriptor is owned by the Ethernet DMA or CPU.  We
       * cannot perform the TX poll if we are unable to accept another packet for
       * transmission.
       *
       * In a race condition, ETH_TDES0_OWN may be cleared BUT still not available
       * because stm32_freeframe() has not yet run.  If stm32_freeframe() has run,
       * the buffer1 pointer (tdes2) will be nullified (and inflight should be <
       * CONFIG_STM32_ETH_NTXDESC).
       */

      if ((priv->txhead->tdes0 & ETH_TDES0_OWN) != 0 ||
           priv->txhead->tdes2 != 0)
        {
          /* We have to terminate the poll if we have no more descriptors
           * available for another transfer.
           */

          return -EBUSY;
        }

      /* We have the descriptor, we can continue the poll. Allocate a new
       * buffer for the poll.
       */

      dev->d_buf = stm32_allocbuffer(priv);

      /* We can't continue the poll if we have no buffers */

      if (dev->d_buf == NULL)
        {
          /* Terminate the poll. */

          return -ENOMEM;
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: stm32_dopoll
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the the received frame.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_dopoll(FAR struct stm32_ethmac_s *priv)
{
  FAR struct uip_driver_s *dev = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or
   * CPU.  We cannot perform the TX poll if we are unable to accept
   * another packet for transmission.
   *
   * In a race condition, ETH_TDES0_OWN may be cleared BUT still not available
   * because stm32_freeframe() has not yet run.  If stm32_freeframe() has run,
   * the buffer1 pointer (tdes2) will be nullified (and inflight should be <
   * CONFIG_STM32_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ETH_TDES0_OWN) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then poll uIP for new XMIT data.
       * Allocate a buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = stm32_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          (void)uip_poll(dev, stm32_uiptxpoll);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              stm32_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }
}

/****************************************************************************
 * Function: stm32_enableint
 *
 * Description:
 *   Enable a "normal" interrupt
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_enableint(FAR struct stm32_ethmac_s *priv, uint32_t ierbit)
{
  uint32_t regval;

  /* Enable the specified "normal" interrupt */

  regval  = stm32_getreg(STM32_ETH_DMAIER);
  regval |= (ETH_DMAINT_NIS | ierbit);
  stm32_putreg(regval, STM32_ETH_DMAIER);
}

/****************************************************************************
 * Function: stm32_disableint
 *
 * Description:
 *   Disable a normal interrupt.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_disableint(FAR struct stm32_ethmac_s *priv, uint32_t ierbit)
{
  uint32_t regval;

  /* Disable the "normal" interrupt */

  regval  = stm32_getreg(STM32_ETH_DMAIER);
  regval &= ~ierbit;

  /* Are all "normal" interrupts now disabled? */

  if ((regval & ETH_DMAINT_NORMAL) == 0)
    {
      /* Yes.. disable normal interrupts */
 
      regval &= ~ETH_DMAINT_NIS;
    }

  stm32_putreg(regval, STM32_ETH_DMAIER);
}

/****************************************************************************
 * Function: stm32_freesegment
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors to the the received frame.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_freesegment(FAR struct stm32_ethmac_s *priv,
                              FAR struct eth_rxdesc_s *rxfirst, int segments)
{
  struct eth_rxdesc_s *rxdesc;
  int i;

  nllvdbg("rxfirst: %p segments: %d\n", rxfirst, segments);

  /* Set OWN bit in RX descriptors.  This gives the buffers back to DMA */

  rxdesc = rxfirst;
  for (i = 0; i < segments; i++)
    {
      rxdesc->rdes0 = ETH_RDES0_OWN;
      rxdesc = (struct eth_rxdesc_s *)rxdesc->rdes3;
    }

  /* Reset the segment managment logic */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Check if the RX Buffer unavailable flag is set */

  if ((stm32_getreg(STM32_ETH_DMASR) & ETH_DMAINT_RBUI) != 0)  
    {
      /* Clear RBUS Ethernet DMA flag */

      stm32_putreg(ETH_DMAINT_RBUI, STM32_ETH_DMASR);
      
      /* Resume DMA reception */

      stm32_putreg(0, STM32_ETH_DMARPDR);
    }
}

/****************************************************************************
 * Function: stm32_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors of the received frame.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_recvframe(FAR struct stm32_ethmac_s *priv)
{
  struct eth_rxdesc_s *rxdesc;
  struct eth_rxdesc_s *rxcurr;
  uint8_t *buffer;
  int i;

  nllvdbg("rxhead: %p rxcurr: %p segments: %d\n",
          priv->rxhead, priv->rxcurr, priv->segments);

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!stm32_isfreebuffer(priv))
    {
      nlldbg("No free buffers\n");
      return -ENOMEM;
    }

  /* Scan descriptors owned by the CPU.  Scan until:
   *
   *   1) We find a descriptor still owned by the DMA,
   *   2) We have examined all of the RX descriptors, or
   *   3) All of the TX descriptors are in flight.
   *
   * This last case is obscure.  It is due to that fact that each packet
   * that we receive can generate an unstoppable transmisson.  So we have
   * to stop receiving when we can not longer transmit.  In this case, the
   * transmit logic should also have disabled further RX interrupts.
   */

  rxdesc = priv->rxhead;
  for (i = 0;
       (rxdesc->rdes0 & ETH_RDES0_OWN) == 0 &&
        i < CONFIG_STM32_ETH_NRXDESC &&
        priv->inflight < CONFIG_STM32_ETH_NTXDESC;
       i++)
    {
      /* Check if this is the first segment in the frame */

      if ((rxdesc->rdes0 & ETH_RDES0_FS) != 0 &&
          (rxdesc->rdes0 & ETH_RDES0_LS) == 0)
        {
          priv->rxcurr   = rxdesc;
          priv->segments = 1;   
        }
    
      /* Check if this is an intermediate segment in the frame */

      else if (((rxdesc->rdes0 & ETH_RDES0_LS) == 0)&&
               ((rxdesc->rdes0 & ETH_RDES0_FS) == 0))
        {
          priv->segments++;
        }

      /* Otherwise, it is the last segment in the frame */

      else
        { 
          priv->segments++;

          /* Check if the there is only one segment in the frame */

          if (priv->segments == 1)
            {
              rxcurr = rxdesc;
            }
          else
            {
              rxcurr = priv->rxcurr;
            }

          nllvdbg("rxhead: %p rxcurr: %p segments: %d\n",
              priv->rxhead, priv->rxcurr, priv->segments);

          /* Check if any errors are reported in the frame */

          if ((rxdesc->rdes0 & ETH_RDES0_ES) == 0)
            {
              struct uip_driver_s *dev = &priv->dev;

              /* Get the Frame Length of the received packet: substruct 4
               * bytes of the CRC
               */

              dev->d_len = ((rxdesc->rdes0 & ETH_RDES0_FL_MASK) >> ETH_RDES0_FL_SHIFT) - 4;

              /* Get a buffer from the free list.  We don't even check if
               * this is successful because we already assure the the free
               * list is not empty above.
               */

              buffer = stm32_allocbuffer(priv);

              /* Take the buffer from the RX descriptor of the first free
               * segment, put it into the uIP device structure, then replace
               * the buffer in the RX descriptor with the newly allocated
               * buffer.
               */

              DEBUGASSERT(dev->d_buf == NULL);
              dev->d_buf    = (uint8_t*)rxcurr->rdes2;
              rxcurr->rdes2 = (uint32_t)buffer;

              /* Return success, remebering where we should re-start scanning
               * and resetting the segment scanning logic
               */

              priv->rxhead   = (struct eth_rxdesc_s*)rxdesc->rdes3;
              stm32_freesegment(priv, rxcurr, priv->segments);

              nllvdbg("rxhead: %p d_buf: %p d_len: %d\n",
                      priv->rxhead, dev->d_buf, dev->d_len);

              return OK;
            }
          else
            {
              /* Drop the frame that contains the errors, reset the segment
               * scanning logic, and continue scanning with the next frame.
               */

              nlldbg("DROPPED: RX descriptor errors: %08x\n", rxdesc->rdes0);
              stm32_freesegment(priv, rxcurr, priv->segments);
            }
        }

      /* Try the next descriptor */

      rxdesc = (struct eth_rxdesc_s*)rxdesc->rdes3;
    }

  /* We get here after all of the descriptors have been scanned or when rxdesc points
   * to the first descriptor owned by the DMA.  Remember where we left off.
   */

  priv->rxhead = rxdesc;

  nllvdbg("rxhead: %p rxcurr: %p segments: %d\n",
          priv->rxhead, priv->rxcurr, priv->segments);

  return -EAGAIN; 
}

/****************************************************************************
 * Function: stm32_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_receive(FAR struct stm32_ethmac_s *priv)
{
  struct uip_driver_s *dev = &priv->dev;

  /* Loop while while stm32_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (stm32_recvframe(priv) == OK)
    {
      /* Check if the packet is a valid size for the uIP buffer configuration
       * (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_BUFSIZE)
        {
          nlldbg("DROPPED: Too big: %d\n", dev->d_len);
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      else if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      else if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          nllvdbg("IP frame\n");

          /* Handle ARP on input then give the IP packet to uIP */

          uip_arp_ipin(&priv->dev);
          uip_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
           {
             uip_arp_out(&priv->dev);
             stm32_transmit(priv);
           }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          nllvdbg("ARP frame\n");

          /* Handle ARP packet */

          uip_arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              stm32_transmit(priv);
            }
        }
      else
        {
          nlldbg("DROPPED: Unknown type: %04x\n", BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          stm32_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Function: stm32_freeframe
 *
 * Description:
 *   Scans the TX descriptors and frees the buffers of completed TX transfers.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_freeframe(FAR struct stm32_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  int i;

  nllvdbg("txhead: %p txtail: %p inflight: %d\n",
          priv->txhead, priv->txtail, priv->inflight);

  /* Scan for "in-flight" descriptors owned by the CPU */

  txdesc = priv->txtail;
  if (txdesc)
    {
      DEBUGASSERT(priv->inflight > 0);

      for (i = 0; (txdesc->tdes0 & ETH_TDES0_OWN) == 0; i++)
        {
          /* There should be a buffer assigned to all in-flight
           * TX descriptors.
           */

          nllvdbg("txtail: %p tdes0: %08x tdes2: %08x tdes3: %08x\n",
                  txdesc, txdesc->tdes0, txdesc->tdes2, txdesc->tdes3);

          DEBUGASSERT(txdesc->tdes2 != 0);

          /* Check if this is the first segment of a TX frame. */

          if ((txdesc->tdes0 & ETH_TDES0_FS) != 0)
            {
              /* Yes.. Free the buffer */

              stm32_freebuffer(priv, (uint8_t*)txdesc->tdes2);
            }

          /* In any event, make sure that TDES2 is nullified. */

          txdesc->tdes2 = 0;

          /* Check if this is the last segement of a TX frame */

          if ((txdesc->tdes0 & ETH_TDES0_FS) != 0)
            {
              /* Yes.. Decrement the number of frames "in-flight". */

              priv->inflight--;

              /* If all of the TX descriptors were in-flight, then RX interrupts
               * may have been disabled... we can re-enable them now.
               */

              stm32_enableint(priv, ETH_DMAINT_RI);

              /* If there are no more frames in-flight, then bail. */

              if (priv->inflight <= 0)
                {
                  priv->txtail   = NULL;
                  priv->inflight = 0;
                  return;
                }
            }

          /* Try the next descriptor in the TX chain */

          txdesc = (struct eth_txdesc_s*)txdesc->tdes3;
        }

      /* We get here if (1) there are still frames "in-flight". Remember
       * where we left off.
       */

      priv->txtail = txdesc;

      nllvdbg("txhead: %p txtail: %p inflight: %d\n",
              priv->txhead, priv->txtail, priv->inflight);
    }
}

/****************************************************************************
 * Function: stm32_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void stm32_txdone(FAR struct stm32_ethmac_s *priv)
{
  DEBUGASSERT(priv->txtail != NULL);

  /* Scan the TX desciptor change, returning buffers to free list */

  stm32_freeframe(priv);

  /* If no further xmits are pending, then cancel the TX timeout */

  if (priv->inflight <= 0)
    {
      wd_cancel(priv->txtimeout);

      /* And disable further TX interrupts. */

      stm32_disableint(priv, ETH_DMAINT_TI);
    }

  /* Then poll uIP for new XMIT data */

  stm32_dopoll(priv);
}

/****************************************************************************
 * Function: stm32_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_interrupt(int irq, FAR void *context)
{
  register FAR struct stm32_ethmac_s *priv = &g_stm32ethmac[0];
  uint32_t dmasr;

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */

  dmasr = stm32_getreg(STM32_ETH_DMASR);

  /* Mask only enabled interrupts.  This depends on the fact that the interrupt
   * related bits (0-16) correspond in these two registers.
   */

  dmasr &= stm32_getreg(STM32_ETH_DMAIER);

  /* Check if there are pending "normal" interrupts */

  if ((dmasr & ETH_DMAINT_NIS) != 0)
    {
      /* Yes.. Check if we received an incoming packet, if so, call
       * stm32_receive()
       */

      if ((dmasr & ETH_DMAINT_RI) != 0)
        {
          /* Clear the pending receive interrupt */

          stm32_putreg(ETH_DMAINT_RI, STM32_ETH_DMASR);

          /* Handle the received package */

          stm32_receive(priv);
        }

      /* Check if a packet transmission just completed.  If so, call
       * stm32_txdone(). This may disable further TX interrupts if there
       * are no pending tansmissions.
       */

      if ((dmasr & ETH_DMAINT_TI) != 0)
        {
          /* Clear the pending receive interrupt */

          stm32_putreg(ETH_DMAINT_TI, STM32_ETH_DMASR);

          /* Check if there are pending transmissions */

          stm32_txdone(priv);
        }

      /* Clear the pending normal summary interrupt */

      stm32_putreg(ETH_DMAINT_NIS, STM32_ETH_DMASR);
    }

  /* Handle error interrupt only if CONFIG_DEBUG_NET is eanbled */

#ifdef CONFIG_DEBUG_NET

  /* Check if there are pending "anormal" interrupts */

  if ((dmasr & ETH_DMAINT_AIS) != 0)
    {
      /* Just let the user know what happened */

      nlldbg("Abormal event(s): %08x\n", dmasr);

      /* Clear all pending abnormal events */

      stm32_putreg(ETH_DMAINT_ABNORMAL, STM32_ETH_DMASR);

      /* Clear the pending abnormal summary interrupt */

      stm32_putreg(ETH_DMAINT_AIS, STM32_ETH_DMASR);
    }
#endif
  return OK;
}

/****************************************************************************
 * Function: stm32_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void stm32_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)arg;

  nlldbg("Timeout!\n");

  /* Then reset the hardware.  Just take the interface down, then back
   * up again.
   */

  stm32_ifdown(&priv->dev);
  stm32_ifup(&priv->dev);

  /* Then poll uIP for new XMIT data */

  stm32_dopoll(priv);
}

/****************************************************************************
 * Function: stm32_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void stm32_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)arg;
  FAR struct uip_driver_s   *dev  = &priv->dev;

  /* Check if the next TX descriptor is owned by the Ethernet DMA or CPU.  We
   * cannot perform the timer poll if we are unable to accept another packet
   * for transmission.  Hmmm.. might be bug here.  Does this mean if there is
   * a transmit in progress, we will miss TCP time state updates?
   *
   * In a race condition, ETH_TDES0_OWN may be cleared BUT still not available
   * because stm32_freeframe() has not yet run.  If stm32_freeframe() has run,
   * the buffer1 pointer (tdes2) will be nullified (and inflight should be <
   * CONFIG_STM32_ETH_NTXDESC).
   */

  if ((priv->txhead->tdes0 & ETH_TDES0_OWN) == 0 &&
       priv->txhead->tdes2 == 0)
    {
      /* If we have the descriptor, then perform the timer poll.  Allocate a
       * buffer for the poll.
       */

      DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);
      dev->d_buf = stm32_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          /* Update TCP timing states and poll uIP for new XMIT data. 
           */

          (void)uip_timer(dev, stm32_uiptxpoll, STM32_POLLHSEC);

          /* We will, most likely end up with a buffer to be freed.  But it
           * might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              DEBUGASSERT(dev->d_len == 0);
              stm32_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
    }

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, STM32_WDDELAY, stm32_polltimer, 1, arg);
}

/****************************************************************************
 * Function: stm32_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided 
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ifup(struct uip_driver_s *dev)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)dev->d_private;
  int ret;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Configure the Ethernet interface for DMA operation. */

  ret = stm32_ethconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, STM32_WDDELAY, stm32_polltimer, 1, (uint32_t)priv);

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  up_enable_irq(STM32_IRQ_ETH);

  stm32_checksetup();
  return OK;
}

/****************************************************************************
 * Function: stm32_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ifdown(struct uip_driver_s *dev)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)dev->d_private;
  irqstate_t flags;

  ndbg("Taking the network down\n");

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(STM32_IRQ_ETH);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the stm32_ifup() always
   * successfully brings the interface back up.
   */

  stm32_ethreset(priv);

  /* Mark the device "down" */

  priv->ifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: stm32_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a 
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int stm32_txavail(struct uip_driver_s *dev)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)dev->d_private;
  irqstate_t flags;

  nllvdbg("ifup: %d\n", priv->ifup);

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->ifup)
    {
      /* Poll uIP for new XMIT data */

      stm32_dopoll(priv);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: stm32_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int stm32_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)dev->d_private;

  nllvdbg("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast routing table */
  /* Add the MAC address to the hardware multicast routing table */
#error "Missing logic"

  return OK;
}
#endif

/****************************************************************************
 * Function: stm32_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int stm32_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct stm32_ethmac_s *priv = (FAR struct stm32_ethmac_s *)dev->d_private;

  nllvdbg("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Add the MAC address to the hardware multicast routing table */
#error "Missing logic"

  return OK;
}
#endif

/****************************************************************************
 * Function: stm32_txdescinit
 *
 * Description:
 *   Initializes the DMA TX descriptors in chain mode.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_txdescinit(FAR struct stm32_ethmac_s *priv)
{
  struct eth_txdesc_s *txdesc;
  int i;
  
  /* priv->txhead will point to the first, available TX descriptor in the chain.
   * Set the priv->txhead pointer to the first descriptor in the table.
   */

  priv->txhead = priv->txtable;

  /* priv->txtail will point to the first segment of the oldest pending
   * "in-flight" TX transfer.  NULL means that there are no active TX
   * transfers.
   */

   priv->txtail   = NULL;
   priv->inflight = 0;

  /* Initialize each TX descriptor */   

  for (i = 0; i < CONFIG_STM32_ETH_NTXDESC; i++)
    {
      txdesc = &priv->txtable[i];

      /* Set Second Address Chained bit */

      txdesc->tdes0 = ETH_TDES0_TCH;  
       
#ifdef CHECKSUM_BY_HARDWARE
      /* Enable the checksum insertion for the TX frames */

      txdesc->tdes0 |= ETH_TDES0_CIC_ALL;
#endif

      /* Clear Buffer1 address pointer (buffers will be assigned as they
       * are used)
       */

      txdesc->tdes2 = 0;
    
      /* Initialize the next descriptor with the Next Descriptor Polling Enable */

      if (i < (CONFIG_STM32_ETH_NTXDESC-1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          txdesc->tdes3 = (uint32_t)&priv->txtable[i+1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          txdesc->tdes3 = (uint32_t)priv->txtable;  
        }
    }

  /* Set Transmit Desciptor List Address Register */

  stm32_putreg((uint32_t)priv->txtable, STM32_ETH_DMATDLAR);
}

/****************************************************************************
 * Function: stm32_rxdescinit
 *
 * Description:
 *   Initializes the DMA RX descriptors in chain mode.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_rxdescinit(FAR struct stm32_ethmac_s *priv)
{
  struct eth_rxdesc_s *rxdesc;
  int i;
  
  /* priv->rxhead will point to the first,  RX descriptor in the chain.
   * This will be where we receive the first incomplete frame.
   */

  priv->rxhead = priv->rxtable;

  /* If we accumulate the frame in segments, priv->rxcurr points to the
   * RX descriptor of the first segment in the current TX frame.
   */

  priv->rxcurr   = NULL;
  priv->segments = 0;

  /* Initialize each TX descriptor */   

  for (i = 0; i < CONFIG_STM32_ETH_NRXDESC; i++)
    {
      rxdesc = &priv->rxtable[i];

      /* Set Own bit of the RX descriptor rdes0 */

      rxdesc->rdes0 = ETH_RDES0_OWN;

      /* Set Buffer1 size and Second Address Chained bit and enabled DMA
       * RX desc receive interrupt
       */

      rxdesc->rdes1 = ETH_RDES1_RCH | (uint32_t)CONFIG_STM32_ETH_BUFSIZE;  

      /* Set Buffer1 address pointer */

      rxdesc->rdes2 = (uint32_t)&priv->rxbuffer[i*CONFIG_STM32_ETH_BUFSIZE];
    
      /* Initialize the next descriptor with the Next Descriptor Polling Enable */

      if (i < (CONFIG_STM32_ETH_NRXDESC-1))
        {
          /* Set next descriptor address register with next descriptor base
           * address
           */

          rxdesc->rdes3 = (uint32_t)&priv->rxtable[i+1];
        }
      else
        {
          /* For last descriptor, set next descriptor address register equal
           * to the first descriptor base address
           */

          rxdesc->rdes3 = (uint32_t)priv->rxtable;  
        }
    }

  /* Set Receive Descriptor List Address Register */

  stm32_putreg((uint32_t)priv->rxtable, STM32_ETH_DMARDLAR);
}

/****************************************************************************
 * Function: stm32_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_phyread(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t *value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MACMIIAR register, preserving CSR Clock Range CR[2:0] bits */

  regval  = stm32_getreg(STM32_ETH_MACMIIAR);
  regval &= ETH_MACMIIAR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the buy bit.
   * the  ETH_MACMIIAR_MW is clear, indicating a read operation.
   */

  regval |= (((uint32_t)phydevaddr << ETH_MACMIIAR_PA_SHIFT) & ETH_MACMIIAR_PA_MASK);
  regval |= (((uint32_t)phyregaddr << ETH_MACMIIAR_MR_SHIFT) & ETH_MACMIIAR_MR_MASK);
  regval |= ETH_MACMIIAR_MB;

  stm32_putreg(regval, STM32_ETH_MACMIIAR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_READ_TIMEOUT; timeout++)
    {
      if ((stm32_getreg(STM32_ETH_MACMIIAR) & ETH_MACMIIAR_MB) == 0)
        {
          *value = (uint16_t)stm32_getreg(STM32_ETH_MACMIIDR);
          return OK;
        }
    }

  ndbg("MII transfer timed out: phydevaddr: %04x phyregaddr: %04x\n",
       phydevaddr, phyregaddr);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: stm32_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Parameters:
 *   phydevaddr - The PHY device address
 *   phyregaddr - The PHY register address
 *   value - The 16-bit value to write to the PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_phywrite(uint16_t phydevaddr, uint16_t phyregaddr, uint16_t value)
{
  volatile uint32_t timeout;
  uint32_t regval;

  /* Configure the MACMIIAR register, preserving CSR Clock Range CR[2:0] bits */

  regval  = stm32_getreg(STM32_ETH_MACMIIAR);
  regval &= ETH_MACMIIAR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the  ETH_MACMIIAR_MW is set, indicating a write operation.
   */

  regval |= (((uint32_t)phydevaddr << ETH_MACMIIAR_PA_SHIFT) & ETH_MACMIIAR_PA_MASK);
  regval |= (((uint32_t)phyregaddr << ETH_MACMIIAR_MR_SHIFT) & ETH_MACMIIAR_MR_MASK);
  regval |= (ETH_MACMIIAR_MB | ETH_MACMIIAR_MW);

  /* Write the value into the MACIIDR register before setting the new MACMIIAR
   * register value.
   */

  stm32_putreg(value, STM32_ETH_MACMIIDR);
  stm32_putreg(regval, STM32_ETH_MACMIIAR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < PHY_WRITE_TIMEOUT; timeout++)
    {
      if ((stm32_getreg(STM32_ETH_MACMIIAR) & ETH_MACMIIAR_MB) == 0)
        {
          return OK;
        }
    }

  ndbg("MII transfer timed out: phydevaddr: %04x phyregaddr: %04x value: %04x\n",
       phydevaddr, phyregaddr, value);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: stm32_dm9161
 *
 * Description:
 *   Special workaround for the Davicom DM9161 PHY is required.  On power,
 *   up, the PHY is not usually configured correctly but will work after
 *   a powered-up reset.  This is really a workaround for some more
 *   fundamental issue with the PHY clocking initialization, but the
 *   root cause has not been studied (nor will it be with this workaround).
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PHY_DM9161
static inline int stm32_dm9161(FAR struct stm32_ethmac_s *priv)
{
  uint16_t phyval;
  int ret;

  /* Read the PHYID1 register;  A failure to read the PHY ID is one
   * indication that check if the DM9161 PHY CHIP is not ready.
   */

  ret = stm32_phyread(CONFIG_STM32_PHYADDR, MII_PHYID1, &phyval);
  if (ret < 0)
    {
      ndbg("Failed to read the PHY ID1: %d\n", ret);
      return ret;
    }

  /* If we failed to read the PHY ID1 register, the reset the MCU to recover */

  else if (phyval == 0xffff)
    {
      up_systemreset();
    }

  nvdbg("PHY ID1: 0x%04X\n", phyval);

  /* Now check the "DAVICOM Specified Configuration Register (DSCR)", Register 16 */

  ret = stm32_phyread(CONFIG_STM32_PHYADDR, 16, &phyval);
  if (ret < 0)
    {
      ndbg("Failed to read the PHY Register 0x10: %d\n", ret);
      return ret;
    }

  /* Bit 8 of the DSCR register is zero, the the DM9161 has not selected RMII.
   * If RMII is not selected, then reset the MCU to recover.
   */
 
  else if ((phyval & (1 << 8)) == 0)
    {
      up_systemreset();
    }

  return OK;
}
#endif

/****************************************************************************
 * Function: stm32_phyinit
 *
 * Description:
 *  Configure the PHY and determine the link speed/duplex.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_phyinit(FAR struct stm32_ethmac_s *priv)
{
  volatile uint32_t timeout;
  uint32_t regval;
  uint16_t phyval;
  int ret;

  /* Assume 10MBps and half duplex */

  priv->mbps100 = 0;
  priv->fduplex = 0;

  /* Setup up PHY clocking by setting the SR field in the MACMIIAR register */

  regval  = stm32_getreg(STM32_ETH_MACMIIAR);
  regval &= ~ETH_MACMIIAR_CR_MASK;
  regval |= ETH_MACMIIAR_CR;
  stm32_putreg(regval, STM32_ETH_MACMIIAR);

  /* Put the PHY in reset mode */

  ret = stm32_phywrite(CONFIG_STM32_PHYADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      ndbg("Failed to reset the PHY: %d\n", ret);
      return ret;
    }
  up_mdelay(PHY_RESET_DELAY);

  /* Perform any necessary, board-specific PHY initialization */

#ifdef CONFIG_STM32_PHYINIT
  ret = stm32_phy_boardinitialize(0);
  if (ret < 0)
    {
      ndbg("Failed to initialize the PHY: %d\n", ret);
      return ret;
    }
#endif

  /* Special workaround for the Davicom DM9161 PHY is required. */

#ifdef CONFIG_PHY_DM9161
  ret = stm32_dm9161(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Perform auto-negotion if so configured */

#ifdef CONFIG_STM32_AUTONEG
  /* Wait for link status */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = stm32_phyread(CONFIG_STM32_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          ndbg("Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_LINKSTATUS) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      ndbg("Timed out waiting for link status: %04x\n", phyval);
      return -ETIMEDOUT;
    }

  /* Enable auto-gegotiation */

  ret = stm32_phywrite(CONFIG_STM32_PHYADDR, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      ndbg("Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = stm32_phyread(CONFIG_STM32_PHYADDR, MII_MSR, &phyval);
      if (ret < 0)
        {
          ndbg("Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      ndbg("Timed out waiting for auto-negotiation\n");
      return -ETIMEDOUT;
    }
 
  /* Read the result of the auto-negotiation from the PHY-specific register */

  ret = stm32_phyread(CONFIG_STM32_PHYADDR, CONFIG_STM32_PHYSR, &phyval);
  if (ret < 0)
    {
      ndbg("Failed to read PHY status register\n");
      return ret;
    }

  /* Remember the selected speed and duplex modes */

  nvdbg("PHYSR[%d]: %04x\n", CONFIG_STM32_PHYSR, phyval);

  /* Different PHYs present speed and mode information in different ways.  IF
   * This CONFIG_STM32_PHYSR_ALTCONFIG is selected, this indicates that the PHY
   * represents speed and mode information are combined, for example, with
   * separate bits for 10HD, 100HD, 10FD and 100FD.
   */

#ifdef CONFIG_STM32_PHYSR_ALTCONFIG
  switch (phyval & CONFIG_STM32_PHYSR_ALTMODE)
    {
      default:
      case CONFIG_STM32_PHYSR_10HD:
        priv->fduplex = 0;
        priv->mbps100 = 0;
        break;

      case CONFIG_STM32_PHYSR_100HD:
        priv->fduplex = 0;
        priv->mbps100 = 1;
        break;

      case CONFIG_STM32_PHYSR_10FD:
        priv->fduplex = 1;
        priv->mbps100 = 0;
        break;

      case CONFIG_STM32_PHYSR_100FD:
        priv->fduplex = 1;
        priv->mbps100 = 1;
        break;
    }

  /* Different PHYs present speed and mode information in different ways.  Some
   * will present separate information for speed and mode (this is the default).
   * Those PHYs, for example, may provide a 10/100 Mbps indication and a separate
   * full/half duplex indication.
   */

#else
  if ((phyval & CONFIG_STM32_PHYSR_MODE) == CONFIG_STM32_PHYSR_FULLDUPLEX)
    {
      priv->fduplex = 1;
    }

  if ((phyval & CONFIG_STM32_PHYSR_SPEED) == CONFIG_STM32_PHYSR_100MBPS)
    {
      priv->mbps100 = 1;
    }
#endif

#else /* Auto-negotion not selected */

  phyval = 0;
#ifdef CONFIG_STM32_ETHFD
  phyval |= MII_MCR_FULLDPLX;
#endif
#ifdef CONFIG_STM32_ETH100MBPS
  phyval |= MII_MCR_SPEED100;
#endif

  ret = stm32_phywrite(CONFIG_STM32_PHYADDR, MII_MCR, phyval);
  if (ret < 0)
    {
     ndbg("Failed to write the PHY MCR: %d\n", ret);
      return ret;
    }
  up_mdelay(PHY_CONFIG_DELAY);

  /* Remember the selected speed and duplex modes */

#ifdef CONFIG_STM32_ETHFD
  priv->fduplex = 1;
#endif
#ifdef CONFIG_STM32_ETH100MBPS
  priv->mbps100 = 1;
#endif
#endif

  ndbg("Duplex: %s Speed: %d MBps\n",
       priv->fduplex ? "FULL" : "HALF",
       priv->mbps100 ? 100 : 10);

  return OK;
}

/************************************************************************************
 * Name: stm32_selectmii
 *
 * Description:
 *   Selects the MII inteface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_MII
static inline void stm32_selectmii(void)
{
  uint32_t regval;

#ifdef CONFIG_STM32_CONNECTIVITYLINE
  regval  = getreg32(STM32_AFIO_MAPR);
  regval &= ~AFIO_MAPR_MII_RMII_SEL;
  putreg32(regval, STM32_AFIO_MAPR);
#else
  regval  = getreg32(STM32_SYSCFG_PMC);
  regval &= ~SYSCFG_PMC_MII_RMII_SEL;
  putreg32(regval, STM32_SYSCFG_PMC);
#endif
}
#endif

/************************************************************************************
 * Name: stm32_selectrmii
 *
 * Description:
 *   Selects the RMII inteface.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void stm32_selectrmii(void)
{
  uint32_t regval;

#ifdef CONFIG_STM32_CONNECTIVITYLINE
  regval  = getreg32(STM32_AFIO_MAPR);
  regval |= AFIO_MAPR_MII_RMII_SEL;
  putreg32(regval, STM32_AFIO_MAPR);
#else
  regval  = getreg32(STM32_SYSCFG_PMC);
  regval |= SYSCFG_PMC_MII_RMII_SEL;
  putreg32(regval, STM32_SYSCFG_PMC);
#endif
}

/****************************************************************************
 * Function: stm32_ethgpioconfig
 *
 * Description:
 *  Configure GPIOs for the Ethernet interface.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void stm32_ethgpioconfig(FAR struct stm32_ethmac_s *priv)
{
  /* Configure GPIO pins to support Ethernet */

#if defined(CONFIG_STM32_MII) || defined(CONFIG_STM32_RMII)

  /* MDC and MDIO are common to both modes */

  stm32_configgpio(GPIO_ETH_MDC);
  stm32_configgpio(GPIO_ETH_MDIO);

  /* Set up the MII interface */

#if defined(CONFIG_STM32_MII)

  /* Select the MII interface */

  stm32_selectmii();

  /* Provide clocking via MCO, MCO1 or MCO2:
   *
   * "MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or PLL
   *  clock (through a configurable prescaler) on PA8 pin."
   *
   * "MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or
   *  PLLI2S clock (through a configurable prescaler) on PC9 pin."
   */

# if defined(CONFIG_STM32_MII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO1);
  stm32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

# elif defined(CONFIG_STM32_MII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO2);
  stm32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

# elif defined(CONFIG_STM32_MII_MCO)
  /* Setup MCO pin for alternative usage */

  stm32_configgpio(GPIO_MCO);
  stm32_mcoconfig(BOARD_CFGR_MCO_SOURCE);
# endif

  /* MII interface pins (17):  
   *
   * MII_TX_CLK, MII_TXD[3:0], MII_TX_EN, MII_RX_CLK, MII_RXD[3:0], MII_RX_ER,
   * MII_RX_DV, MII_CRS, MII_COL, MDC, MDIO
   */

  stm32_configgpio(GPIO_ETH_MII_COL);
  stm32_configgpio(GPIO_ETH_MII_CRS);
  stm32_configgpio(GPIO_ETH_MII_RXD0);
  stm32_configgpio(GPIO_ETH_MII_RXD1);
  stm32_configgpio(GPIO_ETH_MII_RXD2);
  stm32_configgpio(GPIO_ETH_MII_RXD3);
  stm32_configgpio(GPIO_ETH_MII_RX_CLK);
  stm32_configgpio(GPIO_ETH_MII_RX_DV);
  stm32_configgpio(GPIO_ETH_MII_RX_ER);
  stm32_configgpio(GPIO_ETH_MII_TXD0);
  stm32_configgpio(GPIO_ETH_MII_TXD1);
  stm32_configgpio(GPIO_ETH_MII_TXD2);
  stm32_configgpio(GPIO_ETH_MII_TXD3);
  stm32_configgpio(GPIO_ETH_MII_TX_CLK);
  stm32_configgpio(GPIO_ETH_MII_TX_EN);

  /* Set up the RMII interface. */

#elif defined(CONFIG_STM32_RMII)

  /* Select the RMII interface */

  stm32_selectrmii();

  /* Provide clocking via MCO, MCO1 or MCO2:
   *
   * "MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or PLL
   *  clock (through a configurable prescaler) on PA8 pin."
   *
   * "MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or
   *  PLLI2S clock (through a configurable prescaler) on PC9 pin."
   */

# if defined(CONFIG_STM32_RMII_MCO1)
  /* Configure MC01 to drive the PHY.  Board logic must provide MC01 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO1);
  stm32_mco1config(BOARD_CFGR_MC01_SOURCE, BOARD_CFGR_MC01_DIVIDER);

# elif defined(CONFIG_STM32_RMII_MCO2)
  /* Configure MC02 to drive the PHY.  Board logic must provide MC02 clocking
   * info.
   */

  stm32_configgpio(GPIO_MCO2);
  stm32_mco2config(BOARD_CFGR_MC02_SOURCE, BOARD_CFGR_MC02_DIVIDER);

# elif defined(CONFIG_STM32_RMII_MCO)
  /* Setup MCO pin for alternative usage */

  stm32_configgpio(GPIO_MCO);
  stm32_mcoconfig(BOARD_CFGR_MCO_SOURCE);
# endif

  /* RMII interface pins (7):
   *
   * RMII_TXD[1:0], RMII_TX_EN, RMII_RXD[1:0], RMII_CRS_DV, MDC, MDIO,
   * RMII_REF_CLK
   */

  stm32_configgpio(GPIO_ETH_RMII_CRS_DV);
  stm32_configgpio(GPIO_ETH_RMII_REF_CLK);
  stm32_configgpio(GPIO_ETH_RMII_RXD0);
  stm32_configgpio(GPIO_ETH_RMII_RXD1);
  stm32_configgpio(GPIO_ETH_RMII_TXD0);
  stm32_configgpio(GPIO_ETH_RMII_TXD1);
  /* stm32_configgpio(GPIO_ETH_RMII_TX_CLK); not needed? */
  stm32_configgpio(GPIO_ETH_RMII_TX_EN);

#endif
#endif

  /* Enable pulse-per-second (PPS) output signal */

  stm32_configgpio(GPIO_ETH_PPS_OUT);
}

/****************************************************************************
 * Function: stm32_ethreset
 *
 * Description:
 *  Reset the Ethernet block.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_ethreset(FAR struct stm32_ethmac_s *priv)
{
  uint32_t regval;

  /* Reset the Ethernet on the AHB bus (F1 Connectivity Line) or AHB1 bus (F2
   * and F4)
   */

#if defined(CONFIG_STM32_CONNECTIVITYLINE)
  regval  = stm32_getreg(STM32_RCC_AHBRSTR);
  regval |= RCC_AHBRSTR_ETHMACRST;
  stm32_putreg(regval, STM32_RCC_AHBRSTR);

  regval &= ~RCC_AHBRSTR_ETHMACRST;
  stm32_putreg(regval, STM32_RCC_AHBRSTR);
#else
  regval  = stm32_getreg(STM32_RCC_AHB1RSTR);
  regval |= RCC_AHB1RSTR_ETHMACRST;
  stm32_putreg(regval, STM32_RCC_AHB1RSTR);

  regval &= ~RCC_AHB1RSTR_ETHMACRST;
  stm32_putreg(regval, STM32_RCC_AHB1RSTR);
#endif

  /* Perform a software reset by setting the SR bit in the DMABMR register.
   * This Resets all MAC subsystem internal registers and logic.  After this
   * reset all the registers holds their reset values.
   */

  regval  = stm32_getreg(STM32_ETH_DMABMR);
  regval |= ETH_DMABMR_SR;
  stm32_putreg(regval, STM32_ETH_DMABMR);

  /* Wait for software reset to complete. The SR bit is cleared automatically
   * after the reset operation has completed in all of the core clock domains.
   */

  while ((stm32_getreg(STM32_ETH_DMABMR) & ETH_DMABMR_SR) != 0);
}

/****************************************************************************
 * Function: stm32_macconfig
 *
 * Description:
 *  Configure the Ethernet MAC for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_macconfig(FAR struct stm32_ethmac_s *priv)
{
  uint32_t regval;

  /* Set up the MACCR register */

  regval  = stm32_getreg(STM32_ETH_MACCR);
  regval &= ~MACCR_CLEAR_BITS;
  regval |= MACCR_SET_BITS;

  if (priv->fduplex)
    {
      /* Set the DM bit for full duplex support */

      regval |= ETH_MACCR_DM;
    }

  if (priv->mbps100)
    {
      /* Set the FES bit for 100Mbps fast ethernet support */

      regval |= ETH_MACCR_FES;
    }

  stm32_putreg(regval, STM32_ETH_MACCR);

  /* Set up the MACFFR register */

  regval  = stm32_getreg(STM32_ETH_MACFFR);
  regval &= ~MACFFR_CLEAR_BITS;
  regval |= MACFFR_SET_BITS;
  stm32_putreg(regval, STM32_ETH_MACFFR);

  /* Set up the MACHTHR and MACHTLR registers */

  stm32_putreg(0, STM32_ETH_MACHTHR);
  stm32_putreg(0, STM32_ETH_MACHTLR);

  /* Setup up the MACFCR register */

  regval  = stm32_getreg(STM32_ETH_MACFCR);
  regval &= ~MACFCR_CLEAR_MASK;
  regval |= MACFCR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_MACFCR);

  /* Setup up the MACVLANTR register */

  stm32_putreg(0, STM32_ETH_MACVLANTR);

  /* DMA Configuration */
  /* Set up the DMAOMR register */

  regval  = stm32_getreg(STM32_ETH_DMAOMR);
  regval &= ~DMAOMR_CLEAR_MASK;
  regval |= DMAOMR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_DMAOMR);

  /* Set up the DMABMR register */

  regval  = stm32_getreg(STM32_ETH_DMABMR);
  regval &= ~DMABMR_CLEAR_MASK;
  regval |= DMABMR_SET_MASK;
  stm32_putreg(regval, STM32_ETH_DMABMR);

  return OK;
}

/****************************************************************************
 * Function: stm32_macaddress
 *
 * Description:
 *   Configure the selected MAC address.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_macaddress(FAR struct stm32_ethmac_s *priv)
{
  FAR struct uip_driver_s *dev = &priv->dev;
  uint32_t regval;

  nllvdbg("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          dev->d_ifname,
          dev->d_mac.ether_addr_octet[0], dev->d_mac.ether_addr_octet[1],
          dev->d_mac.ether_addr_octet[2], dev->d_mac.ether_addr_octet[3],
          dev->d_mac.ether_addr_octet[4], dev->d_mac.ether_addr_octet[5]);

  /* Set the MAC address high register */
  
  regval = ((uint32_t)dev->d_mac.ether_addr_octet[5] << 8) |
            (uint32_t)dev->d_mac.ether_addr_octet[4];
  stm32_putreg(regval, STM32_ETH_MACA0HR);

  /* Set the MAC address low register */
  
  regval = ((uint32_t)dev->d_mac.ether_addr_octet[3] << 24) |
           ((uint32_t)dev->d_mac.ether_addr_octet[2] << 16) |
           ((uint32_t)dev->d_mac.ether_addr_octet[1] <<  8) | 
            (uint32_t)dev->d_mac.ether_addr_octet[0];
  stm32_putreg(regval, STM32_ETH_MACA0LR);
}

/****************************************************************************
 * Function: stm32_macenable
 *
 * Description:
 *  Enable normal MAC operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_macenable(FAR struct stm32_ethmac_s *priv)
{
  uint32_t regval;

  /* Set the MAC address */

  stm32_macaddress(priv);

  /* Enable transmit state machine of the MAC for transmission on the MII */  

  regval  = stm32_getreg(STM32_ETH_MACCR);
  regval |= ETH_MACCR_TE;
  stm32_putreg(regval, STM32_ETH_MACCR);

  /* Flush Transmit FIFO */

  regval  = stm32_getreg(STM32_ETH_DMAOMR);
  regval |= ETH_DMAOMR_FTF;
  stm32_putreg(regval, STM32_ETH_DMAOMR);

  /* Enable receive state machine of the MAC for reception from the MII */  

  /* Enables or disables the MAC reception. */

  regval  = stm32_getreg(STM32_ETH_MACCR);
  regval |= ETH_MACCR_RE;
  stm32_putreg(regval, STM32_ETH_MACCR);
 
  /* Start DMA transmission */

  regval  = stm32_getreg(STM32_ETH_DMAOMR);
  regval |= ETH_DMAOMR_ST;
  stm32_putreg(regval, STM32_ETH_DMAOMR);
 
  /* Start DMA reception */

  regval  = stm32_getreg(STM32_ETH_DMAOMR);
  regval |= ETH_DMAOMR_SR;
  stm32_putreg(regval, STM32_ETH_DMAOMR);

  /* Enable Ethernet DMA interrupts.
   *
   * The STM32 hardware supports two interrupts: (1) one dedicated to normal
   * Ethernet operations and the other, used only for the Ethernet wakeup
   * event.  The wake-up interrupt is not used by this driver.
   *
   * The first Ethernet vector is reserved for interrupts generated by the
   * MAC and the DMA.  The MAC provides PMT and time stamp trigger interrupts,
   * neither of which are used by this driver.
   */

  stm32_putreg(ETH_MACIMR_ALLINTS, STM32_ETH_MACIMR);
  
  /* Ethernet DMA supports two classes of interrupts: Normal interrupt
   * summary (NIS) and Abnormal interrupt summary (AIS) with a variety
   * individual normal and abnormal interrupting events.  Here only
   * the normal receive event is enabled (unless DEBUG is enabled).  Transmit
   * events will only be enabled when a transmit interrupt is expected.
   */

  stm32_putreg((ETH_DMAINT_RECV_ENABLE | ETH_DMAINT_ERROR_ENABLE), STM32_ETH_DMAIER);
  return OK;
}

/****************************************************************************
 * Function: stm32_ethconfig
 *
 * Description:
 *  Configure the Ethernet interface for DMA operation.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ethconfig(FAR struct stm32_ethmac_s *priv)
{
  int ret;

  /* NOTE: The Ethernet clocks were initialized early in the boot-up
   * sequence in stm32_rcc.c.
   */

  /* Reset the Ethernet block */

  nllvdbg("Reset the Ethernet block\n");
  stm32_ethreset(priv);

  /* Initialize the PHY */

  nllvdbg("Initialize the PHY\n");
  ret = stm32_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the MAC and DMA */

  nllvdbg("Initialize the MAC and DMA\n");
  ret = stm32_macconfig(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the free buffer list */

  stm32_initbuffer(priv);

  /* Initialize TX Descriptors list: Chain Mode */

  stm32_txdescinit(priv);

  /* Initialize RX Descriptors list: Chain Mode  */

  stm32_rxdescinit(priv);

  /* Enable normal MAC operation */

  nllvdbg("Enable normal operation\n");
  return stm32_macenable(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then board specific logic
 *   must implement up_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if STM32_NETHERNET == 1
static inline
#endif

int stm32_ethinitialize(int intf)
{
  struct stm32_ethmac_s *priv;

  nvdbg("intf: %d\n", intf);

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < STM32_NETHERNET);
  priv = &g_stm32ethmac[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct stm32_ethmac_s));
  priv->dev.d_ifup    = stm32_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = stm32_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = stm32_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = stm32_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = stm32_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = (void*)g_stm32ethmac; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll       = wd_create();   /* Create periodic poll timer */
  priv->txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Configure GPIO pins to support Ethernet */

  stm32_ethgpioconfig(priv);

  /* Attach the IRQ to the driver */

  if (irq_attach(STM32_IRQ_ETH, stm32_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Put the interface in the down state. */

  stm32_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->dev);
  return OK;
}

/****************************************************************************
 * Function: up_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in up_initialize.c.  If STM32_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of up_netinitialize() that calls stm32_ethinitialize() with
 *   the appropriate interface number.
 *
 * Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if STM32_NETHERNET == 1
void up_netinitialize(void)
{
  (void)stm32_ethinitialize(0);
}
#endif

#endif /* STM32_NETHERNET > 0 */
#endif /* CONFIG_NET && CONFIG_STM32_ETHMAC */
