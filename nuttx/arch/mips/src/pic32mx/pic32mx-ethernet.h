/********************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-ethernet.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_ETHERNET_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_ETHERNET_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Register Offsets *************************************************************************/

/* Controller and DMA Engine Configuration/Status Registers */

#define PIC32MX_ETH_CON1_OFFSET        0x0000  /* Ethernet Controller Control 1 Register */
#define PIC32MX_ETH_CON1CLR_OFFSET     0x0004
#define PIC32MX_ETH_CON1SET_OFFSET     0x0008
#define PIC32MX_ETH_CON1INV_OFFSET     0x000c
#define PIC32MX_ETH_CON2_OFFSET        0x0010  /* Ethernet Controller Control 2 Register */
#define PIC32MX_ETH_CON2CLR_OFFSET     0x0014
#define PIC32MX_ETH_CON2SET_OFFSET     0x0018
#define PIC32MX_ETH_CON2INV_OFFSET     0x001c
#define PIC32MX_ETH_TXST_OFFSET        0x0020  /* Ethernet Controller TX Packet Descriptor Start Address Register */
#define PIC32MX_ETH_TXSTCLR_OFFSET     0x0024
#define PIC32MX_ETH_TXSTSET_OFFSET     0x0028
#define PIC32MX_ETH_TXSTINV_OFFSET     0x002c
#define PIC32MX_ETH_RXST_OFFSET        0x0030  /* Ethernet Controller RX Packet Descriptor Start Address Register */
#define PIC32MX_ETH_RXSTCLR_OFFSET     0x0034
#define PIC32MX_ETH_RXSTSET_OFFSET     0x0038
#define PIC32MX_ETH_RXSTINV_OFFSET     0x003c

#define PIC32MX_ETH_IEN_OFFSET         0x00c0  /* Ethernet Controller Interrupt Enable Register */
#define PIC32MX_ETH_IENCLR_OFFSET      0x00c4
#define PIC32MX_ETH_IENSET_OFFSET      0x00c8
#define PIC32MX_ETH_IENINV_OFFSET      0x00cc
#define PIC32MX_ETH_IRQ_OFFSET         0x00d0  /* Ethernet Controller Interrupt Request Register */
#define PIC32MX_ETH_IRQCLR_OFFSET      0x00d4
#define PIC32MX_ETH_IRQSET_OFFSET      0x00d8
#define PIC32MX_ETH_IRQINV_OFFSET      0x00dc
#define PIC32MX_ETH_STAT_OFFSET        0x00e0  /* Ethernet Controller Status Register */

/* RX Filtering Configuration Registers */

#define PIC32MX_ETH_RXFC_OFFSET        0x00a0  /* Ethernet Controller Receive Filter Configuration Register */
#define PIC32MX_ETH_RXFCCLR_OFFSET     0x00a4
#define PIC32MX_ETH_RXFCSET_OFFSET     0x00a8
#define PIC32MX_ETH_RXFCINV_OFFSET     0x00ac

#define PIC32MX_ETH_HT0_OFFSET         0x0040  /* Ethernet Controller Hash Table 0 Register */
#define PIC32MX_ETH_HT0CLR_OFFSET      0x0044
#define PIC32MX_ETH_HT0SET_OFFSET      0x0048
#define PIC32MX_ETH_HT0INV_OFFSET      0x004c
#define PIC32MX_ETH_HT1_OFFSET         0x0050  /* Ethernet Controller Hash Table 1 Register */
#define PIC32MX_ETH_HT1CLR_OFFSET      0x0054
#define PIC32MX_ETH_HT1SET_OFFSET      0x0058
#define PIC32MX_ETH_HT1INV_OFFSET      0x005c
#define PIC32MX_ETH_PMM0_OFFSET        0x0060  /* Ethernet Controller Pattern Match Mask 0 Register */
#define PIC32MX_ETH_PMM0CLR_OFFSET     0x0064
#define PIC32MX_ETH_PMM0SET_OFFSET     0x0068
#define PIC32MX_ETH_PMM0INV_OFFSET     0x006c
#define PIC32MX_ETH_PMM1_OFFSET        0x0070  /* Ethernet Controller Pattern Match Mask 1 Register */
#define PIC32MX_ETH_PMM1CLR_OFFSET     0x0074
#define PIC32MX_ETH_PMM1SET_OFFSET     0x0078
#define PIC32MX_ETH_PMM1INV_OFFSET     0x007c
#define PIC32MX_ETH_PMCS_OFFSET        0x0080  /* Ethernet Controller Pattern Match Checksum Register */
#define PIC32MX_ETH_PMCSCLR_OFFSET     0x0084
#define PIC32MX_ETH_PMCSSET_OFFSET     0x0088
#define PIC32MX_ETH_PMCSINV_OFFSET     0x008c
#define PIC32MX_ETH_PMO_OFFSET         0x0090  /* Ethernet Controller Pattern Match Offset Register */
#define PIC32MX_ETH_PMOCLR_OFFSET      0x0094
#define PIC32MX_ETH_PMOSET_OFFSET      0x0098
#define PIC32MX_ETH_PMOINV_OFFSET      0x009c

/* Flow Control Configuring Register */

#define PIC32MX_ETH_RXWM_OFFSET        0x00b0  /* Ethernet Controller Receive Watermarks Register */
#define PIC32MX_ETH_RXWMCLR_OFFSET     0x00b4
#define PIC32MX_ETH_RXWMSET_OFFSET     0x00b8
#define PIC32MX_ETH_RXWMINV_OFFSET     0x00bc

/* Ethernet Statistics Registers */

#define PIC32MX_ETH_RXOVFLOW_OFFSET    0x0100  /* Ethernet Controller Receive Overflow Statistics Register */
#define PIC32MX_ETH_RXOVFLOWCLR_OFFSET 0x0104
#define PIC32MX_ETH_RXOVFLOWSET_OFFSET 0x0108
#define PIC32MX_ETH_RXOVFLOWINV_OFFSET 0x010c
#define PIC32MX_ETH_FRMTXOK_OFFSET     0x0110  /* Ethernet Controller Frames Transmitted OK Statistics Register */
#define PIC32MX_ETH_FRMTXOKCLR_OFFSET  0x0114
#define PIC32MX_ETH_FRMTXOKSET_OFFSET  0x0118
#define PIC32MX_ETH_FRMTXOKINV_OFFSET  0x011c
#define PIC32MX_ETH_SCOLFRM_OFFSET     0x0120  /* Ethernet Controller Single Collision Frames Statistics Register */
#define PIC32MX_ETH_SCOLFRMCLR_OFFSET  0x0124
#define PIC32MX_ETH_SCOLFRMSET_OFFSET  0x0128
#define PIC32MX_ETH_SCOLFRMINV_OFFSET  0x012c
#define PIC32MX_ETH_MCOLFRM_OFFSET     0x0130  /* Ethernet Controller Multiple Collision Frames Statistics Register */
#define PIC32MX_ETH_MCOLFRMCLR_OFFSET  0x0134
#define PIC32MX_ETH_MCOLFRMSET_OFFSET  0x0138
#define PIC32MX_ETH_MCOLFRMINV_OFFSET  0x013c
#define PIC32MX_ETH_FRMRXOK_OFFSET     0x0140  /* Ethernet Controller Frames Received OK Statistics Register */
#define PIC32MX_ETH_FRMRXOKCLR_OFFSET  0x0144
#define PIC32MX_ETH_FRMRXOKSET_OFFSET  0x0148
#define PIC32MX_ETH_FRMRXOKINV_OFFSET  0x014c
#define PIC32MX_ETH_FCSERR_OFFSET      0x0150  /* Ethernet Controller Frame Check Sequence Error Statistics Register */
#define PIC32MX_ETH_FCSERRCLR_OFFSET   0x0154
#define PIC32MX_ETH_FCSERRSET_OFFSET   0x0158
#define PIC32MX_ETH_FCSERRINV_OFFSET   0x015c
#define PIC32MX_ETH_ALGNERR_OFFSET     0x0160  /* Ethernet Controller Alignment Errors Statistics Register */
#define PIC32MX_ETH_ALGNERRCLR_OFFSET  0x0164
#define PIC32MX_ETH_ALGNERRSET_OFFSET  0x0168
#define PIC32MX_ETH_ALGNERRINV_OFFSET  0x016c

/* MAC Configuration Registers */

#define PIC32MX_EMAC1_CFG1_OFFSET      0x0200  /* Ethernet Controller MAC Configuration 1 Register */
#define PIC32MX_EMAC1_CFG1CLR_OFFSET   0x0204
#define PIC32MX_EMAC1_CFG1SET_OFFSET   0x0208
#define PIC32MX_EMAC1_CFG1INV_OFFSET   0x020c
#define PIC32MX_EMAC1_CFG2_OFFSET      0x0210  /* Ethernet Controller MAC Configuration 2 Register */
#define PIC32MX_EMAC1_CFG2CLR_OFFSET   0x0214
#define PIC32MX_EMAC1_CFG2SET_OFFSET   0x0218
#define PIC32MX_EMAC1_CFG2INV_OFFSET   0x021c
#define PIC32MX_EMAC1_IPGT_OFFSET      0x0220  /* Ethernet Controller MAC Back-to-Back Interpacket Gap Register */
#define PIC32MX_EMAC1_IPGTCLR_OFFSET   0x0224
#define PIC32MX_EMAC1_IPGTSET_OFFSET   0x0228
#define PIC32MX_EMAC1_IPGTINV_OFFSET   0x022c
#define PIC32MX_EMAC1_IPGR_OFFSET      0x0230  /* Ethernet Controller MAC Non-Back-to-Back Interpacket Gap Register */
#define PIC32MX_EMAC1_IPGRCLR_OFFSET   0x0234
#define PIC32MX_EMAC1_IPGRSET_OFFSET   0x0238
#define PIC32MX_EMAC1_IPGRINV_OFFSET   0x023c
#define PIC32MX_EMAC1_CLRT_OFFSET      0x0240  /* Ethernet Controller MAC Collision Window/Retry Limit Register */
#define PIC32MX_EMAC1_CLRTCLR_OFFSET   0x0244
#define PIC32MX_EMAC1_CLRTSET_OFFSET   0x0248
#define PIC32MX_EMAC1_CLRTINV_OFFSET   0x024c
#define PIC32MX_EMAC1_MAXF_OFFSET      0x0250  /* Ethernet Controller MAC Maximum Frame Length Register */
#define PIC32MX_EMAC1_MAXFCLR_OFFSET   0x0254
#define PIC32MX_EMAC1_MAXFSET_OFFSET   0x0258
#define PIC32MX_EMAC1_MAXFINV_OFFSET   0x025c
#define PIC32MX_EMAC1_SUPP_OFFSET      0x0260  /* Ethernet Controller MAC PHY Support Register */
#define PIC32MX_EMAC1_SUPPCLR_OFFSET   0x0264
#define PIC32MX_EMAC1_SUPPSET_OFFSET   0x0268
#define PIC32MX_EMAC1_SUPPINV_OFFSET   0x026c
#define PIC32MX_EMAC1_TEST_OFFSET      0x0270  /* Ethernet Controller MAC Test Register */
#define PIC32MX_EMAC1_TESTCLR_OFFSET   0x0274
#define PIC32MX_EMAC1_TESTSET_OFFSET   0x0278
#define PIC32MX_EMAC1_TESTINV_OFFSET   0x027c

#define PIC32MX_EMAC1_SA0_OFFSET       0x0300  /* Ethernet Controller MAC Station Address 0 Register */
#define PIC32MX_EMAC1_SA0CLR_OFFSET    0x0304
#define PIC32MX_EMAC1_SA0SET_OFFSET    0x0308
#define PIC32MX_EMAC1_SA0INV_OFFSET    0x030c
#define PIC32MX_EMAC1_SA1_OFFSET       0x0310  /* Ethernet Controller MAC Station Address 1 Register */
#define PIC32MX_EMAC1_SA1CLR_OFFSET    0x0314
#define PIC32MX_EMAC1_SA1SET_OFFSET    0x0318
#define PIC32MX_EMAC1_SA1INV_OFFSET    0x031c
#define PIC32MX_EMAC1_SA2_OFFSET       0x0320  /* Ethernet Controller MAC Station Address 2 Register */
#define PIC32MX_EMAC1_SA2CLR_OFFSET    0x0324
#define PIC32MX_EMAC1_SA2SET_OFFSET    0x0328
#define PIC32MX_EMAC1_SA2INV_OFFSET    0x032c

/* MII Management Registers */

#define PIC32MX_EMAC1_MCFG_OFFSET      0x0280  /* Ethernet Controller MAC MII Management Configuration Register */
#define PIC32MX_EMAC1_MCFGCLR_OFFSET   0x0284
#define PIC32MX_EMAC1_MCFGSET_OFFSET   0x0288
#define PIC32MX_EMAC1_MCFGINV_OFFSET   0x028c
#define PIC32MX_EMAC1_MCMD_OFFSET      0x0290  /* Ethernet Controller MAC MII Management Command Register */
#define PIC32MX_EMAC1_MCMDCLR_OFFSET   0x0294
#define PIC32MX_EMAC1_MCMDSET_OFFSET   0x0298
#define PIC32MX_EMAC1_MCMDINV_OFFSET   0x029c
#define PIC32MX_EMAC1_MADR_OFFSET      0x02a0  /* Ethernet Controller MAC MII Management Address Register */
#define PIC32MX_EMAC1_MADRCLR_OFFSET   0x02a4
#define PIC32MX_EMAC1_MADRSET_OFFSET   0x02a8
#define PIC32MX_EMAC1_MADRINV_OFFSET   0x02ac
#define PIC32MX_EMAC1_MWTD_OFFSET      0x02b0  /* Ethernet Controller MAC MII Management Write Data Register */
#define PIC32MX_EMAC1_MWTDCLR_OFFSET   0x02b4
#define PIC32MX_EMAC1_MWTDSET_OFFSET   0x02b8
#define PIC32MX_EMAC1_MWTDINV_OFFSET   0x02bc
#define PIC32MX_EMAC1_MRDD_OFFSET      0x02c0  /* Ethernet Controller MAC MII Management Read Data Register */
#define PIC32MX_EMAC1_MRDDCLR_OFFSET   0x02c4
#define PIC32MX_EMAC1_MRDDSET_OFFSET   0x02c8
#define PIC32MX_EMAC1_MRDDINV_OFFSET   0x02cc
#define PIC32MX_EMAC1_MIND_OFFSET      0x02d0  /* Ethernet Controller MAC MII Management Indicators Register */
#define PIC32MX_EMAC1_MINDCLR_OFFSET   0x02d4
#define PIC32MX_EMAC1_MINDSET_OFFSET   0x02d8
#define PIC32MX_EMAC1_MINDINV_OFFSET   0x02dc

/* Register Addresses ***********************************************************************/

/* Controller and DMA Engine Configuration/Status Registers */

#define PIC32MX_ETH_CON1               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON1_OFFSET)
#define PIC32MX_ETH_CON1CLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON1CLR_OFFSET)
#define PIC32MX_ETH_CON1SET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON1SET_OFFSET)
#define PIC32MX_ETH_CON1INV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON1INV_OFFSET)
#define PIC32MX_ETH_CON2               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON2_OFFSET)
#define PIC32MX_ETH_CON2CLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON2CLR_OFFSET)
#define PIC32MX_ETH_CON2SET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON2SET_OFFSET)
#define PIC32MX_ETH_CON2INV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_CON2INV_OFFSET)
#define PIC32MX_ETH_TXST               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_TXST_OFFSET)
#define PIC32MX_ETH_TXSTCLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_TXSTCLR_OFFSET)
#define PIC32MX_ETH_TXSTSET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_TXSTSET_OFFSET)
#define PIC32MX_ETH_TXSTINV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_TXSTINV_OFFSET)
#define PIC32MX_ETH_RXST               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXST_OFFSET)
#define PIC32MX_ETH_RXSTCLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXSTCLR_OFFSET)
#define PIC32MX_ETH_RXSTSET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXSTSET_OFFSET)
#define PIC32MX_ETH_RXSTINV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXSTINV_OFFSET)
#define PIC32MX_ETH_IEN                (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IEN_OFFSET)
#define PIC32MX_ETH_IENCLR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IENCLR_OFFSET)
#define PIC32MX_ETH_IENSET             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IENSET_OFFSET)
#define PIC32MX_ETH_IENINV             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IENINV_OFFSET)
#define PIC32MX_ETH_IRQ                (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IRQ_OFFSET)
#define PIC32MX_ETH_IRQCLR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IRQCLR_OFFSET)
#define PIC32MX_ETH_IRQSET             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IRQSET_OFFSET)
#define PIC32MX_ETH_IRQINV             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_IRQINV_OFFSET)
#define PIC32MX_ETH_STAT               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_STAT_OFFSET)

/* RX Filtering Configuration Registers */

#define PIC32MX_ETH_RXFC               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXFC_OFFSET)
#define PIC32MX_ETH_RXFCCLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXFCCLR_OFFSET)
#define PIC32MX_ETH_RXFCSET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXFCSET_OFFSET)
#define PIC32MX_ETH_RXFCINV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXFCINV_OFFSET)
#define PIC32MX_ETH_HT0                (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT0_OFFSET)
#define PIC32MX_ETH_HT0CLR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT0CLR_OFFSET)
#define PIC32MX_ETH_HT0SET             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT0SET_OFFSET)
#define PIC32MX_ETH_HT0INV             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT0INV_OFFSET)
#define PIC32MX_ETH_HT1                (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT1_OFFSET)
#define PIC32MX_ETH_HT1CLR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT1CLR_OFFSET)
#define PIC32MX_ETH_HT1SET             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT1SET_OFFSET)
#define PIC32MX_ETH_HT1INV             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_HT1INV_OFFSET)
#define PIC32MX_ETH_PMM0               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM0_OFFSET)
#define PIC32MX_ETH_PMM0CLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM0CLR_OFFSET)
#define PIC32MX_ETH_PMM0SET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM0SET_OFFSET)
#define PIC32MX_ETH_PMM0INV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM0INV_OFFSET)
#define PIC32MX_ETH_PMM1               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM1_OFFSET)
#define PIC32MX_ETH_PMM1CLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM1CLR_OFFSET)
#define PIC32MX_ETH_PMM1SET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM1SET_OFFSET)
#define PIC32MX_ETH_PMM1INV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMM1INV_OFFSET)
#define PIC32MX_ETH_PMCS               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMCS_OFFSET)
#define PIC32MX_ETH_PMCSCLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMCSCLR_OFFSET)
#define PIC32MX_ETH_PMCSSET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMCSSET_OFFSET)
#define PIC32MX_ETH_PMCSINV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMCSINV_OFFSET)
#define PIC32MX_ETH_PMO                (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMO_OFFSET)
#define PIC32MX_ETH_PMOCLR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMOCLR_OFFSET)
#define PIC32MX_ETH_PMOSET             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMOSET_OFFSET)
#define PIC32MX_ETH_PMOINV             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_PMOINV_OFFSET)

/* Flow Control Configuring Register */

#define PIC32MX_ETH_RXWM               (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXWM_OFFSET)
#define PIC32MX_ETH_RXWMCLR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXWMCLR_OFFSET)
#define PIC32MX_ETH_RXWMSET            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXWMSET_OFFSET)
#define PIC32MX_ETH_RXWMINV            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXWMINV_OFFSET)

/* Ethernet Statistics Registers */

#define PIC32MX_ETH_RXOVFLOW           (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXOVFLOW_OFFSET)
#define PIC32MX_ETH_RXOVFLOWCLR        (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXOVFLOWCLR_OFFSET)
#define PIC32MX_ETH_RXOVFLOWSET        (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXOVFLOWSET_OFFSET)
#define PIC32MX_ETH_RXOVFLOWINV        (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_RXOVFLOWINV_OFFSET)
#define PIC32MX_ETH_FRMTXOK            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMTXOK_OFFSET)
#define PIC32MX_ETH_FRMTXOKCLR         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMTXOKCLR_OFFSET)
#define PIC32MX_ETH_FRMTXOKSET         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMTXOKSET_OFFSET)
#define PIC32MX_ETH_FRMTXOKINV         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMTXOKINV_OFFSET)
#define PIC32MX_ETH_SCOLFRM            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_SCOLFRM_OFFSET)
#define PIC32MX_ETH_SCOLFRMCLR         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_SCOLFRMCLR_OFFSET)
#define PIC32MX_ETH_SCOLFRMSET         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_SCOLFRMSET_OFFSET)
#define PIC32MX_ETH_SCOLFRMINV         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_SCOLFRMINV_OFFSET)
#define PIC32MX_ETH_MCOLFRM            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_MCOLFRM_OFFSET)
#define PIC32MX_ETH_MCOLFRMCLR         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_MCOLFRMCLR_OFFSET)
#define PIC32MX_ETH_MCOLFRMSET         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_MCOLFRMSET_OFFSET)
#define PIC32MX_ETH_MCOLFRMINV         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_MCOLFRMINV_OFFSET)
#define PIC32MX_ETH_FRMRXOK            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMRXOK_OFFSET)
#define PIC32MX_ETH_FRMRXOKCLR         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMRXOKCLR_OFFSET)
#define PIC32MX_ETH_FRMRXOKSET         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMRXOKSET_OFFSET)
#define PIC32MX_ETH_FRMRXOKINV         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FRMRXOKINV_OFFSET)
#define PIC32MX_ETH_FCSERR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FCSERR_OFFSET)
#define PIC32MX_ETH_FCSERRCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FCSERRCLR_OFFSET)
#define PIC32MX_ETH_FCSERRSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FCSERRSET_OFFSET)
#define PIC32MX_ETH_FCSERRINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_FCSERRINV_OFFSET)
#define PIC32MX_ETH_ALGNERR            (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_ALGNERR_OFFSET)
#define PIC32MX_ETH_ALGNERRCLR         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_ALGNERRCLR_OFFSET)
#define PIC32MX_ETH_ALGNERRSET         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_ALGNERRSET_OFFSET)
#define PIC32MX_ETH_ALGNERRINV         (PIC32MX_ETHERNET_K1BASE+PIC32MX_ETH_ALGNERRINV_OFFSET)

/* MAC Configuration Registers */

#define PIC32MX_EMAC1_CFG1             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG1_OFFSET)
#define PIC32MX_EMAC1_CFG1CLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG1CLR_OFFSET)
#define PIC32MX_EMAC1_CFG1SET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG1SET_OFFSET)
#define PIC32MX_EMAC1_CFG1INV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG1INV_OFFSET)
#define PIC32MX_EMAC1_CFG2             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG2_OFFSET)
#define PIC32MX_EMAC1_CFG2CLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG2CLR_OFFSET)
#define PIC32MX_EMAC1_CFG2SET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG2SET_OFFSET)
#define PIC32MX_EMAC1_CFG2INV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CFG2INV_OFFSET)
#define PIC32MX_EMAC1_IPGT             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGT_OFFSET)
#define PIC32MX_EMAC1_IPGTCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGTCLR_OFFSET)
#define PIC32MX_EMAC1_IPGTSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGTSET_OFFSET)
#define PIC32MX_EMAC1_IPGTINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGTINV_OFFSET)
#define PIC32MX_EMAC1_IPGR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGR_OFFSET)
#define PIC32MX_EMAC1_IPGRCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGRCLR_OFFSET)
#define PIC32MX_EMAC1_IPGRSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGRSET_OFFSET)
#define PIC32MX_EMAC1_IPGRINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_IPGRINV_OFFSET)
#define PIC32MX_EMAC1_CLRT             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CLRT_OFFSET)
#define PIC32MX_EMAC1_CLRTCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CLRTCLR_OFFSET)
#define PIC32MX_EMAC1_CLRTSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CLRTSET_OFFSET)
#define PIC32MX_EMAC1_CLRTINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_CLRTINV_OFFSET)
#define PIC32MX_EMAC1_MAXF             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MAXF_OFFSET)
#define PIC32MX_EMAC1_MAXFCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MAXFCLR_OFFSET)
#define PIC32MX_EMAC1_MAXFSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MAXFSET_OFFSET)
#define PIC32MX_EMAC1_MAXFINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MAXFINV_OFFSET)
#define PIC32MX_EMAC1_SUPP             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SUPP_OFFSET)
#define PIC32MX_EMAC1_SUPPCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SUPPCLR_OFFSET)
#define PIC32MX_EMAC1_SUPPSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SUPPSET_OFFSET)
#define PIC32MX_EMAC1_SUPPINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SUPPINV_OFFSET)
#define PIC32MX_EMAC1_TEST             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_TEST_OFFSET)
#define PIC32MX_EMAC1_TESTCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_TESTCLR_OFFSET)
#define PIC32MX_EMAC1_TESTSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_TESTSET_OFFSET)
#define PIC32MX_EMAC1_TESTINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_TESTINV_OFFSET)
#define PIC32MX_EMAC1_SA0              (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA0_OFFSET)
#define PIC32MX_EMAC1_SA0CLR           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA0CLR_OFFSET)
#define PIC32MX_EMAC1_SA0SET           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA0SET_OFFSET)
#define PIC32MX_EMAC1_SA0INV           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA0INV_OFFSET)
#define PIC32MX_EMAC1_SA1              (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA1_OFFSET)
#define PIC32MX_EMAC1_SA1CLR           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA1CLR_OFFSET)
#define PIC32MX_EMAC1_SA1SET           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA1SET_OFFSET)
#define PIC32MX_EMAC1_SA1INV           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA1INV_OFFSET)
#define PIC32MX_EMAC1_SA2              (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA2_OFFSET)
#define PIC32MX_EMAC1_SA2CLR           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA2CLR_OFFSET)
#define PIC32MX_EMAC1_SA2SET           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA2SET_OFFSET)
#define PIC32MX_EMAC1_SA2INV           (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_SA2INV_OFFSET)

/* MII Management Registers */

#define PIC32MX_EMAC1_MCFG             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCFG_OFFSET)
#define PIC32MX_EMAC1_MCFGCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCFGCLR_OFFSET)
#define PIC32MX_EMAC1_MCFGSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCFGSET_OFFSET)
#define PIC32MX_EMAC1_MCFGINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCFGINV_OFFSET)
#define PIC32MX_EMAC1_MCMD             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCMD_OFFSET)
#define PIC32MX_EMAC1_MCMDCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCMDCLR_OFFSET)
#define PIC32MX_EMAC1_MCMDSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCMDSET_OFFSET)
#define PIC32MX_EMAC1_MCMDINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MCMDINV_OFFSET)
#define PIC32MX_EMAC1_MADR             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MADR_OFFSET)
#define PIC32MX_EMAC1_MADRCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MADRCLR_OFFSET)
#define PIC32MX_EMAC1_MADRSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MADRSET_OFFSET)
#define PIC32MX_EMAC1_MADRINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MADRINV_OFFSET)
#define PIC32MX_EMAC1_MWTD             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MWTD_OFFSET)
#define PIC32MX_EMAC1_MWTDCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MWTDCLR_OFFSET)
#define PIC32MX_EMAC1_MWTDSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MWTDSET_OFFSET)
#define PIC32MX_EMAC1_MWTDINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MWTDINV_OFFSET)
#define PIC32MX_EMAC1_MRDD             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MRDD_OFFSET)
#define PIC32MX_EMAC1_MRDDCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MRDDCLR_OFFSET)
#define PIC32MX_EMAC1_MRDDSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MRDDSET_OFFSET)
#define PIC32MX_EMAC1_MRDDINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MRDDINV_OFFSET )
#define PIC32MX_EMAC1_MIND             (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MIND_OFFSET)
#define PIC32MX_EMAC1_MINDCLR          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MINDCLR_OFFSET)
#define PIC32MX_EMAC1_MINDSET          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MINDSET_OFFSET)
#define PIC32MX_EMAC1_MINDINV          (PIC32MX_ETHERNET_K1BASE+PIC32MX_EMAC1_MINDINV_OFFSET)

/* Register Bit-Field Definitions ***********************************************************/

/* Controller and DMA Engine Configuration/Status Registers */
/* Ethernet Controller Control 1 Register */

#define ETH_CON1_BUFCDEC               (1 << 0)  /* Bit 0: : Descriptor Buffer Count Decrement bit */
                                                 /* Bit 1-3: Reserved */
#define ETH_CON1_MANFC                 (1 << 4)  /* Bit 4:  Manual Flow Control bit */
                                                 /* Bit 5-6: Reserved */
#define ETH_CON1_AUTOFC                (1 << 7)  /* Bit 7:  Automatic Flow Control bit */
#define ETH_CON1_RXEN                  (1 << 8)  /* Bit 8:  Receive Enable bit */
#define ETH_CON1_TXRTS                 (1 << 9)  /* Bit 9:  Transmit Request to Send bit */
                                                 /* Bit 10-12: Reserved */
#define ETH_CON1_SIDL                  (1 << 13) /* Bit 13: Ethernet Stop in Idle Mode bit */
                                                 /* Bit 14: Reserved */
#define ETH_CON1_ON                    (1 << 15) /* Bit 15: Ethernet ON bit */
#define ETH_CON1_PTV_SHIFT             (16)      /* Bits 16-31: PAUSE Timer Value bits */
#define ETH_CON1_PTV_MASK              (0xffff << ETH_CON1_PTV_SHIFT)

/* Ethernet Controller Control 2 Register */
                                                 /* Bits 0-3: Reserved */
#define ETH_CON2_RXBUFSZ_SHIFT         (4)       /* Bits 4-10: RX Data Buffer Size for All RX Descriptors */
#define ETH_CON2_RXBUFSZ_MASK          (0x7f << ETH_CON2_RXBUFSZ_SHIFT)
#  define ETH_CON2_RXBUFSZ(n)          (((n) >> 4) << ETH_CON2_RXBUFSZ_SHIFT) /* n=16, 32, 48, ... 2032 */
                                                 /* Bits 11-31: Reserved */

/* Ethernet Controller TX Packet Descriptor Start Address Register (32-bit address) */
/* Ethernet Controller RX Packet Descriptor Start Address Register (32-bit address) */

/* Ethernet Controller Interrupt Enable Register */
/* Ethernet Controller Interrupt Request Register */

#define ETH_INT_RXOVFLW                (1 << 0)  /* Bit 0:  Receive FIFO overflow interrupt */
#define ETH_INT_RXBUFNA                (1 << 1)  /* Bit 1:  Receive buffer not available interrupt */
#define ETH_INT_TXABORT                (1 << 2)  /* Bit 2:  Transmitter abort interrupt */
#define ETH_INT_TXDONE                 (1 << 3)  /* Bit 3:  Transmitter done interrupt */
                                                 /* Bit 4:  Reserved */
#define ETH_INT_RXACT                  (1 << 5)  /* Bit 5:  RX activity interrupt */
#define ETH_INT_PKTPEND                (1 << 6)  /* Bit 6:  Packet pending interrupt */
#define ETH_INT_RXDONE                 (1 << 7)  /* Bit 7:  Receiver done interrupt */
#define ETH_INT_FWMARK                 (1 << 8)  /* Bit 8:  Full watermark interrupt */
#define ETH_INT_EWMARK                 (1 << 9)  /* Bit 9:  Empty watermark interrupt */
                                                 /* Bits 10-12: Reserved */
#define ETH_INT_RXBUSE                 (1 << 13) /* Bit 13: Receive BVCI bus error interrupt */
#define ETH_INT_TXBUSE                 (1 << 14) /* Bit 14: Transmit BVCI bus error interrupt */
                                                 /* Bits 15-31: Reserved */
#define ETH_INT_ALLINTS                (0x000063ef)

/* Ethernet Controller Status Register */

                                                 /* Bits 0-4: Reserved */
#define ETH_STAT_RXBUSY                (1 << 5)  /* Bit 5:  Receive busy */
#define ETH_STAT_TXBUSY                (1 << 6)  /* Bit 6:  Transmit busy */
#define ETH_STAT_ETHBUSY               (1 << 7)  /* Bit 7:  Ethernet module busy */
                                                 /* Bits 8-15: Reserved */
#define ETH_STAT_BUFCNT_SHIFT          (18)      /* Bits 16-23: Packet buffer count */
#define ETH_STAT_BUFCNT_MASK           (0xff << ETH_STAT_BUFCNT_SHIFT)
                                                 /* Bits 24-31: Reserved */

/* RX Filtering Configuration Registers */
/* Ethernet Controller Receive Filter Configuration Register */

#define ETH_RXFC_BCEN                  (1 << 0)  /* Bit 0:  Broadcast filter enable */
#define ETH_RXFC_MCEN                  (1 << 1)  /* Bit 1:  Multicast filter enable */
#define ETH_RXFC_NOTMEEN               (1 << 2)  /* Bit 2:  Not Me unicast filter enable */
#define ETH_RXFC_UCEN                  (1 << 3)  /* Bit 3:  Unicast filter enable */
#define ETH_RXFC_RUNTEN                (1 << 4)  /* Bit 4:  Runt enable */
#define ETH_RXFC_RUNTERREN             (1 << 5)  /* Bit 5:  Runt error collection enable */
#define ETH_RXFC_CRCOKEN               (1 << 6)  /* Bit 6:  CRC OK enable enable */
#define ETH_RXFC_CRCERREN              (1 << 7)  /* Bit 7:  CRC error collection enable */
#define ETH_RXFC_PMMODE_SHIFT          (8)       /* Bits 8-11: Pattern match mode */
#define ETH_RXFC_PMMODE_MASK           (15 << ETH_RXFC_PMMODE_SHIFT)
#  define ETH_RXFC_PMMODE_DISABLED     (0 << ETH_RXFC_PMMODE_SHIFT) /* Pattern match is always unsuccessful */
#  define ETH_RXFC_PMMODE_PMCKSUM      (1 << ETH_RXFC_PMMODE_SHIFT) /* PM checksum matches */
#  define ETH_RXFC_PMMODE_DASTA        (2 << ETH_RXFC_PMMODE_SHIFT) /* PM checksum matches & DA==STA */
/* #define ETH_RXFC_PMMODE_DASTA       (3 << ETH_RXFC_PMMODE_SHIFT)    PM checksum matches & DA==STA */
#  define ETH_RXFC_PMMODE_DAUCAST      (4 << ETH_RXFC_PMMODE_SHIFT) /* PM checksum matches & DA==Unicast address */
/* #define ETH_RXFC_PMMODE_DAUCAST     (5 << ETH_RXFC_PMMODE_SHIFT)    PM checksum matches & DA==Unicast address */
#  define ETH_RXFC_PMMODE_DABCAST      (6 << ETH_RXFC_PMMODE_SHIFT) /* PM checksum matches & DA==Broadcast address */
/* #define ETH_RXFC_PMMODE_DABCAST     (7 << ETH_RXFC_PMMODE_SHIFT)    PM checksum matches & DA==Broadcast address */
#  define ETH_RXFC_PMMODE_HASH         (8 << ETH_RXFC_PMMODE_SHIFT) /* PM checksum matches & Hash Table Filter match */
#  define ETH_RXFC_PMMODE_MAGIC        (9 << ETH_RXFC_PMMODE_SHIFT) /* PM checksum matches & Packet = Magic Packet */
#define ETH_RXFC_NOTPM                 (1 << 12) /* Bit 12: Pattern match inversion */
                                                 /* Bit 13: Reserved */
#define ETH_RXFC_MPEN                  (1 << 14) /* Bit 14: Magic packet enable */
#define ETH_RXFC_HTEN                  (1 << 15) /* Bit 15: Hash table filtering enable */
                                                 /* Bits 16-31: Reserved */
/* Ethernet Controller Hash Table 0 Register */

#define ETH_HT0_BYTE0_SHIFT            (0)       /* Bits 0-7: Hash table byte 0, HT[0-7] */
#define ETH_HT0_BYTE0_MASK             (0xff << ETH_HT0_BYTE0_SHIFT)
#define ETH_HT0_BYTE1_SHIFT            (8)       /* Bits 8-15: Hash table byte 1, HT[8-15] */
#define ETH_HT0_BYTE1_MASK             (0xff << ETH_HT0_BYTE1_SHIFT)
#define ETH_HT0_BYTE2_SHIFT            (16)      /* Bits 16-23: Hash table byte 2, HT[16-23] */
#define ETH_HT0_BYTE2_MASK             (0xff << ETH_HT0_BYTE2_SHIFT)
#define ETH_HT0_BYTE3_SHIFT            (24)      /* Bits 24-31: Hash table byte 3, HT[24-31] */
#define ETH_HT0_BYTE3_MASK             (0xff << ETH_HT0_BYTE3_SHIFT)

/* Ethernet Controller Hash Table 1 Register */

#define ETH_HT1_BYTE4_SHIFT            (0)       /* Bits 0-7: Hash table byte 4, HT[32-39] */
#define ETH_HT1_BYTE4_MASK             (0xff << ETH_HT1_BYTE4_SHIFT)
#define ETH_HT1_BYTE5_SHIFT            (8)       /* Bits 8-15: Hash table byte 5, HT[40-47] */
#define ETH_HT1_BYTE5_MASK             (0xff << ETH_HT1_BYTE5_SHIFT)
#define ETH_HT1_BYTE6_SHIFT            (16)      /* Bits 16-23: Hash table byte 6, HT[48-55] */
#define ETH_HT1_BYTE6_MASK             (0xff << ETH_HT1_BYTE6_SHIFT)
#define ETH_HT1_BYTE7_SHIFT            (24)      /* Bits 24-31: Hash table byte 7, HT[56-63] */
#define ETH_HT1_BYTE7_MASK             (0xff << ETH_HT1_BYTE7_SHIFT)

/* Ethernet Controller Pattern Match Mask 0 Register */

#define ETH_PMM0_MASK0_SHIFT           (0)       /* Bits 0-7: Patch mask 0, PMM[0-7] */
#define ETH_PMM0_MASK0_MASK            (0xff << ETH_PMM0_MASK0_SHIFT)
#define ETH_PMM0_MASK1_SHIFT           (8)       /* Bits 8-15: Patch mask 1, PMM[8-15] */
#define ETH_PMM0_MASK1_MASK            (0xff << ETH_PMM0_MASK1_SHIFT)
#define ETH_PMM0_MASK2_SHIFT           (16)      /* Bits 16-23: Patch mask 2, PMM[16-23] */
#define ETH_PMM0_MASK2_MASK            (0xff << ETH_PMM0_MASK2_SHIFT)
#define ETH_PMM0_MASK3_SHIFT           (24)      /* Bits 24-31: Patch mask 3, PMM[24-31] */
#define ETH_PMM0_MASK3_MASK            (0xff << ETH_PMM0_MASK3_SHIFT)

/* Ethernet Controller Pattern Match Mask 1 Register */

#define ETH_PMM1_MASK4_SHIFT           (0)       /* Bits 0-7: Patch mask 4, PMM[32-39] */
#define ETH_PMM1_MASK4_MASK            (0xff << ETH_PMM1_MASK4_SHIFT)
#define ETH_PMM1_MASK5_SHIFT           (8)       /* Bits 8-15: Patch mask 5, PMM[40-47] */
#define ETH_PMM1_MASK5_MASK            (0xff << ETH_PMM1_MASK5_SHIFT)
#define ETH_PMM1_MASK6_SHIFT           (16)      /* Bits 16-23: Patch mask 6, PMM[48-55] */
#define ETH_PMM1_MASK6_MASK            (0xff << ETH_PMM1_MASK6_SHIFT)
#define ETH_PMM1_MASK7_SHIFT           (24)      /* Bits 24-31: Patch mask 7, PMM[56-63] */
#define ETH_PMM1_MASK7_MASK            (0xff << ETH_PMM1_MASK7_SHIFT)

/* Ethernet Controller Pattern Match Checksum Register */

#define ETH_PMCS_CKSM0_SHIFT           (0)       /* Bits 0-7: Pattern match checksum 0 bits, PMCS[0-7] */
#define ETH_PMCS_CKSM0_MASK            (0xff << ETH_PMCS_CKSM0_SHIFT)
#define ETH_PMCS_CKSM1_SHIFT           (8)       /* Bits 8-15: Pattern match checksum 1 bits, PMCS[8-15] */
#define ETH_PMCS_CKSM1_MASK            (0xff << ETH_PMCS_CKSM1_SHIFT)

/* Ethernet Controller Pattern Match Offset Register */

#define ETH_PMO_MASK                   (0xffff)

/* Flow Control Configuring Register */
/* Ethernet Controller Receive Watermarks Register */

#define ETH_RXWM_RXEWM_SHIFT           (0)       /* Bits 0-7: Receive empty watermark bits */
#define ETH_RXWM_RXEWM_MASK            (0xff << ETH_RXWM_RXEWM_SHIFT)
#define ETH_RXWM_RXFWM_SHIFT           (8)       /* Bits 8-15: Receive full watermark bits */
#define ETH_RXWM_RXFWM_MASK            (0xff << ETH_RXWM_RXFWM_SHIFT)

/* Ethernet Statistics Registers */
/* Ethernet Controller Receive Overflow Statistics Register */

#define ETH_RXOVFLOW_MASK              (0xffff)

/* Ethernet Controller Frames Transmitted OK Statistics Register */

#define ETH_FRMTXOK_MASK               (0xffff)

/* Ethernet Controller Single Collision Frames Statistics Register */

#define ETH_SCOLFRM_MASK               (0xffff)

/* Ethernet Controller Multiple Collision Frames Statistics Register */

#define ETH_MCOLFRM_MASK               (0xffff)

/* Ethernet Controller Frames Received OK Statistics Register */

#define ETH_FRMRXOK_MASK               (0xffff)

/* Ethernet Controller Frame Check Sequence Error Statistics Register */

#define ETH_FCSERR_MASK                (0xffff)

/* Ethernet Controller Alignment Errors Statistics Register */

#define ETH_ALGNERR_MASK               (0xffff)

/* MAC Configuration Registers */
/* Ethernet Controller MAC Configuration 1 Register */

#define EMAC1_CFG1_RXEN                (1 << 0)  /* Bit 0:  MAC Receive enable */
#define EMAC1_CFG1_PASSALL             (1 << 1)  /* Bit 1:  MAC Pass all all receive frames */
#define EMAC1_CFG1_RXPAUSE             (1 << 2)  /* Bit 2:  MAC RX flow control bit */
#define EMAC1_CFG1_TXPAUSE             (1 << 3)  /* Bit 3:  MAC TX flow control */
#define EMAC1_CFG1_LOOPBACK            (1 << 4)  /* Bit 4:  MAC loopback mode */
                                                 /* Bits 5-7: Reserved */
#define EMAC1_CFG1_TXRST               (1 << 8)  /* Bit 8:  Reset TX function */
#define EMAC1_CFG1_MCSTXRST            (1 << 9)  /* Bit 9:  Reset MCS/TX */
#define EMAC1_CFG1_RXRST               (1 << 10) /* Bit 10: Reset RX */
#define EMAC1_CFG1_MCSRXRST            (1 << 11) /* Bit 11: Reset MCS/RX */
                                                 /* Bits 12-13: Reserved */
#define EMAC1_CFG1_SIMRST              (1 << 14) /* Bit 14: Simulation reset */
#define EMAC1_CFG1_SOFTRST             (1 << 15) /* Bit 15: Soft reset */
                                                 /* Bits 16-31: Reserved */
/* Ethernet Controller MAC Configuration 2 Register */

#define EMAC1_CFG2_FULLDPLX            (1 << 0)  /* Bit 0:  Full duplex operation */
#define EMAC1_CFG2_LENGTHCK            (1 << 1)  /* Bit 1:  Frame length checking */
#define EMAC1_CFG2_HUGEFRM             (1 << 2)  /* Bit 2:  Huge frame enable */
#define EMAC1_CFG2_DELAYCRC            (1 << 3)  /* Bit 3:  Delayed CRC */
#define EMAC1_CFG2_CRCEN               (1 << 4)  /* Bit 4:  CRC enable */
#define EMAC1_CFG2_PADCRCEN            (1 << 5)  /* Bit 5:  Pad/CRC enable */
#define EMAC1_CFG2_VLANPADEN           (1 << 6)  /* Bit 6:  VLAN pad enable */
#define EMAC1_CFG2_AUTOPADEN           (1 << 7)  /* Bit 7:  Auto detect pad enable */
#define EMAC1_CFG2_PUREPRE             (1 << 8)  /* Bit 8:  Pure preamble enforcement */
#define EMAC1_CFG2_LONGPRE             (1 << 9)  /* Bit 9:  Long preamble enforcement */
                                                 /* Bits 10-11: Reserved */
#define EMAC1_CFG2_NOBKOFF             (1 << 12) /* Bit 12: No backoff */
#define EMAC1_CFG2_BPNOBKOFF           (1 << 13) /* Bit 13: Back pressure/no backoff */
#define EMAC1_CFG2_EXCESSDFR           (1 << 14) /* Bit 14: Excess defer */
                                                 /* Bits 15-31: Reserved */
/* Ethernet Controller MAC Back-to-Back Interpacket Gap Register */

#define EMAC1_IPGT_SHIFT               (0)       /* Bits 0-6 */
#define EMAC1_IPGT_MASK                (0x7f << EMAC1_IPGT_SHIFT)
                                                 /* Bits 7-31: Reserved */
/* Ethernet Controller MAC Non-Back-to-Back Interpacket Gap Register */

#define EMAC1_IPGR_GAP2_SHIFT          (0)       /* Bits 0-6: Gap part 2 */
#define EMAC1_IPGR_GAP2_MASK           (0x7f << EMAC1_IPGR_GAP2_SHIFT)
                                                 /* Bit 7: Reserved */
#define EMAC1_IPGR_GAP1_SHIFT          (8)       /* Bits 8-18: Gap part 1 */
#define EMAC1_IPGR_GAP1_MASK           (0x7f << EMAC1_IPGR_GAP2_SHIFT)
                                                 /* Bits 15-31: Reserved */
/* Ethernet Controller MAC Collision Window/Retry Limit Register */

#define EMAC1_CLRT_RETX_SHIFT          (0)       /* Bits 0-3: Retransmission maximum */
#define EMAC1_CLRT_RETX_MASK           (15 << EMAC1_CLRT_RETX_SHIFT)
                                                 /* Bits 4-7: Reserved */
#define EMAC1_CLRT_CWINDOW_SHIFT       (8)       /* Bits 8-13: Collision window */
#define EMAC1_CLRT_CWINDOW_MASK        (0x3f << EMAC1_CLRT_CWINDOW_SHIFT)
                                                 /* Bits 14-31: Reserved */
/* Ethernet Controller MAC Maximum Frame Length Register */

#define EMAC1_MAXF_SHIFT               (0)       /* Bits 0-15 */
#define EMAC1_MAXF_MASK                (0xffff << EMAC1_MAXF_SHIFT)
                                                 /* Bits 16-31: Reserved */
/* Ethernet Controller MAC PHY Support Register */
                                                 /* Bits 0-7: Reserved */
#define EMAC1_SUPP_SPEEDRMII           (1 << 8)  /* Bit 8:  RMII Speed0=10Bps 1=100Bps */
                                                 /* Bits 9-10: Reserved */
#define EMAC1_SUPP_RESETRMII           (1 << 11) /* Bit 11: Reset RMII Logic */
                                                 /* Bits 12-31: Reserved */
/* Ethernet Controller MAC Test Register */

#define EMAC1_TEST_SHRTQNTA            (1 << 0)  /* Bit 0:  Shortcut pause quanta */
#define EMAC1_TEST_TESTPAUSE           (1 << 1)  /* Bit 1:  Test pause */
#define EMAC1_TEST_TESTBP              (1 << 2)  /* Bit 2:  Test packpressure */
                                                 /* Bits 3-31: Reserved */
/* Ethernet Controller MAC Station Address 0 Register */

#define EMAC1_SA0_STNADDR6_SHIFT       (0)       /* Bits 0-7: Station address 5th octet */
#define EMAC1_SA0_STNADDR6_MASK        (0xff << EMAC1_SA0_STNADDR6_SHIFT)
#define EMAC1_SA0_STNADDR5_SHIFT       (8)       /* Bits 8-15: Station address 6th octet */
#define EMAC1_SA0_STNADDR5_MASK        (0xff << EMAC1_SA0_STNADDR5_SHIFT)
                                                 /* Bits 16-31: Reserved */
/* Ethernet Controller MAC Station Address 1 Register */

#define EMAC1_SA1_STNADDR4_SHIFT       (0)       /* Bits 0-7: Station address 4th octet */
#define EMAC1_SA1_STNADDR4_MASK        (0xff << EMAC1_SA0_STNADDR4_SHIFT)
#define EMAC1_SA1_STNADDR3_SHIFT       (8)       /* Bits 8-15: Station address 3rd octet */
#define EMAC1_SA1_STNADDR3_MASK        (0xff << EMAC1_SA0_STNADDR3_SHIFT)
                                                 /* Bits 16-31: Reserved */
/* Ethernet Controller MAC Station Address 2 Register */

#define EMAC1_SA2_STNADDR2_SHIFT       (0)       /* Bits 0-7: Station address 2nd octet */
#define EMAC1_SA2_STNADDR2_MASK        (0xff << EMAC1_SA2_STNADDR2_SHIFT)
#define EMAC1_SA2_STNADDR1_SHIFT       (8)       /* Bits 8-15: Station address 1st octet */
#define EMAC1_SA2_STNADDR1_MASK        (0xff << EMAC1_SA2_STNADDR1_SHIFT)
                                                 /* Bits 16-31: Reserved */
/* MII Management Registers */

/* Ethernet Controller MAC MII Management Configuration Register */

#define EMAC1_MCFG_SCANINC             (1 << 0)  /* Bit 0:  Scan increment */
#define EMAC1_MCFG_NOPRE               (1 << 1)  /* Bit 1:  Suppress preamble */
#define EMAC1_MCFG_CLKSEL_SHIFT        (2)       /* Bits 2-5: Clock select */
#define EMAC1_MCFG_CLKSEL_MASK         (15 << EMAC1_MCFG_CLKSEL_SHIFT)
#  define EMAC1_MCFG_CLKSEL_DIV4       (0 << EMAC1_MCFG_CLKSEL_SHIFT)
#  define EMAC1_MCFG_CLKSEL_DIV6       (2 << EMAC1_MCFG_CLKSEL_SHIFT)
#  define EMAC1_MCFG_CLKSEL_DIV8       (3 << EMAC1_MCFG_CLKSEL_SHIFT)
#  define EMAC1_MCFG_CLKSEL_DIV10      (4 << EMAC1_MCFG_CLKSEL_SHIFT)
#  define EMAC1_MCFG_CLKSEL_DIV14      (5 << EMAC1_MCFG_CLKSEL_SHIFT)
#  define EMAC1_MCFG_CLKSEL_DIV20      (6 << EMAC1_MCFG_CLKSEL_SHIFT)
#  define EMAC1_MCFG_CLKSEL_DIV40      (8 << EMAC1_MCFG_CLKSEL_SHIFT)
                                                 /* Bits 6-14: Reserved */
#define EMAC1_MCFG_MGMTRST             (1 << 15) /* Bit 15: Reset MII mgmt */
                                                 /* Bits 16-31: Reserved */

/* Ethernet Controller MAC MII Management Command Register */

#define EMAC1_MCMD_READ                (1 << 0)  /* Bit 0:  Single read cycle */
#define EMAC1_MCMD_SCAN                (1 << 1)  /* Bit 1:  Continuous read cycles */
                                                 /* Bits 2-31: Reserved */
#define EMAC1_MCMD_WRITE               (0)

/* Ethernet Controller MAC MII Management Address Register */

#define EMAC1_MADR_REGADDR_SHIFT       (0)       /* Bits 0-4: Register address */
#define EMAC1_MADR_REGADDR_MASK        (31 << EMAC1_MADR_REGADDR_SHIFT)
                                                 /* Bits 7-5: Reserved */
#define EMAC1_MADR_PHYADDR_SHIFT       (8)       /* Bits 8-12: PHY address */
#define EMAC1_MADR_PHYADDR_MASK        (31 << EMAC1_MADR_PHYADDR_SHIFT)
                                                 /* Bits 13-31: Reserved */
/* Ethernet Controller MAC MII Management Write Data Register */

#define EMAC1_MWTD_SHIFT               (0)       /* Bits 0-15 */
#define EMAC1_MWTD_MASK                (0xffff << EMAC1_MWTD_SHIFT)
                                                 /* Bits 16-31: Reserved */
/* Ethernet Controller MAC MII Management Read Data Register */

#define EMAC1_MRDD_SHIFT               (0)       /* Bits 0-15 */
#define EMAC1_MRDD_MASK                (0xffff << EMAC1_MRDD_SHIFT)
                                                 /* Bits 16-31: Reserved */
/* Ethernet Controller MAC MII Management Indicators Register */

#define EMAC1_MIND_MIIMBUSY            (1 << 0)  /* Bit 0:  Busy */
#define EMAC1_MIND_SCAN                (1 << 1)  /* Bit 1:  Scanning */
#define EMAC1_MIND_NOTVALID            (1 << 2)  /* Bit 2:  Not valid */
#define EMAC1_MIND_LINKFAIL            (1 << 3)  /* Bit 3:  MII link fail */
                                                 /* Bits 4-31: Reserved */

/* Descriptors Offsets **********************************************************************/

/* Tx descriptor offsets.  The NEXTED field is only present if NPV=1 */

#define PIC32MX_TXDESC_STATUS          0x00      /* Various status bits (32-bits) */
#define PIC32MX_TXDESC_ADDRESS         0x04      /* Data buffer address (32-bits) */
#define PIC32MX_TXDESC_TSV1            0x08      /* Transmit filter status vector 1 (32-bits) */
#define PIC32MX_TXDESC_TSV2            0x0c      /* Transmit filter status vector 2 (32-bits) */
#define PIC32MX_TXLINEAR_SIZE          0x10      /* Size in bytes of one linear Tx descriptor */

#define PIC32MX_TXDESC_NEXTED          0x10      /* Next Ethernet Descriptor (ED) */
#define PIC32MX_TXLINKED_SIZE          0x14      /* Size in bytes of one linked Tx descriptor */

/* Tx descriptor uint32_t* indices */

#define TXDESC_STATUS                  0         /* Various status bits (32-bits) */
#define TXDESC_ADDRESS                 1         /* Data buffer address (32-bits) */
#define TXDESC_TSV1                    2         /* Transmit filter status vector 1 (32-bits) */
#define TXDESC_TSV2                    3         /* Transmit filter status vector 2 (32-bits) */
#define TXLINEAR_SIZE                  4         /* Size in 32-bit words of one linear Tx descriptor */

#define TXDESC_NEXTED                  4         /* Next Ethernet Descriptor (ED) */
#define TXLINKED_SIZE                  5         /* Size in 32-bit words of one linked Tx descriptor */

/* Rx descriptor offsets.  The NEXTED field is only present if NPV=1 */

#define PIC32MX_RXDESC_STATUS          0x00      /* Various status bits (32-bits) */
#define PIC32MX_RXDESC_ADDRESS         0x04      /* Data buffer address (32-bits) */
#define PIC32MX_RXDESC_RSV1            0x08      /* Receive filter status vector 1 and checksum (32-bits) */
#define PIC32MX_RXDESC_RSV2            0x0c      /* Receive filter status vector 2 (32-bits) */
#define PIC32MX_RXLINEAR_SIZE          0x10      /* Size in bytes of one linear Rx descriptor */

#define PIC32MX_RXDESC_NEXTED          0x10      /* Next Ethernet Descriptor (ED) */
#define PIC32MX_RXLINKED_SIZE          0x14      /* Size in bytes of one linked Rx descriptor */

/* Rx descriptor offsets uint32_t* indices */

#define RXDESC_STATUS                  0         /* Various status bits (32-bits) */
#define RXDESC_ADDRESS                 1         /* Data buffer address (32-bits) */
#define RXDESC_RSV1                    2         /* Receive filter status vector 1 and checksum (32-bits) */
#define RXDESC_RSV2                    3         /* Receive filter status vector 2 (32-bits) */
#define RXLINEAR_SIZE                  4         /* Size in 32-bit words of one linear Rx descriptor */

#define RXDESC_NEXTED                  4         /* Next Ethernet Descriptor (ED) */
#define RXLINKED_SIZE                  5         /* Size in 32-bit words of one linked Rx descriptor */

/* Descriptor Bit Definitions ***************************************************************/
/* Tx descriptor status bit definitions */
                                                 /* Bits 0-6: Reserved */
#define TXDESC_STATUS_EOWN             (1 << 7)  /* Bit 7:  1=Ethernet controller owns  */
#define TXDESC_STATUS_SOWN             (0)       /*         0=Software owns  */
#define TXDESC_STATUS_NPV              (1 << 8)  /* Bit 8:  Next ED pointer valid enable */
#define TXDESC_STATUS_USER1_SHIFT      (9)       /* Bits 9-15: User-defined  */
#define TXDESC_STATUS_USER1_MASK       (0x7f << TXDESC_STATUS_USER2_SHIFT)
#define TXDESC_STATUS_BYTECOUNT_SHIFT  (16)       /* Bits 16-26: Byte Count */
#define TXDESC_STATUS_BYTECOUNT_MASK   (0x7ff << TXDESC_STATUS_BYTECOUNT_SHIFT)
#define TXDESC_STATUS_USER2_SHIFT      (27)       /* Bits 27-29: User-defined  */
#define TXDESC_STATUS_USER2_MASK       (7 << TXDESC_STATUS_USER1_SHIFT)
#define TXDESC_STATUS_EOP              (1 << 30) /* Bit 30: End of packet enable */
#define TXDESC_STATUS_SOP              (1 << 31) /* Bit 31: Start of packet enable  */

/* Tx descriptor Transmit filter status vector bit definitions */

#define TXDESC_TSV1_BYTECOUNT_SHIFT    (0)       /* Bits 0-15: TSV[32-47] Total bytes transmitted */
#define TXDESC_TSV1_BYTECOUNT_MASK     (0xffff << TXDESC_TSV1_BYTECOUNT_SHIFT)
#define TXDESC_TSV1_CONTROL            (1 << 16) /* Bit 16: TSV48 Transmit Control Frame */
#define TXDESC_TSV1_PAUSE              (1 << 17) /* Bit 17: TSV49 Transmit PAUSE Control Frame */
#define TXDESC_TSV1_BPAPPLIED          (1 << 18) /* Bit 18: TSV50 Transmit Backpressure Applied */
#define TXDESC_TSV1_VLAN               (1 << 19) /* Bit 19: TSV51 Transmit VLAN Tagged Frame */
                                                 /* Bits 20-23: Reserved */
#define TXDESC_TSV1_USER_SHIFT         (24)      /* Bits 24-31: User-defined  */
#define TXDESC_TSV1_USER_MASK          (0xff << TXDESC_STATUS_USER1_SHIFT)

#define TXDESC_TSV2_BYTECOUNT_SHIFT    (0)       /* Bits 0-15: TSV15:0 Transmit Byte Count */
#define TXDESC_TSV2_BYTECOUNT_MASK     (0xffff << TXDESC_TSV2_BYTECOUNT_SHIFT)
#define TXDESC_TSV2_COLCOUNT_SHIFT     (0)       /* Bits 16-19: TSV16-19 Transmit Collision Count */
#define TXDESC_TSV2_COLCOUNT_MASK      (15 << TXDESC_TSV2_COLCOUNT_SHIFT)
#define TXDESC_TSV2_TXCRCE             (1 << 20) /* Bit 20: TSV20 Transmit CRC Error */
#define TXDESC_TSV2_TXLCE              (1 << 21) /* Bit 21: TSV21 Transmit Length Check Error */
#define TXDESC_TSV2_TXOOR              (1 << 22) /* Bit 22: TSV22 Transmit Length Out Of Range */
#define TXDESC_TSV2_TXDONE             (1 << 23) /* Bit 23: TSV23 Transmit Done */
#define TXDESC_TSV2_MCAST              (1 << 24) /* Bit 24: TSV24 Transmit Multicast */
#define TXDESC_TSV2_BCAST              (1 << 25) /* Bit 25: TSV25 Transmit Broadcast */
#define TXDESC_TSV2_PKTDFR             (1 << 26) /* Bit 26: TSV26 Transmit Packet Defer */
#define TXDESC_TSV2_EXCESSDFR          (1 << 27) /* Bit 27: TSV27 Transmit Excessive Defer */
#define TXDESC_TSV2_MAXOL              (1 << 28) /* Bit 28: TSV28 Transmit Maximum Collision */
#define TXDESC_TSV2_TXLC               (1 << 29) /* Bit 29: TSV29 Transmit Late Collision */
#define TXDESC_TSV2_TXGIANT            (1 << 30) /* Bit 30: TSV30 Transmit Giant */
#define TXDESC_TSV2_TXUR               (1 << 31) /* Bit 31: TSV31 Transmit Under-run */

/* Rx descriptor status bit definitions */
                                                 /* Bits 0-6: Reserved */
#define RXDESC_STATUS_EOWN             (1 << 7)  /* Bit 7:  1=Ethernet controller owns  */
#define RXDESC_STATUS_SOWN             (0)       /*         0=Software owns  */
#define RXDESC_STATUS_NPV              (1 << 8)  /* Bit 8:  Next ED pointer valid enable */
                                                 /* Bits 9-15: Reserved  */
#define RXDESC_STATUS_BYTECOUNT_SHIFT  (16)      /* Bits 16-26: Byte Count */
#define RXDESC_STATUS_BYTECOUNT_MASK   (0x7ff << RXDESC_STATUS_BYTECOUNT_SHIFT)
                                                 /* Bits 27-29: Reserved  */
#define RXDESC_STATUS_EOP              (1 << 30) /* Bit 30: End of packet enable */
#define RXDESC_STATUS_SOP              (1 << 31) /* Bit 31: Start of packet enable */

/* Rx descriptor receive filter status vector bit definitions */

#define RXDESC_RSV1_CHECKSUM_SHIFT     (0)      /* Bits 0-15: RX Packet Payload Checksum */
#define RXDESC_RSV1_CHECKSUM_MASK      (0xffff << RXDESC_RSV1_CHECKSUM_SHIFT)
#define RXDESC_RSV1_USER_SHIFT         (16)      /* Bits 16-23: User defined */
#define RXDESC_RSV1_USER_MASK          (0xff << RXDESC_RSV1_USER_SHIFT)
#define RXDESC_RSV1_RUNT               (1 << 24) /* Bit 24: RXF_RSV0 Runt packet */
#define RXDESC_RSV1_NOTANDNOT          (1 << 25) /* Bit 25: RXF_RSV1 NOT (Unicast match) AND NOT (Multicast Match) */
#define RXDESC_RSV1_HTMATCH            (1 << 26) /* Bit 26: RXF_RSV2 Hash Table match */
#define RXDESC_RSV1_MAGIC              (1 << 27) /* Bit 27: RXF_RSV3 Magic Packet match */
#define RXDESC_RSV1_PATMATCH           (1 << 28) /* Bit 28: RXF_RSV4 Pattern Match match */
#define RXDESC_RSV1_UCASTMATCH         (1 << 29) /* Bit 29: RXF_RSV5 Unicast match */
#define RXDESC_RSV1_BCASTMATCH         (1 << 30) /* Bit 30: RXF_RSV6 Broadcast match */
#define RXDESC_RSV1_MCASTMATCH         (1 << 31) /* Bit 31: RXF_RSV7 Multicast match */

#define RXDESC_RSV2_BYTECOUNT_SHIFT    (0)       /* Bits 0-15:  RSV0-15 Received Byte Count */
#define RXDESC_RSV2_BYTECOUNT_MASK     (0xffff << RXDESC_RSV2_BYTECOUNT_SHIFT)
#define RXDESC_RSV2_LONGDROP           (1 << 16) /* Bit 16: RSV16 Long Event/Drop Event */
#define RXDESC_RSV2_RXDVSEEN           (1 << 17) /* Bit 17: RSV17 RXDV Event Previously Seen */
#define RXDESC_RSV2_CARSEEN            (1 << 18) /* Bit 18: RSV18 Carrier Event Previously Seen */
#define RXDESC_RSV2_CODE               (1 << 19) /* Bit 19: RSV19 Receive Code Violation */
#define RXDESC_RSV2_CRCERR             (1 << 20) /* Bit 20: RSV20 CRC Error */
#define RXDESC_RSV2_LENCHK             (1 << 21) /* Bit 21: RSV21 Length Check Error */
#define RXDESC_RSV2_OOR                (1 << 22) /* Bit 22: RSV22 Length Out of Range */
#define RXDESC_RSV2_OK                 (1 << 23) /* Bit 23: RSV23 Received Ok */
#define RXDESC_RSV2_MCAST              (1 << 24) /* Bit 24: RSV24 Receive Multicast Packet */
#define RXDESC_RSV2_BCAST              (1 << 25) /* Bit 25: RSV25 Receive Broadcast Packet */
#define RXDESC_RSV2_DRIBBLE            (1 << 26) /* Bit 26: RSV26 Dribble Nibble */
#define RXDESC_RSV2_CONTROL            (1 << 27) /* Bit 27: RSV27 Receive Control Frame */
#define RXDESC_RSV2_PAUSE              (1 << 28) /* Bit 28: RSV28 Receive Pause Control Frame */
#define RXDESC_RSV2_UNKNOWNOP          (1 << 29) /* Bit 29: RSV29 Receive Unknown Op code */
#define RXDESC_RSV2_VLAN               (1 << 30) /* Bit 30: RSV30 Receive VLAN Type Detected */
                                                 /* Bit 31: RSV31 Reserved */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

#ifndef __ASSEMBLY__

/* Descriptors as structures */

/* Tx descriptor with NPV=0 */

struct pic32mx_txlinear_s
{
  uint32_t status;                               /* Various status bits (32-bits) */
  uint32_t address;                              /* Data buffer address (32-bits) */
  uint32_t tsv1;                                 /* Transmit filter status vector 1 (32-bits) */
  uint32_t tsv2;                                 /* Transmit filter status vector 2 (32-bits) */
};

/* Tx descriptor with NPV=1 */

struct pic32mx_txdesc_s
{
  uint32_t status;                               /* Various status bits (32-bits) */
  uint32_t address;                              /* Data buffer address (32-bits) */
  uint32_t tsv1;                                 /* Transmit filter status vector 1 (32-bits) */
  uint32_t tsv2;                                 /* Transmit filter status vector 2 (32-bits) */
  uint32_t nexted;                               /* Next Ethernet Descriptor (ED) */
};

/* Rx descriptor with NPV=0 */

struct pic32mx_rxlinear_s
{
  uint32_t status;                               /* Various status bits (32-bits) */
  uint32_t address;                              /* Data buffer address (32-bits) */
  uint32_t rsv1;                                 /* Receive filter status vector 1 and checksum (32-bits) */
  uint32_t rsv2;                                 /* Receive filter status vector 2 (32-bits) */
};

/* Rx descriptor with NPV=1 */

struct pic32mx_rxdesc_s
{
  uint32_t status;                               /* Various status bits (32-bits) */
  uint32_t address;                              /* Data buffer address (32-bits) */
  uint32_t rsv1;                                 /* Receive filter status vector 1 and checksum (32-bits) */
  uint32_t rsv2;                                 /* Receive filter status vector 2 (32-bits) */
  uint32_t nexted;                               /* Next Ethernet Descriptor (ED) */
};

/********************************************************************************************
 * Inline Functions
 ********************************************************************************************/

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_ETHERNET_H */
