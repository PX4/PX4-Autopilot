/************************************************************************************
 * arch/arm/src/lm/chip/lm_epi.h
 *
 *   Copyright (C) 2009-2013 Max Neklyudov. All rights reserved.
 *   Author: Max Neklyudov <macscomp@gmail.com>
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM_EPI_H
#define __ARCH_ARM_SRC_LM_CHIP_LM_EPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* External Peripheral Interface Register Offsets ***********************************/

#define LM_EPI_CFG_OFFSET             0x000
#define LM_EPI_SDRAMCFG_OFFSET        0x010
#define LM_EPI_ADDRMAP_OFFSET         0x01C
#define LM_EPI_STAT_OFFSET            0x060
#define LM_EPI_BAUD_OFFSET            0x004

/* External Peripheral Interface Register Addresses *********************************/

#define LM_EPI0_CFG                   (LM_EPI0_BASE + LM_EPI_CFG_OFFSET)
#define LM_EPI0_SDRAMCFG              (LM_EPI0_BASE + LM_EPI_SDRAMCFG_OFFSET)
#define LM_EPI0_ADDRMAP               (LM_EPI0_BASE + LM_EPI_ADDRMAP_OFFSET)
#define LM_EPI0_STAT                  (LM_EPI0_BASE + LM_EPI_STAT_OFFSET)
#define LM_EPI0_BAUD                  (LM_EPI0_BASE + LM_EPI_BAUD_OFFSET)

/* External Peripheral Interface Register Bit Definitions ***************************/

/* EPI Configuration (EPICFG), offset 0x000 */

#define EPI_CFG_MODE_SHIFT            0         /* Bits 3-0: Mode Select */
#define EPI_CFG_MODE_MASK             (0x1f << EPI_CFG_MODE_SHIFT)
#  define EPI_CFG_MODE_SDRAM          (0x11 << EPI_CFG_MODE_SHIFT)  /* SDRAM + BLKEN */

/* EPI Address Map (EPIADDRMAP), offset 0x01C */

#define EPI_ADDRMAP_ERADR_SHIFT       0       /* Bits 1-0: External RAM Address */
#define EPI_ADDRMAP_ERADR_MASK        (0x3 << EPI_ADDRMAP_ERADR_SHIFT)
#  define EPI_ADDRMAP_ERADR_6         (0x1 << EPI_ADDRMAP_ERADR_SHIFT)
#  define EPI_ADDRMAP_ERADR_8         (0x2 << EPI_ADDRMAP_ERADR_SHIFT)
#define EPI_ADDRMAP_ERSZ_SHIFT        2       /* Bits 3-2: External RAM Size */
#define EPI_ADDRMAP_ERSZ_MASK         (0x3 << EPI_ADDRMAP_ERSZ_SHIFT)
#  define EPI_ADDRMAP_ERSZ_256B       (0x0 << EPI_ADDRMAP_ERSZ_SHIFT)
#  define EPI_ADDRMAP_ERSZ_64KB       (0x1 << EPI_ADDRMAP_ERSZ_SHIFT)
#  define EPI_ADDRMAP_ERSZ_16MB       (0x2 << EPI_ADDRMAP_ERSZ_SHIFT)
#  define EPI_ADDRMAP_ERSZ_512MB      (0x3 << EPI_ADDRMAP_ERSZ_SHIFT)

/* EPI Status (EPISTAT), offset 0x060 */

#define EPI_STAT_INITSEQ_SHIFT        6       /* Bits 6: Initialization Sequence */
#define EPI_STAT_INITSEQ_MASK         (0x1 << EPI_STAT_INITSEQ_SHIFT)

/* EPI SDRAM Configuration (EPISDRAMCFG), offset 0x010 */

#define EPI_SDRAMCFG_SIZE_SHIFT       0       /* Bits 1-0: Size of SDRAM */
#define EPI_SDRAMCFG_SIZE_MASK        (3 << EPI_SDRAMCFG_SIZE_SHIFT)
#  define EPI_SDRAMCFG_SIZE_8MB       (0x0 << EPI_SDRAMCFG_SIZE_SHIFT)
#  define EPI_SDRAMCFG_SIZE_16MB      (0x1 << EPI_SDRAMCFG_SIZE_SHIFT)
#  define EPI_SDRAMCFG_SIZE_32MB      (0x2 << EPI_SDRAMCFG_SIZE_SHIFT)
#  define EPI_SDRAMCFG_SIZE_64MB      (0x3 << EPI_SDRAMCFG_SIZE_SHIFT)
#define EPI_SDRAMCFG_RFSH_SHIFT       16      /* Bits 26-16: Refresh Counter */
#define EPI_SDRAMCFG_RFSH_MASK        (0x7FF << EPI_SDRAMCFG_RFSH_SHIFT)
#  define EPI_SDRAMCFG_RFSH(n)        ((n) << EPI_SDRAMCFG_RFSH_SHIFT)
#define EPI_SDRAMCFG_FREQ_SHIFT       30      /* EPI Frequency Range */
#define EPI_SDRAMCFG_FREQ_MASK        (3 << EPI_SDRAMCFG_FREQ_SHIFT)
#  define EPI_SDRAMCFG_FREQ_0_15MHZ   (0x0 << EPI_SDRAMCFG_FREQ_SHIFT)
#  define EPI_SDRAMCFG_FREQ_15_30MHZ  (0x1 << EPI_SDRAMCFG_FREQ_SHIFT)
#  define EPI_SDRAMCFG_FREQ_30_50MHZ  (0x2 << EPI_SDRAMCFG_FREQ_SHIFT)
#  define EPI_SDRAMCFG_FREQ_50_100MHZ (0x3 << EPI_SDRAMCFG_FREQ_SHIFT)

/* EPI Main Baud Rate (EPIBAUD), offset 0x004 */

#define EPI_BAUD_COUNT0_SHIFT         0
#define EPI_BAUD_COUNT0_MASK          (0xFFFF << EPI_BAUD_COUNT0_SHIFT)
#  define EPI_BAUD_COUNT0(n)          ((n) << EPI_BAUD_COUNT0_SHIFT)

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM_EPI_H */
