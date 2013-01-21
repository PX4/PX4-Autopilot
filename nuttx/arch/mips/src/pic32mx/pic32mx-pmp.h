/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-pmp.h
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
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PMP_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PMP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_PMP_CON_OFFSET     0x0000 /* Parallel Port Control Register */
#define PIC32MX_PMP_CONCLR_OFFSET  0x0004 /* Parallel Port Control Clear Register */
#define PIC32MX_PMP_CONSET_OFFSET  0x0008 /* Parallel Port Control Set Register */
#define PIC32MX_PMP_CONINV_OFFSET  0x000c /* Parallel Port Control Invert Register */
#define PIC32MX_PMP_MODE_OFFSET    0x0010 /* Parallel Port Mode Register */
#define PIC32MX_PMP_MODECLR_OFFSET 0x0014 /* Parallel Port Mode Clear Register */
#define PIC32MX_PMP_MODESET_OFFSET 0x0018 /* Parallel Port Mode Set Register */
#define PIC32MX_PMP_MODEINV_OFFSET 0x001c /* Parallel Port Mode Invert Register */
#define PIC32MX_PMP_ADDR_OFFSET    0x0020 /* Parallel Port Address Register */
#define PIC32MX_PMP_ADDRCLR_OFFSET 0x0024 /* Parallel Port Address Clear Register */
#define PIC32MX_PMP_ADDRSET_OFFSET 0x0028 /* Parallel Port Address Set Register */
#define PIC32MX_PMP_ADDRINV_OFFSET 0x002c /* Parallel Port Address Invert Register */
#define PIC32MX_PMP_DOUT_OFFSET    0x0030 /* Parallel Port Data Output Register */
#define PIC32MX_PMP_DOUTCLR_OFFSET 0x0034 /* Parallel Port Data Output Clear Register */
#define PIC32MX_PMP_DOUTSET_OFFSET 0x0038 /* Parallel Port Data Output Set Register */
#define PIC32MX_PMP_DOUTINV_OFFSET 0x003c /* Parallel Port Data Output Invert Register */
#define PIC32MX_PMP_DIN_OFFSET     0x0040 /* Parallel Port Data Input Register */
#define PIC32MX_PMP_DINCLR_OFFSET  0x0044 /* Parallel Port Data Input Clear Register */
#define PIC32MX_PMP_DINSET_OFFSET  0x0048 /* Parallel Port Data Input Set Register */
#define PIC32MX_PMP_DININV_OFFSET  0x004c /* Parallel Port Data Input Invert Register */
#define PIC32MX_PMP_AEN_OFFSET     0x0050 /* Parallel Port Pin Enable Register */
#define PIC32MX_PMP_AENCLR_OFFSET  0x0054 /* Parallel Port Pin Enable Clear Register */
#define PIC32MX_PMP_AENSET_OFFSET  0x0058 /* Parallel Port Pin Enable Set Register */
#define PIC32MX_PMP_AENINV_OFFSET  0x005c /* Parallel Port Pin Enable Invert Register */
#define PIC32MX_PMP_STAT_OFFSET    0x0060 /* Parallel Port Status Register */
#define PIC32MX_PMP_STATCLR_OFFSET 0x0064 /* Parallel Port Status Clear Register */
#define PIC32MX_PMP_STATSET_OFFSET 0x0068 /* Parallel Port Status Set Register */
#define PIC32MX_PMP_STATINV_OFFSET 0x006c /* Parallel Port Status Invert Register */

/* Register Addresses ***************************************************************/

#define PIC32MX_PMP_CON            (PIC32MX_PMP_K1BASE+PIC32MX_PMP_CON_OFFSET)
#define PIC32MX_PMP_CONCLR         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_CONCLR_OFFSET)
#define PIC32MX_PMP_CONSET         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_CONSET_OFFSET)
#define PIC32MX_PMP_CONINV         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_CONINV_OFFSET)
#define PIC32MX_PMP_MODE           (PIC32MX_PMP_K1BASE+PIC32MX_PMP_MODE_OFFSET)
#define PIC32MX_PMP_MODECLR        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_MODECLR_OFFSET)
#define PIC32MX_PMP_MODESET        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_MODESET_OFFSET)
#define PIC32MX_PMP_MODEINV        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_MODEINV_OFFSET)
#define PIC32MX_PMP_ADDR           (PIC32MX_PMP_K1BASE+PIC32MX_PMP_ADDR_OFFSET)
#define PIC32MX_PMP_ADDRCLR        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_ADDRCLR_OFFSET)
#define PIC32MX_PMP_ADDRSET        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_ADDRSET_OFFSET)
#define PIC32MX_PMP_ADDRINV        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_ADDRINV_OFFSET)
#define PIC32MX_PMP_DOUT           (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DOUT_OFFSET)
#define PIC32MX_PMP_DOUTCLR        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DOUTCLR_OFFSET)
#define PIC32MX_PMP_DOUTSET        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DOUTSET_OFFSET)
#define PIC32MX_PMP_DOUTINV        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DOUTINV_OFFSET)
#define PIC32MX_PMP_DIN            (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DIN_OFFSET)
#define PIC32MX_PMP_DINCLR         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DINCLR_OFFSET)
#define PIC32MX_PMP_DINSET         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DINSET_OFFSET)
#define PIC32MX_PMP_DININV         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_DININV_OFFSET)
#define PIC32MX_PMP_AEN            (PIC32MX_PMP_K1BASE+PIC32MX_PMP_AEN_OFFSET)
#define PIC32MX_PMP_AENCLR         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_AENCLR_OFFSET)
#define PIC32MX_PMP_AENSET         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_AENSET_OFFSET)
#define PIC32MX_PMP_AENINV         (PIC32MX_PMP_K1BASE+PIC32MX_PMP_AENINV_OFFSET)
#define PIC32MX_PMP_STAT           (PIC32MX_PMP_K1BASE+PIC32MX_PMP_STAT_OFFSET)
#define PIC32MX_PMP_STATCLR        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_STATCLR_OFFSET)
#define PIC32MX_PMP_STATSET        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_STATSET_OFFSET)
#define PIC32MX_PMP_STATINV        (PIC32MX_PMP_K1BASE+PIC32MX_PMP_STATINV_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Parallel Port Control Register */

#define PMP_CON_RDSP               (1 << 0)  /* Bit 0:  Read strobe polarity */
#define PMP_CON_WRSP               (1 << 1)  /* Bit 1:  Write strobe polarity */
#define PMP_CON_CS1P               (1 << 3)  /* Bit 3:  Chip select 0 polarity */
#define PMP_CON_CS2P               (1 << 4)  /* Bit 4:  Chip select 1 polarity */
#define PMP_CON_ALP                (1 << 5)  /* Bit 5:  Address latch polarity */
#define PMP_CON_CSF_SHIFT          (6)       /* Bits 6-7: Chip select function */
#define PMP_CON_CSF_MASK           (3 << PMP_CON_CSF_SHIFT)
#  define PMP_CON_CSF_ADDR1415     (0 << PMP_CON_CSF_SHIFT) /* PMCS2/PMCS1 = address bits 15 and 14 */
#  define PMP_CON_CSF_CS2ADDR14    (1 << PMP_CON_CSF_SHIFT) /* PMCS2 = Chip Select, PMCS1 = address bit 14 */
#  define PMP_CON_CSF_CS12         (2 << PMP_CON_CSF_SHIFT) /* PMCS2/PMCS1 = Chip Select */
#define PMP_CON_PTRDEN             (1 << 8)  /* Bit 8:  Read/write strobe port enable */
#define PMP_CON_PTWREN             (1 << 9)  /* Bit 9:  Write enable strobe port enable */
#define PMP_CON_PMPTTL             (1 << 10) /* Bit 10: PMP module TTL input buffer select */
#define PMP_CON_ADRMUX_SHIFT       (11)      /* Bits 11-12: Address/data multiplexing selection */
#define PMP_CON_ADRMUX_MASK        (3 << PMP_CON_ADRMUX_SHIFT)
#  define PMP_CON_ADRMUX_NONE      (0 << PMP_CON_ADRMUX_SHIFT) /* Address and data appear separate */
#  define PMP_CON_ADRMUX_BYTE      (1 << PMP_CON_ADRMUX_SHIFT) /* LS address are mux'ed on PMD 7:0 MS on PMA 15:8 */
#  define PMP_CON_ADRMUX_MUX8      (2 << PMP_CON_ADRMUX_SHIFT) /* Address mux'ed on PMD 7:0 */
#  define PMP_CON_ADRMUX_MUX16     (3 << PMP_CON_ADRMUX_SHIFT) /* Address mux'ed on PMD 15:0 */
#define PMP_CON_SIDL               (1 << 13) /* Bit 13: Stop in idle mode */
#define PMP_CON_FRZ                (1 << 14) /* Bit 14: Freeze in debug exception mode */
#define PMP_CON_ON                 (1 << 15) /* Bit 15: Parallel master port enable */

/* Parallel Port Mode Register */

#define PMP_MODE_WAITE_SHIFT       (0)       /* Bits 0-1: Data hold after R/W strobe wait states */
#define PMP_MODE_WAITE_MASK        (3 << PMP_MODE_WAITE_SHIFT)
#  define PMP_MODE_WAITE_WR(n)     ((n-1) << PMP_MODE_WAITE_SHIFT) /* Wait of n TPB n=1..4 */
#  define PMP_MODE_WAITE_RD(n)     ((n) << PMP_MODE_WAITE_SHIFT)   /* Wait of n TPB n=0..3 */
#define PMP_MODE_WAITM_SHIFT       (2)       /* Bits 2-5: Data R/W strobe wait states */
#define PMP_MODE_WAITM_MASK        (15 << PMP_MODE_WAITM_SHIFT)
#  define PMP_MODE_WAITM(n)        ((n-1) << PMP_MODE_WAITM_SHIFT) /* Wait of n TPB n=1..16 */
#define PMP_MODE_WAITB_SHIFT       (6)       /* Bits 6-7: Data setup to R/W strobe wait states */
#define PMP_MODE_WAITB_MASK        (3 << PMP_MODE_WAITB_SHIFT)
#  define PMP_MODE_WAITB_1TPB      (0 << PMP_MODE_WAITB_SHIFT) /* Data wait of 1 TPB */
#  define PMP_MODE_WAITB_2TPB      (1 << PMP_MODE_WAITB_SHIFT) /* Data wait of 2 TPB */
#  define PMP_MODE_WAITB_3TPB      (2 << PMP_MODE_WAITB_SHIFT) /* Data wait of 3 TPB */
#  define PMP_MODE_WAITB_4TPB      (3 << PMP_MODE_WAITB_SHIFT) /* Data wait of 4 TPB */
#define PMP_MODE_MODE_SHIFT        (8)       /* Bits 8-9: Parallel port mode select */
#define PMP_MODE_MODE_MASK         (3 << PMP_MODE_MODE_SHIFT)
#  define PMP_MODE_MODE_LEGACY     (0 << PMP_MODE_MODE_SHIFT) /* Legacy parallel slave port */
#  define PMP_MODE_MODE_SLAVE      (1 << PMP_MODE_MODE_SHIFT) /* Enhanced slave mode */
#  define PMP_MODE_MODE_MODE2      (2 << PMP_MODE_MODE_SHIFT) /* Master mode 2 */
#  define PMP_MODE_MODE_MODE1      (3 << PMP_MODE_MODE_SHIFT) /* Master mode 1 */
#define PMP_MODE_MODE16            (1 << 10) /* Bit 10: 1=16-bit mode */
#define PMP_MODE_MODE8             (0)       /*         0=8-bit mode */
#define PMP_MODE_INCM_SHIFT        (11)      /* Bits 11-12: Increment Mode */
#define PMP_MODE_INCM_MASK         (3 << PMP_MODE_INCM_SHIFT)
#  define PMP_MODE_INCM_NONE       (0 << PMP_MODE_INCM_SHIFT) /* No incr or decr of addr */
#  define PMP_MODE_INCM_INCR       (1 << PMP_MODE_INCM_SHIFT) /* Incr addr on R/W cycle */
#  define PMP_MODE_INCM_DECR       (2 << PMP_MODE_INCM_SHIFT) /* Decr addr on R/Wcycle */
#  define PMP_MODE_INCM_SLAVE      (3 << PMP_MODE_INCM_SHIFT) /* Slave mode auto-increment */
#define PMP_MODE_IRQM_SHIFT        (13)      /* Bits 13-14: Interrupt request mode */
#define PMP_MODE_IRQM_MASK         (3 << PMP_MODE_IRQM_SHIFT)
#  define PMP_MODE_IRQM_NONE       (0 << PMP_MODE_IRQM_SHIFT) /* No Interrupt generated */
#  define PMP_MODE_IRQM_RW         (1 << PMP_MODE_IRQM_SHIFT) /* Interrupt at end of R/W cycle */
#  define PMP_MODE_IRQM_BUFFER     (2 << PMP_MODE_IRQM_SHIFT) /* R/W buffer 3 or write PMA=11 */
#define PMP_MODE_BUSY              (1 << 15) /* Bit 15: Busy (master mode only) */

/* Parallel Port Address Register */

#define PMP_ADDR_ADDR_SHIFT        (0)       /* Bits 0-13: Destination address */
#define PMP_ADDR_ADDR_MASK         (0x3fff << PMP_ADDR_ADDR_SHIFT)
#define PMP_ADDR_CS1EN             (1 << 14) /* Bit 14: Chip select 1 */
#define PMP_ADDR_CS2EN             (1 << 15) /* Bit 15: Chip select 2 */

/* Parallel Port Data Output Register -- 32-bit data register */

/* Parallel Port Data Input Register -- 32-bit data register */

/* Parallel Port Pin Enable Register */

#define PMP_AEN_PMALEN_SHIFT       (0)       /* PTEN 0-1: PMALH/PMALL strobe enable */
#define PMP_AEN_PMALEN_MASK        (3 << PMP_AEN_STROBEN_SHIFT)
#define PMP_AEN_ADDR_SHIFT         (2)       /* PTEN 2-13: PMP address port enable */
#define PMP_AEN_ADDR_MASK          (0xfff << PMP_AEN_STROBEN_SHIFT)
#define PMP_AEN_PMCSEN_SHIFT       (14)      /* PTEN 14-15: PMCSx Strobe enable */
#define PMP_AEN_PMCSEN_MASK        (3 << PMP_AEN_STROBEN_SHIFT)

/* Parallel Port Status Register */

#define PMP_STAT_OBNE(n)           (1 << (n))
#define PMP_STAT_OB0E              (1 << 0)  /* Bit 0:  Output buffer 0 status empty bits */
#define PMP_STAT_OB1E              (1 << 1)  /* Bit 1:  Output buffer 1 status empty bits */
#define PMP_STAT_OB2E              (1 << 2)  /* Bit 2:  Output buffer 2 status empty bits */
#define PMP_STAT_OB3E              (1 << 3)  /* Bit 3:  Output buffer 3 status empty bits */
#define PMP_STAT_OBUF              (1 << 6)  /* Bit 6:  Output buffer underflow status */
#define PMP_STAT_OBE               (1 << 7)  /* Bit 7:  Output buffer empty status */
#define PMP_STAT_IBNF(n)           (1 << (n+8))
#define PMP_STAT_IB0F              (1 << 8)  /* Bit 8:  Input buffer 0 status full */
#define PMP_STAT_IB1F              (1 << 9)  /* Bit 9:  Input buffer 1 status full */
#define PMP_STAT_IB2F              (1 << 10) /* Bit 10: Input buffer 2 status full */
#define PMP_STAT_IB3F              (1 << 11) /* Bit 11: Input buffer 3 status full */
#define PMP_STAT_IBOV              (1 << 14) /* Bit 14: Input buffer overflow status */
#define PMP_STAT_IBF               (1 << 15) /* Bit 15: Input buffer full status */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

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
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PMP_H */
