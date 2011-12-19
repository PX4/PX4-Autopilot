/********************************************************************************************
 * arch/mips/include/mips32/cp0.h
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

#ifndef __ARCH_MIPS_INCLUDE_MIPS32_CP0_H
#define __ARCH_MIPS_INCLUDE_MIPS32_CP0_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* CP0 Register Addresses *******************************************************************/

#ifdef __ASSEMBLY__
#  define MIPS32_CP0_INDEX1         $0,0   /* Index into the TLB array */
#  define MIPS32_CP0_RANDOM1        $1,0   /* Randomly generated index into the TLB array */
#  define MIPS32_CP0_ENTRYLO01      $2,0   /* LS TLB entry for even-numbered pages */
#  define MIPS32_CP0_ENTRYLO11      $3,0   /* LS TLB entry for odd-numbered pages */
#  define MIPS32_CP0_CONTEXT2       $4,0   /* Page table address */
#  define MIPS32_CP0_PAGEMASK1      $5,0   /* Variable page sizes in TLB entries */
#  define MIPS32_CP0_WIRED1         $6,0   /* umber of fixed (“wired”) TLB entries */
#  define MIPS32_CP0_BADVADDR       $8,0   /* Address of most recent exception */
#  define MIPS32_CP0_COUNT          $9,0   /* Processor cycle count */
#  define MIPS32_CP0_ENTRYHI1       $10,0  /* High-order portion of the TLB entry */
#  define MIPS32_CP0_COMPARE        $11,0  /* Timer interrupt control */
#  define MIPS32_CP0_STATUS         $12,0  /* Processor status and control */
#  define MIPS32_CP0_CAUSE          $13,0  /* Cause of last general exception */
#  define MIPS32_CP0_EPC            $14,0  /* Program counter at last exception */
#  define MIPS32_CP0_PRID           $15,0  /* Processor identification and revision */
#  define MIPS32_CP0_CONFIG         $16,0  /* Configuration register */
#  define MIPS32_CP0_CONFIG1        $16,1  /* Configuration register 1 */
#  define MIPS32_CP0_CONFIG2        $16,2  /* Configuration register 3 */
#  define MIPS32_CP0_CONFIG3        $16,2  /* Configuration register 3 */
#  define MIPS32_CP0_LLADDR         $17,0  /* Load linked address */
#  define MIPS32_CP0_WATCHLO2       $18,0  /* LS Watchpoint address */
#  define MIPS32_CP0_WATCHHI2       $19,0  /* MS Watchpoint address and mask */
#  define MIPS32_CP0_DEBUG3         $23,0  /* Debug control and exception status */
#  define MIPS32_CP0_DEPC3          $24,0  /* Program counter at last debug exception */
#  define MIPS32_CP0_ERRCTL         $26,0  /* Controls access data CACHE instruction */
#  define MIPS32_CP0_TAGLO          $28,0  /* LS portion of cache tag interface */
#  define MIPS32_CP0_DATALO         $28,1  /* LS portion of cache tag interface */
#  define MIPS32_CP0_TAGHI          $29,0  /* MS portion of cache tag interface */
#  define MIPS32_CP0_DATAHI         $29,1  /* MS portion of cache tag interface */
#  define MIPS32_CP0_ERROREPC2      $30,0  /* Program counter at last error */
#  define MIPS32_CP0_DESAVE3        $31,0  /* Debug handler scratchpad register */
#endif

/* CP0 Registers ****************************************************************************/

/* Register Number: 0 Sel: 0 Name: Index
 * Function: Index into the TLB array
 * Compliance Level: Required for TLB-based MMUs; Optional otherwise.
 */

#define CP0_INDEX_SHIFT             (0)       /* Bits 0-(n-1): TLB Index */
#define CP0_INDEX_MASK              (0x7fffffff)
#define CP0_INDEX_P                 (1 << 31) /* Bit 31: Probe failure */

/* Register Number: 1 Sel: 0 Name: Random
 * Function: Randomly generated index into the TLB array
 * Compliance Level: Required for TLB-based MMUs; Optional otherwise.
 *
 *   This is a 32-bit register containing a random TLB index.  The valid width is some fixed
 *   number, 'n', but the upper bits are padded so that no fields need be defined for this
 *   register.
 */

/* Register Number: 2 Sel: 0 Name: EntryLo0
 * Function: Low-order portion of the TLB entry for even-numbered virtual pages
 * Compliance Level: EntryLo0 is Required for a TLB-based MMU; Optional
 *   otherwise.
 *
 * Register Number: 3 Sel: 0 Name: EntryLo1
 * Function: Low-order portion of the TLB entry for odd-numbered virtual pages
 * Compliance Level: EntryLo1 is Required for a TLB-based MMU; Optional otherwise.
 */

#define CP0_ENTRYLO_G               (1 << 0)  /* Bit 0: Global bit */
#define CP0_ENTRYLO_V               (1 << 1)  /* Bit 1: Valild bit */
#define CP0_ENTRYLO_D               (1 << 2)  /* Bit 2: Dirty bit */
#define CP0_ENTRYLO_C_SHIFT         (3)       /* Bits 3-5: Coherency attribute */
#define CP0_ENTRYLO_C_MASK          (7 << CP0_ENTRYLO_CSHIFT)
#  define CP0_ENTRYLO_UNCACHED      (2 << CP0_ENTRYLO_CSHIFT)
#  define CP0_ENTRYLO_CACHEABLE     (3 << CP0_ENTRYLO_CSHIFT)
#define CP0_ENTRYLO_PFN_SHIFT       (6)       /* Bits 6-29: Page frame number */
#define CP0_ENTRYLO_PFN_MASK        (0x00ffffff << CP0_ENTRYLO_CSHIFT)

/* Register Number: 4 Sel: 0 Name: Context
 * Function: Pointer to page table entry in memory
 * Compliance Level: Required for TLB-based MMUs; Optional otherwise.
 */

#define CP0_CONTEXT_BADVPN2_SHIFT   (4)       /* Bits 4-22: Virtual address that cause an excpetion */
#define CP0_CONTEXT_BADVPN2_MASK    (0x0007ffff << CP0_CONTEXT_BADVPN2_SHIFT)
#define CP0_CONTEXT_PTEBASE_SHIFT   (23)      /* Bits 23-31: Page table base address */
#define CP0_CONTEXT_PTEBASE_MASK    (0x000001ff << CP0_CONTEXT_PTEBASE_SHIFT)

/* Register Number: 5 Sel: 0 Name: PageMask
 * Function: Control for variable page size in TLB entries.
 * Compliance Level: Required for TLB-based MMUs; Optional otherwise.
 */

#define CP0_PAGEMASK_SHIFT          (13)      /* Bits 13-28: Page mask */
#define CP0_PAGEMASK_MASK           (0xffff << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_4KB          (0x0000 << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_16KB         (0x0003 << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_64KB         (0x000f << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_256KB        (0x003f << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_1MB          (0x00ff << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_4MB          (0x03ff << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_16MB         (0x0fff << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_64MB         (0x3fff << CP0_PAGEMASK_SHIFT)
#  define CP0_PAGEMASK_256MB        (0xffff << CP0_PAGEMASK_SHIFT)

/* Register Number: 6 Sel: 0 Name: Wired
 * Function: Controls the number of fixed (“wired”) TLB entries
 * Compliance Level: Required for TLB-based MMUs; Optional otherwise.
 *
 *   This is a 32-bit register containing the TLB wired boundary.  The valid width is some
 *   fixed number, 'n', but the upper bits are padded so that no fields need be defined for
 *   this register.
 *
 * Register Number: 7 Sel: all (Reserved for future extensions)
 *
 * Register Number: 8 Sel: 0 Name: BadVAddr
 * Function: Reports the address for the most recent address-related exception
 * Compliance Level: Required.
 *
 *   This register contains a 32-bit address value; No fields need be defined for this
 *   register.
 *
 * Register Number: 9 Sel: 0 Name: Count
 * Function: Processor cycle count
 * Compliance Level: Required.
 *
 *   This register contains a 32-bit count value; No fields need be defined for this
 *   register.
 *
 * Register Number: 9 Sel: 6-7 (Available for implementation dependent user)
 */

/* Register Number: 10 Sel: 0 Name: EntryHi
 * Function: High-order portion of the TLB entry
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 */

#define CP0_ENTRYHI_ASID_SHIFT      (0)       /* Bits 0-7: Address space identifier */
#define CP0_ENTRYHI_ASID_MASK       (0xff << CP0_ENTRYHI_ASID_SHIFT)
#define CP0_ENTRYHI_VPN2_SHIFT      (13)      /* Bits 13-31: Virtual address */
#define CP0_ENTRYHI_VPN2_MASK       (0x0007ffff << CP0_ENTRYHI_VPN2_SHIFT)

/* Register Number: 11 Sel: 0 Name: Compare
 * Function: Timer interrupt control
 * Compliance Level: Required.
 *
 *   This register contains a 32-bit compare value; No fields need be defined for this
 *   register.
 *
 * Register Number: 11 Sel: 6-7 (Available for implementation dependent user)
 */

/* Register Number: 12 Sel: 0 Name: Status
 * Function: Processor status and control
 * Compliance Level: Required.
 */

#define CP0_STATUS_IE               (1 << 0)  /* Bit 0: Interrupt Enable */
#define CP0_STATUS_EXL              (1 << 1)  /* Bit 1: Exception Level */
#define CP0_STATUS_ERL              (1 << 2)  /* Bit 2: Error Level */
#define CP0_STATUS_KSU_SHIFT        (3)       /* Bits 3-4: Operating mode (with supervisor mode) */
#define CP0_STATUS_KSU_MASK         (3 << CP0_STATUS_KSU_SHIFT)
#  define CP0_STATUS_KSU_KERNEL     (0 << CP0_STATUS_KSU_SHIFT)
#  define CP0_STATUS_KSU_SUPER      (1 << CP0_STATUS_KSU_SHIFT)
#  define CP0_STATUS_KSU_USER       (2 << CP0_STATUS_KSU_SHIFT)
#define CP0_STATUS_UM               (1 << 4)  /* Bit 4: Operating mode == USER (No supervisor mode) */
#define CP0_STATUS_UX               (1 << 5)  /* Bit 5: Enables 64-bit user address space (Not MIPS32) */
#define CP0_STATUS_SX               (1 << 6)  /* Bit 6: Enables 64-bit supervisor address space (Not MIPS32) */
#define CP0_STATUS_KX               (1 << 7)  /* Bit 7: Enables 64-bit kernel address space (Not MIPS32) */
#define CP0_STATUS_IM_SHIFT         (8)       /* Bits 8-15: Interrupt Mask */
#define CP0_STATUS_IM_MASK          (0xff << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM_SWINTS      (0x03 << CP0_STATUS_IM_SHIFT) /* IM0-1 = Software interrupts */
#  define CP0_STATUS_IM0            (0x01 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM1            (0x02 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM_HWINTS      (0x7c << CP0_STATUS_IM_SHIFT) /* IM2-6 = Hardware interrupts */
#  define CP0_STATUS_IM2            (0x04 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM3            (0x08 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM4            (0x10 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM5            (0x20 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM6            (0x40 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM_TIMER       (0x80 << CP0_STATUS_IM_SHIFT) /* IM7 = Hardware/Timer/Perf interrupts */
#  define CP0_STATUS_IM7            (0x80 << CP0_STATUS_IM_SHIFT)
#  define CP0_STATUS_IM_ALL         (0xff << CP0_STATUS_IM_SHIFT)
#define CP0_STATUS_IMPL_SHIFT       (16)      /* Bits 16-17: Implementation dependent */
#define CP0_STATUS_IMPL_MASK        (3 << CP0_STATUS_IMPL_SHIFT)
#define CP0_STATUS_NMI              (1 << 19) /* Bit 19: Reset exception due to an NMI */
#define CP0_STATUS_SR               (1 << 20) /* Bit 20: Reset exception due to a Soft Reset */
#define CP0_STATUS_TS               (1 << 21) /* Bit 21: TLB detected match on multiple entries */
#define CP0_STATUS_BEV              (1 << 22) /* Bit 22: Location of exception vectors 1->Bootstrap */
#define CP0_STATUS_PX               (1 << 23) /* Bit 23: Enables 64-bit operations (Not MIPS32) */
#define CP0_STATUS_MX               (1 << 24) /* Bit 24: Enables MDMX™ (Not MIPS32) */
#define CP0_STATUS_RE               (1 << 25) /* Bit 25: Enable reverse-endian memory in user mode */
#define CP0_STATUS_FR               (1 << 26) /* Bit 26: Controls the floating point register mode (Not MIPS32) */
#define CP0_STATUS_RP               (1 << 27) /* Bit 27: Enables reduced power mode */
#define CP0_STATUS_CU0              (1 << 28) /* Bit 28: Controls access to coprocessor 0 */
#define CP0_STATUS_CU1              (1 << 29) /* Bit 29: Controls access to coprocessor 1 */
#define CP0_STATUS_CU2              (1 << 30) /* Bit 30: Controls access to coprocessor 2 */
#define CP0_STATUS_CU3              (1 << 31) /* Bit 31: Controls access to coprocessor 3 */

/* Register Number: 13 Sel: 0 Name: Cause
 * Function: Cause of last general exception
 * Compliance Level: Required.
 */

#define CP0_CAUSE_EXCCODE_SHIFT     (2)       /* Bits 2-6: Exception code */
#define CP0_CAUSE_EXCCODE_MASK      (31 << CP0_CAUSE_EXCCODE_SHIFT)
# define CP0_CAUSE_EXCCODE_INT      (0 << CP0_CAUSE_EXCCODE_SHIFT)  /* Interrupt */
# define CP0_CAUSE_EXCCODE_TLBL     (2 << CP0_CAUSE_EXCCODE_SHIFT)  /* TLB exception (load or instruction fetch) */
# define CP0_CAUSE_EXCCODE_TLBS     (3 << CP0_CAUSE_EXCCODE_SHIFT)  /* TLB exception (store) */
# define CP0_CAUSE_EXCCODE_ADEL     (4 << CP0_CAUSE_EXCCODE_SHIFT)  /* Address error exception (load or instruction fetch) */
# define CP0_CAUSE_EXCCODE_ADES     (5 << CP0_CAUSE_EXCCODE_SHIFT)  /* Address error exception (store) */
# define CP0_CAUSE_EXCCODE_IBE      (6 << CP0_CAUSE_EXCCODE_SHIFT)  /* Bus error exception (instruction fetch) */
# define CP0_CAUSE_EXCCODE_DBE      (7 << CP0_CAUSE_EXCCODE_SHIFT)  /* Bus error exception (data reference: load or store) */
# define CP0_CAUSE_EXCCODE_SYS      (8 << CP0_CAUSE_EXCCODE_SHIFT)  /* Syscall exception */
# define CP0_CAUSE_EXCCODE_BP       (9 << CP0_CAUSE_EXCCODE_SHIFT)  /* Breakpoint exception */
# define CP0_CAUSE_EXCCODE_RI       (10 << CP0_CAUSE_EXCCODE_SHIFT) /* Reserved instruction exception */
# define CP0_CAUSE_EXCCODE_CPU      (11 << CP0_CAUSE_EXCCODE_SHIFT) /* Coprocessor Unusable exception */
# define CP0_CAUSE_EXCCODE_OV       (12 << CP0_CAUSE_EXCCODE_SHIFT) /* Arithmetic Overflow exception */
# define CP0_CAUSE_EXCCODE_TR       (13 << CP0_CAUSE_EXCCODE_SHIFT) /* Trap exception */
# define CP0_CAUSE_EXCCODE_FPE      (15 << CP0_CAUSE_EXCCODE_SHIFT) /* Floating point exception */
# define CP0_CAUSE_EXCCODE_C2E      (18 << CP0_CAUSE_EXCCODE_SHIFT) /* Precise Coprocessor 2 exceptions */
# define CP0_CAUSE_EXCCODE_MDMX     (22 << CP0_CAUSE_EXCCODE_SHIFT) /* MDMX Unusable (MIPS64) */
# define CP0_CAUSE_EXCCODE_WATCH    (23 << CP0_CAUSE_EXCCODE_SHIFT) /* WatchHi/WatchLo address */
# define CP0_CAUSE_EXCCODE_MCHECK   (24 << CP0_CAUSE_EXCCODE_SHIFT) /* Machine check */
# define CP0_CAUSE_EXCCODE_CACHEERR (30 << CP0_CAUSE_EXCCODE_SHIFT) /* Cache error */
#define CP0_CAUSE_IP0               (1 << 8)  /* Bit 8: Controls request for software interrupt 0 */
#define CP0_CAUSE_IP1               (1 << 9)  /* Bit 9: Controls request for software interrupt 1 */
#define CP0_CAUSE_IP_SHIFT          (10)      /* Bits 10-15:  Pending external interrupts */
#define CP0_CAUSE_IP_MASK           (0x3f << CP0_CAUSE_IP_SHIFT)
#  define CP0_CAUSE_IP2             (0x10 << CP0_CAUSE_IP_SHIFT) /*  Hardware interrupt 0 */
#  define CP0_CAUSE_IP3             (0x11 << CP0_CAUSE_IP_SHIFT) /*  Hardware interrupt 1 */
#  define CP0_CAUSE_IP4             (0x12 << CP0_CAUSE_IP_SHIFT) /*  Hardware interrupt 2 */
#  define CP0_CAUSE_IP5             (0x13 << CP0_CAUSE_IP_SHIFT) /*  Hardware interrupt 3 */
#  define CP0_CAUSE_IP6             (0x14 << CP0_CAUSE_IP_SHIFT) /*  Hardware interrupt 4 */
#  define CP0_CAUSE_IP7             (0x15 << CP0_CAUSE_IP_SHIFT) /*  Hardware interrupt 5, timer or performance counter interrupt */
#define CP0_CAUSE_WP                (1 << 22) /* Watch exception was deferred */
#define CP0_CAUSE_IV                (1 << 23) /* Bit 23:  Interrupt exception uses special interrupt vector */
#define CP0_CAUSE_CE_SHIFT          (28)      /* Bits 28-29: Coprocessor unit number fo Coprocessor Unusable exception */
#define CP0_CAUSE_CE_MASK           (3 << CP0_CAUSE_CE_SHIFT)
#define CP0_CAUSE_BD                (1 << 31) /* Bit 31:  Last exception occurred in a branch delay slot */

/* Register Number: 14 Sel: 0 Name: EPC
 * Function: Program counter at last exception
 * Compliance Level: Required.
 *
 *   This register contains a 32-bit address value; No fields need be defined for this
 *   register.
 */

/* Register Number: 15 Sel: 0 Name: PRId
 * Function: Processor identification and revision
 * Compliance Level: Required.
 */

#define CP0_PRID_REV_SHIFT          (0)       /* Bits 0-7: Revision number of the processor */
#define CP0_PRID_REV_MASK           (0xff << CP0_PRID_REV_SHIFT)
#define CP0_PRID_PROCID_SHIFT       (8)       /* Bits 8-15: Type of processor */
#define CP0_PRID_PROCID_MASK        (0xff << CP0_PRID_PROCID_SHIFT)
#define CP0_PRID_COMPANY_SHIFT      (16)      /* Bits 16-23:  Company ID */
#define CP0_PRID_COMPANY_MASK       (0xff << CP0_PRID_COMPANY_SHIFT)
#define CP0_PRID_OPTIONS_SHIFT      (24)      /* Bits 24-31: Company-dependent options */
#define CP0_PRID_OPTIONS_MASK       (0xff << CP0_PRID_OPTIONS_SHIFT)

/* Register Number: 16 Sel: 0 Name: Config
 * Function: Configuration register
 * Compliance Level: Required.
 */

#define CP0_CONFIG_K0_SHIFT         (0)       /* Bits 0-2: KSEG0 coherency algorithm */
#define CP0_CONFIG_K0_MASK          (7 << CP0_CONFIG_K0_SHIFT)
#  define CP0_CONFIG_K0_UNCACHED    (2 << CP0_CONFIG_K0_SHIFT)
#  define CP0_CONFIG_K0_CACHEABLE   (3 << CP0_CONFIG_K0_SHIFT)
#define CP0_CONFIG_MT_SHIFT         (7)      /* Bits 7-9: MMU Type */
#define CP0_CONFIG_MT_MASK          (7 << CP0_CONFIG_MT_SHIFT)
#  define CP0_CONFIG_MT_NONE        (0 << CP0_CONFIG_MT_SHIFT) /* None */
#  define CP0_CONFIG_MT_TLB         (1 << CP0_CONFIG_MT_SHIFT) /* Standard TLB */
#  define CP0_CONFIG_MT_BAT         (2 << CP0_CONFIG_MT_SHIFT) /* Standard BAT */
#  define CP0_CONFIG_MT_FIXED       (3 << CP0_CONFIG_MT_SHIFT) /* Standard fixed mapping */
#define CP0_CONFIG_AR_SHIFT         (10)     /* Bits 10-12: Architecture revision level */
#define CP0_CONFIG_AR_MASK          (7 << CP0_CONFIG_AR_SHIFT)
#  define CP0_CONFIG_AR_REV1        (0 << CP0_CONFIG_AR_SHIFT)
#  define CP0_CONFIG_AR_REV2        (1 << CP0_CONFIG_AR_SHIFT)
#define CP0_CONFIG_AT_SHIFT         (13)     /* Bits 13-14: Architecture type implemented by the processor */
#define CP0_CONFIG_AT_MASK          (3 << CP0_CONFIG_AT_SHIFT)
#  define CP0_CONFIG_AT_MIPS32      (0 << CP0_CONFIG_AT_SHIFT) /* MIPS32 */
#  define CP0_CONFIG_AT_MIPS64CMP   (0 << CP0_CONFIG_AT_SHIFT) /* MIPS64 with 32-bit compatibility segments */
#  define CP0_CONFIG_AT_MIPS64      (1 << CP0_CONFIG_AT_SHIFT) /* MIPS64 with access to all address segments */
#define CP0_CONFIG_BE               (1 << 15) /* Bit 15: Processor is running in big-endian mode */
#define CP0_CONFIG_IMPL_SHIFT       (16)      /* Bits 16-30: Implementation dependent */
#define CP0_CONFIG_IMPL_MASK        (0x7fff << CP0_CONFIG_IMPL_SHIFT)
#define CP0_CONFIG_M                (1 << 31) /* Bit 31: Config1 register is implemented at select=1 */

/* Register Number: 16 Sel: 1 Name: Config1
 * Function: Configuration register 1
 * Compliance Level: Required.
 */

#define CP0_CONFIG1_FP              (1 << 0 FPU implemented
#define CP0_CONFIG1_EP              (1 << 1 EJTAG implemented
#define CP0_CONFIG1_CA              (1 << 2 Code compression (MIPS16) implemented
#define CP0_CONFIG1_WR              (1 << 3 Watch registers implemented
#define CP0_CONFIG1_PC              (1 << 4 Performance Counter registers implemented
#define CP0_CONFIG1_MD              (1 << 5 MDMX ASE implemented (MIPS64)
#define CP0_CONFIG1_C2              (1 << 6 Coprocessor 2 implemented
#define CP0_CONFIG1_DA_SHIFT        (7)       /* Bits 7-9: Dcache associativity */
#define CP0_CONFIG1_DA_MASK         (7 << CP0_CONFIG1_DA_SHIFT)
#  define CP0_CONFIG1_DA_DIRECT     (0 << CP0_CONFIG1_DA_SHIFT) /* Direct mapped */
#  define CP0_CONFIG1_DA_2WAY       (1 << CP0_CONFIG1_DA_SHIFT) /* 2-way */
#  define CP0_CONFIG1_DA_3WAY       (2 << CP0_CONFIG1_DA_SHIFT) /* 3-way */
#  define CP0_CONFIG1_DA_4WAY       (3 << CP0_CONFIG1_DA_SHIFT) /* 4-way */
#  define CP0_CONFIG1_DA_5WAY       (4 << CP0_CONFIG1_DA_SHIFT) /* 5-way */
#  define CP0_CONFIG1_DA_6WAY       (5 << CP0_CONFIG1_DA_SHIFT) /* 6-way */
#  define CP0_CONFIG1_DA_7WAY       (6 << CP0_CONFIG1_DA_SHIFT) /* 7-way */
#  define CP0_CONFIG1_DA_8WAY       (7 << CP0_CONFIG1_DA_SHIFT) /* 8-way */
#define CP0_CONFIG1_DL_SHIFT        (10)      /* Bits 10-12:  Dcache line size */
#define CP0_CONFIG1_DL_MASK         (7 << CP0_CONFIG1_DL_SHIFT)
#  define CP0_CONFIG1_DL_NONE       (0 << CP0_CONFIG1_DL_SHIFT) /* No Dcache present */
#  define CP0_CONFIG1_DL_4BYTES     (1 << CP0_CONFIG1_DL_SHIFT) /* 4 bytes */
#  define CP0_CONFIG1_DL_8BYTES     (2 << CP0_CONFIG1_DL_SHIFT) /* 8 bytes */
#  define CP0_CONFIG1_DL_16BYTES    (3 << CP0_CONFIG1_DL_SHIFT) /* 16 bytes */
#  define CP0_CONFIG1_DL_32BYTES    (4 << CP0_CONFIG1_DL_SHIFT) /* 32 bytes */
#  define CP0_CONFIG1_DL_64BYTES    (5 << CP0_CONFIG1_DL_SHIFT) /* 64 bytes */
#  define CP0_CONFIG1_DL_128BYTES   (6 << CP0_CONFIG1_DL_SHIFT) /* 128 bytes */
#define CP0_CONFIG1_DS_SHIFT        (13)      /* Bits 13-15: Dcache sets per way */
#define CP0_CONFIG1_DS_MASK         (7 << CP0_CONFIG1_DS_SHIFT)
#  define CP0_CONFIG1_DS_64SETS     (0 << CP0_CONFIG1_DS_SHIFT)
#  define CP0_CONFIG1_DS_128SETS    (1 << CP0_CONFIG1_DS_SHIFT)
#  define CP0_CONFIG1_DS_256SETS    (2 << CP0_CONFIG1_DS_SHIFT)
#  define CP0_CONFIG1_DS_512SETS    (3 << CP0_CONFIG1_DS_SHIFT)
#  define CP0_CONFIG1_DS_1024SETS   (4 << CP0_CONFIG1_DS_SHIFT)
#  define CP0_CONFIG1_DS_2048SETS   (5 << CP0_CONFIG1_DS_SHIFT)
#  define CP0_CONFIG1_DS_4096SETS   (6 << CP0_CONFIG1_DS_SHIFT)
#define CP0_CONFIG1_IA_SHIFT        (16)      /* Bits 16-18: Icache associativity */
#define CP0_CONFIG1_IA_MASK         (7 << CP0_CONFIG1_IA_SHIFT)
#  define CP0_CONFIG1_IA_DIRECT     (0 << CP0_CONFIG1_IA_SHIFT) /* Direct mapped */
#  define CP0_CONFIG1_IA_2WAY       (1 << CP0_CONFIG1_IA_SHIFT) /* 2-way */
#  define CP0_CONFIG1_IA_3WAY       (2 << CP0_CONFIG1_IA_SHIFT) /* 3-way */
#  define CP0_CONFIG1_IA_4WAY       (3 << CP0_CONFIG1_IA_SHIFT) /* 4-way */
#  define CP0_CONFIG1_IA_5WAY       (4 << CP0_CONFIG1_IA_SHIFT) /* 5-way */
#  define CP0_CONFIG1_IA_6WAY       (5 << CP0_CONFIG1_IA_SHIFT) /* 6-way */
#  define CP0_CONFIG1_IA_7WAY       (6 << CP0_CONFIG1_IA_SHIFT) /* 7-way */
#  define CP0_CONFIG1_IA_8WAY       (7 << CP0_CONFIG1_IA_SHIFT) /* 8-way */
#define CP0_CONFIG1_IL_SHIFT        (19)      /* Bits 19-21: Icache line size */
#define CP0_CONFIG1_IL_MASK         (7 << CP0_CONFIG1_IL_SHIFT)
#  define CP0_CONFIG1_IL_NONE       (0 << CP0_CONFIG1_IL_SHIFT) /* No Dcache present */
#  define CP0_CONFIG1_IL_4BYTES     (1 << CP0_CONFIG1_IL_SHIFT) /* 4 bytes */
#  define CP0_CONFIG1_IL_8BYTES     (2 << CP0_CONFIG1_IL_SHIFT) /* 8 bytes */
#  define CP0_CONFIG1_IL_16BYTES    (3 << CP0_CONFIG1_IL_SHIFT) /* 16 bytes */
#  define CP0_CONFIG1_IL_32BYTES    (4 << CP0_CONFIG1_IL_SHIFT) /* 32 bytes */
#  define CP0_CONFIG1_IL_64BYTES    (5 << CP0_CONFIG1_IL_SHIFT) /* 64 bytes */
#  define CP0_CONFIG1_IL_128BYTES   (6 << CP0_CONFIG1_IL_SHIFT) /* 128 bytes */
#define CP0_CONFIG1_IS_SHIFT        (22)      /* Bits 22-24: Icache sets per way */
#define CP0_CONFIG1_IS_MASK         (7 << CP0_CONFIG1_IS_SHIFT)
#  define CP0_CONFIG1_IS_64SETS     (0 << CP0_CONFIG1_IS_SHIFT)
#  define CP0_CONFIG1_IS_128SETS    (1 << CP0_CONFIG1_IS_SHIFT)
#  define CP0_CONFIG1_IS_256SETS    (2 << CP0_CONFIG1_IS_SHIFT)
#  define CP0_CONFIG1_IS_512SETS    (3 << CP0_CONFIG1_IS_SHIFT)
#  define CP0_CONFIG1_IS_1024SETS   (4 << CP0_CONFIG1_IS_SHIFT)
#  define CP0_CONFIG1_IS_2048SETS   (5 << CP0_CONFIG1_IS_SHIFT)
#  define CP0_CONFIG1_IS_4096SETS   (6 << CP0_CONFIG1_IS_SHIFT)
#define CP0_CONFIG1_MMUSIZE_SHIFT   (25)      /* Bits 25-30: Number of entries in the TLB minus one */
#define CP0_CONFIG1_MMUSIZE_MASK    (0x3f << CP0_CONFIG1_MMUSIZE_SHIFT)
#define CP0_CONFIG1_M               (1 << 31) /* Bit 31: Config2 register is present */

/* Register Number: 16 Sel: 2 Name: Config2
 * Function: Configuration register 2
 * Compliance Level: Optional.
 */

#define CP0_CONFIG2_TBS_SHIFT       (0)       /* Bits 0-30: Configuration of the level 2 and level 3 caches */
#define CP0_CONFIG2_TBS_MASK        (0x7fffffff << CP0_CONFIG2_TBS_SHIFT)
#define CP0_CONFIG2_M               (1 << 31) /* Bit 31: Config3 register is present */

/* Register Number: 16 Sel: 3 Name: Config3
 * Function: Configuration register 3
 * Compliance Level: Optional.
 */
#define CP0_CONFIG3_TL              (1 << 0)  /* Bit 0: Trace Logic implemented */
#define CP0_CONFIG3_SM              (1 << 1)  /* Bit 1: SmartMIPS™ ASE implemented */
#define CP0_CONFIG3_M               (1 << 31) /* Bit 31: Config4 register is present */

/* Register Number: 16 Sel: 6-7 (Available for implementation dependent use) */

/* Register Number: 17 Sel: 0 Name: LLAddr
 * Function: Load linked address
 * Compliance Level: Optional.
 *
 *   This register contains a 32-bit address value; No fields need be defined for this
 *   register.
 */

/* Register Number: 18 Sel: 0-n Name: WatchLo
 * Function: Watchpoint address
 * Compliance Level: Optional.
 */

#define CP0_WATCHLO_W               (1 << 0) /* Bit 0: Exceptions are enabled for stores */
#define CP0_WATCHLO_R               (1 << 1) /* Bit 0: Exceptions are enabled for loads */
#define CP0_WATCHLO_I               (1 << 2) /* Bit 0: Exceptions are enabled for instructions */
#define CP0_WATCHLO_VADDR_SHIFT     (3)      /* Bits 3-31: Virtual address to match */
#define CP0_WATCHLO_VADDR_MASK      (0x1fffffff << CP0_WATCHLO_VADDR_SHIFT)

/* Register Number: 19 Sel: 0-n Name: WatchHi
 * Function: Watchpoint control
 * Compliance Level: Optional.
 */

#define CP0_WATCHHI_MASK_SHIFT      (3)        /* Bits 3-11:  Mask that qualifies the WatchLo address */
#define CP0_WATCHHI_MASK_MASK       (0x1ff << CP0_WATCHHI_MASK_SHIFT)
#define CP0_WATCHHI_ASID_SHIFT      (16)       /* Bits 16-23:  ASID value which to match that in the EntryHi register */
#define CP0_WATCHHI_ASID_MASK       (0xff << CP0_WATCHHI_ASID_SHIFT)
#define CP0_WATCHHI_G               (1 << 30) /* Bit 30: Any address matcing the WatchLo addr will cause an exception */
#define CP0_WATCHHI_M               (1 << 31) /* Bit 30: Another pair of WatchHi/WatchLo registers at select n+1 */

/* Register Number: 20 Sel: 0 Name: XContext
 * Function: in 64-bit implementations
 *
 * Register Number: 21 Sel: all (Reserved for future extensions)
 *
 * Register Number: 22 Sel: all Available for implementation dependent use)
 *
 * Register Number: 23 Sel: 0 Name: Debug
 * Function: EJTAG Debug register
 * Compliance Level: Optional, part of the EJTAG specification.
 *
 * Register Number: 24 Sel: 0 Name: DEPC
 * Function: Program counter at last EJTAG debug exception
 * Compliance Level: Optional, part of the EJTAG specification.
 *
 *   This register contains a 32-bit address value; No fields need be defined for this
 *   register.
 */

/* Register Number: 25 Sel: 0-n Name: PerfCnt
 * Function: Performance counter interface
 * Compliance Level: Recommended.
 */

#define CP0_PERFCNT_EXL             (1 << 0)  /* Bit 0: Enables event counting when STATUS EXL=1 ERL=0 */
#define CP0_PERFCNT_K               (1 << 1)  /* Bit 1: Enables event counting in Kernel Mode (EXL=0 ERL=0) */
#define CP0_PERFCNT_S               (1 << 2)  /* Bit 2: Enables event counting in Supervisor Mode */
#define CP0_PERFCNT_U               (1 << 3)  /* Bit 3: Enables event counting in User Mode */
#define CP0_PERFCNT_IE              (1 << 4)  /* Bit 4: Interrupt Enable */
#define CP0_PERFCNT_EVENT_SHIFT     (5)       /* Bits 5-10:  Event to be counted */
#define CP0_PERFCNT_EVENT_MASK      (0xffff << CP0_PERFCNT_EVENT_SHIFT)
#define CP0_PERFCNT_M               (1 << 31) /* Bit 31: Another pair of performance registers at n+2 and n+3 */

/* Register Number: 26 Sel: 0 Name: ErrCtl
 * Function: Parity/ECC error control and status
 * Compliance Level: Optional.
 *
 *   The bit definitions within the ErrCtl register are implementation dependent.
 *
 * Register Number: 27 Sel: 0-3 Name: CacheErr
 * Function: Cache parity error control and status
 * Compliance Level: Optional.
 *
 *   The bit definitions within the CacheErr register are implementation dependent.
 *
 * Register Number: 28 Sel: 0 Name: TagLo
 * Function: Low-order portion of cache tag interface
 * Compliance Level: Required if a cache is implemented; Optional otherwise.
 *
 *   The bit definitions within the TagLo register are implementation dependent.
 *
 * Register Number: 28 Sel: 1, 3 Name: DataLo
 * Function: The DataLo and DataHi registers are read-only registers that
 *   act as the interface to the cache data array and are intended for
 *   diagnostic operation only.
 * Compliance Level: Optional.
 *
 *   The bit definitions within the DataLo register are implementation dependent.
 *
 * Register Number: 29 Sel: 0 Name: TagHi
 * Function: High-order portion of cache tag interface
 * Compliance Level: Required if a cache is implemented; Optional otherwise.
 *
 *   The bit definitions within the TagHi register are implementation dependent.
 *
 * Register Number: 29 Sel: 1, 3 Name: DataHi
 * Function: The DataLo and DataHi registers are read-only registers that
 *   act as the interface to the cache data array and are intended for
 *   diagnostic operation only.
 * Compliance Level: Optional.
 *
 *   The bit definitions within the DataHi register are implementation dependent.
 *
 * Register Number: 30 Sel: 0 Name: ErrorEPC
 * Function: Program counter at last error
 * Compliance Level: Required.
 *
 *   This register contains a 32-bit address value; No fields need be defined for this
 *   register.
 *
 * Register Number: 31 Sel: 0 Name: DESAVE
 * Function: EJTAG debug exception save register
 * Compliance Level: Optional, part of the EJTAG specification.
 *
 *   This register contains a 32-bit address value; No fields need be defined for this
 *   register.
 */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

#ifndef __ASSEMBLY__

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
#endif /* __ARCH_MIPS_INCLUDE_MIPS32_CP0_H */
