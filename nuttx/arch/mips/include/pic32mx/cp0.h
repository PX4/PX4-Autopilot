/****************************************************************************
 * arch/mips/include/pic32mx/cp0.h
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_INCLUDE_PIC32MX_CP0_H
#define __ARCH_MIPS_INCLUDE_PIC32MX_CP0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/mips32/cp0.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* CP0 Register Addresses ***************************************************/

#ifdef __ASSEMBLY__
#  define PIC32MX_CP0_HWRENA   $7,0   /* Enables access via RDHWR hardware registers */
#  define PIC32MX_CP0_BADVADDR $8,0   /* Address of most recent exception */
#  define PIC32MX_CP0_COUNT    $9,0   /* Processor cycle count */
#  define PIC32MX_CP0_COMPARE  $11,0  /* Timer interrupt control */
#  define PIC32MX_CP0_STATUS   $12,0  /* Processor status and control */
#  define PIC32MX_CP0_INTCTL   $12,1  /* Interrupt system status and control */
#  define PIC32MX_CP0_SRSCTL   $12,2  /* Shadow register set status and control */
#  define PIC32MX_CP0_SRSMAP   $12,3  /* Maps from vectored interrupt to a shadow set */
#  define PIC32MX_CP0_CAUSE    $13,0  /* Cause of last general exception */
#  define PIC32MX_CP0_EPC      $14,0  /* Program counter at last exception */
#  define PIC32MX_CP0_PRID     $15,0  /* Processor identification and revision */
#  define PIC32MX_CP0_EBASE    $15,1  /* Exception vector base register */
#  define PIC32MX_CP0_CONFIG   $16,0  /* Configuration register */
#  define PIC32MX_CP0_CONFIG1  $16,1  /* Configuration register 1 */
#  define PIC32MX_CP0_CONFIG2  $16,2  /* Configuration register 3 */
#  define PIC32MX_CP0_CONFIG3  $16,3  /* Configuration register 3 */
#  define PIC32MX_CP0_DEBUG    $23,3  /* Debug control and exception status */
#  define PIC32MX_CP0_DEPC     $24,3  /* Program counter at last debug exception */
#  define PIC32MX_CP0_ERREPC   $30,3  /* Program counter at last error */
#  define PIC32MX_CP0_DESAVE   $31,3  /* Debug handler scratchpad register */
#endif

/* CP0 Registers ************************************************************/

/* Register Number: 0-6: Reserved
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 */

/* Register Number: 7 Sel: 0 Name: HWREna
 * Function: Enables access via the RDHWR instruction to selected hardware
 *   registers in non-privileged mode.
 * Compliance Level: (Reserved for future extensions)
 */

#define CP0_HWRENA_SHIFT            (0)       /* Bits 0-3: Enable access to a hardware resource */
#define CP0_HWRENA_MASK             (15 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT0           (1 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT1           (2 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT2           (4 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT3           (8 << CP0_HWRENA_SHIFT)

/* Register Number: 8 Sel: 0 Name: BadVAddr
 * Function: Reports the address for the most recent address-related
 *   exception
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 9 Sel: 0 Name: Count
 * Function: Processor cycle count
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 10 Reserved.
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 *
 * Register Number: 11 Sel: 0 Name: Compare
 * Function: Timer interrupt control
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 12 Sel: 0 Name: Status
 * Function: Processor status and control
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *   NOTES:
 *   1. The following are reserved bits in the PIC32: 
 *      CP0_STATUS_UX   Bit 5: Enables 64-bit user address space (Not MIPS32)
 *      CP0_STATUS_SX   Bit 6: Enables 64-bit supervisor address space (Not MIPS32)
 *      CP0_STATUS_KX   Bit 7: Enables 64-bit kernel address space (Not MIPS32)
 *      CP0_STATUS_IMPL Bits 16-17: Implementation dependent
 *      CP0_STATUS_TS   Bit 21: TLB detected match on multiple entries
 *      CP0_STATUS_PX   Bit 23: Enables 64-bit operations (Not MIPS32)
 *      CP0_STATUS_MX   Bit 24: Enables MDMX™ (Not MIPS32)
 */

#undef CP0_STATUS_UX
#undef CP0_STATUS_SX
#undef CP0_STATUS_KX
#undef CP0_STATUS_IMPL
#undef CP0_STATUS_IMPL_SHIFT
#undef CP0_STATUS_IMPL_MASK
#undef CP0_STATUS_TS
#undef CP0_STATUS_PX
#undef CP0_STATUS_MX

/*   2. The following field is of a different width.  Apparently, it
 *      excludes the software interrupt bits.
 *
 *      CP0_STATUS_IM   Bits 8-15: Interrupt Mask
 *      Vs.
 *      CP0_STATUS_IPL  Bits 10-15: Interrupt priority level
 *                      Bitss 8-9 reserved
 */

#define CP0_STATUS_IPL_SHIFT        (10)   /*  Bits 10-15: Interrupt priority level */
#define CP0_STATUS_IPL_MASK         (0x3f << CP0_STATUS_IPL_SHIFT)

/*   3. Supervisor mode not supported
 *       CP0_STATUS_KSU Bits 3-4: Operating mode (with supervisor mode)
 */

#undef CP0_STATUS_KSU_SHIFT
#undef CP0_STATUS_KSU_MASK
#undef CP0_STATUS_KSU_KERNEL
#undef CP0_STATUS_KSU_SUPER
#undef CP0_STATUS_KSU_USER

/* Register Number: 12 Sel: 1 Name: IntCtl */

#define CP0_INTCTL_VS_SHIFT         (5)       /* Bits 5-9: Vector spacing bits */
#define CP0_INTCTL_VS_MASK          (0x1f << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_0BYTES      (0x00 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_32BYTES     (0x01 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_64BYTES     (0x02 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_128BYTES    (0x04 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_256BYTES    (0x08 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_512BYTES    (0x10 << CP0_INTCTL_VS_SHIFT)

/* Register Number: 12 Sel: 2 Name: SRSCtl */

#define CP0_SRSCTL_CSS_SHIFT        (0)       /* Bits 0-3: Current shadow bit set */
#define CP0_SRSCTL_CSS_MASK         (15 << CP0_SRSCTL_CSS_SHIFT)
#define CP0_SRSCTL_PSS_SHIFT        (6)       /* Bits 6-9: Previous shadow set */
#define CP0_SRSCTL_PSS_MASK         (15 << CP0_SRSCTL_PSS_SHIFT)
#define CP0_SRSCTL_ESS_SHIFT        (12)      /* Bits 12-15: Exception shadow sets */
#define CP0_SRSCTL_ESS_MASK         (15 << CP0_SRSCTL_ESS_SHIFT)
#define CP0_SRSCTL_EICSS_SHIFT      (18)      /* Bits 18-21: External interrupt controller shadow sets */
#define CP0_SRSCTL_EICSS_MASK       (15 << CP0_SRSCTL_EICSS_SHIFT)
#define CP0_SRSCTL_HSS_SHIFT        (26)      /* Bits 26-29: High shadow sets */
#define CP0_SRSCTL_HSS_MASK         (15 << CP0_SRSCTL_HSS_SHIFT)
#  define CP0_SRSCTL_HSS_1SET       (0 << CP0_SRSCTL_HSS_SHIFT) /* One shadow set (normal GPR set) */
#  define CP0_SRSCTL_HSS_2SETS      (1 << CP0_SRSCTL_HSS_SHIFT) /* Two shadow sets */
#  define CP0_SRSCTL_HSS_4SETS      (3 << CP0_SRSCTL_HSS_SHIFT) /* Four shadow sets */

/* Register Number: 12 Sel: 3 Name: SRSMap */

#define CP0_SRSMAP_SSV0_SHIFT       (0)       /* Bits 0-3: Shadow set vector 0 */
#define CP0_SRSMAP_SSV0_MASK        (15 << CP0_SRSMAP_SSV0_SHIFT) 
#define CP0_SRSMAP_SSV1_SHIFT       (4)       /* Bits 4-7: Shadow set vector 1 */
#define CP0_SRSMAP_SSV1_MASK        (15 << CP0_SRSMAP_SSV1_SHIFT) 
#define CP0_SRSMAP_SSV2_SHIFT       (8)       /* Bits 8-11: Shadow set vector 2 */
#define CP0_SRSMAP_SSV2_MASK        (15 << CP0_SRSMAP_SSV2_SHIFT) 
#define CP0_SRSMAP_SSV3_SHIFT       (12)      /* Bits 12-15: Shadow set vector 3 */
#define CP0_SRSMAP_SSV3_MASK        (15 << CP0_SRSMAP_SSV3_SHIFT) 
#define CP0_SRSMAP_SSV4_SHIFT       (16)      /* Bits 16-19: Shadow set vector 4 */
#define CP0_SRSMAP_SSV4_MASK        (15 << CP0_SRSMAP_SSV4_SHIFT) 
#define CP0_SRSMAP_SSV5_SHIFT       (20)      /* Bits 20-23: Shadow set vector 5 */
#define CP0_SRSMAP_SSV5_MASK        (15 << CP0_SRSMAP_SSV5_SHIFT) 
#define CP0_SRSMAP_SSV6_SHIFT       (24)      /* Bits 24-27: Shadow set vector 6 */
#define CP0_SRSMAP_SSV6_MASK        (15 << CP0_SRSMAP_SSV6_SHIFT) 
#define CP0_SRSMAP_SSV7_SHIFT       (28)      /* Bits 28-31: Shadow set vector 7 */
#define CP0_SRSMAP_SSV7_MASK        (15 << CP0_SRSMAP_SSV7_SHIFT) 

/* Register Number: 13 Sel: 0 Name: Cause
 * Function: Cause of last general exception
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *   NOTES: The following bits are added in the PIC32: 
 */
 
#define CP0_CAUSE_R                 (1 << 26) /* Bit 26: R bit */
#define CP0_CAUSE_DC                (1 << 27) /* Bit 27: Disable count */
#define CP0_CAUSE_TI                (1 << 30) /* Bit 30: Timer interrupt bit *.

/* Register Number: 14 Sel: 0 Name: EPC
 * Function: Program counter at last exception
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 15 Sel: 0 Name: PRId
 * Function: Processor identification and revision
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *   NOTE: Slightly different bit interpretations of some fields:
 */

#define CP0_PRID_PATCH_SHIFT        (5)       /* Bits 0-1: Patch level */
#define CP0_PRID_PATCH_MASK         (3 << CP0_PRID_PATCH_SHIFT)
#define CP0_PRID_MINOR_SHIFT        (2)       /* Bits 2-4: Minor revision number */
#define CP0_PRID_MINOR_MASK         (7 << CP0_PRID_MINOR_SHIFT)
#define CP0_PRID_MAJOR_SHIFT        (5)       /* Bits 5-7: Major revision number */
#define CP0_PRID_MAJOR_MASK         (7 << CP0_PRID_MAJOR_SHIFT)

#undef CP0_PRID_OPTIONS_SHIFT
#undef CP0_PRID_OPTIONS_MASK

/* Register Number: 15 Sel: 1 Name: EBASE */

#define CP_EBASE_CPUNUM_SHIFT      (0)        /* Bits 0-9: CPU number */
#define CP_EBASE_CPUNUM_MASK       (0x3ff << CP_EBASE_CPUNUM_SHIFT)
#define CP_EBASE_SHIFT             (12)       /* Bits 30-31=10, Bits 12-29: Exception base */
#define CP_EBASE_MASK              (0x3ffff << CP_EBASE_SHIFT)

/* Register Number: 16 Sel: 0 Name: Config
 * Function: Configuration register
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *   1. PIC32MX is always little-endian.
 *   2. Implementation specific bits defined.
 */

#undef CP0_CONFIG_IMPL_SHIFT
#undef CP0_CONFIG_IMPL_MASK

#define CP0_CONFIG_K23_SHIFT       (0)       /* Bits 28-30:  KSEG2 and KSEG3 cacheability */
#define CP0_CONFIG_K23_MASK        (7 << CP0_CONFIG_K23_SHIFT)
#  define CP0_CONFIG_K23_UNCACHED  (2 << CP0_CONFIG_K23_SHIFT)
#  define CP0_CONFIG_K23_CACHEABLE (3 << CP0_CONFIG_K23_SHIFT)
#define CP0_CONFIG_KU_SHIFT         (0)       /* Bits 0-2: KUSEG and USEG cacheability */
#define CP0_CONFIG_KU_MASK          (7 << CP0_CONFIG_KU_SHIFT)
#  define CP0_CONFIG_KU_UNCACHED    (2 << CP0_CONFIG_KU_SHIFT)
#  define CP0_CONFIG_KU_CACHEABLE   (3 << CP0_CONFIG_KU_SHIFT)
#define CP0_CONFIG_UDI              (1 << 22) /* Bit 22: User defined bit */
#define CP0_CONFIG_SB               (1 << 21) /* Bit 32: Simple BE bus mode bit */
#define CP0_CONFIG_MDU              (1 << 20) /* Multipley/Divide unit bit */
#define CP0_CONFIG_DS               (1 << 16) /* Dual SRAM bit */

/* Register Number: 16 Sel: 1 Name: Config1
 * Function: Configuration register 1
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 16 Sel: 2 Name: Config2
 * Function: Configuration register 2
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

#undef CP0_CONFIG2_TBS_SHIFT
#undef CP0_CONFIG2_TBS_MASK

/* Register Number: 16 Sel: 3 Name: Config3
 * Function: Configuration register 3
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

#define CP0_CONFIG3_SP              (1 << 4)  /* Bit 4: Support page bit */
#define CP0_CONFIG3_VINT            (1 << 5)  /* Bit 5: Vector interrupt bit */
#define CP0_CONFIG3_VEIC            (1 << 6)  /* Bit 6: External interrupt controller supported */

/* Register Number: 17-22 Reserved
 * Compliance Level: Optional.
 */

/* Register Number: 23 Sel: 0 Name: Debug
 * Function: EJTAG Debug register
 * Compliance Level: Optional.
 */

#define CP0_DEBUG_DSS               (1 << 0) /* Bit 0: Debug single-step exception */
#define CP0_DEBUG_DBP               (1 << 1) /* Bit 1: Debug software breakpoint exception */
#define CP0_DEBUG_DDBL              (1 << 2) /* Bit 2: Debug data break exception on load */
#define CP0_DEBUG_DDBS              (1 << 3) /* Bit 3: Debug data break exception on store */
#define CP0_DEBUG_DIB               (1 << 4) /* Bit 4: Debug instruction break exception */
#define CP0_DEBUG_DINT              (1 << 5) /* Bit 5: Debug interrupt exception */
#define CP0_DEBUG_SST               (1 << 8) /* Bit 6: Enable debug single step exception */
#define CP0_DEBUG_NOSST             (1 << 9) /* Bit 7: No single step feature available */
#define CP0_DEBUG_DEXCCODE_SHIFT    (10)     /* Bits 10-14: Cause of latest exception in DEBUG mode */
#define CP0_DEBUG_DEXCCODE_MASK     (31 << CP0_DEBUG_DEXCCODE_SHIFT)
#define CP0_DEBUG_VER_SHIFT         (15)     /* Bits 15-17: EJTAG version */
#define CP0_DEBUG_VER_MASK          (7 << CP0_DEBUG_VER_SHIFT)
#define CP0_DEBUG_DDBLIMPR          (1 << 18) /* Bit 18: Imprecise debug data break load instruction */
#define CP0_DEBUG_DDBSIMPR          (1 << 19) /* Bit 19: Imprecise debug data break store instruction */
#define CP0_DEBUG_IEXI              (1 << 20) /* Bit 20: Imprecise error exception inhibit */
#define CP0_DEBUG_DBUSEP            (1 << 21) /* Bit 21: Data access bus error exception pending */
#define CP0_DEBUG_CACHEEP           (1 << 22) /* Bit 22: Imprecise cache error exception is pending */
#define CP0_DEBUG_MCHECKP           (1 << 23) /* Bit 23: Imprecise machine check exception is pending */
#define CP0_DEBUG_IBUSEP            (1 << 24) /* Bit 24: Bus error exception pending */
#define CP0_DEBUG_COUNTDM           (1 << 25) /* Bit 25: Count register behavior (1=running) */
#define CP0_DEBUG_HALT              (1 << 26) /* Bit 26: Internal system bus clock stopped */
#define CP0_DEBUG_DOZE              (1 << 27) /* Bit 27: Processor in low power mode */
#define CP0_DEBUG_LSNM              (1 << 28) /* Bit 28: Load/store in DSEG goes to main memory */
#define CP0_DEBUG_NODCR             (1 << 29) /* Bit 29: No DSEG preset */
#define CP0_DEBUG_DM                (1 << 30) /* Bit 30: Processor is operating in DEBUG mode */
#define CP0_DEBUG_DBD               (1 << 31) /* Bit 31: Last debug exception occurred in a dely slot */

/* Register Number: 23 Sel: ? Name: Debug2
 * Is this documented anywhere?
 */

/* Register Number: 24 Sel: 0 Name: DEPC
 * Function: Program counter at last EJTAG debug exception
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 25-29 Reserved
 * Compliance Level: Recommended/Optional.
 *
 * Register Number: 30 Sel: 0 Name: ErrorEPC
 * Function: Program counter at last error
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 31 Sel: 0 Name: DeSAVE
 * Function: EJTAG debug exception save register
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
#endif /* __ARCH_MIPS_INCLUDE_PIC32MX_CP0_H */
