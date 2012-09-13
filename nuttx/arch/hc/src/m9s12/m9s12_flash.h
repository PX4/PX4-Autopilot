/************************************************************************************
 * arch/hc/src/m9s12/m9s12_flash.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_FLASH_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_FLASH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Flash memory map *****************************************************************
 *
 * 0x4000-0x7fff: 16Kb Fixed FLASH EEPROM (Page 3e)
 * 0x8000-0xbfff: 16Kb Page window
 * 0xc000-0xffff: 16Kb Fixed FLASH EEPROM (Page 3f)
 * (see chip.h)
 *
 * The M9S12 implements 6 bits of the PPAGE register which gives it a
 * 1 Mbyte program memory address space that is accessed through the PPAGE
 * window. The lower 768K portion of the address space, accessed with PPAGE
 * values $00 through $2F, is reserved for external memory when the part is
 * operated in expanded mode. The upper 256K of the address space, accessed
 * with PPAGE values $30 through $3F.
 */

#define HCS12_BTLDR_BASE   0xf800 /* 0xf800-0xffff: 2Kb Protected bootloader FLASH */

/* User-Accessible utility subroutines provided by the serial monitor:
 *
 * PutChar -     Sends the character in A out SCI0
 *
 *               PutChar: brclr   SCI0SR1,TDRE,PutChar ;wait for Tx ready
 *                        staa    SCI0DRL       ;send character from A
 *                        rts
 *
 * GetChar -     Wait indefinitely for a character to be received via SCI.
 *               Return received character in A.
 *
 *               GetChar: brset  SCI0SR1,RDRF,RxReady ;exit loop when RDRF=1
 *                        bra    GetChar              ;loop till RDRF set
 *               RxReady: ldaa   SCI0DRL              ;read character into A
 *                        rts                         ;return
 * EraseAllCmd - Use repeated page erase commands to erase all flash
 *               except bootloader in protected block at the end of flash, and mass
 *               erase all EEPROM locations
 * DoOnStack -   Copy to stack and execute from RAM
 * WriteD2IX -   Write the data in D (word) to the address in IX
 *               The location may be RAM, FLASH, EEPROM, or a register
 *               if FLASH or EEPROM, the operation is completed before return
 *               IX and A preserved, returns Z=1 (.EQ.) if OK
 */

#ifdef CONFIG_HCS12_SERIALMON
#  define PutChar          0xfee6 /* Sends the character in A out SCI0 */
#  define GetChar          0xfee9 /* Return character received from SCIO in A */
#  define EraseAllCmd      0xfeec /* Erase all flash (except bootloader) */
#  define DoOnStack        0xfeef /* Copy to stack and execute from RAM */
#  define WriteD2IX        0xfef2 /* Write the data in D (word) to the address in IX.
                                   * The location may be RAM, FLASH, EEPROM, or a register */

/* Serial monitor version */

#  define SWID_DEVID       0xfef8
#  define SWID_DATE        0xfefa
#  define SWID_YEAR        0xfefc
#  define SWID_VER         0xfefe

/* Interrupts */

#  define HCS12_UVECTOR_BASE 0xf780 /* 0xf780–0xf7fe: User vector base */
#endif

/* FLASH interface */

#define HCS12_BACKDOOR_KEY 0xff00 /* 0xff00-0xff07: Backdoor comparison keys */
                                  /* 0xff08-0xff0c: Reserved */
#define HCS12_FLASH_PROT   0xff0d /* 0xff0d:        Flash protection byte */
                                  /* 0xff0e:        Reserved */
#define HCS12_FLASH_OPT    0xff0f /* 0xff0f:        Flash options/security byte */

/* Interrupts */

#define HCS12_VECTOR_BASE  0xff80 /* 0xff80–0xfffe: Actual vector base */

/* Register Offsets *****************************************************************/

#define HCS12_FLASH_FCLKDIV_OFFSET (HCS12_FLASH_BASE+0x00) /* Flash Clock Divider Register */
#define HCS12_FLASH_FSEC_OFFSET    (HCS12_FLASH_BASE+0x03) /* Flash Security Register  */
#define HCS12_FLASH_FCNFG_OFFSET   (HCS12_FLASH_BASE+0x03) /* Flash Configuration Register */
#define HCS12_FLASH_FPROT_OFFSET   (HCS12_FLASH_BASE+0x04) /* Flash Protection Register */
#define HCS12_FLASH_FSTAT_OFFSET   (HCS12_FLASH_BASE+0x05) /* Flash Status Register */
#define HCS12_FLASH_FCMD_OFFSET    (HCS12_FLASH_BASE+0x06) /* Flash Command Register */
#define HCS12_FLASH_FADDRHI_OFFSET (HCS12_FLASH_BASE+0x08) /* Flash High Address Register */
#define HCS12_FLASH_FADDRLO_OFFSET (HCS12_FLASH_BASE+0x09) /* Flash Low Address Register */
#define HCS12_FLASH_FDATAHI_OFFSET (HCS12_FLASH_BASE+0x0a) /* Flash High Data Register */
#define HCS12_FLASH_FDATALO_OFFSET (HCS12_FLASH_BASE+0x0b) /* Flash Low Data Register (*/

/* Register Addresses ***************************************************************/

#define HCS12_FLASH_FCLKDIV        (HCS12_REG_BASE+HCS12_FLASH_FCLKDIV_OFFSET)
#define HCS12_FLASH_FSEC           (HCS12_REG_BASE+HCS12_FLASH_FSEC_OFFSET)
#define HCS12_FLASH_FCNFG          (HCS12_REG_BASE+HCS12_FLASH_FCNFG_OFFSET)
#define HCS12_FLASH_FPROT          (HCS12_REG_BASE+HCS12_FLASH_FPROT_OFFSET)
#define HCS12_FLASH_FSTAT          (HCS12_REG_BASE+HCS12_FLASH_FSTAT_OFFSET)
#define HCS12_FLASH_FCMD           (HCS12_REG_BASE+HCS12_FLASH_FCMD_OFFSET)
#define HCS12_FLASH_FADDRHI        (HCS12_REG_BASE+HCS12_FLASH_FADDRHI_OFFSET)
#define HCS12_FLASH_FADDRLO        (HCS12_REG_BASE+HCS12_FLASH_FADDRLO_OFFSET)
#define HCS12_FLASH_FDATAHI        (HCS12_REG_BASE+HCS12_FLASH_FDATAHI_OFFSET)
#define HCS12_FLASH_FDATALO        (HCS12_REG_BASE+HCS12_FLASH_FDATALO_OFFSET)

/* Register Bit Definitions *********************************************************/

#define FLASH_FCLKDIV_FDIV_SHIFT   (0)      /* Bits 0-5: Clock Divider Bits */
#define FLASH_FCLKDIV_FDIV_MASK    (0x3f << FLASH_FCLKDIV_FDIV_SHIFT)
#define FLASH_FCLKDIV_PRDIV8       (1 << 6) /* Bit 6: Enable Prescaler by 8 */
#define FLASH_FCLKDIV_FDIVLD       (1 << 7) /* Bit 7: Clock Divider Loaded */

#define FLASH_FSET_SEC_SHIFT       (0)      /* Bits 0-1: Memory Security Bits */
#define FLASH_FSET_SEC_MASK        (3 << FLASH_FSET_SEC_SHIFT)
#  define FLASH_FSET_SEC_UNSECURED (2 << FLASH_FSET_SEC_SHIFT)
#define FLASH_FSET_KEYEN           (1 << 7) /* Bit 7: Backdoor Key Enable */
#define FLASH_FSET_NV_SHIFT        (2)      /* Bits 2-6: Nonvolatile Flag Bits */
#define FLASH_FSET_NV_MASK         (0x1f << FLASH_FSET_NV_SHIFT)

#define FLASH_FCNG_KEYACC          (1 << 5) /* Bit 5: Enable Security Key Writing */
#define FLASH_FCNG_CCIE            (1 << 6) /* Bit 6: Command Complete Interrupt Enable */
#define FLASH_FCNG_CBEIE           (1 << 7) /* Bit 7: Command Buffer Empty Interrupt Enable */

#define FLASH_FPROT_FPLS_SHIFT     (0)      /* Bits 0-1: Flash Protection Lower Address Size */
#define FLASH_FPROT_FPLS_MASK      (3 << FLASH_FPROT_FPLS_SHIFT)
#  define FLASH_FPROT_FPLS_512B    (0 << FLASH_FPROT_FPLS_SHIFT)
#  define FLASH_FPROT_FPLS_1KB     (1 << FLASH_FPROT_FPLS_SHIFT)
#  define FLASH_FPROT_FPLS_2KB     (2 << FLASH_FPROT_FPLS_SHIFT)
#  define FLASH_FPROT_FPLS_4KB     (3 << FLASH_FPROT_FPLS_SHIFT)
#define FLASH_FPROT_FPLDIS         (1 << 2) /* Bit 2: Flash Protection Lower address range Disable */
#define FLASH_FPROT_FPHS_SHIFT     (3)      /* Bits 3-4: Flash Protection Higher Address Size */
#define FLASH_FPROT_FPHS_MASK      (3 << FLASH_FPROT_FPHS_SHIFT)
#  define FLASH_FPROT_FPHS_2KB     (0 << FLASH_FPROT_FPHS_SHIFT)
#  define FLASH_FPROT_FPHS_4KB     (1 << FLASH_FPROT_FPHS_SHIFT)
#  define FLASH_FPROT_FPHS_8KB     (2 << FLASH_FPROT_FPHS_SHIFT)
#  define FLASH_FPROT_FPHS_16KB    (3 << FLASH_FPROT_FPHS_SHIFT)
#define FLASH_FPROT_FPHDIS         (1 << 5) /* Bit 5: Flash Protection Higher address range disable */
#define FLASH_FPROT_FPOPEN         (1 << 7) /* Bit 7: Opens the Flash block for program or erase */

#define FLASH_FSTAT_BLANK          (1 << 2) /* Bit 2: Array has been verified as erased */
#define FLASH_FSTAT_ACCERR         (1 << 4) /* Bit 4: Flash access error */
#define FLASH_FSTAT_PVIOL          (1 << 5) /* Bit 5: Protection violation */
#define FLASH_FSTAT_CCIF           (1 << 6) /* Bit 6: Command complete interrupt flag */
#define FLASH_FSTAT_CBEIF          (1 << 7) /* Bit 7: Command buffer empty interrupt flag */

#define FLASH_FCMD_ERASEVERIFY     0x05     /* Erase Verify */
#define FLASH_FCMD_WORDPROGRM      0x20     /* Word Program */
#define FLASH_FCMD_SECTORERASE     0x40     /* Sector Erase */
#define FLASH_FCMD_MASSERASE       0x41     /* Mass Erase */

#define FLASH_FADDRHI_MASK         0x7f

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_FLASH_H */
