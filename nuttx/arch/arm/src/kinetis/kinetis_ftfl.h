/************************************************************************************
 * arch/arm/src/kinetis/kinetis_ftfl.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_FTFL_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_FTFL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_FTFL_FSTAT_OFFSET   0x0000 /* Flash Status Register */
#define KINETIS_FTFL_FCNFG_OFFSET   0x0001 /* Flash Configuration Register */
#define KINETIS_FTFL_FSEC_OFFSET    0x0002 /* Flash Security Register */
#define KINETIS_FTFL_FOPT_OFFSET    0x0003 /* Flash Option Register */

#define KINETIS_FTFL_FCCOB3_OFFSET  0x0004 /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB2_OFFSET  0x0005 /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB1_OFFSET  0x0006 /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB0_OFFSET  0x0007 /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB7_OFFSET  0x0008 /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB6_OFFSET  0x0009 /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB5_OFFSET  0x000a /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB4_OFFSET  0x000b /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOBB_OFFSET  0x000c /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOBA_OFFSET  0x000d /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB9_OFFSET  0x000e /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FCCOB8_OFFSET  0x000f /* Flash Common Command Object Registers */
#define KINETIS_FTFL_FPROT3_OFFSET  0x0010 /* Program Flash Protection Registers */
#define KINETIS_FTFL_FPROT2_OFFSET  0x0011 /* Program Flash Protection Registers */
#define KINETIS_FTFL_FPROT1_OFFSET  0x0012 /* Program Flash Protection Registers */
#define KINETIS_FTFL_FPROT0_OFFSET  0x0013 /* Program Flash Protection Registers */
#define KINETIS_FTFL_FEPROT_OFFSET  0x0016 /* EEPROM Protection Register */
#define KINETIS_FTFL_FDPROT_OFFSET  0x0017 /* Data Flash Protection Register */

/* Register Addresses ***************************************************************/

#define KINETIS_FTFL_FSTAT          (KINETIS_FTFL_BASE+KINETIS_FTFL_FSTAT_OFFSET)
#define KINETIS_FTFL_FCNFG          (KINETIS_FTFL_BASE+KINETIS_FTFL_FCNFG_OFFSET)
#define KINETIS_FTFL_FSEC           (KINETIS_FTFL_BASE+KINETIS_FTFL_FSEC_OFFSET)
#define KINETIS_FTFL_FOPT           (KINETIS_FTFL_BASE+KINETIS_FTFL_FOPT_OFFSET)
#define KINETIS_FTFL_FCCOB3         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB3_OFFSET)
#define KINETIS_FTFL_FCCOB2         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB2_OFFSET)
#define KINETIS_FTFL_FCCOB1         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB1_OFFSET)
#define KINETIS_FTFL_FCCOB0         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB0_OFFSET)
#define KINETIS_FTFL_FCCOB7         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB7_OFFSET)
#define KINETIS_FTFL_FCCOB6         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB6_OFFSET)
#define KINETIS_FTFL_FCCOB5         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB5_OFFSET)
#define KINETIS_FTFL_FCCOB4         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB4_OFFSET)
#define KINETIS_FTFL_FCCOBB         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOBB_OFFSET)
#define KINETIS_FTFL_FCCOBA         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOBA_OFFSET)
#define KINETIS_FTFL_FCCOB9         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB9_OFFSET)
#define KINETIS_FTFL_FCCOB8         (KINETIS_FTFL_BASE+KINETIS_FTFL_FCCOB8_OFFSET)
#define KINETIS_FTFL_FPROT3         (KINETIS_FTFL_BASE+KINETIS_FTFL_FPROT3_OFFSET)
#define KINETIS_FTFL_FPROT2         (KINETIS_FTFL_BASE+KINETIS_FTFL_FPROT2_OFFSET)
#define KINETIS_FTFL_FPROT1         (KINETIS_FTFL_BASE+KINETIS_FTFL_FPROT1_OFFSET)
#define KINETIS_FTFL_FPROT0         (KINETIS_FTFL_BASE+KINETIS_FTFL_FPROT0_OFFSET)
#define KINETIS_FTFL_FEPROT         (KINETIS_FTFL_BASE+KINETIS_FTFL_FEPROT_OFFSET)
#define KINETIS_FTFL_FDPROT         (KINETIS_FTFL_BASE+KINETIS_FTFL_FDPROT_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Flash Status Register */

#define FTFL_FSTAT_MGSTAT0          (1 << 0)  /* Bit 0:  Memory Controller Command Completion Status Flag */
                                              /* Bits 1-3: Reserved */
#define FTFL_FSTAT_FPVIOL           (1 << 4)  /* Bit 4:  Flash Protection Violation Flag */
#define FTFL_FSTAT_ACCERR           (1 << 5)  /* Bit 5:  Flash Access Error Flag */
#define FTFL_FSTAT_RDCOLERR         (1 << 6)  /* Bit 6:  FTFL Read Collision Error Flag */
#define FTFL_FSTAT_CCIF             (1 << 7)  /* Bit 7:  Command Complete Interrupt Flag */

/* Flash Configuration Register */

#define FTFL_FCNFG_EEERDY           (1 << 0)  /* Bit 0:  FEEPROM backup data copied to FlexRAM */
#define FTFL_FCNFG_RAMRDY           (1 << 1)  /* Bit 1:  RAM Ready */
#define FTFL_FCNFG_PFLSH            (1 << 2)  /* Bit 2:  FTFL configuration */
#define FTFL_FCNFG_SWAP             (1 << 3)  /* Bit 3:  Swap */
#define FTFL_FCNFG_ERSSUSP          (1 << 4)  /* Bit 4:  Erase Suspend */
#define FTFL_FCNFG_ERSAREQ          (1 << 5)  /* Bit 5:  Erase All Request */
#define FTFL_FCNFG_RDCOLLIE         (1 << 6)  /* Bit 6:  Read Collision Error Interrupt Enable */
#define FTFL_FCNFG_CCIE             (1 << 7)  /* Bit 7:  Command Complete Interrupt Enable */

/* Flash Security Register */

#define FTFL_FSEC_SEC_SHIFT         (0)       /* Bits 0-1: Flash Security */
#define FTFL_FSEC_SEC_MASK          (3 << FTFL_FSEC_SEC_SHIFT)
#  define FTFL_FSEC_SEC_SECURE      (0 << FTFL_FSEC_SEC_SHIFT) /* 00,01,11: status is secure */
#  define FTFL_FSEC_SEC_UNSECURE    (2 << FTFL_FSEC_SEC_SHIFT) /* 10: status is insecure */
#define FTFL_FSEC_FSLACC_SHIFT      (2)       /* Bits 2-3: Freescale Failure Analysis Access Code */
#define FTFL_FSEC_FSLACC_MASK       (3 << FTFL_FSEC_FSLACC_SHIFT)
#  define FTFL_FSEC_FSLACC_GRANTED  (0 << FTFL_FSEC_FSLACC_SHIFT) /* 00 or 11: Access granted */
#  define FTFL_FSEC_FSLACC_DENIED   (1 << FTFL_FSEC_FSLACC_SHIFT) /* 01 or 10: Access denied */
#define FTFL_FSEC_MEEN_SHIFT        (4)       /* Bits 4-5: Mass Erase Enable Bits */
#define FTFL_FSEC_MEEN_MASK         (3 << FTFL_FSEC_MEEN_SHIFT)
#  define FTFL_FSEC_MEEN_ENABLED    (0 << FTFL_FSEC_MEEN_SHIFT) /* All values are enabled */
#define FTFL_FSEC_KEYEN_SHIFT       (6)       /* Bits 6-7: Backdoor Key Security Enable */
#define FTFL_FSEC_KEYEN_MASK        (3 << FTFL_FSEC_KEYEN_SHIFT)
#  define FTFL_FSEC_KEYEN_DISABLED  (1 << FTFL_FSEC_KEYEN_SHIFT) /* All values are disabled */

/* Flash Option Register (32-bits, see Chip Configuration details) */
/* Flash Common Command Object Registers (8-bit flash command data) */
/* Program Flash Protection Registers (8-bit flash protection data) */
/* EEPROM Protection Register (8-bit eeprom protection data) */
/* Data Flash Protection Register (8-bit data flash protection data) */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_FTFL_H */
