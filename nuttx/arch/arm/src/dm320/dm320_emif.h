/************************************************************************************
 * dm320/dm320_emif.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#ifndef __DM320_DM320_EMIF_H
#define __DM320_DM320_EMIF_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* External Memory Interface (EMIF) Registers */

#define DM320_EMIF_CS0CTRL1     (DM320_PERIPHERALS_VADDR + 0x0A00) /* CS0 Control Register #1 */
#define DM320_EMIF_CS0CTRL2     (DM320_PERIPHERALS_VADDR + 0x0A02) /* CS0 Control Register #2 */
#define DM320_EMIF_CS0CTRL3     (DM320_PERIPHERALS_VADDR + 0x0A04) /* CS0 Control Register #3 */
#define DM320_EMIF_CS1CTRL1A    (DM320_PERIPHERALS_VADDR + 0x0A06) /* CS1 Control Register #1A */
#define DM320_EMIF_CS1CTRL1B    (DM320_PERIPHERALS_VADDR + 0x0A08) /* CS1 Control Register #1B */
#define DM320_EMIF_CS2CTRL2     (DM320_PERIPHERALS_VADDR + 0x0A0A) /* CS1 Control Register #2 */
#define DM320_EMIF_CS2CTRL1     (DM320_PERIPHERALS_VADDR + 0x0A0C) /* CS2 Control Register #1 */
#define DM320_EMIF_CS1CTRL2     (DM320_PERIPHERALS_VADDR + 0x0A0E) /* CS2 Control Register #2 */
#define DM320_EMIF_CS3CTRL1     (DM320_PERIPHERALS_VADDR + 0x0A10) /* CS3 Control Register #1 */
#define DM320_EMIF_CS3CTRL2     (DM320_PERIPHERALS_VADDR + 0x0A12) /* CS3 Control Register #2 */
#define DM320_EMIF_CS4CTRL1     (DM320_PERIPHERALS_VADDR + 0x0A14) /* CS4 Control Register #1 */
#define DM320_EMIF_CS4CTRL2     (DM320_PERIPHERALS_VADDR + 0x0A16) /* CS4 Control Register #2 */
#define DM320_EMIF_BUSCTRL      (DM320_PERIPHERALS_VADDR + 0x0A18) /* Bus Control Register */
#define DM320_EMIF_BUSRLS       (DM320_PERIPHERALS_VADDR + 0x0A1A) /* Bus Release Control Register */
#define DM320_EMIF_CFCTRL1      (DM320_PERIPHERALS_VADDR + 0x0A1C) /* CFC ControlRegister #1 */
#define DM320_EMIF_CFCTRL2      (DM320_PERIPHERALS_VADDR + 0x0A1E) /* CFC ControlRegister#2 */
#define DM320_EMIF_SMCTRL       (DM320_PERIPHERALS_VADDR + 0x0A20) /* SmartMedia Control Register */
#define DM320_EMIF_BUSINTEN     (DM320_PERIPHERALS_VADDR + 0x0A22) /* Bus Interrupt Enable Register */
#define DM320_EMIF_BUSSTS       (DM320_PERIPHERALS_VADDR + 0x0A24) /* Bus Status Register */
#define DM320_EMIF_BUSWAITMD    (DM320_PERIPHERALS_VADDR + 0x0A26) /* Bus Wait Mode Register */
#define DM320_EMIF_ECC1CP       (DM320_PERIPHERALS_VADDR + 0x0A28) /* ECC Area 1 CP Register */
#define DM320_EMIF_ECC1LP       (DM320_PERIPHERALS_VADDR + 0x0A2A) /* ECC Area 1 LP Register */
#define DM320_EMIF_ECC2CP       (DM320_PERIPHERALS_VADDR + 0x0A2C) /* ECC Area 2 CP Register */
#define DM320_EMIF_ECC2LP       (DM320_PERIPHERALS_VADDR + 0x0A2E) /* ECC Area 2 LP Register */
#define DM320_EMIF_ECC3CP       (DM320_PERIPHERALS_VADDR + 0x0A30) /* ECC Area 3 CP Register */
#define DM320_EMIF_ECC3LP       (DM320_PERIPHERALS_VADDR + 0x0A32) /* ECC Area 3 LP Register */
#define DM320_EMIF_ECC4CP       (DM320_PERIPHERALS_VADDR + 0x0A34) /* ECC Area 4 CP Register */
#define DM320_EMIF_ECC4LP       (DM320_PERIPHERALS_VADDR + 0x0A36) /* ECC Area 4 LP Register */
#define DM320_EMIF_ECC5CP       (DM320_PERIPHERALS_VADDR + 0x0A38) /* ECC Area 5 CP Register */
#define DM320_EMIF_ECC5LP       (DM320_PERIPHERALS_VADDR + 0x0A3A) /* ECC Area 5 LP Register */
#define DM320_EMIF_ECC6CP       (DM320_PERIPHERALS_VADDR + 0x0A3C) /* ECC Area 6 CP Register */
#define DM320_EMIF_ECC6LP       (DM320_PERIPHERALS_VADDR + 0x0A3E) /* ECC Area 6 LP Register */
#define DM320_EMIF_ECC7CP       (DM320_PERIPHERALS_VADDR + 0x0A40) /* ECC Area 7 CP Register */
#define DM320_EMIF_ECC7LP       (DM320_PERIPHERALS_VADDR + 0x0A42) /* ECC Area 7 LP Register */
#define DM320_EMIF_ECC8CP       (DM320_PERIPHERALS_VADDR + 0x0A44) /* ECC Area 8 CP Register */
#define DM320_EMIF_ECC8LP       (DM320_PERIPHERALS_VADDR + 0x0A46) /* ECC Area 8 LP Register */
#define DM320_EMIF_ECCCLR       (DM320_PERIPHERALS_VADDR + 0x0A48) /* ECC Clear Register */
#define DM320_EMIF_PAGESZ       (DM320_PERIPHERALS_VADDR + 0x0A4A) /* SmartMedia Page Size Register */
#define DM320_EMIF_PRIORCTL     (DM320_PERIPHERALS_VADDR + 0x0A4C) /* Priority control for DMA */
#define DM320_EMIF_IMGDSPDEST   (DM320_PERIPHERALS_VADDR + 0x0A4E) /* DSP/IMGBUF DMA destination */
#define DM320_EMIF_IMGDSPADDH   (DM320_PERIPHERALS_VADDR + 0x0A50) /* DSP/IMGBUF high address */
#define DM320_EMIF_IMGDSPADDL   (DM320_PERIPHERALS_VADDR + 0x0A52) /* DSP/IMGBUG low address */
#define DM320_EMIF_AHBADDH      (DM320_PERIPHERALS_VADDR + 0x0A54) /* AHB high address */
#define DM320_EMIF_AHBADDL      (DM320_PERIPHERALS_VADDR + 0x0A56) /* AHB low address */
#define DM320_EMIF_MTCADDH      (DM320_PERIPHERALS_VADDR + 0x0A58) /* MTC high address */
#define DM320_EMIF_MTCADDL      (DM320_PERIPHERALS_VADDR + 0x0A5A) /* MTC low address */
#define DM320_EMIF_DMASIZE      (DM320_PERIPHERALS_VADDR + 0x0A5C) /* DMA Transfer Size Register */
#define DM320_EMIF_DMAMTCSEL    (DM320_PERIPHERALS_VADDR + 0x0A5E) /* DMA Device Select Register */
#define DM320_EMIF_DMACTL       (DM320_PERIPHERALS_VADDR + 0x0A60) /* DMA Control Register */
#define DM320_EMIF_TEST         (DM320_PERIPHERALS_VADDR + 0x0A62) /* Test Register.Do not use */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __DM320_DM320_EMIF_H */
