/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_lcd.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_LCD_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_LCD_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* LCD register base address offset into the APB2 domain ****************************************/

#define LPC31_LCD_VBASE                (LPC31_APB2_VSECTION+LPC31_APB2_LCD_OFFSET)
#define LPC31_LCD_PBASE                (LPC31_APB2_PSECTION+LPC31_APB2_LCD_OFFSET)

/* LCD register offsets (with respect to the LCD base) ******************************************/

#define LPC31_LCD_STATUS_OFFSET        0x000 /* Status register */
#define LPC31_LCD_CONTROL_OFFSET       0x004 /* Control register */
#define LPC31_LCD_INTRAW_OFFSET        0x008 /* Interrupt Raw register */
#define LPC31_LCD_INTCLEAR_OFFSET      0x00c /* Interrupt Clear register */
#define LPC31_LCD_INTMASK_OFFSET       0x010 /* Interrupt Mask Register */
#define LPC31_LCD_READCMD_OFFSET       0x014 /* Read Command register */
#define LPC31_LCD_INSTBYTE_OFFSET      0x020 /* Instruction Byte Register */
#define LPC31_LCD_DATABYTE_OFFSET      0x030 /* Data Byte Register */
#define LPC31_LCD_INSTWORD_OFFSET      0x040 /* Instruction Word register */
#define LPC31_LCD_DATAWORD_OFFSET      0x080 /* Data Word register */

/* LCD register (virtual) addresses *************************************************************/

#define LPC31_LCD_STATUS               (LPC31_LCD_VBASE+LPC31_LCD_STATUS_OFFSET)
#define LPC31_LCD_CONTROL              (LPC31_LCD_VBASE+LPC31_LCD_CONTROL_OFFSET)
#define LPC31_LCD_INTRAW               (LPC31_LCD_VBASE+LPC31_LCD_INTRAW_OFFSET)
#define LPC31_LCD_INTCLEAR             (LPC31_LCD_VBASE+LPC31_LCD_INTCLEAR_OFFSET)
#define LPC31_LCD_INTMASK              (LPC31_LCD_VBASE+LPC31_LCD_INTMASK_OFFSET)
#define LPC31_LCD_READCMD              (LPC31_LCD_VBASE+LPC31_LCD_READCMD_OFFSET)
#define LPC31_LCD_INSTBYTE             (LPC31_LCD_VBASE+LPC31_LCD_INSTBYTE_OFFSET)
#define LPC31_LCD_DATABYTE             (LPC31_LCD_VBASE+LPC31_LCD_DATABYTE_OFFSET)
#define LPC31_LCD_INSTWORD             (LPC31_LCD_VBASE+LPC31_LCD_INSTWORD_OFFSET)
#define LPC31_LCD_DATAWORD             (LPC31_LCD_VBASE+LPC31_LCD_DATAWORD_OFFSET)

/* LCD register bit definitions *****************************************************************/
/* LCD interface Status Register LCD_STATUS, address 0x15000400 */

#define LCD_STATUS_COUNTER_SHIFT         (5)       /* Bits 5-9: Current value of the FIFO counter */
#define LCD_STATUS_COUNTER_MASK          (0x1f << LCD_STATUS_COUNTER_SHIFT)
#define LCD_STATUS_INTERFACEBUSY         (1 << 4)  /* Bit 4:  LCD interface still reading value */
#define LCD_STATUS_INTREADVALID          (1 << 3)  /* Bit 3:  Value read from the LCD controller is valid */
#define LCD_STATUS_INTFIFOOVERRUN        (1 << 2)  /* Bit 2:  Value written is larger than the FIFO can hold */
#define LCD_STATUS_INTFIFOHALFEMPTY      (1 << 1)  /* Bit 1:  FIFO is less then half full */
#define LCD_STATUS_INTFIFOEMPTY          (1 << 0)  /* Bit 0:  FIFO is empty */

/* LCD interface Control register LCD_CONTROL, address 0x15000404 */

#define LCD_CONTROL_BYASYNCRELCLK        (1 << 20) /* Bit 20: Bypass PCLK & LCDLCOK asynch logic */
#define LCD_CONTROL_IF16                 (1 << 19) /* Bit 19: Interface to 16 bit LCD-Controller*/
#define LCD_CONTROL_LOOPBACK             (1 << 18) /* Bit 18: LCD Interface in Loopback mode*/
#define LCD_CONTROL_MSBFIRST             (1 << 17) /* Bit 17: Send multi-byte data MSB first*/
#define LCD_CONTROL_INVERTERD            (1 << 16) /* Bit 16: Invert polarity of E_RD*/
#define LCD_CONTROL_INVERTCS             (1 << 15) /* Bit 15: Invert CS*/
#define LCD_CONTROL_BUSYRSVALUE          (1 << 14) /* Bit 14: Busy check on RS=1*/
#define LCD_CONTROL_BUSYBITNR_SHIFT      (10)      /* Bits 10-13: Bit that represents busy flag*/
#define LCD_CONTROL_BUSYBITNR_MASK       (15 << LCD_CONTROL_BUSYBITNR_SHIFT)
#define LCD_CONTROL_BUSYVALUE            (1 << 9)  /* Bit 9:  LCD controller is busy if bit=1*/
#define LCD_CONTROL_BUSYFLAGCHECK        (1 << 8)  /* Bit 8:  Enable the busy-flag-checking*/
#define LCD_CONTROL_SERRDPOSS_SHIFT      (9)       /* Bits 6-7: 7:6 Serial sample mode*/
#define LCD_CONTROL_SERRDPOSS_MASK       (3 << LCD_CONTROL_SERRDPOSS_SHIFT)
#  define LCD_CONTROL_SERRDPOSS_START    (0 << LCD_CONTROL_SERRDPOSS_SHIFT) /* Sample at beginning of cycle*/
#  define LCD_CONTROL_SERRDPOSS_FOURTH   (1 << LCD_CONTROL_SERRDPOSS_SHIFT) /* Sample at 0.25 * cycle*/
#  define LCD_CONTROL_SERRDPOSS_HALF     (2 << LCD_CONTROL_SERRDPOSS_SHIFT) /* Sample at 0.5 * cycle*/
#  define LCD_CONTROL_SERRDPOSS_3FOURTHS (3 << LCD_CONTROL_SERRDPOSS_SHIFT) /* Sample at 0.75 * cycle*/
#define LCD_CONTROL_SERCLKSHIFT_SHIFT    (4)       /* Bits 4-5: Serial clock mode*/
#define LCD_CONTROL_SERCLKSHIFT_MASK     (3 << LCD_CONTROL_SERCLKSHIFT_SHIFT)
#  define LCD_CONTROL_SERCLKSHIFT_MODE0  (0 << LCD_CONTROL_SERCLKSHIFT_SHIFT) /* Clock mode 0*/
#  define LCD_CONTROL_SERCLKSHIFT_MODE1  (1 << LCD_CONTROL_SERCLKSHIFT_SHIFT) /* Clock mode 1*/
#  define LCD_CONTROL_SERCLKSHIFT_MODE2  (2 << LCD_CONTROL_SERCLKSHIFT_SHIFT) /* Clock mode 2*/
#  define LCD_CONTROL_SERCLKSHIFT_MODE3  (3 << LCD_CONTROL_SERCLKSHIFT_SHIFT) /* Clock mode 3*/
#define LCD_CONTROL_4BIT                 (1 << 3)  /* Bit 2:  LCD interface 4 bit mode (vs 8)*/
#define LCD_CONTROL_MI                   (1 << 2)  /* Bit 2:  LCD interface 6800 mode (vs 8080 mode)*/
#define LCD_CONTROL_PS                   (1 << 1)  /* Bit 1:  LCD interface serial mode (vs parallel)*/

/* LCD interface Interrupt Raw register LCD_INTRAW, address 0x15000408
 * LCD interface Interrupt Clear register LCD_INTCLEAR, address 0x1500040c
 * LCD interface Interrupt Mask register LCD_INTMASK, address 0x15000410
 */

#define LCD_INT_READVALID                (1 << 3)  /* Bit 3:  Value that has been read from the LCD controller */
#define LCD_INT_OVERRUN                  (1 << 2)  /* Bit 2:  FIFO overrun */
#define LCD_INT_FIFOHALFEMPTY            (1 << 1)  /* Bit 1:  FIFO is less then half full */
#define LCD_INT_FIFO_EMPTY               (1 << 0)  /* Bit 0:  FIFO is empty */

/* LCD interface Read Command register LCD_READCMD, address 0x15000414 */

#define LCD_READCMD_DATABYTE             (1 << 0)  /* Bit 0: Read in DATA byte (vs INST) */

/* LCD interface Instruction Byte register LCD_INSTBYTE, address 0x15000420 */

#define LCD_INSTBYTE_WORD_SHIFT          (0)       /* Bits 0-15: 16 bit mode = 15:0 Instruction */
#define LCD_INSTBYTE_WORD_MASK           (0xffff << LCD_INSTBYTE_WORD_SHIFT)
#define LCD_INSTBYTE_BYTE_SHIFT          (0)       /* Bits 0-7: 8 bit mode = 7:0 Instruction */
#define LCD_INSTBYTE_BYTE_MASK           (0xff << LCD_INSTBYTE_BYTE_SHIFT)

/* LCD interface Data Byte register LCD_DATABYTE, address 0x15000430 */

#define LCD_DATABYTE_WORD_SHIFT          (0)       /* Bits 0-15: 16 bit mode = 15:0 Instruction */
#define LCD_DATABYTE_WORD_MASK           (0xffff << LCD_IDATABYTE_WORD_SHIFT)
#define LCD_DATABYTE_BYTE_SHIFT          (0)       /* Bits 0-7: 8 bit mode = 7:0 Instruction */
#define LCD_DATABYTE_BYTE_MASK           (0xff << LCD_IDATABYTE_BYTE_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_LCD_H */
