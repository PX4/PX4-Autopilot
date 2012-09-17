/************************************************************************************
 * dm320/dm320_gio.h
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

#ifndef __DM320_DM320GIO_H
#define __DM320_DM320GIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* General I/O Registers */

#define DM320_GIO_DIR0    (DM320_PERIPHERALS_VADDR + 0x0580) /* GIO Direction Register 0 */
#define DM320_GIO_DIR1    (DM320_PERIPHERALS_VADDR + 0x0582) /* GIO Direction Register 1 */
#define DM320_GIO_DIR2    (DM320_PERIPHERALS_VADDR + 0x0584) /* GIO Direction Register 2 */
#define DM320_GIO_INV0    (DM320_PERIPHERALS_VADDR + 0x0586) /* GIO Inversion Register 0 */
#define DM320_GIO_INV1    (DM320_PERIPHERALS_VADDR + 0x0588) /* GIO Inversion Register 1 */
#define DM320_GIO_INV2    (DM320_PERIPHERALS_VADDR + 0x058A) /* GIO Inversion Register 2 */
#define DM320_GIO_BITSET0 (DM320_PERIPHERALS_VADDR + 0x058C) /* GIO Bit Set Register 0 */
#define DM320_GIO_BITSET1 (DM320_PERIPHERALS_VADDR + 0x058E) /* GIO Bit Set Register 1 */
#define DM320_GIO_BITSET2 (DM320_PERIPHERALS_VADDR + 0x0590) /* GIO Bit Set Register 2 */
#define DM320_GIO_BITCLR0 (DM320_PERIPHERALS_VADDR + 0x0592) /* GIO Bit Clear Register 0 */
#define DM320_GIO_BITCLR1 (DM320_PERIPHERALS_VADDR + 0x0594) /* GIO Bit Clear Register 1 */
#define DM320_GIO_BITCLR2 (DM320_PERIPHERALS_VADDR + 0x0596) /* GIO Bit Clear Register 2 */
#define DM320_GIO_IRQPORT (DM320_PERIPHERALS_VADDR + 0x0598) /* GIO IRQ Port Setting Register */
#define DM320_GIO_IRQEDGE (DM320_PERIPHERALS_VADDR + 0x059A) /* GIO IRQ Edge Setting Register */
#define DM320_GIO_CHAT0   (DM320_PERIPHERALS_VADDR + 0x059C) /* GIO Chatter Setting Register 0 */
#define DM320_GIO_CHAT1   (DM320_PERIPHERALS_VADDR + 0x059E) /* GIO Chatter Setting Register 1 */
#define DM320_GIO_CHAT2   (DM320_PERIPHERALS_VADDR + 0x05A0) /* GIO Chatter Setting Register 2 */
#define DM320_GIO_NCHAT   (DM320_PERIPHERALS_VADDR + 0x05A2) /* GIO Chatter Value Register */
#define DM320_GIO_FSEL0   (DM320_PERIPHERALS_VADDR + 0x05A4) /* GIO Function Select Register 0 */
#define DM320_GIO_FSEL1   (DM320_PERIPHERALS_VADDR + 0x05A6) /* GIO Function Select Register 1 */
#define DM320_GIO_FSEL2   (DM320_PERIPHERALS_VADDR + 0x05A8) /* GIO Function Select Register 2 */
#define DM320_GIO_FSEL3   (DM320_PERIPHERALS_VADDR + 0x05AA) /* GIO Function Select Register 3 */

/* Macros for GIO access */

#define _GIO_READ_REG(pin, reg0, reg1, reg2, bval) \
  do { \
    register uint32_t _reg; register int _pin; \
    if ((pin) < 16) { _reg = (reg0); _pin = (pin); } \
    else if ((pin) < 32) { _reg = (reg1); _pin = ((pin) - 16); } \
    else { _reg = (reg2); _pin = ((pin) - 32); } \
    bval = ((getreg16(_reg) & (1<<_pin)) != 0); \
  }

#define _GIO_SET_REG(pin, reg0, reg1, reg2) \
  do { \
    register uint32_t _reg; register int _pin; \
    if ((pin) < 16) { _reg = (reg0); _pin = (pin); } \
    else if ((pin) < 32) { _reg = (reg1); _pin = ((pin) - 16); } \
    else { _reg = (reg2); _pin = ((pin) - 32); } \
    putreg16((getreg16(_reg) | (1 << _pin)), _reg); \
  } while (0)

#define _GIO_CLEAR_REG(pin, reg0, reg1, reg2) \
  do { \
    register uint32_t _reg; register int _pin; \
    if ((pin) < 16) { _reg = (reg0); _pin = (pin); } \
    else if ((pin) < 32) { _reg = (reg1); _pin = ((pin) - 16); } \
    else { _reg = (reg2); _pin = ((pin) - 32); } \
    putreg16((getreg16(_reg) & ~(1 << _pin)), _reg); \
  } while (0)

/* Select GIO input or output */

#define GIO_INPUT(pin) \
  _GIO_SET_REG((pin), DM320_GIO_DIR0, DM320_GIO_DIR1, DM320_GIO_DIR2)
#define GIO_OUTPUT(pin) \
  _GIO_CLEAR_REG((pin), DM320_GIO_DIR0, DM320_GIO_DIR1, DM320_GIO_DIR2)

/* Select inverted or non-inverted GIO */

#define GIO_INVERTED(pin) \
  _GIO_SET_REG((pin), DM320_GIO_INV0, DM320_GIO_INV1, DM320_GIO_INV2)
#define GIO_NONINVERTED(pin) \
  _GIO_CLEAR_REG((pin), DM320_GIO_INV0, DM320_GIO_INV1, DM320_GIO_INV2)

/* Set and clear outputs */

#define GIO_SET_OUTPUT(pin) \
  _GIO_SET_REG((pin), DM320_GIO_BITSET0, DM320_GIO_BITSET1, DM320_GIO_BITSET2)
#define GIO_CLEAR_OUTPUT(pin) \
 _GIO_SET_REG((pin), DM320_GIO_BITCLR0, DM320_GIO_BITCLR1, DM320_GIO_BITCLR2)

/* Read input */

#define GIO_READ_INPUT(pin, bval) \
  _GIO_READ_REG((pin), DM320_GIO_BITSET0, DM320_GIO_BITSET1, DM320_GIO_BITSET2, (bval))

/* Configure GIO pins */

#define _GIO_SET_CONFIG(reg, sh, val) \
    putreg16(((getreg16(reg) & ~(3 << sh)) | (val << sh)), (reg))

#define GIO_CONFIGURE(pin, val) \
  do {\
    if ((pin) < 10) _GIO_SET_CONFIG(DM320_GIO_FSEL0, 0, (val)); \
    else if ((pin) < 17) _GIO_SET_CONFIG(DM320_GIO_FSEL0, 2*((pin)-9), (val)); \
    else if ((pin) < 25) _GIO_SET_CONFIG(DM320_GIO_FSEL1, 2*((pin)-17), (val)); \
    else if ((pin) < 33) _GIO_SET_CONFIG(DM320_GIO_FSEL2, 2*((pin)-25), (val)); \
    else _GIO_SET_CONFIG(DM320_GIO_FSEL3, 2*((pin)-33), (val)); \
  }

/* Configure GIO interrupts (pins 1-15) */

#define GIO_INTERRUPT(pin) \
  if (pin < 16) putreg16((getreg16(DM320_GIO_IRQPORT) | (1<<(pin))), DM320_GIO_IRQPORT)
#define GIO_NONINTERRUPT(pin) \
  if (pin < 16) putreg16((getreg16(DM320_GIO_IRQPORT) & ~(1<<(pin))), DM320_GIO_IRQPORT)
#define GIO_FALLINGEDGE(pin) \
  if (pin < 16) { \
    putreg16((getreg16(DM320_GIO_IRQEDGE) & ~(1<<(pin))), DM320_GIO_IRQEDGE) \
    putreg16((getreg16(DM320_GIO_INV0) & ~(1<<(pin))), DM320_GIO_INV0); \
  }
#define GIO_RISINGEDGE(pin) \
  if (pin < 16) { \
    putreg16((getreg16(DM320_GIO_IRQEDGE) & ~(1<<(pin))), DM320_GIO_IRQEDGE); \
    putreg16((getreg16(DM320_GIO_INV0) | (1<<(pin))), DM320_GIO_INV0); \
  }
#define GIO_BOTHEDGES(pin) \
  if (pin < 16) { \
    putreg16((getreg16(DM320_GIO_IRQEDGE) | (1<<(pin))), DM320_GIO_IRQEDGE); \
    putreg16((getreg16(DM320_GIO_INV0) & ~(1<<(pin))), DM320_GIO_INV0); \
  }

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __DM320_DM320_GIO_H */
