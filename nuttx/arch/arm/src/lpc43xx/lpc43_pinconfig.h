/********************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_pinconfig.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_PINCONFIG_H
#define __ARCH_ARM_SRC_LPC43XX_PINCONFIG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/* Include the chip capabilities and SCU definitions file */

#include "chip.h"
#include "chip/lpc43_scu.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Each configurable pin can be individually configured by software in several modes. The
 * following definitions provide the bit encoding that is used to define a pin configuration.
 * Note that these pins do not corresponding GPIO ports and pins.
 *
 * 20-bit Encoding:
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .FFF UUDD IGWS SSSP PPPP
 */

/* Alternate function number:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .FFF .... .... .... ....
 */

#define PINCONF_FUNC_SHIFT            (16)       /* Bits 16-18: Alternate function number */
#define PINCONF_FUNC_MASK             (7 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC(n)             ((n) << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC0               (0 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC1               (1 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC2               (2 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC3               (3 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC4               (4 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC5               (5 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC6               (6 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC7               (7 << PINCONF_FUNC_SHIFT)

/* Pull-up/down resisters.  These selections are available for all pins but may not
 * make sense for all pins.  NOTE: that both pull up and down is not precluded.
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... UU.. .... .... ....
 */

#define PINCONF_PULLUP                (1 << 15) /* Bit 15: 1=Pull-up */
#define PINCONF_PULLDOWN              (1 << 14) /* Bit 14: 1=Pull-down */
#define PINCONF_FLOAT                 (0)       /* Bit 14-15=0 if neither */

#define PINCONF_IS_PULLUP(p)          (((p) & PINCONF_PULLUP) != 0)
#define PINCONF_IS_PULLDOWN(p)        (((p) & PINCONF_PULLDOWN) != 0)
#define PINCONF_IS_FLOAT(p)           (((p) & (PINCONF_PULLUP|PINCONF_PULLDOWN) == 0)

/* Drive strength.  These selections are available only for high-drive pins
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... ..DD .... .... ....
 */

#define PINCONF_DRIVE_SHIFT           (12)       /* Bits 12-13 = Pin drive strength */
#define PINCONF_DRIVE_MASK            (3 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_NORMAL        (0 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_MEDIUM        (1 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_HIGH          (2 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_ULTRA         (3 << PINCONF_DRIVE_SHIFT)

/* Input buffer enable
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... I... .... ....
 */

#define PINCONF_INBUFFER              (1 << 11)  /* Bit 11: 1=Enabled input buffer */
#define PINCONF_INBUFFER_ENABLED(p)   (((p) & PINCONF_INBUFFER) != 0)

/* Glitch filter enable
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .G.. .... ....
 */

#define PINCONF_GLITCH                (1 << 10)  /* Bit 10: 1=Glitch filter enable */
#define PINCONF_GLITCH_ENABLE(p)      (((p) & PINCONF_GLITCH) == 0)

/* Slew rate
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..W. .... ....
 */

#define PINCONF_SLEW_FAST             (1 << 9)   /* Bit 9: 1=Alternate function */
#define PINCONF_SLEW_SLOW             (0)        /* Bit 9: 0=Normal function */

#define PINCONF_IS_SLEW_FAST(p)       (((p) & PINCONF_SLEW_FAST) != 0)
#define PINCONF_IS_SLOW_SLOW(p)       (((p) & PINCONF_SLEW_FAST) == 0)

/* Pin configuration sets:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...S SSS. ....
 */

#define PINCONF_PINS_SHIFT            (5)        /* Bits 5-8: Pin set */
#define PINCONF_PINS_MASK             (15 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS0               (0 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS1               (1 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS2               (2 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS3               (3 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS4               (4 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS5               (5 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS6               (6 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS7               (7 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS8               (8 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS9               (9 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSA               (10 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSB               (11 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSC               (12 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSD               (13 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSE               (14 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSF               (15 << PINCONF_PINS_SHIFT)

/* Pin numbers:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... ...P PPPP
 */
 
#define PINCONF_PIN_SHIFT             (0)        /* Bits 0-4: Pin number */
#define PINCONF_PIN_MASK              (31 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_0               (0 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_1               (1 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_2               (2 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_3               (3 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_4               (4 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_5               (5 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_6               (6 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_7               (7 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_8               (8 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_9               (9 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_10              (10 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_11              (11 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_12              (12 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_13              (13 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_14              (14 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_15              (15 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_16              (16 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_17              (17 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_18              (18 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_19              (19 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_20              (20 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_21              (21 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_22              (22 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_23              (23 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_24              (24 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_25              (25 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_26              (26 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_27              (27 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_28              (28 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_29              (29 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_30              (30 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_31              (31 << PINCONF_PIN_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

/********************************************************************************************
 * Name: lpc43_pin_config
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 * Input Value:
 *   20-bit encoded value describing the pin.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ********************************************************************************************/

EXTERN int lpc43_pin_config(uint32_t pinconf);

/********************************************************************************************
 * Function:  lpc43_pin_dump
 *
 * Description:
 *   Dump all pin configuration registers associated with the provided pin configuration
 *
 ********************************************************************************************/

#ifdef CONFIG_DEBUG
EXTERN int lpc43_pin_dump(uint32_t pinconf, const char *msg);
#else
#  define lpc43_pin_dump(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LPC43XX_PINCONFIG_H */
