/********************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_gpio.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_GPIO_H
#define __ARCH_ARM_SRC_LPC43XX_GPIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/* Include the chip capabilities and GPIO definitions file */

#include "chip.h"
#include "chip/lpc43_gpio.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
#define NUM_GPIO_PORTS                8
#define NUM_GPIO_PINS                 32

/* Each configurable pin can be individually configured by software in several modes. The
 * following definitions provide the bit encoding that is used to define a pin configuration.
 * Note that these pins do not corresponding GPIO ports and pins.
 *
 * 16-bit Encoding:
 *             1111 1100 0000 0000
 *             5432 1098 7654 3210
 *             ---- ---- ---- ----
 * Normal:    .MM. .... PPPB BBBB
 * Interrupt: .MMG GPII PPPB BBBB
 */

/* GPIO mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .MM. .... .... ....
 */

#define GPIO_MODE_SHIFT            (13)       /* Bits 13-14: Mode of the GPIO pin */
#define GPIO_MODE_MASK             (3 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INPUT          (1 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_OUTPUT         (2 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INTERRUPT      (3 << GPIO_MODE_SHIFT)

#define GPIO_IS_OUTPUT(p)          ((p) & GPIO_MODE_MASK) == GPIO_MODE_INPUT)
#define GPIO_IS_INPUT(p)           ((p) & GPIO_MODE_MASK) == GPIO_MODE_OUTPUT)
#define GPIO_IS_INTERRUPT(p)       ((p) & GPIO_MODE_MASK) == GPIO_MODE_INTERRUPT)

/* Group Interrupt Selection (valid only for interrupt GPIO pins):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ...G G... .... ....
 */

#define GPIO_GRPINT_SHIFT          (11)       /* Bits 11-12: Group interrupt selection */
#define GPIO_GRPINT_MASK           (3 << GPIO_GRPINT_SHIFT)
#  define GPIO_GRPINT_NONE         (0 << GPIO_GRPINT_SHIFT) /* 00 Not a member of a group */
#  define GPIO_GRPINT_GROUP0       (2 << GPIO_GRPINT_SHIFT) /* 10 Member of group 0 */
#  define GPIO_GRPINT_GROUP1       (3 << GPIO_GRPINT_SHIFT) /* 11 Member of group 1 */

#define _GPIO_GRPINT               (1 << (GPIO_GRPINT_SHIFT+1)) /* Bit 12: 1=Member of a group */
#define _GPIO_GRPNO                (1 << GPIO_GRPINT_SHIFT)     /* Bit 11: Group number */

#define GPIO_IS_GRPINT(p)          ((p) & _GPIO_GRPINT) != 0)
#define GPIO_GRPPNO(p)             ((p) & _GPIO_GRPNO) >> GPIO_GRPINT_SHIFT)

/* Group Interrupt Polarity (valid only for interrupt GPIO group interrupts ):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .P.. .... ....
 */

#define GPIO_POLARITY              (1 << 10) /* Bit 10: Group Polarity */

#define GPIO_POLARITY_HI           GPIO_POLARITY
#define GPIO_POLARITY_LOW          0

#define GPIO_IS_POLARITY_HI(p)     (((p) & GPIO_POLARITY) != 0)
#define GPIO_IS_POLARITY_LOW(p)    (((p) & GPIO_POLARITY) == 0)

/* Interrupt Configuration (valid only for interrupt GPIO pins):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... ..II .... ....
 */

#define GPIO_INT_SHIFT             (8)        /* Bits 8-9: Interrupt mode */
#define GPIO_INT_MASK              (3 << GPIO_INT_SHIFT)
#  define GPIO_INT_LEVEL_LOW       (0 << GPIO_INT_SHIFT) /* 00 Edge=NO, Active=LOW */
#  define GPIO_INT_LEVEL_HI        (1 << GPIO_INT_SHIFT) /* 01 Edge=NO, Active=HIGH */
#  define GPIO_INT_EDGE_FALLING    (2 << GPIO_INT_SHIFT) /* 10 Edge=YES, Active=LOW */
#  define GPIO_INT_EDGE_RISING     (3 << GPIO_INT_SHIFT) /* 11 Edge=YES, Active=LOW */

#define _GPIO_ACTIVE_HI            (1 << GPIO_INT_SHIFT)
#define _GPIO_EDGE                 (1 << (GPIO_INT_SHIFT+1))

#define GPIO_IS_ACTIVE_HI(p)       ((p) & _GPIO_ACTIVE_HI) != 0)
#define GPIO_IS_ACTIVE_LOW(p)      ((p) & _GPIO_ACTIVE_HI) == 0)
#define GPIO_IS_EDGE(p)            ((p) & _GPIO_EDGE) != 0)
#define GPIO_IS_LEVEL(p)           ((p) & _GPIO_EDGE) == 0)

/* GPIO Port Number:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... GPII .... ....
 */

#define GPIO_PORT_SHIFT               (4)        /* Bits 4-6: Port number */
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0                  (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1                  (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2                  (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3                  (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4                  (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5                  (5 << GPIO_PORT_SHIFT)
#  define GPIO_PORT6                  (6 << GPIO_PORT_SHIFT)
#  define GPIO_PORT7                  (7 << GPIO_PORT_SHIFT)

/* GPIO Pin Number:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT                (0)        /* Bits 0-5: Pin number */
#define GPIO_PIN_MASK                 (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16                  (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17                  (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18                  (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19                  (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20                  (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21                  (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22                  (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23                  (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24                  (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25                  (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26                  (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27                  (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28                  (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29                  (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30                  (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31                  (31 << GPIO_PIN_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/* Base addresses for each GPIO block */

extern const uint32_t g_gpiobase[NUM_GPIO_PORTS];

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

/********************************************************************************************
 * Name: lpc43_gpioconfig
 *
 * Description:
 *   Configure a GPIO based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ************************************************************************************/

EXTERN int lpc43_gpioconfig(uint16_t gpiocfg);

/************************************************************************************
 * Name: lpc43_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

EXTERN void lpc43_gpiowrite(uint16_t gpiocfg, bool value);

/************************************************************************************
 * Name: lpc43_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Returned Value:
 *   The boolean state of the input pin
 *
 ************************************************************************************/

EXTERN bool lpc43_gpioread(uint16_t gpiocfg);

/************************************************************************************
 * Name: lpc43_gpioattach
 *
 * Description:
 *   Attach and enable a GPIO interrupts on the selected GPIO pin, receiving the
 *   interrupt with the selected interrupt handler.  The GPIO interrupt may be
 *   disabled by providing a NULL value for the interrupt handler function pointer.
 *
 * Parameters:
 *  - gpiocfg: GPIO pin identification
 *  - func:   Interrupt handler
 *
 * Returns:
 *  The previous value of the interrupt handler function pointer.  This value may,
 *  for example, be used to restore the previous handler when multiple handlers are
 *  used.
 *
 ************************************************************************************/

EXTERN xcpt_t lpc43_gpioattach(uint16_t gpiocfg, xcpt_t func);

/************************************************************************************
 * Function:  lpc43_dumpgpio
 *
 * Description:
 *   Dump all pin configuration registers associated with the provided base address
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG
EXTERN int lpc43_dumpgpio(uint16_t gpiocfg, const char *msg);
#else
#  define lpc43_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_LPC43XX_GPIO_H */
