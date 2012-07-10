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
#include <nuttx/irq.h>

/* Include the chip capabilities and GPIO definitions file */

#include "chip.h"
#include "chip/lpc43_gpio.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Max number of GPIO ports and the maximum number of pins per port */

#define NUM_GPIO_PORTS                8
#define NUM_GPIO_PINS                 32
#define NUM_GPIO_NGROUPS              2

/* Each configurable pin can be individually configured by software in several modes. The
 * following definitions provide the bit encoding that is used to define a pin configuration.
 * Note that these pins do not corresponding GPIO ports and pins.
 *
 * 16-bit Encoding:
 *                   1111 1100 0000 0000
 *                   5432 1098 7654 3210
 *                   ---- ---- ---- ----
 * Normal GPIO:      MMV. .... PPPB BBBB
 * Normal Interrupt: MMCC CIII PPPB BBBB
 * Group  Interrupt: MM.N P... PPPB BBBB
 */

/* GPIO mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * MM.. .... .... ....
 */

#define GPIO_MODE_SHIFT            (14)       /* Bits 14-15: Mode of the GPIO pin */
#define GPIO_MODE_MASK             (3 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INPUT          (0 << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_MODE_OUTPUT         (1 << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_MODE_PININTR        (2 << GPIO_MODE_SHIFT) /* GPIO pin interrupt */
#  define GPIO_MODE_GRPINTR        (3 << GPIO_MODE_SHIFT) /* GPIO group interrupt */

#define GPIO_IS_OUTPUT(p)          (((p) & GPIO_MODE_MASK) == GPIO_MODE_INPUT)
#define GPIO_IS_INPUT(p)           (((p) & GPIO_MODE_MASK) == GPIO_MODE_OUTPUT)
#define GPIO_IS_PININT(p)          (((p) & GPIO_MODE_MASK) == GPIO_MODE_PININTR)
#define GPIO_IS_GRPINTR(p)         (((p) & GPIO_MODE_MASK) == GPIO_MODE_GRPINTR)

/* Initial value (for GPIO outputs only)
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ..V. .... .... ....
 */

#define GPIO_VALUE_ONE             (1 << 13) /* Bit 13: 1=High */
#define GPIO_VALUE_ZERO            (0)       /* Bit 13: 0=Low */

#define GPIO_IS_ONE(p)             (((p) & GPIO_VALUE_ONE) != 0)
#define GPIO_IS_ZERO(p)            (((p) & GPIO_VALUE_ONE) == 0)

/* Group Interrupt Group Selection (valid only for GPIO group interrupts):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ...N .... .... ....
 */

#define GPIO_GRPINT_GROUPNO        (1 << 12) /* Bit 12: 1=Member of group 1 */
#define GPIO_GRPINT_GROUP0         (0)
#define GPIO_GRPINT_GROUP1         GPIO_GRPINT_GROUPNO

#define GPIO_IS_GROUP0(p)          (((p) & GPIO_GRPINT_GROUPNO) == 0)
#define GPIO_IS_GROUP1(p)          (((p) & GPIO_GRPINT_GROUPNO) != 0)

/* Group Interrupt Polarity (valid only for GPIO group interrupts):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... P... .... ....
 */

#define GPIO_POLARITY              (1 << 11) /* Bit 11: Group Polarity */
#define GPIO_POLARITY_HI           GPIO_POLARITY
#define GPIO_POLARITY_LOW          0

#define GPIO_IS_POLARITY_HI(p)     (((p) & GPIO_POLARITY) != 0)
#define GPIO_IS_POLARITY_LOW(p)    (((p) & GPIO_POLARITY) == 0)

/* Pin interrupt number (valid only for GPIO pin interrupts)
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ..CC C... .... ....
 */

#define GPIO_PININT_SHIFT          (10)        /* Bits 11-13: Pin interrupt number */
#define GPIO_PININT_MASK           (7 << GPIO_PININT_SHIFT)
#  define GPIO_PININT0             (0 << GPIO_PININT_SHIFT)
#  define GPIO_PININT1             (1 << GPIO_PININT_SHIFT)
#  define GPIO_PININT2             (2 << GPIO_PININT_SHIFT)
#  define GPIO_PININT3             (3 << GPIO_PININT_SHIFT)
#  define GPIO_PININT4             (4 << GPIO_PININT_SHIFT)
#  define GPIO_PININT5             (5 << GPIO_PININT_SHIFT)
#  define GPIO_PININT6             (6 << GPIO_PININT_SHIFT)
#  define GPIO_PININT7             (7 << GPIO_PININT_SHIFT)

/* Pin interrupt configuration (valid only for GPIO pin interrupts)
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .III .... ....
 */

#define _GPIO_INT_LEVEL            (1 << 10)  /* Bit 10: 1=Level (vs edge) */
#define _GPIO_INT_HIGH             (1 << 9)   /* Bit 9:  1=High level or rising edge */
#define _GPIO_INT_LOW              (1 << 8)   /* Bit 8:  1=Low level or falling edge */

#define GPIO_INT_SHIFT             (8)        /* Bits 8-10: Interrupt mode */
#define GPIO_INT_MASK              (7 << GPIO_INT_SHIFT)
#  define GPIO_INT_LEVEL_HI        (1 << GPIO_INT_SHIFT) /* 001 Edge=NO  LOW=0 HIGH=1 */
#  define GPIO_INT_LEVEL_LOW       (2 << GPIO_INT_SHIFT) /* 010 Edge=NO  LOW=1 HIGH=0 */
#  define GPIO_INT_EDGE_RISING     (5 << GPIO_INT_SHIFT) /* 101 Edge=YES LOW=0 HIGH=1 */
#  define GPIO_INT_EDGE_FALLING    (6 << GPIO_INT_SHIFT) /* 110 Edge=YES LOW=1 HIGH=0 */
#  define GPIO_INT_EDGE_BOTH       (7 << GPIO_INT_SHIFT) /* 111 Edge=YES LOW=1 HIGH=1 */

#define GPIO_IS_ACTIVE_HI(p)       (((p) & _GPIO_INT_HIGH)  != 0)
#define GPIO_IS_ACTIVE_LOW(p)      (((p) & _GPIO_INT_LOW)   != 0)
#define GPIO_IS_EDGE(p)            (((p) & _GPIO_INT_LEVEL) == 0)
#define GPIO_IS_LEVEL(p)           (((p) & _GPIO_INT_LEVEL) != 0)

/* GPIO Port Number:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... PPP. ....
 */

#define GPIO_PORT_SHIFT            (5)        /* Bits 5-7: Port number */
#define GPIO_PORT_MASK             (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0               (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1               (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2               (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3               (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4               (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5               (5 << GPIO_PORT_SHIFT)
#  define GPIO_PORT6               (6 << GPIO_PORT_SHIFT)
#  define GPIO_PORT7               (7 << GPIO_PORT_SHIFT)

/* GPIO Pin Number:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             (0)        /* Bits 0-5: Pin number */
#define GPIO_PIN_MASK              (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10               (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11               (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12               (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13               (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14               (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15               (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16               (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17               (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18               (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19               (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20               (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21               (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22               (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23               (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24               (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25               (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26               (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27               (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28               (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29               (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30               (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31               (31 << GPIO_PIN_SHIFT)

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
 * Name: lpc43_gpio_config
 *
 * Description:
 *   Configure a GPIO based on bit-encoded description of the pin.  NOTE: The pin *must*
 *   have first been configured for GPIO usage with a corresponding call to lpc43_pin_config.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ********************************************************************************************/

EXTERN int lpc43_gpio_config(uint16_t gpiocfg);

/********************************************************************************************
 * Name: lpc43_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Returned Value:
 *   None
 *
 ********************************************************************************************/

EXTERN void lpc43_gpio_write(uint16_t gpiocfg, bool value);

/********************************************************************************************
 * Name: lpc43_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Returned Value:
 *   The boolean state of the input pin
 *
 ********************************************************************************************/

EXTERN bool lpc43_gpio_read(uint16_t gpiocfg);

/********************************************************************************************
 * Function:  lpc43_gpio_dump
 *
 * Description:
 *   Dump all pin configuration registers associated with the provided base address
 *
 ********************************************************************************************/

#ifdef CONFIG_DEBUG
EXTERN int lpc43_gpio_dump(uint16_t gpiocfg, const char *msg);
#else
#  define lpc43_gpio_dump(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LPC43XX_GPIO_H */
