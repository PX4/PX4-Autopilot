/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_internal.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_INTERNAL_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "at32uc3_config.h"

#ifdef CONFIG_AVR32_GPIOIRQ
#  include <nuttx/irq.h>
#endif

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to at32uc3_configgpio() ********************************/

/* 16-bit Encoding:
 * PERIPHERAL: FMMI UXXG PPPB BBBB with G=0
 * GPIO:       FMMI UVOG PPPB BBBB with G=1
 */

/* Glitch Filter Enable:
 * F... .... .... ....
 */

 #define GPIO_GLITCH               (1 << 15) /* Bit 15: Glitch filter enable */

/* Interrupt modes (valid only if GPIO_INTR==1)
 * .MM. .... .... ....
 */

#define GPIO_INTMODE_SHIFT         (13)      /* Bits 13-14: Interrupt mode */
#define GPIO_INTMODE_MASK          (3 << GPIO_INTMODE_SHIFT)
#  define GPIO_INTMODE_BOTH        (0 << GPIO_INTMODE_SHIFT)
#  define GPIO_INTMODE_RISING      (1 << GPIO_INTMODE_SHIFT)
#  define GPIO_INTMODE_FALLING     (2 << GPIO_INTMODE_SHIFT)

#  define GPIO_IMR0                (1 << GPIO_INTMODE_SHIFT)
#  define GPIO_IMR1                (2 << GPIO_INTMODE_SHIFT)

/* Interrupt enable
 * ...I .... .... ....
 */

#define GPIO_INTR                  (1 << 12) /* Bit 12: Interrupt enable */

/* Pull-up enable
 * .... U... .... ....
 */

#define GPIO_PULLUP                (1 << 11) /* Bit 11: Pull-up enable */

/* Output value (Valid only if GPIO_ENABLE and GPIO_OUTPUT)
 * .... .V.. .... ....
 */

#define GPIO_VALUE                 (1 << 10) /* Bit 10: Output value */
#define GPIO_HIGH                  GPIO_VALUE
#define GPIO_LOW                   (0)

/* Input/Ouptut (Valid only if GPIO_ENABLE)
 * .... ..O. .... ....
 */

#define GPIO_OUTPUT                (1 << 9) /* Bit 9: Output driver enable */
#define GPIO_INPUT                 (0)

/* Peripheral MUX setting (valid only if GPIO_PERIPH)
 * .... .XX. .... ....
 */

#define GPIO_FUNC_SHIFT            (9)       /* Bits 9-10: Peripheral MUX */
#define GPIO_FUNC_MASK             (3 << GPIO_FUNC_SHIFT)
#  define GPIO_FUNCA               (0 << GPIO_FUNC_SHIFT) /* PMR0=0 PMR1=0 */
#  define GPIO_FUNCB               (1 << GPIO_FUNC_SHIFT) /* PMR0=1 PMR1=0 */
#  define GPIO_FUNCC               (2 << GPIO_FUNC_SHIFT) /* PMR0=0 PMR1=1 */
#  define GPIO_FUNCD               (3 << GPIO_FUNC_SHIFT) /* PMR0=1 PMR1=1 */

#  define GPIO_PMR0                (1 << GPIO_FUNC_SHIFT)
#  define GPIO_PMR1                (2 << GPIO_FUNC_SHIFT)

/* GPIO Enable (1) or Peripheral Enable (0)
 * .... .... .... .... .... ...G .... ....
 */

#define GPIO_ENABLE                (1 << 8)  /* Bit 8:  GPIO enable */
#define GPIO_PERIPH                (0)


/* Port Number
 * .... .... .... .... .... .... PPP. ....
 */

#define GPIO_PORT_SHIFT            (5)       /* Bits 5-7: Port number */
#define GPIO_PORT_MASK             (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA               (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB               (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC               (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD               (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTE               (4 << GPIO_PORT_SHIFT)

/* Pin number:
 * .... .... .... .... .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             (0)       /* Bits 0-4: Port number */
#define GPIO_PIN_MASK              (0x1f << GPIO_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_clkinit
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h file.
 *
 ****************************************************************************/

EXTERN void up_clkinitialize(void);

/****************************************************************************
 * Name: usart_reset
 *
 * Description:
 *   Reset a USART.
 *
 ****************************************************************************/

EXTERN void usart_reset(uintptr_t usart_base);

/****************************************************************************
 * Name: usart_configure
 *
 * Description:
 *   Configure a USART as a RS-232 UART.
 *
 ****************************************************************************/

void usart_configure(uintptr_t usart_base, uint32_t baud, unsigned int parity,
                     unsigned int nbits, bool stop2);

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the intialization sequence to configure the serial console
 *   uart (only).
 *
 ****************************************************************************/

EXTERN void up_consoleinit(void);

/****************************************************************************
 * Name: up_boardinit
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory configs/<board-name>/up_boot.c.
 *
 ****************************************************************************/

EXTERN void up_boardinitialize(void);

/****************************************************************************
 * Name: at32uc3_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

EXTERN int at32uc3_configgpio(uint16_t cfgset);

/****************************************************************************
 * Name: at32uc3_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

EXTERN void at32uc3_gpiowrite(uint16_t pinset, bool value);

/****************************************************************************
 * Name: at32uc3_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

EXTERN bool at32uc3_gpioread(uint16_t pinset);

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 *   CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 *   interrupts on.
 *
 * Assumptions:
 *   Called during the early boot sequence before global interrupts have
 *   been enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR32_GPIOIRQ
EXTERN void weak_function gpio_irqinitialize(void);
#endif

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provide 'isr'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 *   CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 *   interrupts on.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR32_GPIOIRQ
EXTERN int gpio_irqattach(int irq, xcpt_t newisr, xcpt_t *oldisr);
#endif

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 *   CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 *   interrupts on.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR32_GPIOIRQ
EXTERN void gpio_irqenable(int irq);
#endif

/*****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 *   CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 *   interrupts on.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR32_GPIOIRQ
EXTERN void gpio_irqdisable(int irq);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_INTERNAL_H */

