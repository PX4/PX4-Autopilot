/****************************************************************************
 *  arch/arm/src/kinetis/kinetis_pinirq.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"

#include "kinetis_internal.h"
#include "kinetis_port.h"

#ifdef CONFIG_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The Kinetis port interrupt logic is very flexible and will program interrupts on
 * most all pin events.  In order to keep the memory usage to a minimum, the NuttX
 * port supports enabling interrupts on a per-port basis.
 */

#if defined (CONFIG_KINETIS_PORTAINTS) || defined (CONFIG_KINETIS_PORTBINTS) || \
    defined (CONFIG_KINETIS_PORTCINTS) || defined (CONFIG_KINETIS_PORTDINTS) || \
    defined (CONFIG_KINETIS_PORTEINTS)
#  define HAVE_PORTINTS 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Per pin port interrupt vectors.  NOTE:  Not all pins in each port
 * correspond to externally available GPIOs.  However, I believe that the
 * Kinesis will support interrupts even if the pin is not available as
 * a GPIO. Hence, we need to support all 32 pins for each port.  To keep the
 * memory usage at a minimum, the logic may be configure per port.
 */

#ifdef CONFIG_KINETIS_PORTAINTS
static xcpt_t g_portaisrs[32];
#endif
#ifdef CONFIG_KINETIS_PORTBINTS
static xcpt_t g_portbisrs[32];
#endif
#ifdef CONFIG_KINETIS_PORTCINTS
static xcpt_t g_portcisrs[32];
#endif
#ifdef CONFIG_KINETIS_PORTDINTS
static xcpt_t g_portdisrs[32];
#endif
#ifdef CONFIG_KINETIS_PORTEINTS
static xcpt_t g_porteisrs[32];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_portinterrupt
 *
 * Description:
 *   Common port interrupt handling.
 *
 ****************************************************************************/

#ifdef HAVE_PORTINTS
static int kinetis_portinterrupt(int irq, FAR void *context,
                                uintptr_t addr, xcpt_t *isrtab)
{
  uint32_t isfr = getreg32(addr);
  int i;

  /* Examine each pin in the port */

  for (i = 0; i < 32 && isfr != 0; i++)
    {
      /* A bit set in the ISR means that an interrupt is pending for this
       * pin.  If the pin is programmed for level sensitive inputs, then
       * the interrupt handling logic MUST disable the interrupt (or cause
       * the level to change) to prevent infinite interrupts.
       */

      uint32_t bit = (1 << i);
      if ((isfr & bit ) != 0)
        {
          /* I think that bits may be set in the ISFR for DMA activities
           * well.  So, no error is declared if there is no registered
           * interrupt handler for the pin.
           */

          if (isrtab[i])
            {
              /* There is a registered interrupt handler... invoke it */

              (void)isrtab[i](irq, context);
            }

          /* Writing a one to the ISFR register will clear the pending
           * interrupt.  If pin is configured to generate a DMA request
           * then the ISFR bit will be cleared automatically at the
           * completion of the requested DMA transfer. If configured for
           * a level sensitive interrupt and the pin remains asserted and
           * the bit will set again immediately after it is cleared.
           */

          isfr &= ~bit;
          putreg32(bit, addr);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: kinetis_portXinterrupt
 *
 * Description:
 *   Handle interrupts arriving on individual ports
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_PORTAINTS
static int kinetis_portainterrupt(int irq, FAR void *context)
{
  return kinetis_portinterrupt(irq, context, KINETIS_PORTA_ISFR, g_portaisrs);
}
#endif
#ifdef CONFIG_KINETIS_PORTBINTS
static int kinetis_portbinterrupt(int irq, FAR void *context)
{
  return kinetis_portinterrupt(irq, context, KINETIS_PORTB_ISFR, g_portbisrs);
}
#endif
#ifdef CONFIG_KINETIS_PORTCINTS
static int kinetis_portcinterrupt(int irq, FAR void *context)
{
  return kinetis_portinterrupt(irq, context, KINETIS_PORTC_ISFR, g_portcisrs);
}
#endif
#ifdef CONFIG_KINETIS_PORTDINTS
static int kinetis_portdinterrupt(int irq, FAR void *context)
{
  return kinetis_portinterrupt(irq, context, KINETIS_PORTD_ISFR, g_portdisrs);
}
#endif
#ifdef CONFIG_KINETIS_PORTEINTS
static int kinetis_porteinterrupt(int irq, FAR void *context)
{
  return kinetis_portinterrupt(irq, context, KINETIS_PORTE_ISFR, g_porteisrs);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_pinirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void kinetis_pinirqinitialize(void)
{
#ifdef CONFIG_KINETIS_PORTAINTS
  (void)irq_attach(KINETIS_IRQ_PORTA, kinetis_portainterrupt);
  putreg32(0xffffffff, KINETIS_PORTA_ISFR);
  up_enable_irq(KINETIS_IRQ_PORTA);
#endif
#ifdef CONFIG_KINETIS_PORTBINTS
  (void)irq_attach(KINETIS_IRQ_PORTB, kinetis_portbinterrupt);
  putreg32(0xffffffff, KINETIS_PORTB_ISFR);
  up_enable_irq(KINETIS_IRQ_PORTB);
#endif
#ifdef CONFIG_KINETIS_PORTCINTS
  (void)irq_attach(KINETIS_IRQ_PORTC, kinetis_portcinterrupt);
  putreg32(0xffffffff, KINETIS_PORTC_ISFR);
  up_enable_irq(KINETIS_IRQ_PORTC);
#endif
#ifdef CONFIG_KINETIS_PORTDINTS
  (void)irq_attach(KINETIS_IRQ_PORTD, kinetis_portdinterrupt);
  putreg32(0xffffffff, KINETIS_PORTD_ISFR);
  up_enable_irq(KINETIS_IRQ_PORTD);
#endif
#ifdef CONFIG_KINETIS_PORTEINTS
  (void)irq_attach(KINETIS_IRQ_PORTE, kinetis_porteinterrupt);
  putreg32(0xffffffff, KINETIS_PORTE_ISFR);
  up_enable_irq(KINETIS_IRQ_PORTE);
#endif
}

/****************************************************************************
 * Name: kinetis_pinirqattach
 *
 * Description:
 *   Attach a pin interrupt handler.  The normal initalization sequence is:
 *
 *   1. Call kinetis_pinconfig() to configure the interrupting pin (pin interrupts
 *      will be disabled.
 *   2. Call kinetis_pinirqattach() to attach the pin interrupt handling function.
 *   3. Call kinetis_pinirqenable() to enable interrupts on the pin.
 *
 * Parameters:
 *  - pinset:  Pin configuration
 *  - pinisr:  Pin interrupt service routine
 *
 * Returns:
 *   The previous value of the interrupt handler function pointer.  This
 *   value may, for example, be used to restore the previous handler whe
 *   multiple handlers are used.
 *
 ****************************************************************************/

xcpt_t kinetis_pinirqattach(uint32_t pinset, xcpt_t pinisr)
{
#ifdef HAVE_PORTINTS
  xcpt_t      *isrtab;
  xcpt_t       oldisr;
  irqstate_t   flags;
  unsigned int port;
  unsigned int pin;

  /* It only makes sense to call this function for input pins that are configured
   * as interrupts.
   */

  DEBUGASSERT((pinset & _PIN_INTDMA_MASK) == _PIN_INTERRUPT);
  DEBUGASSERT((pinset & _PIN_IO_MASK) == _PIN_INPUT);

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  /* Get the table associated with this port */

  DEBUGASSERT(port < KINETIS_NPORTS);
  flags = irqsave();
  switch (port)
    {
#ifdef CONFIG_KINETIS_PORTAINTS
      case KINETIS_PORTA :
        isrtab = g_portaisrs;
        break;
#endif
#ifdef CONFIG_KINETIS_PORTBINTS
      case KINETIS_PORTB :
        isrtab = g_portbisrs;
        break;
#endif
#ifdef CONFIG_KINETIS_PORTCINTS
      case KINETIS_PORTC :
        isrtab = g_portcisrs;
        break;
#endif
#ifdef CONFIG_KINETIS_PORTDINTS
      case KINETIS_PORTD :
        isrtab = g_portdisrs;
        break;
#endif
#ifdef CONFIG_KINETIS_PORTEINTS
      case KINETIS_PORTE :
        isrtab = g_porteisrs;
        break;
#endif
      default:
        return NULL;
    }

   /* Get the old PIN ISR and set the new PIN ISR */

   oldisr      = isrtab[pin];
   isrtab[pin] = pinisr;

   /* And return the old PIN isr address */

   return oldisr;
#else
   return NULL;
#endif /* HAVE_PORTINTS */
}

/************************************************************************************
 * Name: kinetis_pinirqenable
 *
 * Description:
 *   Enable the interrupt for specified pin IRQ
 *
 ************************************************************************************/

void kinetis_pinirqenable(uint32_t pinset)
{
#ifdef HAVE_PORTINTS
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < KINETIS_NPORTS);
  if (port < KINETIS_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  KINETIS_PORT_BASE(port);

      /* Modify the IRQC field of the port PCR register in order to enable
       * the interrupt.
       */

      regval = getreg32(base + KINETIS_PORT_PCR_OFFSET(pin));
      regval &= ~PORT_PCR_IRQC_MASK;

      switch (pinset & _PIN_INT_MASK)
        {
          case PIN_INT_ZERO : /* Interrupt when logic zero */
            regval |= PORT_PCR_IRQC_ZERO;
            break;

          case PIN_INT_RISING : /* Interrupt on rising edge*/
            regval |= PORT_PCR_IRQC_RISING;
            break;

          case PIN_INT_BOTH : /* Interrupt on falling edge */
            regval |= PORT_PCR_IRQC_FALLING;
            break;

          case PIN_DMA_FALLING : /* nterrupt on either edge */
            regval |= PORT_PCR_IRQC_BOTH;
            break;

          case PIN_INT_ONE : /* IInterrupt when logic one */
            regval |= PORT_PCR_IRQC_ONE;
            break;

          default:
            return;
        }

      putreg32(regval, base + KINETIS_PORT_PCR_OFFSET(pin));
    }
#endif /* HAVE_PORTINTS */
}

/************************************************************************************
 * Name: kinetis_pinirqdisable
 *
 * Description:
 *   Disable the interrupt for specified pin
 *
 ************************************************************************************/

void kinetis_pinirqdisable(uint32_t pinset)
{
#ifdef HAVE_PORTINTS
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < KINETIS_NPORTS);
  if (port < KINETIS_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  KINETIS_PORT_BASE(port);

      /* Clear the IRQC field of the port PCR register in order to disable
       * the interrupt.
       */

      regval = getreg32(base + KINETIS_PORT_PCR_OFFSET(pin));
      regval &= ~PORT_PCR_IRQC_MASK;
      putreg32(regval, base + KINETIS_PORT_PCR_OFFSET(pin));
    }
#endif /* HAVE_PORTINTS */
}
#endif /* CONFIG_GPIO_IRQ */
