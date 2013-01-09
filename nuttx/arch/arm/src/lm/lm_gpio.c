/****************************************************************************
 * arch/arm/src/lm/lm_gpio.c
 * arch/arm/src/chip/lm_gpio.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
#include "lm_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These definitions are part of the implementation of the  GPIO pad
 * configuration of Table 9-1 in the LM3S6918 data sheet.
 */

#define AFSEL_SHIFT            5
#define AFSEL_1                (1 << AFSEL_SHIFT) /* Set/clear bit in GPIO AFSEL register */
#define AFSEL_0                0
#define AFSEL_X                0

#define DIR_SHIFT              4
#define DIR_1                  (1 << DIR_SHIFT)   /* Set/clear bit in GPIO DIR register */
#define DIR_0                  0
#define DIR_X                  0

#define ODR_SHIFT              3
#define ODR_1                  (1 << ODR_SHIFT)   /* Set/clear bit in GPIO ODR register */
#define ODR_0                  0
#define ODR_X                  0

#define DEN_SHIFT              2
#define DEN_1                  (1 << DEN_SHIFT)   /* Set/clear bit in GPIO DEN register */
#define DEN_0                  0
#define DEN_X                  0

#define PUR_SHIFT              1
#define PUR_1                  (1 << PUR_SHIFT)   /* Set/clear bit in GPIO PUR register */
#define PUR_0                  0
#define PUR_X                  0

#define PDR_SHIFT              0
#define PDR_1                  (1 << PDR_SHIFT)   /* Set/clear bit in GPIO PDR register */
#define PDR_0                  0
#define PDR_X                  0

#define GPIO_INPUT_SETBITS     (AFSEL_0 | DIR_0 | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_INPUT_CLRBITS     (AFSEL_1 | DIR_1 | ODR_1 | DEN_0 | PUR_X | PDR_X)

#define GPIO_OUTPUT_SETBITS    (AFSEL_0 | DIR_1 | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_OUTPUT_CLRBITS    (AFSEL_1 | DIR_0 | ODR_1 | DEN_0 | PUR_X | PDR_X)

#define GPIO_ODINPUT_SETBITS   (AFSEL_0 | DIR_0 | ODR_1 | DEN_1 | PUR_X | PDR_X)
#define GPIO_ODINPUT_CLRBITS   (AFSEL_1 | DIR_1 | ODR_0 | DEN_0 | PUR_X | PDR_X)

#define GPIO_ODOUTPUT_SETBITS  (AFSEL_0 | DIR_1 | ODR_1 | DEN_1 | PUR_X | PDR_X)
#define GPIO_ODOUTPUT_CLRBITS  (AFSEL_1 | DIR_0 | ODR_0 | DEN_0 | PUR_X | PDR_X)

#define GPIO_PFODIO_SETBITS    (AFSEL_1 | DIR_X | ODR_1 | DEN_1 | PUR_X | PDR_X)
#define GPIO_PFODIO_CLRBITS    (AFSEL_0 | DIR_X | ODR_0 | DEN_0 | PUR_X | PDR_X)

#define GPIO_PFIO_SETBITS      (AFSEL_1 | DIR_X | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_PFIO_CLRBITS      (AFSEL_0 | DIR_X | ODR_1 | DEN_0 | PUR_X | PDR_X)

#define GPIO_ANINPUT_SETBITS   (AFSEL_0 | DIR_0 | ODR_0 | DEN_0 | PUR_0 | PDR_0)
#define GPIO_ANINPUT_CLRBITS   (AFSEL_1 | DIR_1 | ODR_1 | DEN_1 | PUR_1 | PDR_1)

#define GPIO_INTERRUPT_SETBITS (AFSEL_0 | DIR_0 | ODR_0 | DEN_1 | PUR_X | PDR_X)
#define GPIO_INTERRUPT_CLRBITS (AFSEL_1 | DIR_1 | ODR_1 | DEN_0 | PUR_X | PDR_X)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_func_s
{
  uint8_t setbits;  /* A set of GPIO register bits to set */
  uint8_t clrbits;  /* A set of GPIO register bits to clear */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_func_s g_funcbits[] =
{
  {GPIO_INPUT_SETBITS,     GPIO_INPUT_CLRBITS},     /* GPIO_FUNC_INPUT */
  {GPIO_OUTPUT_SETBITS,    GPIO_OUTPUT_CLRBITS},    /* GPIO_FUNC_OUTPUT */
  {GPIO_ODINPUT_SETBITS,   GPIO_ODINPUT_CLRBITS},   /* GPIO_FUNC_ODINPUT */
  {GPIO_ODOUTPUT_SETBITS,  GPIO_ODOUTPUT_CLRBITS},  /* GPIO_FUNC_ODOUTPUT */
  {GPIO_PFODIO_SETBITS,    GPIO_PFODIO_CLRBITS},    /* GPIO_FUNC_PFODIO */
  {GPIO_PFIO_SETBITS,      GPIO_PFIO_CLRBITS},      /* GPIO_FUNC_PFIO */
  {GPIO_ANINPUT_SETBITS,   GPIO_ANINPUT_CLRBITS},   /* GPIO_FUNC_ANINPUT */
  {GPIO_INTERRUPT_SETBITS, GPIO_INTERRUPT_CLRBITS}, /* GPIO_FUNC_INTERRUPT */
};

static const uint32_t g_gpiobase[LM3S_NPORTS] =
{
  /* All support LM3S parts have at least 7 ports, GPIOA-G */

  LM3S_GPIOA_BASE, LM3S_GPIOB_BASE, LM3S_GPIOC_BASE, LM3S_GPIOD_BASE,
  LM3S_GPIOE_BASE, LM3S_GPIOF_BASE, LM3S_GPIOG_BASE,

  /* GPIOH exists on the LM3S6918 and th LM3S6B96, but not on the LM3S6965 or LM3S8962*/

#if LM3S_NPORTS > 7
  LM3S_GPIOH_BASE,
#endif

  /* GPIOJ exists on the LM3S6B96, but not on the LM3S6918 or LM3S6965 or LM3S8962*/

#if LM3S_NPORTS > 8
  LM3S_GPIOJ_BASE,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_gpiobaseaddress
 *
 * Description:
 *   Given a GPIO enumeration value, return the base address of the
 *   associated GPIO registers.
 *
 ****************************************************************************/

static uint32_t lm_gpiobaseaddress(unsigned int port)
{
  uint32_t gpiobase = 0;
  if (port < LM3S_NPORTS)
    {
      gpiobase = g_gpiobase[port];
    }
  return gpiobase;
}

/****************************************************************************
 * Name: lm_gpiofunc
 *
 * Description:
 *   Configure GPIO registers for a specific function
 *
 ****************************************************************************/

static void lm_gpiofunc(uint32_t base, uint32_t pinno, const struct gpio_func_s *func)
{
  uint32_t setbit;
  uint32_t clrbit;
  uint32_t regval;

  /* Set/clear/ignore the GPIO ODR bit. "The GPIO ODR register is the open drain
   * control register. Setting a bit in this register enables the open drain
   * configuration of the corresponding GPIO pad. When open drain mode is enabled,
   * the corresponding bit should also be set in the GPIO Digital Input Enable
   * (GPIO DEN) register ... Corresponding bits in the drive strength registers
   * (GPIO DR2R, GPIO DR4R, GPIO DR8R, and GPIO SLR ) can be set to achieve the
   * desired rise and fall times. The GPIO acts as an open drain input if the
   * corresponding bit in the GPIO DIR register is set to 0; and as an open
   * drain output when set to 1."
   */

  setbit = (((uint32_t)func->setbits >> ODR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> ODR_SHIFT) & 1) << pinno;

  regval = getreg32(base + LM3S_GPIO_ODR_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + LM3S_GPIO_ODR_OFFSET);

  /* Set/clear the GPIO PUR bit. "The GPIOPUR register is the pull-up control
   * register. When a bit is set to 1, it enables a weak pull-up resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPUR automatically clears the
   * corresponding bit in the GPIO Pull-Down Select (GPIOPDR) register ..."
   */

  setbit = (((uint32_t)func->setbits >> PUR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> PUR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      regval = getreg32(base + LM3S_GPIO_PUR_OFFSET);
      regval &= ~clrbit;
      regval |= setbit;
      putreg32(regval, base + LM3S_GPIO_PUR_OFFSET);
    }

  /* Set/clear the GPIO PDR bit. "The GPIOPDR register is the pull-down control
   * register. When a bit is set to 1, it enables a weak pull-down resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPDR automatically clears
   * the corresponding bit in the GPIO Pull-Up Select (GPIOPUR) register ..."
   */

  setbit = (((uint32_t)func->setbits >> PDR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> PDR_SHIFT) & 1) << pinno;

  if (setbit || clrbit)
    {
      regval = getreg32(base + LM3S_GPIO_PDR_OFFSET);
      regval &= ~clrbit;
      regval |= setbit;
      putreg32(regval, base + LM3S_GPIO_PDR_OFFSET);
    }

  /* Set/clear the GPIO DEN bit. "The GPIODEN register is the digital enable
   * register. By default, with the exception of the GPIO signals used for JTAG/SWD
   * function, all other GPIO signals are configured out of reset to be undriven
   * (tristate). Their digital function is disabled; they do not drive a logic
   * value on the pin and they do not allow the pin voltage into the GPIO receiver.
   * To use the pin in a digital function (either GPIO or alternate function), the
   * corresponding GPIODEN bit must be set."
   */

  setbit = (((uint32_t)func->setbits >> DEN_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> DEN_SHIFT) & 1) << pinno;

  regval = getreg32(base + LM3S_GPIO_DEN_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + LM3S_GPIO_DEN_OFFSET);

  /* Set/clear/ignore the GPIO DIR bit. "The GPIODIR register is the data
   * direction register. Bits set to 1 in the GPIODIR register configure
   * the corresponding pin to be an output, while bits set to 0 configure the
   * pins to be inputs. All bits are cleared by a reset, meaning all GPIO
   * pins are inputs by default.
   */

  setbit = (((uint32_t)func->setbits >> DIR_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> DIR_SHIFT) & 1) << pinno;

  regval = getreg32(base + LM3S_GPIO_DIR_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + LM3S_GPIO_DIR_OFFSET);

  /* Set/clear/ignore the GPIO AFSEL bit. "The GPIOAFSEL register is the mode
   * control select register. Writing a 1 to any bit in this register selects
   * the hardware control for the corresponding GPIO line. All bits are cleared
   * by a reset, therefore no GPIO line is set to hardware control by default."
   *
   * NOTE: In order so set JTAG/SWD GPIOs, it is also necessary to lock, commit
   * and unlock the GPIO.  That is not implemented here.
   */

  setbit = (((uint32_t)func->setbits >> AFSEL_SHIFT) & 1) << pinno;
  clrbit = (((uint32_t)func->clrbits >> AFSEL_SHIFT) & 1) << pinno;

  regval = getreg32(base + LM3S_GPIO_AFSEL_OFFSET);
  regval &= ~clrbit;
  regval |= setbit;
  putreg32(regval, base + LM3S_GPIO_AFSEL_OFFSET);
}

/****************************************************************************
 * Name: lm_gpiopadstrength
 *
 * Description:
 *   Set up pad strength and pull-ups
 *
 ****************************************************************************/

static inline void lm_gpiopadstrength(uint32_t base, uint32_t pin, uint32_t cfgset)
{
  int strength = (cfgset & GPIO_STRENGTH_MASK) >> GPIO_STRENGTH_SHIFT;
  uint32_t regoffset;
  uint32_t regval;
  uint32_t slrset;
  uint32_t slrclr;

  /* Prepare bits to disable slew */

  slrset = 0;
  slrclr = pin;

  switch (strength)
    {
      case 0: /* 2mA pad drive strength */
        {
          /* "The GPIODR2R register is the 2-mA drive control register. It
           * allows for each GPIO signal in the port to be individually configured
           * without affecting the other pads. When writing a DRV2 bit for a GPIO
           * signal, the corresponding DRV4 bit in the GPIO DR4R register and the
           * DRV8 bit in the GPIODR8R register are automatically cleared by hardware."
           */

          regoffset = LM3S_GPIO_DR2R_OFFSET;
        }
        break;

      case 1: /* 4mA pad drive strength */
        {
          /* "The GPIODR4R register is the 4-mA drive control register. It allows
           * for each GPIO signal in the port to be individually configured without
           * affecting the other pads. When writing the DRV4 bit for a GPIO signal,
           * the corresponding DRV2 bit in the GPIO DR2R register and the DRV8 bit
           * in the GPIO DR8R register are automatically cleared by hardware."
           */

          regoffset = LM3S_GPIO_DR4R_OFFSET;
        }
        break;

      case 3: /* 8mA Pad drive with slew rate control */
        {
          /* "The GPIOSLR register is the slew rate control register. Slew rate
           * control is only available when using the 8-mA drive strength option
           * via the GPIO 8-mA Drive Select (GPIODR8R) register..."
           */

          slrset = pin;
          slrclr = 0;
        }
        /* Fall through */

      case 2: /* 8mA pad drive strength (without slew rate control) */
        {
          /* "The GPIODR8R register is the 8-mA drive control register. It
           * allows for each GPIO signal in the port to be individually configured
           * without affecting the other pads. When writing the DRV8 bit for a GPIO
           * signal, the corresponding DRV2 bit in the GPIO DR2R register and the
           * DRV4 bit in the GPIO DR4R register are automatically cleared by hardware."
           */

          regoffset = LM3S_GPIO_DR8R_OFFSET;
        }
        break;
    }

  /* Set the selected pad strength and set/clear optional slew rate control */

  regval = getreg32(base + regoffset);
  regval |= pin;
  putreg32(regval, base + regoffset);

  regval  = getreg32(base + LM3S_GPIO_SLR_OFFSET);
  regval &= slrclr;
  regval |= slrset;
  putreg32(regval, base + LM3S_GPIO_SLR_OFFSET);
}

/****************************************************************************
 * Name: lm_gpiopadtype
 *
 * Description:
 *   Set up pad strength and pull-ups.  Some of these values may be over-
 *   written by lm_gpiofunc, depending on the function selection.  Others
 *   are optional for different function selections.
 *
 ****************************************************************************/

static inline void lm_gpiopadtype(uint32_t base, uint32_t pin, uint32_t cfgset)
{
  int padtype  = (cfgset & GPIO_PADTYPE_MASK) >> GPIO_PADTYPE_SHIFT;
#if 0 /* always overwritten by lm_gpiofunc */
  uint32_t odrset;
  uint32_t odrclr;
#endif
  uint32_t purset;
  uint32_t purclr;
  uint32_t pdrset;
  uint32_t pdrclr;
#if 0 /* always overwritten by lm_gpiofunc */
  uint32_t denset;
  uint32_t denclr;
#endif
  uint32_t regval;

  /* Assume digital GPIO function, push-pull with no pull-up or pull-down */

#if 0 /* always overwritten by lm_gpiofunc */
  odrset = 0;
  odrclr = pin;
#endif
  purset = 0;
  purclr = pin;
  pdrset = 0;
  pdrclr = pin;
#if 0 /* always overwritten by lm_gpiofunc */
  denset = pin;
  denclr = 0;
#endif

  switch (padtype)
    {
      case 0: /* Push-pull */
      default:
        {
        }
        break;

      case 1: /* Push-pull with weak pull-up */
        {
          purset = pin;
          purclr = 0;
        }
        break;
      case 2: /* Push-pull with weak pull-down */
        {
          pdrset = pin;
          pdrclr = 0;
        }
        break;
      case 3: /* Open-drain */
        {
#if 0 /* always overwritten by lm_gpiofunc */
          odrset = pin;
          odrclr = 0;
#endif
        }
        break;
      case 4: /* Open-drain with weak pull-up */
        {
#if 0 /* always overwritten by lm_gpiofunc */
          odrset = pin;
          odrclr = 0;
#endif
          purset = pin;
          purclr = 0;
        }
        break;
      case 5: /* Open-drain with weak pull-down */
        {
#if 0 /* always overwritten by lm_gpiofunc */
          odrset = pin;
          odrclr = 0;
#endif
          pdrset = pin;
          pdrclr = 0;
        }
        break;
      case 6: /* Analog comparator */
        {
#if 0 /* always overwritten by lm_gpiofunc */
          denset = 0;
          denclr = pin;
#endif
        }
        break;
    }

  /* Set/clear the GPIO ODR bit. "The GPIO ODR register is the open drain
   * control register. Setting a bit in this register enables the open drain
   * configuration of the corresponding GPIO pad. When open drain mode is enabled,
   * the corresponding bit should also be set in the GPIO Digital Input Enable
   * (GPIO DEN) register ... Corresponding bits in the drive strength registers
   * (GPIO DR2R, GPIO DR4R, GPIO DR8R, and GPIO SLR ) can be set to achieve the
   * desired rise and fall times. The GPIO acts as an open drain input if the
   * corresponding bit in the GPIO DIR register is set to 0; and as an open
   * drain output when set to 1."
   */

#if 0 /* always overwritten by lm_gpiofunc */
  regval = getreg32(base + LM3S_GPIO_ODR_OFFSET);
  regval &= ~odrclr;
  regval |= odrset;
  putreg32(regval, base + LM3S_GPIO_ODR_OFFSET);
#endif

  /* Set/clear the GPIO PUR bit. "The GPIOPUR register is the pull-up control
   * register. When a bit is set to 1, it enables a weak pull-up resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPUR automatically clears the
   * corresponding bit in the GPIO Pull-Down Select (GPIOPDR) register ..."
   */

  regval = getreg32(base + LM3S_GPIO_PUR_OFFSET);
  regval &= ~purclr;
  regval |= purset;
  putreg32(regval, base + LM3S_GPIO_PUR_OFFSET);

  /* Set/clear the GPIO PDR bit. "The GPIOPDR register is the pull-down control
   * register. When a bit is set to 1, it enables a weak pull-down resistor on the
   * corresponding GPIO signal. Setting a bit in GPIOPDR automatically clears
   * the corresponding bit in the GPIO Pull-Up Select (GPIOPUR) register ..."
   */

  regval = getreg32(base + LM3S_GPIO_PDR_OFFSET);
  regval &= ~pdrclr;
  regval |= pdrset;
  putreg32(regval, base + LM3S_GPIO_PDR_OFFSET);

  /* Set/clear the GPIO DEN bit. "The GPIODEN register is the digital enable
   * register. By default, with the exception of the GPIO signals used for JTAG/SWD
   * function, all other GPIO signals are configured out of reset to be undriven
   * (tristate). Their digital function is disabled; they do not drive a logic
   * value on the pin and they do not allow the pin voltage into the GPIO receiver.
   * To use the pin in a digital function (either GPIO or alternate function), the
   * corresponding GPIODEN bit must be set."
   */

#if 0 /* always overwritten by lm_gpiofunc */
  regval = getreg32(base + LM3S_GPIO_DEN_OFFSET);
  regval &= ~denclr;
  regval |= denset;
  putreg32(regval, base + LM3S_GPIO_DEN_OFFSET);
#endif
}

/****************************************************************************
 * Name: lm_initoutput
 *
 * Description:
 *   Set the GPIO output value
 *
 ****************************************************************************/

static inline void lm_initoutput(uint32_t cfgset)
{
  bool value = ((cfgset & GPIO_VALUE_MASK) != GPIO_VALUE_ZERO);
  lm_gpiowrite(cfgset, value);
}

/****************************************************************************
 * Name: lm_interrupt
 *
 * Description:
 *   Configure the interrupt pin.
 *
 ****************************************************************************/

static inline void lm_interrupt(uint32_t base, uint32_t pin, uint32_t cfgset)
{
  int inttype = (cfgset & GPIO_INT_MASK) >> GPIO_INT_SHIFT;
  uint32_t regval;
  uint32_t isset;
  uint32_t isclr;
  uint32_t ibeset;
  uint32_t ibeclr;
  uint32_t iveset;
  uint32_t iveclr;

  /* Mask and clear the GPIO interrupt
   *
   * "The GPIOIM register is the interrupt mask register. Bits set to High in
   * GPIO IM allow the corresponding pins to trigger their individual interrupts
   * and the combined GPIO INTR line. Clearing a bit disables interrupt triggering
   * on that pin. All bits are cleared by a reset."
   */

  regval  = getreg32(base + LM3S_GPIO_IM_OFFSET);
  regval &= ~pin;
  putreg32(regval, base + LM3S_GPIO_IM_OFFSET);

  /* "The GPIOICR register is the interrupt clear register. Writing a 1 to a bit
   * in this register clears the corresponding interrupt edge detection logic
   * register. Writing a 0 has no effect."
   */

  regval  = getreg32(base + LM3S_GPIO_ICR_OFFSET);
  regval |= pin;
  putreg32(regval, base + LM3S_GPIO_ICR_OFFSET);

  /* Assume rising edge */

  isset  = 0;            /* Not level sensed */
  isclr  = pin;
  ibeset = 0;            /* Single edge */
  ibeclr = pin;
  iveset = pin;          /* Rising edge or high levels*/
  iveclr = 0;

  /* Then handle according to the selected interrupt type */

  switch (inttype)
    {
      case 0:            /* Interrupt on falling edge */
        {
          iveset = 0;    /* Falling edge or low levels*/
          iveclr = pin;
        }
        break;

      case 1:           /* Interrupt on rising edge */
      default:
        break;

      case 2:           /* Interrupt on both edges */
        {
          ibeset = pin; /* Both edges */
          ibeclr = 0;
        }
        break;

      case 3:           /* Interrupt on low level */
        {
          isset = pin;  /* Level sensed */
          isclr = 0;
          iveset = 0;   /* Falling edge or low levels*/
          iveclr = pin;
        }
        break;

      case 4:           /* Interrupt on high level */
        {
          isset = pin;  /* Level sensed */
          isclr = 0;
        }
        break;
    }

  /* "The GPIO IS register is the interrupt sense register. Bits set to
   * 1 in GPIOIS configure the corresponding pins to detect levels, while
   * bits set to 0 configure the pins to detect edges. All bits are cleared
   * by a reset.
   */

  regval  = getreg32(base + LM3S_GPIO_IS_OFFSET);
  regval &= isclr;
  regval |= isset;
  putreg32(regval, base + LM3S_GPIO_IS_OFFSET);

  /* "The GPIO IBE register is the interrupt both-edges register. When the
   * corresponding bit in the GPIO Interrupt Sense (GPIO IS) register ... is
   * set to detect edges, bits set to High in GPIO IBE configure the
   * corresponding pin to detect both rising and falling edges, regardless
   * of the corresponding bit in the GPIO Interrupt Event (GPIO IEV) register ...
   * Clearing a bit configures the pin to be controlled by GPIOIEV. All bits
   * are cleared by a reset.
   */

  regval  = getreg32(base + LM3S_GPIO_IBE_OFFSET);
  regval &= ibeclr;
  regval |= ibeset;
  putreg32(regval, base + LM3S_GPIO_IBE_OFFSET);

  /* "The GPIOIEV register is the interrupt event register. Bits set to
   * High in GPIO IEV configure the corresponding pin to detect rising edges
   * or high levels, depending on the corresponding bit value in the GPIO
   * Interrupt Sense (GPIO IS) register... Clearing a bit configures the pin to
   * detect falling edges or low levels, depending on the corresponding bit
   * value in GPIOIS. All bits are cleared by a reset.
   */

  regval  = getreg32(base + LM3S_GPIO_IEV_OFFSET);
  regval &= iveclr;
  regval |= iveset;
  putreg32(regval, base + LM3S_GPIO_IEV_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int lm_configgpio(uint32_t cfgset)
{
  irqstate_t   flags;
  unsigned int func;
  unsigned int port;
  unsigned int pinno;
  uint32_t     pin;
  uint32_t     base;
  uint32_t     regval;

  /* Decode the basics */

  func  = (cfgset & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT;
  port  = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno = (cfgset & GPIO_NUMBER_MASK);
  pin   = (1 <<pinno);

  DEBUGASSERT(func <= GPIO_FUNC_MAX);

  /* Get the base address associated with the GPIO port */

  base = lm_gpiobaseaddress(port);
  DEBUGASSERT(base != 0);

  /* The following requires exclusive access to the GPIO registers */

  flags = irqsave();

  /* Enable clocking for this GPIO peripheral. "To use the GPIO, the peripheral
   * clock must be enabled by setting the appropriate GPIO Port bit field (GPIOn)
   * in the RCGC2 register."
   */

  regval = getreg32(LM3S_SYSCON_RCGC2);
  regval |= SYSCON_RCGC2_GPIO(port);
  putreg32(regval, LM3S_SYSCON_RCGC2);

  /* First, set the port to digital input.  This is the safest state in which
   * to perform reconfiguration.
   */

  lm_gpiofunc(base, pinno, &g_funcbits[0]);

  /* Then set up pad strengths and pull-ups.  These setups should be done before
   * setting up the function because some function settings will over-ride these
   * user options.
   */

  lm_gpiopadstrength(base, pin, cfgset);
  lm_gpiopadtype(base, pin, cfgset);

  /* Then set up the real pin function */

  lm_gpiofunc(base, pinno, &g_funcbits[func]);

  /* Special GPIO digital output pins */

  if (func == 1 || func == 3)
    {
      lm_initoutput(cfgset);
    }


  /* Special setup for interrupt GPIO pins */

  else if (func == 7)
    {
      lm_interrupt(base, pin, cfgset);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: lm_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lm_gpiowrite(uint32_t pinset, bool value)
{
  unsigned int port;
  unsigned int pinno;
  uint32_t     base;

  /* Decode the basics */

  port  = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno = (pinset & GPIO_NUMBER_MASK);

  /* Get the base address associated with the GPIO port */

  base  = lm_gpiobaseaddress(port);

  /* "The GPIO DATA register is the data register. In software control mode,
   *  values written in the GPIO DATA register are transferred onto the GPIO
   *  port pins if the respective pins have been configured as outputs through
   *  the GPIO Direction (GPIO DIR) register ...
   *
   * "In order to write to GPIO DATA, the corresponding bits in the mask,
   *  resulting from the address bus bits [9:2], must be High. Otherwise, the
   *  bit values remain unchanged by the write. 
   *
   * "... All bits are cleared by a reset."
   */

  putreg32((uint32_t)value << pinno, base + LM3S_GPIO_DATA_OFFSET + (1 << (pinno + 2)));
}

/****************************************************************************
 * Name: lm_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool lm_gpioread(uint32_t pinset, bool value)
{
  unsigned int port;
  unsigned int pinno;
  uint32_t     base;

  /* Decode the basics */

  port  = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pinno = (pinset & GPIO_NUMBER_MASK);

  /* Get the base address associated with the GPIO port */

  base = lm_gpiobaseaddress(port);

  /* "... the values read from this register are determined for each bit
   *  by the mask bit derived from the address used to access the data register,
   *  bits [9:2]. Bits that are 1 in the address mask cause the corresponding
   *  bits in GPIODATA to be read, and bits that are 0 in the address mask cause
   *  the corresponding bits in GPIO DATA to be read as 0, regardless of their
   *  value.
   *
   * "A read from GPIO DATA returns the last bit value written if the respective
   *  pins are configured as outputs, or it returns the value on the
   *  corresponding input pin when these are configured as inputs. All bits
   *  are cleared by a reset."
   */

  return (getreg32(base + LM3S_GPIO_DATA_OFFSET + (1 << (pinno + 2))) != 0);
}

