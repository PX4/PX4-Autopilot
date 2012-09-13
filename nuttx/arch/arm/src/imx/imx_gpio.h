/************************************************************************************
 * arch/arm/src/imx/imx_gpio.h
 * arch/arm/src/chip/imx_gpio.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_IMX_GPIO_H
#define __ARCH_ARM_IMX_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif
#include "up_arch.h"                     /* getreg32(), putreg32() */
 
/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPIO Register Offsets ************************************************************/

#define GPIO_DDIR_OFFSET         0x0000  /* Data Direction Register */
#define GPIO_OCR1_OFFSET         0x0004  /* Output Configuration Register 1 */
#define GPIO_OCR2_OFFSET         0x0008  /* Output Configuration Register 2 */
#define GPIO_ICONFA1_OFFSET      0x000c  /* Input Configuration Register A1 */
#define GPIO_ICONFA2_OFFSET      0x0010  /* Input Configuration Register A2 */
#define GPIO_ICONFB1_OFFSET      0x0014  /* Input Configuration Register B1 */
#define GPIO_ICONFB2_OFFSET      0x0018  /* Input Configuration Register B2 */
#define GPIO_DR_OFFSET           0x001c  /* Data Register */
#define GPIO_GIUS_OFFSET         0x0020  /* GPIO In Use Register */
#define GPIO_SSR_OFFSET          0x0024  /* Sample Status Register */
#define GPIO_ICR1_OFFSET         0x0028  /* Interrupt Configuration Register 1 */
#define GPIO_ICR2_OFFSET         0x002c  /* Interrupt Configuration Register 2 */
#define GPIO_IMR_OFFSET          0x0030  /* Interrupt Mask Register */
#define GPIO_ISR_OFFSET          0x0034  /* Interrupt Status Register */
#define GPIO_GPR_OFFSET          0x0038  /* General Purpose Register */
#define GPIO_SWR_OFFSET          0x003c  /* Software Reset Register */
#define GPIO_PUEN_OFFSET         0x0040  /* Pull_Up Enable Register */

#define GPIO_PTA_OFFSET          0x0000  /* Port A offset */
#define GPIO_PTB_OFFSET          0x0100  /* Port B offset */
#define GPIO_PTC_OFFSET          0x0200  /* Port C offset */
#define GPIO_PTD_OFFSET          0x0300  /* Port D offset */

#define GPIOA                    0       /* Port A index */
#define GPIOB                    1       /* Port B index */
#define GPIOC                    2       /* Port C index */
#define GPIOD                    3       /* Port D index */
#define GPIO_PT_OFFSET(n)        (GPIO_PTA_OFFSET + (n)*0x0100)

/* GPIO Register Addresses **********************************************************/

#define IMX_PTA_VBASE            (IMX_GPIO_VBASE + GPIO_PTA_OFFSET)
#define IMX_PTB_VBASE            (IMX_GPIO_VBASE + GPIO_PTB_OFFSET)
#define IMX_PTC_VBASE            (IMX_GPIO_VBASE + GPIO_PTC_OFFSET)
#define IMX_PTD_VBASE            (IMX_GPIO_VBASE + GPIO_PTD_OFFSET)
#define IMX_PT_VBASE(n)          (IMX_GPIO_VBASE + GPIO_PT_OFFSET(n))

#define IMX_GPIOA_DDIR           (IMX_PTA_VBASE + GPIO_DDIR_OFFSET)
#define IMX_GPIOA_OCR1           (IMX_PTA_VBASE + GPIO_OCR1_OFFSET)
#define IMX_GPIOA_OCR2           (IMX_PTA_VBASE + GPIO_OCR2_OFFSET)
#define IMX_GPIOA_ICONFA1        (IMX_PTA_VBASE + GPIO_ICONFA1_OFFSET)
#define IMX_GPIOA_ICONFA2        (IMX_PTA_VBASE + GPIO_ICONFA2_OFFSET)
#define IMX_GPIOA_ICONFB1        (IMX_PTA_VBASE + GPIO_ICONFB1_OFFSET)
#define IMX_GPIOA_ICONFB2        (IMX_PTA_VBASE + GPIO_ICONFB2_OFFSET)
#define IMX_GPIOA_DR             (IMX_PTA_VBASE + GPIO_DR_OFFSET)
#define IMX_GPIOA_GIUS           (IMX_PTA_VBASE + GPIO_GIUS_OFFSET)
#define IMX_GPIOA_SSR            (IMX_PTA_VBASE + GPIO_SSR_OFFSET)
#define IMX_GPIOA_ICR1           (IMX_PTA_VBASE + GPIO_ICR1_OFFSET)
#define IMX_GPIOA_ICR2           (IMX_PTA_VBASE + GPIO_ICR2_OFFSET)
#define IMX_GPIOA_IMR            (IMX_PTA_VBASE + GPIO_IMR_OFFSET)
#define IMX_GPIOA_ISR            (IMX_PTA_VBASE + GPIO_ISR_OFFSET)
#define IMX_GPIOA_GPR            (IMX_PTA_VBASE + GPIO_GPR_OFFSET)
#define IMX_GPIOA_SWR            (IMX_PTA_VBASE + GPIO_SWR_OFFSET)
#define IMX_GPIOA_PUEN           (IMX_PTA_VBASE + GPIO_PUEN_OFFSET)

#define IMX_GPIOB_DDIR           (IMX_PTB_VBASE + GPIO_DDIR_OFFSET)
#define IMX_GPIOB_OCR1           (IMX_PTB_VBASE + GPIO_OCR1_OFFSET)
#define IMX_GPIOB_OCR2           (IMX_PTB_VBASE + GPIO_OCR2_OFFSET)
#define IMX_GPIOB_ICONFA1        (IMX_PTB_VBASE + GPIO_ICONFA1_OFFSET)
#define IMX_GPIOB_ICONFA2        (IMX_PTB_VBASE + GPIO_ICONFA2_OFFSET)
#define IMX_GPIOB_ICONFB1        (IMX_PTB_VBASE + GPIO_ICONFB1_OFFSET)
#define IMX_GPIOB_ICONFB2        (IMX_PTB_VBASE + GPIO_ICONFB2_OFFSET)
#define IMX_GPIOB_DR             (IMX_PTB_VBASE + GPIO_DR_OFFSET)
#define IMX_GPIOB_GIUS           (IMX_PTB_VBASE + GPIO_GIUS_OFFSET)
#define IMX_GPIOB_SSR            (IMX_PTB_VBASE + GPIO_SSR_OFFSET)
#define IMX_GPIOB_ICR1           (IMX_PTB_VBASE + GPIO_ICR1_OFFSET)
#define IMX_GPIOB_ICR2           (IMX_PTB_VBASE + GPIO_ICR2_OFFSET)
#define IMX_GPIOB_IMR            (IMX_PTB_VBASE + GPIO_IMR_OFFSET)
#define IMX_GPIOB_ISR            (IMX_PTB_VBASE + GPIO_ISR_OFFSET)
#define IMX_GPIOB_GPR            (IMX_PTB_VBASE + GPIO_GPR_OFFSET)
#define IMX_GPIOB_SWR            (IMX_PTB_VBASE + GPIO_SWR_OFFSET)
#define IMX_GPIOB_PUEN           (IMX_PTB_VBASE + GPIO_PUEN_OFFSET)

#define IMX_GPIOC_DDIR           (IMX_PTC_VBASE + GPIO_DDIR_OFFSET)
#define IMX_GPIOC_OCR1           (IMX_PTC_VBASE + GPIO_OCR1_OFFSET)
#define IMX_GPIOC_OCR2           (IMX_PTC_VBASE + GPIO_OCR2_OFFSET)
#define IMX_GPIOC_ICONFA1        (IMX_PTC_VBASE + GPIO_ICONFA1_OFFSET)
#define IMX_GPIOC_ICONFA2        (IMX_PTC_VBASE + GPIO_ICONFA2_OFFSET)
#define IMX_GPIOC_ICONFB1        (IMX_PTC_VBASE + GPIO_ICONFB1_OFFSET)
#define IMX_GPIOC_ICONFB2        (IMX_PTC_VBASE + GPIO_ICONFB2_OFFSET)
#define IMX_GPIOC_DR             (IMX_PTC_VBASE + GPIO_DR_OFFSET)
#define IMX_GPIOC_GIUS           (IMX_PTC_VBASE + GPIO_GIUS_OFFSET)
#define IMX_GPIOC_SSR            (IMX_PTC_VBASE + GPIO_SSR_OFFSET)
#define IMX_GPIOC_ICR1           (IMX_PTC_VBASE + GPIO_ICR1_OFFSET)
#define IMX_GPIOC_ICR2           (IMX_PTC_VBASE + GPIO_ICR2_OFFSET)
#define IMX_GPIOC_IMR            (IMX_PTC_VBASE + GPIO_IMR_OFFSET)
#define IMX_GPIOC_ISR            (IMX_PTC_VBASE + GPIO_ISR_OFFSET)
#define IMX_GPIOC_GPR            (IMX_PTC_VBASE + GPIO_GPR_OFFSET)
#define IMX_GPIOC_SWR            (IMX_PTC_VBASE + GPIO_SWR_OFFSET)
#define IMX_GPIOC_PUEN           (IMX_PTC_VBASE + GPIO_PUEN_OFFSET)

#define IMX_GPIOD_DDIR           (IMX_PTD_VBASE + GPIO_DDIR_OFFSET)
#define IMX_GPIOD_OCR1           (IMX_PTD_VBASE + GPIO_OCR1_OFFSET)
#define IMX_GPIOD_OCR2           (IMX_PTD_VBASE + GPIO_OCR2_OFFSET)
#define IMX_GPIOD_ICONFA1        (IMX_PTD_VBASE + GPIO_ICONFA1_OFFSET)
#define IMX_GPIOD_ICONFA2        (IMX_PTD_VBASE + GPIO_ICONFA2_OFFSET)
#define IMX_GPIOD_ICONFB1        (IMX_PTD_VBASE + GPIO_ICONFB1_OFFSET)
#define IMX_GPIOD_ICONFB2        (IMX_PTD_VBASE + GPIO_ICONFB2_OFFSET)
#define IMX_GPIOD_DR             (IMX_PTD_VBASE + GPIO_DR_OFFSET)
#define IMX_GPIOD_GIUS           (IMX_PTD_VBASE + GPIO_GIUS_OFFSET)
#define IMX_GPIOD_SSR            (IMX_PTD_VBASE + GPIO_SSR_OFFSET)
#define IMX_GPIOD_ICR1           (IMX_PTD_VBASE + GPIO_ICR1_OFFSET)
#define IMX_GPIOD_ICR2           (IMX_PTD_VBASE + GPIO_ICR2_OFFSET)
#define IMX_GPIOD_IMR            (IMX_PTD_VBASE + GPIO_IMR_OFFSET)
#define IMX_GPIOD_ISR            (IMX_PTD_VBASE + GPIO_ISR_OFFSET)
#define IMX_GPIOD_GPR            (IMX_PTD_VBASE + GPIO_GPR_OFFSET)
#define IMX_GPIOD_SWR            (IMX_PTD_VBASE + GPIO_SWR_OFFSET)
#define IMX_GPIOD_PUEN           (IMX_PTD_VBASE + GPIO_PUEN_OFFSET)

#define IMX_GPIO_DDIR(n)         (IMX_PT_VBASE(n) + GPIO_DDIR_OFFSET)
#define IMX_GPIO_OCR1(n)         (IMX_PT_VBASE(n) + GPIO_OCR1_OFFSET)
#define IMX_GPIO_OCR2(n)         (IMX_PT_VBASE(n) + GPIO_OCR2_OFFSET)
#define IMX_GPIO_ICONFA1(n)      (IMX_PT_VBASE(n) + GPIO_ICONFA1_OFFSET)
#define IMX_GPIO_ICONFA2(n)      (IMX_PT_VBASE(n) + GPIO_ICONFA2_OFFSET)
#define IMX_GPIO_ICONFB1(n)      (IMX_PT_VBASE(n) + GPIO_ICONFB1_OFFSET)
#define IMX_GPIO_ICONFB2(n)      (IMX_PT_VBASE(n) + GPIO_ICONFB2_OFFSET)
#define IMX_GPIO_DR(n)           (IMX_PT_VBASE(n) + GPIO_DR_OFFSET)
#define IMX_GPIO_GIUS(n)         (IMX_PT_VBASE(n) + GPIO_GIUS_OFFSET)
#define IMX_GPIO_SSR(n)          (IMX_PT_VBASE(n) + GPIO_SSR_OFFSET)
#define IMX_GPIO_ICR1(n)         (IMX_PT_VBASE(n) + GPIO_ICR1_OFFSET)
#define IMX_GPIO_ICR2(n)         (IMX_PT_VBASE(n) + GPIO_ICR2_OFFSET)
#define IMX_GPIO_IMR(n)          (IMX_PT_VBASE(n) + GPIO_IMR_OFFSET)
#define IMX_GPIO_ISR(n)          (IMX_PT_VBASE(n) + GPIO_ISR_OFFSET)
#define IMX_GPIO_GPR(n)          (IMX_PT_VBASE(n) + GPIO_GPR_OFFSET)
#define IMX_GPIO_SWR(n)          (IMX_PT_VBASE(n) + GPIO_SWR_OFFSET)
#define IMX_GPIO_PUEN(n)         (IMX_PT_VBASE(n) + GPIO_PUEN_OFFSET)

/* GPIO Register Bit Definitions ****************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_GPIO_H */

#ifndef __ASSEMBLY__

/* Handler circular include... This file includes up_arch.h, but this file is
 * included by up_arch.h (via chip.h) BEFORE getreg32 is defined.
 */

#if !defined(__ARCH_ARM_IMX_GPIOHELPERS_H) && defined(getreg32)
#define __ARCH_ARM_IMX_GPIOHELPERS_H

/* Select whether the pin is an input or output */

static inline void imxgpio_dirout(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_DDIR(port));
  regval |= (1 << bit);
  putreg32(regval, IMX_GPIO_DDIR(port));
}

static inline void imxgpio_dirin(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_DDIR(port));
  regval &= ~(1 << bit);
  putreg32(regval, IMX_GPIO_DDIR(port));
}

/* Select input configuration */

static inline void imxgpio_ocrain(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_OCR1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_OCR2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_ocrbin(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_OCR1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_OCR2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= (1 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_ocrcin(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_OCR1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_OCR2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= (2 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_ocrodrin(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_OCR1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_OCR2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval |= (3 << shift);
  putreg32(regval, regaddr);
}

/* Input configuration */

static inline void imxgpio_aoutgpio(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFA1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFA2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_aoutisr(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFA1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFA2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= (1 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_aout0(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFA1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFA2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= (2 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_aout1(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFA1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFA2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval |= (3 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_boutgpio(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFB1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFB2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_boutisr(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFB1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFB2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= (1 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_bout0(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFB1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFB2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval &= ~(3 << shift);
  regval |= (2 << shift);
  putreg32(regval, regaddr);
}

static inline void imxgpio_bout1(int port, int bit)
{
  uint32_t regval;
  uint32_t regaddr;
  int      shift;

  if (bit < 16)
    {
      regaddr = IMX_GPIO_ICONFB1(port);
      shift   = (bit << 1);
    }
  else
    {
      regaddr = IMX_GPIO_ICONFB2(port);
      shift   = ((bit - 16) << 1);
    }

  regval  = getreg32(regaddr);
  regval |= (3 << shift);
  putreg32(regval, regaddr);
}

/* Select whether the pin is used for its GPIO function or for
 * its peripheral function.  Also select the primary or alternate
 * peripheral function.
 */

static inline void imxgpio_gpiofunc(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_GIUS(port));
  regval |= (1 << bit);
  putreg32(regval, IMX_GPIO_GIUS(port));
}

static inline void imxgpio_peripheralfunc(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_GIUS(port));
  regval &= ~(1 << bit);
  putreg32(regval, IMX_GPIO_GIUS(port));
}

static inline void imxgpio_altperipheralfunc(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_GPR(port));
  regval |= (1 << bit);
  putreg32(regval, IMX_GPIO_GPR(port));
}

static inline void imxgpio_primaryperipheralfunc(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_GPR(port));
  regval &= ~(1 << bit);
  putreg32(regval, IMX_GPIO_GPR(port));
}

/* Enable/disable pullups */

static inline void imxgpio_pullupenable(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_PUEN(port));
  regval |= (1 << bit);
  putreg32(regval, IMX_GPIO_PUEN(port));
}

static inline void imxgpio_pullupdisable(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_PUEN(port));
  regval &= ~(1 << bit);
  putreg32(regval, IMX_GPIO_PUEN(port));
}

static inline void imxgpio_setoutput(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_DR(port));
  regval |= (1 << bit);
  putreg32(regval, IMX_GPIO_DR(port));
}

static inline void imxgpio_clroutput(int port, int bit)
{
  uint32_t regval = getreg32(IMX_GPIO_DR(port));
  regval &= ~(1 << bit);
  putreg32(regval, IMX_GPIO_DR(port));
}

/* Useful functions for normal configurations */

extern void imxgpio_configoutput(int port, int bit, int value);
extern void imxgpio_configinput(int port, int bit);

extern void imxgpio_configpfoutput(int port, int bit);
extern void imxgpio_configpfinput(int port, int bit);

#endif

#endif  /* __ARCH_ARM_IMX_GPIOHELPERS_H */
