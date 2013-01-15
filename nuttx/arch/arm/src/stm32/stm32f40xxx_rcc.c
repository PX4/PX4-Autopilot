/****************************************************************************
 * arch/arm/src/stm32/stm32f40xxx_rcc.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include "stm32_pwr.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_reset
 *
 * Description:
 *   Reset the RCC clock configuration to the default reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  /* Enable the Internal High Speed clock (HSI) */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x00000000, STM32_RCC_CFGR);

  /* Reset HSEON, CSSON and PLLON bits */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSEON|RCC_CR_CSSON|RCC_CR_PLLON);
  putreg32(regval, STM32_RCC_CR);
 
  /* Reset PLLCFGR register to reset default */

  putreg32(RCC_PLLCFG_RESET, STM32_RCC_PLLCFG);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32_RCC_CIR);
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB1ENR register to enabled the
   * selected AHB1 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB1ENR);

  /* Enable GPIOA, GPIOB, .... GPIOI*/

#if STM32_NGPIO > 0
  regval |= (RCC_AHB1ENR_GPIOAEN
#if STM32_NGPIO > 16
             |RCC_AHB1ENR_GPIOBEN
#endif
#if STM32_NGPIO > 32
             |RCC_AHB1ENR_GPIOCEN
#endif
#if STM32_NGPIO > 48
             |RCC_AHB1ENR_GPIODEN
#endif
#if STM32_NGPIO > 64
             |RCC_AHB1ENR_GPIOEEN
#endif
#if STM32_NGPIO > 80
             |RCC_AHB1ENR_GPIOFEN
#endif
#if STM32_NGPIO > 96
             |RCC_AHB1ENR_GPIOGEN
#endif
#if STM32_NGPIO > 112
             |RCC_AHB1ENR_GPIOHEN
#endif
#if STM32_NGPIO > 128
             |RCC_AHB1ENR_GPIOIEN
#endif
             );
#endif

#ifdef CONFIG_STM32_CRC
  /* CRC clock enable */

  regval |= RCC_AHB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32_BKPSRAM
  /* Backup SRAM clock enable */

  regval |= RCC_AHB1ENR_BKPSRAMEN;
#endif

#ifdef CONFIG_STM32_CCMDATARAM
  /* CCM data RAM clock enable */

  regval |= RCC_AHB1ENR_CCMDATARAMEN;
#endif

#ifdef CONFIG_STM32_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32_ETHMAC
  /* Ethernet MAC clocking */

  regval |= (RCC_AHB1ENR_ETHMACEN|RCC_AHB1ENR_ETHMACTXEN|RCC_AHB1ENR_ETHMACRXEN);

#ifdef CONFIG_STM32_ETH_PTP
  /* Precision Time Protocol (PTP) */

  regval |= RCC_AHB1ENR_ETHMACPTPEN;

#endif
#endif

#ifdef CONFIG_STM32_OTGHS
  /* USB OTG HS */

  regval |= (RCC_AHB1ENR_OTGHSEN|RCC_AHB1ENR_OTGHSULPIEN);
#endif

  putreg32(regval, STM32_RCC_AHB1ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB2ENR register to enabled the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB2ENR);

#ifdef CONFIG_STM32_DCMI
  /* Camera interface enable */

  regval |= RCC_AHB2ENR_DCMIEN;
#endif

#ifdef CONFIG_STM32_CRYP
  /* Cryptographic modules clock enable */

  regval |= RCC_AHB2ENR_CRYPEN;
#endif

#ifdef CONFIG_STM32_HASH
  /* Hash modules clock enable */

  regval |= RCC_AHB2ENR_HASHEN;
#endif

#ifdef CONFIG_STM32_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHB2ENR_RNGEN;
#endif

#ifdef CONFIG_STM32_OTGFS
  /* USB OTG FS clock enable */

  regval |= RCC_AHB2ENR_OTGFSEN;
#endif

  putreg32(regval, STM32_RCC_AHB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb3(void)
{
#ifdef CONFIG_STM32_FSMC
  uint32_t regval;

  /* Set the appropriate bits in the AHB3ENR register to enabled the
   * selected AHB3 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB3ENR);

  /* Flexible static memory controller module clock enable */

  regval |= RCC_AHB3ENR_FSMCEN;

  putreg32(regval, STM32_RCC_AHB3ENR);   /* Enable peripherals */
#endif
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1ENR register to enabled the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1ENR);

#ifdef CONFIG_STM32_TIM2
  /* TIM2 clock enable */

  regval |= RCC_APB1ENR_TIM2EN;
#endif

#ifdef CONFIG_STM32_TIM3
  /* TIM3 clock enable */

  regval |= RCC_APB1ENR_TIM3EN;
#endif

#ifdef CONFIG_STM32_TIM4
  /* TIM4 clock enable */

  regval |= RCC_APB1ENR_TIM4EN;
#endif

#ifdef CONFIG_STM32_TIM5
  /* TIM5 clock enable */

  regval |= RCC_APB1ENR_TIM5EN;
#endif

#ifdef CONFIG_STM32_TIM6
  /* TIM6 clock enable */

  regval |= RCC_APB1ENR_TIM6EN;
#endif

#ifdef CONFIG_STM32_TIM7
  /* TIM7 clock enable */

  regval |= RCC_APB1ENR_TIM7EN;
#endif

#ifdef CONFIG_STM32_TIM12
  /* TIM12 clock enable */

  regval |= RCC_APB1ENR_TIM12EN;
#endif

#ifdef CONFIG_STM32_TIM13
  /* TIM13 clock enable */

  regval |= RCC_APB1ENR_TIM13EN;
#endif

#ifdef CONFIG_STM32_TIM14
  /* TIM14 clock enable */

  regval |= RCC_APB1ENR_TIM14EN;
#endif

#ifdef CONFIG_STM32_WWDG
  /* Window watchdog clock enable */

  regval |= RCC_APB1ENR_WWDGEN;
#endif

#ifdef CONFIG_STM32_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1ENR_SPI2EN;
#endif

#ifdef CONFIG_STM32_SPI3
  /* SPI3 clock enable */

  regval |= RCC_APB1ENR_SPI3EN;
#endif

#ifdef CONFIG_STM32_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR_USART2EN;
#endif

#ifdef CONFIG_STM32_USART3
  /* USART3 clock enable */

  regval |= RCC_APB1ENR_USART3EN;
#endif

#ifdef CONFIG_STM32_UART4
  /* UART4 clock enable */

  regval |= RCC_APB1ENR_UART4EN;
#endif

#ifdef CONFIG_STM32_UART5
  /* UART5 clock enable */

  regval |= RCC_APB1ENR_UART5EN;
#endif

#ifdef CONFIG_STM32_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1ENR_I2C1EN;
#endif

#ifdef CONFIG_STM32_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1ENR_I2C2EN;
#endif

#ifdef CONFIG_STM32_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1ENR_I2C3EN;
#endif

#ifdef CONFIG_STM32_CAN1
  /* CAN 1 clock enable */

  regval |= RCC_APB1ENR_CAN1EN;
#endif

#ifdef CONFIG_STM32_CAN2
  /* CAN2 clock enable.  NOTE: CAN2 needs CAN1 clock as well. */

  regval |= (RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN);
#endif

  /* Power interface clock enable.  The PWR block is always enabled so that
   * we can set the internal voltage regulator for maximum performance.
   */

  regval |= RCC_APB1ENR_PWREN;

#if defined (CONFIG_STM32_DAC1) || defined(CONFIG_STM32_DAC2)
  /* DAC interface clock enable */

  regval |= RCC_APB1ENR_DACEN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB2 peripherals.
   */

  regval = getreg32(STM32_RCC_APB2ENR);

#ifdef CONFIG_STM32_TIM1
  /* TIM1 clock enable */

  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32_TIM8
  /* TIM8 clock enable */

  regval |= RCC_APB2ENR_TIM8EN;
#endif

#ifdef CONFIG_STM32_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32_USART6
  /* USART6 clock enable */

  regval |= RCC_APB2ENR_USART6EN;
#endif

#ifdef CONFIG_STM32_ADC1
  /* ADC1 clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
#endif

#ifdef CONFIG_STM32_ADC2
  /* ADC2 clock enable */

  regval |= RCC_APB2ENR_ADC2EN;
#endif

#ifdef CONFIG_STM32_ADC3
  /* ADC3 clock enable */

  regval |= RCC_APB2ENR_ADC3EN;
#endif

#ifdef CONFIG_STM32_SDIO
  /* SDIO clock enable */

  regval |= RCC_APB2ENR_SDIOEN;
#endif

#ifdef CONFIG_STM32_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32_SYSCFG
  /* System configuration controller clock enable */

  regval |= RCC_APB2ENR_SYSCFGEN;
#endif

#ifdef CONFIG_STM32_TIM9
  /* TIM9 clock enable */

  regval |= RCC_APB2ENR_TIM9EN;
#endif

#ifdef CONFIG_STM32_TIM10
  /* TIM10 clock enable */

  regval |= RCC_APB2ENR_TIM10EN;
#endif

#ifdef CONFIG_STM32_TIM11
  /* TIM11 clock enable */

  regval |= RCC_APB2ENR_TIM11EN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 * 
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /* Enable External High-Speed Clock (HSE) */
 
  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
  {
    /* Check if the HSERDY flag is the set in the CR */

    if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
      {
        /* If so, then break-out with timeout > 0 */

        break;
      }
  }

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Select regulator voltage output Scale 1 mode to support system
       * frequencies up to 168 MHz.
       */

      regval  = getreg32(STM32_RCC_APB1ENR);
      regval |= RCC_APB1ENR_PWREN;
      putreg32(regval, STM32_RCC_APB1ENR);

      regval  = getreg32(STM32_PWR_CR);
      regval |= PWR_CR_VOS;
      putreg32(regval, STM32_PWR_CR);

      /* Set the HCLK source/divider */
 
      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_HPRE_MASK;
      regval |= STM32_RCC_CFGR_HPRE;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PCLK2 divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE2_MASK;
      regval |= STM32_RCC_CFGR_PPRE2;
      putreg32(regval, STM32_RCC_CFGR);
  
      /* Set the PCLK1 divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE1_MASK;
      regval |= STM32_RCC_CFGR_PPRE1;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PLL dividers and multiplers to configure the main PLL */

      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN |STM32_PLLCFG_PLLP |
                RCC_PLLCFG_PLLSRC_HSE | STM32_PLLCFG_PLLQ);
      putreg32(regval, STM32_RCC_PLLCFG);

      /* Enable the main PLL */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32_RCC_CR);
 
      /* Wait until the PLL is ready */
  
      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);
 
      /* Enable FLASH prefetch, instruction cache, data cache, and 5 wait states */

#ifdef STM32_FLASH_PREFETCH
      regval = (FLASH_ACR_LATENCY_5 | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
#else
      regval = (FLASH_ACR_LATENCY_5 | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
#endif
      putreg32(regval, STM32_FLASH_ACR);

      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */
  
      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL);
    }
}
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableapb1();
  rcc_enableapb2();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
