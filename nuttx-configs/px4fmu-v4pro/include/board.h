/************************************************************************************
 * configs/px4fmu-v4pro/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Kevin Lopez Alvarez <kevin@drotek.com>
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

#ifndef __NUTTX_CONFIG_PX4FMUV4_PRO_INCLUDE_BOARD_H
#define __NUTTX_CONFIG_PX4FMUV4_PRO_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif


#include <stm32.h>



/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The PX4FMUV4-PRO uses a 24MHz crystal connected to the HSE.
 *
 * This is the "standard" configuration as set up by arch/arm/src/stm32f40xx_rcc.c:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 24000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 12           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 180          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 2            (STM32_PLLCFG_PPQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Use PLLSA1M
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 24MHz
 * LSE - not installed
 */

#define STM32_BOARD_XTAL        24000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (24,000,000 / 12) * 180
 *         = 369,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 360,000,000 / 2 = 180,000,000
 * USB OTG FS, SDIO and RNG Clock is from PLL SAIP
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(12)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(180)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(2)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

/* Configure factors for  PLLSAI clock */

#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(96)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(2)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR_PLLI2SDIVQ    RCC_DCKCFGR_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR_PLLSAIDIVQ    RCC_DCKCFGR_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR_PLLSAIDIVR    RCC_DCKCFGR_PLLSAIDIVR_DIV2
#define STM32_RCC_DCKCFGR_SAI1ASRC      RCC_DCKCFGR_SAI1ASRC_PLLSAI
#define STM32_RCC_DCKCFGR_SAI1BSRC      RCC_DCKCFGR_SAI1BSRC_PLLI2S
#define STM32_RCC_DCKCFGR_TIMPRE        0
#define STM32_RCC_DCKCFGR_48MSEL        RCC_DCKCFGR_48MSEL_PLLSAI
#define STM32_RCC_DCKCFGR_SDMMCSEL      RCC_DCKCFGR_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR_DSISEL        RCC_DCKCFGR_DSISEL_DSIPHY

/* Configure factors for  PLLI2S clock */

#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(96)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(4)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)


#define STM32_SYSCLK_FREQUENCY  180000000ul

/* AHB clock (HCLK) is SYSCLK (180MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (45MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (90MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8-11 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY    STM32_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32_APB1_TIM5_CLKIN
#define BOARD_TIM6_FREQUENCY    STM32_APB1_TIM6_CLKIN
#define BOARD_TIM7_FREQUENCY    STM32_APB1_TIM7_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32_APB2_TIM8_CLKIN
#define BOARD_TIM9_FREQUENCY    STM32_APB2_TIM9_CLKIN
#define BOARD_TIM10_FREQUENCY   STM32_APB2_TIM10_CLKIN
#define BOARD_TIM11_FREQUENCY   STM32_APB2_TIM11_CLKIN
#define BOARD_TIM12_FREQUENCY   STM32_APB1_TIM12_CLKIN
#define BOARD_TIM13_FREQUENCY   STM32_APB1_TIM13_CLKIN
#define BOARD_TIM14_FREQUENCY   STM32_APB1_TIM14_CLKIN

/* SDMMC dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK =48MHz, SDMMC_CK=SDIOCLK/(118+2)=400 KHz
 */

/* Use the Falling edge of the SDIO_CLK clock to change the edge the
 * data and commands are change on
 */

#define SDIO_CLKCR_EDGE SDIO_CLKCR_NEGEDGE

#define SDIO_INIT_CLKDIV         (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(2+2)=12 MHz
 */
//TODO #warning "Check Freq for 24mHz"

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV      (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV      (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA Channel/Stream Selections *****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3 <- may later be used by SPI DMA
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1


/* FLASH wait states
 *
 *  --------- ---------- ----------- ---------
 *  VDD       MAX SYSCLK WAIT STATES OVERDRIVE
 *  --------- ---------- ----------- ---------
 *  1.7-2.1 V   168 MHz    8          OFF
 *  2.1-2.4 V   180 MHz    8          ON
 *  2.4-2.7 V   180 MHz    7          ON
 *  2.7-3.6 V   180 MHz    5          ON
 *  --------- ---------- ----------- --------
 *-
 */

#define BOARD_FLASH_WAITSTATES 5

/* LED definitions ******************************************************************/
/* The px4fmu-v4pro board has numerous LEDs.
 *  FMU_LED_RED, FMU_LED_GREEN & FMU_LED_BLUE are directly connected and
 * can be controlled by software.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

 /* LED index values for use with board_userled() */

 #define BOARD_LED1        0
 #define BOARD_LED2        1
 #define BOARD_LED3        2
 #define BOARD_NLEDS       3

 #define BOARD_LED_RED     BOARD_LED1
 #define BOARD_LED_GREEN   BOARD_LED2
 #define BOARD_LED_BLUE    BOARD_LED3

 /* LED bits for use with board_userled_all() */

 #define BOARD_LED1_BIT    (1 << BOARD_LED1)
 #define BOARD_LED2_BIT    (1 << BOARD_LED2)
 #define BOARD_LED3_BIT    (1 << BOARD_LED3)

 /* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
  * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
  * events as follows:
  *
  *
  *   SYMBOL                     Meaning                      LED state
  *                                                        Red   Green Blue
  *   ----------------------  --------------------------  ------ ------ ----*/

 #define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
 #define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
 #define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
 #define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
 #define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
 #define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
 #define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
 #define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
 #define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

 /* Thus if the Green LED is statically on, NuttX has successfully booted and
  * is, apparently, running normally.  If the Red LED is flashing at
  * approximately 2Hz, then a fatal error has been detected and the system
  * has halted.
  */


/* Alternate function pin selections ************************************************/

/*
 * UARTs.
 */
#define GPIO_USART1_RX	GPIO_USART1_RX_2	/* ESP8266 */
#define GPIO_USART1_TX	GPIO_USART1_TX_2

#define GPIO_USART2_RX	GPIO_USART2_RX_2
#define GPIO_USART2_TX	GPIO_USART2_TX_2
#define GPIO_USART2_RTS	GPIO_USART2_RTS_2
#define GPIO_USART2_CTS	GPIO_USART2_CTS_2

#define GPIO_USART3_RX	GPIO_USART3_RX_3
#define GPIO_USART3_TX	GPIO_USART3_TX_3
#define GPIO_USART3_RTS	GPIO_USART3_RTS_2
#define GPIO_USART3_CTS	GPIO_USART3_CTS_2

#define GPIO_UART4_RX	GPIO_UART4_RX_1
#define GPIO_UART4_TX	GPIO_UART4_TX_1

#define GPIO_USART6_RX	GPIO_USART6_RX_1	/* RC_INPUT */
#define GPIO_USART6_TX	GPIO_USART6_TX_1

#define GPIO_UART7_RX	GPIO_UART7_RX_1
#define GPIO_UART7_TX	GPIO_UART7_TX_1

/* UART8 has no alternate pin config */

/* UART RX DMA configurations */
#define DMAMAP_USART1_RX DMAMAP_USART1_RX_2
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_2

/*
 * CAN
 *
 * CAN1 is routed to the onboard transceiver.
 */
#define GPIO_CAN1_RX	GPIO_CAN1_RX_3
#define GPIO_CAN1_TX	GPIO_CAN1_TX_3

#define GPIO_CAN2_RX	GPIO_CAN2_RX_1
#define GPIO_CAN2_TX	GPIO_CAN2_TX_1

/*
 * I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */
#define GPIO_I2C1_SCL		GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA		GPIO_I2C1_SDA_2
#define GPIO_I2C1_SCL_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

#define GPIO_I2C2_SCL		GPIO_I2C2_SCL_2
#define GPIO_I2C2_SDA		GPIO_I2C2_SDA_2
#define GPIO_I2C2_SCL_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN1)
#define GPIO_I2C2_SDA_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN0)


/*
 * SPI
 *
 * There are sensors on SPI1, and SPI2 is connected to the FRAM.

 */
#define GPIO_SPI1_MISO  GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK   GPIO_SPI1_SCK_1


#define GPIO_SPI2_MISO  GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI  GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK   GPIO_SPI2_SCK_1

#define GPIO_SPI5_MISO  GPIO_SPI5_MISO_1
#define GPIO_SPI5_MOSI  GPIO_SPI5_MOSI_1
#define GPIO_SPI5_SCK   GPIO_SPI5_SCK_1
#define GPIO_SPI5_NSS   GPIO_SPI5_NSS_1


/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void stm32_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __NUTTX_CONFIG_PX4FMUV4_PRO_INCLUDE_BOARD_H */
