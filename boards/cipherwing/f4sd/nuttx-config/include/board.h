/************************************************************************************
 * nuttx-configs/cipherwing-f4sd/include/board.h
 * include/arch/board/board.h
 ************************************************************************************/

#ifndef __CONFIG_CIPHERWINGF4SD_INCLUDE_BOARD_H
#define __CONFIG_CIPHERWINGF4SD_INCLUDE_BOARD_H

#include "board_dma_map.h"
#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32.h"

/*======================== Clocking ========================*/

#define STM32_BOARD_XTAL        8000000ul
#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL: SYSCLK 168 MHz, USB/SDIO/RNG 48 MHz */
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* HCLK = SYSCLK */
#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY

/* APB1 = HCLK/4, APB2 = HCLK/2 */
#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)     /* 42 MHz */
#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)     /* 84 MHz */

/* Timers on APB1/2 run at 2x the APB clocks when prescaled */
#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* (Optional duplicates some PX4-level macros) */
#define BOARD_TIM1_FREQUENCY    STM32_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY    STM32_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32_APB1_TIM5_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32_APB2_TIM8_CLKIN
#define BOARD_TIM12_FREQUENCY   STM32_APB1_TIM12_CLKIN

/*======================== SDIO ========================*/

#define SDIO_INIT_CLKDIV        (118 << SDIO_CLKCR_CLKDIV_SHIFT) /* ~400 kHz */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)   /* 16 MHz */
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)   /* 12 MHz */
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* LED definitions ******************************************************************/

/* We physically have one LED (blue on PB5). */
#define BOARD_LED1              0
#define BOARD_NLEDS             1

#define BOARD_LED_BLUE          BOARD_LED1

/* If CONFIG_ARCH_LEDs/CONFIG_BOARD_CUSTOM_LEDS are set, NuttX uses these
 * symbolic states with board_autoled_on/off(). Define them all so common
 * code (e.g. arm_assert.c) compiles.
 */
#define LED_STARTED             0  /* LED1 */
#define LED_HEAPALLOCATE        1  /* LED2 (unused on this board) */
#define LED_IRQSENABLED         2  /* LED1 */
#define LED_STACKCREATED        3  /* LED1 + LED2 */
#define LED_INIRQ               4  /* LED1 */
#define LED_SIGNAL              5  /* LED2 (unused) */
#define LED_ASSERTION           6  /* LED1 + LED2 */
#define LED_PANIC               7  /* LED1 + LED2 */

/* Optional bit masks if any code uses stm32_setleds() */
#define BOARD_LED1_BIT          (1 << BOARD_LED1)
/* #define BOARD_LED2_BIT      (1 << BOARD_LED2)  // no second LED on this board */

/*======================== Alternate Functions ========================*/
/* UARTs */

/* USART1: PA9 (TX), PA10 (RX) */
#define GPIO_USART1_TX          GPIO_USART1_TX_1
#define GPIO_USART1_RX          GPIO_USART1_RX_1

/* USART2: PA2 (TX), PA3 (RX) */
#define GPIO_USART2_TX          GPIO_USART2_TX_1
#define GPIO_USART2_RX          GPIO_USART2_RX_1

/* USART3: PB10 (TX), PB11 (RX) */
#define GPIO_USART3_TX          GPIO_USART3_TX_1
#define GPIO_USART3_RX          GPIO_USART3_RX_1

/* UART4: PA0 (TX), PA1 (RX) */
#define GPIO_UART4_TX           GPIO_UART4_TX_1
#define GPIO_UART4_RX           GPIO_UART4_RX_1

/* UART5: RX real @ PD2, TX dummy @ PC13 (receive-only) */
#define GPIO_UART5_RX           GPIO_UART5_RX_1                 /* PD2 */
#define GPIO_UART5_TX           (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN13)

/* USART6: dummy placeholders (PWM uses PC6/PC7) */
#define GPIO_USART6_TX          (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN13) /* dummy */
#define GPIO_USART6_RX          (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN12) /* dummy */

/*======================== SPI ========================*/

/* SPI1 – IMU (MPU6000 / ICM20689)
 * CS: PA4 (defined in PX4 board_config.h), SCK: PA5, MISO: PA6, MOSI: PA7
 */
#define GPIO_SPI1_SCK           GPIO_SPI1_SCK_1
#define GPIO_SPI1_MISO          GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI          GPIO_SPI1_MOSI_1

/* SPI2 – SD card: CS PB12 (in PX4 board_config.h), SCK PB13, MISO PB14, MOSI PB15 */
#define GPIO_SPI2_SCK           GPIO_SPI2_SCK_2
#define GPIO_SPI2_MISO          GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI          GPIO_SPI2_MOSI_1

/* SPI3 – Baro/OSD: CS PB3/PA15 (in spi.cpp), SCK PC10, MISO PC11, MOSI PC12 */
#define GPIO_SPI3_SCK           GPIO_SPI3_SCK_2
#define GPIO_SPI3_MISO          GPIO_SPI3_MISO_2
#define GPIO_SPI3_MOSI          GPIO_SPI3_MOSI_2

/*======================== I2C ========================*/

/* I2C1 – External mag: PB8 (SCL), PB9 (SDA) */
#define GPIO_I2C1_SCL           GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA           GPIO_I2C1_SDA_1

#endif /* __CONFIG_CIPHERWINGF4SD_INCLUDE_BOARD_H */

