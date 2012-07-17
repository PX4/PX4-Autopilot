/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_cgu.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CGU_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CGU_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

#define LPC43_FREQ_MON_OFFSET           0x0014  /* Frequency monitor register */
#define LPC43_XTAL_OSC_CTRL_OFFSET      0x0018  /* Crystal oscillator control register */
#define LPC43_PLL0USB_STAT_OFFSET       0x001c  /* PLL0USB status register */
#define LPC43_PLL0USB_CTRL_OFFSET       0x0020  /* PLL0USB control register */
#define LPC43_PLL0USB_MDIV_OFFSET       0x0024  /* PLL0USB M-divider register */
#define LPC43_PLL0USB_NP_DIV_OFFSET     0x0028  /* PLL0USB N/P-divider register */
#define LPC43_PLL0AUDIO_STAT_OFFSET     0x002c  /* PLL0AUDIO status register */
#define LPC43_PLL0AUDIO_CTRL_OFFSET     0x0030  /* PLL0AUDIO control register */
#define LPC43_PLL0AUDIO_MDIV_OFFSET     0x0034  /* PLL0AUDIO M-divider */
#define LPC43_PLL0AUDIO_NP_DIV_OFFSET   0x0038  /* PLL0AUDIO N/P-divider */
#define LPC43_PLL0AUDIO_FRAC_OFFSET     0x003c  /* PLL0AUDIO fractional */
#define LPC43_PLL1_STAT_OFFSET          0x0040  /* PLL1 status register */
#define LPC43_PLL1_CTRL_OFFSET          0x0044  /* PLL1 control register */
#define LPC43_IDIVA_CTRL_OFFSET         0x0048  /* Integer divider A control register */
#define LPC43_IDIVB_CTRL_OFFSET         0x004c  /* Integer divider B control register */
#define LPC43_IDIVC_CTRL_OFFSET         0x0050  /* Integer divider C control register */
#define LPC43_IDIVD_CTRL_OFFSET         0x0054  /* Integer divider D control register */
#define LPC43_IDIVE_CTRL_OFFSET         0x0058  /* Integer divider E control register */
#define LPC43_BASE_SAFE_CLK_OFFSET      0x005c  /* Output stage 0 control register (BASE_SAFE_CLK) */
#define LPC43_BASE_USB0_CLK_OFFSET      0x0060  /* Output stage 1 control register (BASE_USB0_CLK) */
#define LPC43_BASE_PERIPH_CLK_OFFSET    0x0064  /* Output stage 2 control register (BASE_PERIPH_CLK) */
#define LPC43_BASE_USB1_CLK_OFFSET      0x0068  /* Output stage 3 control register (BASE_USB1_CLK) */
#define LPC43_BASE_M4_CLK_OFFSET        0x006c  /* Output stage 4 control register (BASE_M4_CLK) */
#define LPC43_BASE_SPIFI_CLK_OFFSET     0x0070  /* Output stage 5 control register (BASE_SPIFI_CLK) */
#define LPC43_BASE_SPI_CLK_OFFSET       0x0074  /* Output stage 6 control register (BASE_SPI_CLK) */
#define LPC43_BASE_PHYRX_CLK_OFFSET     0x0078  /* Output stage 7 control register (BASE_PHY_RX_CLK) */
#define LPC43_BASE_PHYTX_CLK_OFFSET     0x007c  /* Output stage 8 control register (BASE_PHY_TX_CLK) */
#define LPC43_BASE_APB1_CLK_OFFSET      0x0080  /* Output stage 9 control register (BASE_APB1_CLK) */
#define LPC43_BASE_APB3_CLK_OFFSET      0x0084  /* Output stage 10 control register (BASE_APB3_CLK) */
#define LPC43_BASE_LCD_CLK_OFFSET       0x0088  /* Output stage 11 control register (BASE_LCD_CLK) */
#define LPC43_BASE_VADC_CLK_OFFSET      0x008c  /* Output stage 12 control register (BASE_VADC_CLK) */
#define LPC43_BASE_SDIO_CLK_OFFSET      0x0090  /* Output stage 13 control register (BASE_SDIO_CLK) */
#define LPC43_BASE_SSP0_CLK_OFFSET      0x0094  /* Output stage 14 control register (BASE_SSP0_CLK) */
#define LPC43_BASE_SSP1_CLK_OFFSET      0x0098  /* Output stage 15 control register (BASE_SSP1_CLK) */
#define LPC43_BASE_USART0_CLK_OFFSET    0x009c  /* Output stage 16 control register (BASE_USART0_CLK) */
#define LPC43_BASE_UART1_CLK_OFFSET     0x00a0  /* Output stage 17 control register (BASE_UART1_CLK) */
#define LPC43_BASE_USART2_CLK_OFFSET    0x00a4  /* Output stage 18 control register (BASE_USART2_CLK) */
#define LPC43_BASE_USART3_CLK_OFFSET    0x00a8  /* Output stage 19 control register (BASE_USART3_CLK) */
#define LPC43_BASE_OUT_CLK_OFFSET       0x00ac  /* Output stage 20 control register (BASE_OUT_CLK) */
#define LPC43_BASE_APLL_CLK_OFFSET      0x00c0  /* Output stage 25 control register (BASE_APLL_CLK) */
#define LPC43_BASE_CGU_OUT0_CLK_OFFSET  0x00c4  /* Output stage 26 control register (BASE_CGU_OUT0_CLK) */
#define LPC43_BASE_CGU_OUT1_CLK_OFFSET  0x00c8  /* Output stage 27 control register (BASE_CGU_OUT1_CLK) */

/* Register Addresses *******************************************************************************/

#define LPC43_FREQ_MON                  (LPC43_CGU_BASE+LPC43_FREQ_MON_OFFSET)
#define LPC43_XTAL_OSC_CTRL             (LPC43_CGU_BASE+LPC43_XTAL_OSC_CTRL_OFFSET)
#define LPC43_PLL0USB_STAT              (LPC43_CGU_BASE+LPC43_PLL0USB_STAT_OFFSET)
#define LPC43_PLL0USB_CTRL              (LPC43_CGU_BASE+LPC43_PLL0USB_CTRL_OFFSET)
#define LPC43_PLL0USB_MDIV              (LPC43_CGU_BASE+LPC43_PLL0USB_MDIV_OFFSET)
#define LPC43_PLL0USB_NP_DIV            (LPC43_CGU_BASE+LPC43_PLL0USB_NP_DIV_OFFSET)
#define LPC43_PLL0AUDIO_STAT            (LPC43_CGU_BASE+LPC43_PLL0AUDIO_STAT_OFFSET)
#define LPC43_PLL0AUDIO_CTRL            (LPC43_CGU_BASE+LPC43_PLL0AUDIO_CTRL_OFFSET)
#define LPC43_PLL0AUDIO_MDIV            (LPC43_CGU_BASE+LPC43_PLL0AUDIO_MDIV_OFFSET)
#define LPC43_PLL0AUDIO_NP_DIV          (LPC43_CGU_BASE+LPC43_PLL0AUDIO_NP_DIV_OFFSET)
#define LPC43_PLL0AUDIO_FRAC            (LPC43_CGU_BASE+LPC43_PLL0AUDIO_FRAC_OFFSET)
#define LPC43_PLL1_STAT                 (LPC43_CGU_BASE+LPC43_PLL1_STAT_OFFSET)
#define LPC43_PLL1_CTRL                 (LPC43_CGU_BASE+LPC43_PLL1_CTRL_OFFSET)
#define LPC43_IDIVA_CTRL                (LPC43_CGU_BASE+LPC43_IDIVA_CTRL_OFFSET)
#define LPC43_IDIVB_CTRL                (LPC43_CGU_BASE+LPC43_IDIVB_CTRL_OFFSET)
#define LPC43_IDIVC_CTRL                (LPC43_CGU_BASE+LPC43_IDIVC_CTRL_OFFSET)
#define LPC43_IDIVD_CTRL                (LPC43_CGU_BASE+LPC43_IDIVD_CTRL_OFFSET)
#define LPC43_IDIVE_CTRL                (LPC43_CGU_BASE+LPC43_IDIVE_CTRL_OFFSET)
#define LPC43_BASE_SAFE_CLK             (LPC43_CGU_BASE+LPC43_BASE_SAFE_CLK_OFFSET)
#define LPC43_BASE_USB0_CLK             (LPC43_CGU_BASE+LPC43_BASE_USB0_CLK_OFFSET)
#define LPC43_BASE_PERIPH_CLK           (LPC43_CGU_BASE+LPC43_BASE_PERIPH_CLK_OFFSET)
#define LPC43_BASE_USB1_CLK             (LPC43_CGU_BASE+LPC43_BASE_USB1_CLK_OFFSET)
#define LPC43_BASE_M4_CLK               (LPC43_CGU_BASE+LPC43_BASE_M4_CLK_OFFSET)
#define LPC43_BASE_SPIFI_CLK            (LPC43_CGU_BASE+LPC43_BASE_SPIFI_CLK_OFFSET)
#define LPC43_BASE_SPI_CLK              (LPC43_CGU_BASE+LPC43_BASE_SPI_CLK_OFFSET)
#define LPC43_BASE_PHYRX_CLK            (LPC43_CGU_BASE+LPC43_BASE_PHYRX_CLK_OFFSET)
#define LPC43_BASE_PHYTX_CLK            (LPC43_CGU_BASE+LPC43_BASE_PHYTX_CLK_OFFSET)
#define LPC43_BASE_APB1_CLK             (LPC43_CGU_BASE+LPC43_BASE_APB1_CLK_OFFSET)
#define LPC43_BASE_APB3_CLK             (LPC43_CGU_BASE+LPC43_BASE_APB3_CLK_OFFSET)
#define LPC43_BASE_LCD_CLK              (LPC43_CGU_BASE+LPC43_BASE_LCD_CLK_OFFSET)
#define LPC43_BASE_VADC_CLK             (LPC43_CGU_BASE+LPC43_BASE_VADC_CLK_OFFSET)
#define LPC43_BASE_SDIO_CLK             (LPC43_CGU_BASE+LPC43_BASE_SDIO_CLK_OFFSET)
#define LPC43_BASE_SSP0_CLK             (LPC43_CGU_BASE+LPC43_BASE_SSP0_CLK_OFFSET)
#define LPC43_BASE_SSP1_CLK             (LPC43_CGU_BASE+LPC43_BASE_SSP1_CLK_OFFSET)
#define LPC43_BASE_USART0_CLK           (LPC43_CGU_BASE+LPC43_BASE_USART0_CLK_OFFSET)
#define LPC43_BASE_UART1_CLK            (LPC43_CGU_BASE+LPC43_BASE_UART1_CLK_OFFSET)
#define LPC43_BASE_USART2_CLK           (LPC43_CGU_BASE+LPC43_BASE_USART2_CLK_OFFSET)
#define LPC43_BASE_USART3_CLK           (LPC43_CGU_BASE+LPC43_BASE_USART3_CLK_OFFSET)
#define LPC43_BASE_OUT_CLK              (LPC43_CGU_BASE+LPC43_BASE_OUT_CLK_OFFSET)
#define LPC43_BASE_APLL_CLK             (LPC43_CGU_BASE+LPC43_BASE_APLL_CLK_OFFSET)
#define LPC43_BASE_CGU_OUT0_CLK         (LPC43_CGU_BASE+LPC43_BASE_CGU_OUT0_CLK_OFFSET)
#define LPC43_BASE_CGU_OUT1_CLK         (LPC43_CGU_BASE+LPC43_BASE_CGU_OUT1_CLK_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Frequency monitor register */

#define FREQ_MON_RCNT_SHIFT             (0)       /* Bits 0-8: 9-bit reference clock-counter value */
#define FREQ_MON_RCNT_MASK              (0x1ff << FREQ_MON_RCNT_SHIFT)
#define FREQ_MON_FCNT_SHIFT             (9)       /* Bits 9-22: 14-bit selected clock-counter value */
#define FREQ_MON_FCNT_MASK              (0x3fff << FREQ_MON_FCNT_SHIFT)
#define FREQ_MON_MEAS                   (1 << 23) /* Bit 23: Measure frequency */
#define FREQ_MON_CLKSEL_SHIFT           (24)      /* Bits 24-28: Clock-source selection */
#define FREQ_MON_CLKSEL_MASK            (31 << FREQ_MON_CLKSEL_SHIFT)
#  define FREQ_MON_CLKSEL_32KHZOSC      (0 << FREQ_MON_CLKSEL_SHIFT)  /* 32 kHz oscillator (default) */
#  define FREQ_MON_CLKSEL_IRQ           (1 << FREQ_MON_CLKSEL_SHIFT)  /* IRC */
#  define FREQ_MON_CLKSEL_ENET_RXCLK    (2 << FREQ_MON_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define FREQ_MON_CLKSEL_ENET_TXCLK    (3 << FREQ_MON_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define FREQ_MON_CLKSEL_GPCLKIN       (4 << FREQ_MON_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define FREQ_MON_CLKSEL_XTAL          (6 << FREQ_MON_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define FREQ_MON_CLKSEL_PLL0USB       (7 << FREQ_MON_CLKSEL_SHIFT)  /* PLL0USB */
#  define FREQ_MON_CLKSEL_PLL0AUDIO     (8 << FREQ_MON_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define FREQ_MON_CLKSEL_PLL1          (9 << FREQ_MON_CLKSEL_SHIFT)  /* PLL1 */
#  define FREQ_MON_CLKSEL_IDIVA         (12 << FREQ_MON_CLKSEL_SHIFT) /* IDIVA */
#  define FREQ_MON_CLKSEL_IDIVB         (13 << FREQ_MON_CLKSEL_SHIFT) /* IDIVB */
#  define FREQ_MON_CLKSEL_IDIVC         (14 << FREQ_MON_CLKSEL_SHIFT) /* IDIVC */
#  define FREQ_MON_CLKSEL_IDIVD         (15 << FREQ_MON_CLKSEL_SHIFT) /* IDIVD */
#  define FREQ_MON_CLKSEL_IDIVE         (16 << FREQ_MON_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Crystal oscillator control register */

#define XTAL_OSC_CTRL_ENABLE            (1 << 0)  /* Bit 0:  Oscillator-pad enable */
#define XTAL_OSC_CTRL_BYPASS            (1 << 1)  /* Bit 1:  Configure crystal or external-clock input */
#define XTAL_OSC_CTRL_HF                (1 << 2)  /* Bit 2:  Select frequency range */
                                                  /* Bits 3-31: Reserved */
/* PLL0USB status register */

#define PLL0USB_STAT_LOCK               (1 << 0)  /* Bit 0:  PLL0 lock indicator */
#define PLL0USB_STAT_FR                 (1 << 1)  /* Bit 1:  PLL0 free running indicator */
                                                  /* Bits 2-31: Reserved */
/* PLL0USB control register */

#define PLL0USB_CTRL_PD                 (1 << 0)  /* Bit 0:  PLL0 power down */
#define PLL0USB_CTRL_BYPASS             (1 << 1)  /* Bit 1:  Input clock bypass control */
#define PLL0USB_CTRL_DIRECTI            (1 << 2)  /* Bit 2:  PLL0 direct input */
#define PLL0USB_CTRL_DIRECTO            (1 << 3)  /* Bit 3:  PLL0 direct output */
#define PLL0USB_CTRL_CLKEN              (1 << 4)  /* Bit 4:  PLL0 clock enable */
                                                  /* Bit 5:  Reserved */
#define PLL0USB_CTRL_FRM                (1 << 6)  /* Bit 6:  Free running mode */
                                                  /* Bits 7-10: Reserved */
#define PLL0USB_CTRL_AUTOBLOCK          (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define PLL0USB_CTRL_CLKSEL_SHIFT       (24)      /* Bits 24-28: Clock source selection */
#define PLL0USB_CTRL_CLKSEL_MASK        (31 << PLL0USB_CTRL_CLKSEL_SHIFT)
#  define PLL0USB_CLKSEL_32KHZOSC       (0 << PLL0USB_CTRL_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define PLL0USB_CLKSEL_IRC            (1 << PLL0USB_CTRL_CLKSEL_SHIFT)  /* IRC (default) */
#  define PLL0USB_CLKSEL_ENET_RXCLK     (2 << PLL0USB_CTRL_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define PLL0USB_CLKSEL_ENET_TXCLK     (3 << PLL0USB_CTRL_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define PLL0USB_CLKSEL_GPCLKIN        (4 << PLL0USB_CTRL_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define PLL0USB_CLKSEL_XTAL           (6 << PLL0USB_CTRL_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define PLL0USB_CLKSEL_PLL1           (9 << PLL0USB_CTRL_CLKSEL_SHIFT)  /* PLL1 */
#  define PLL0USB_CLKSEL_IDIVA          (12 << PLL0USB_CTRL_CLKSEL_SHIFT) /* IDIVA */
#  define PLL0USB_CLKSEL_IDIVB          (13 << PLL0USB_CTRL_CLKSEL_SHIFT) /* IDIVB */
#  define PLL0USB_CLKSEL_IDIVC          (14 << PLL0USB_CTRL_CLKSEL_SHIFT) /* IDIVC */
#  define PLL0USB_CLKSEL_IDIVD          (15 << PLL0USB_CTRL_CLKSEL_SHIFT) /* IDIVD */
#  define PLL0USB_CLKSEL_IDIVE          (16 << PLL0USB_CTRL_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* PLL0USB M-divider register */

#define PLL0USB_MDIV_MDEC_SHIFT         (0)      /* Bits 0-16: Decoded M-divider coefficient value (1-131071) */
#define PLL0USB_MDIV_MDEC_MASK          (0x1ffff << PLL0USB_MDIV_MDEC_SHIFT)
#  define PLL0USB_MDIV_MDEC(n)          ((n) << PLL0USB_MDIV_MDEC_SHIFT)
#define PLL0USB_MDIV_SELP_SHIFT         (17)     /* Bits 17-21: Bandwidth select P value */
#define PLL0USB_MDIV_SELP_MASK          (0x1f << PLL0USB_MDIV_SELP_SHIFT)
#  define PLL0USB_MDIV_SELP(n)          ((n) << PLL0USB_MDIV_SELP_SHIFT)
#define PLL0USB_MDIV_SELI_SHIFT         (22)     /* Bits 22-27: Bandwidth select I value */
#define PLL0USB_MDIV_SELI_MASK          (0x3f << PLL0USB_MDIV_SELI_SHIFT)
#  define PLL0USB_MDIV_SELI(n)          ((n) << PLL0USB_MDIV_SELI_SHIFT)
#define PLL0USB_MDIV_SELR_SHIFT         (28)     /* Bits 28-31: Bandwidth select R value */
#define PLL0USB_MDIV_SELR_MASK          (15 << PLL0USB_MDIV_SELR_SHIFT)
#  define PLL0USB_MDIV_SELR(n)          ((n) << PLL0USB_MDIV_SELR_SHIFT)

/* PLL0USB N/P-divider register */

#define PLL0USB_NP_DIV_PDEC_SHIFT       (0)      /* Bits 0-6: Decoded P-divider coefficient value */
#define PLL0USB_NP_DIV_PDEC_MASK        (0x7f << PLL0USB_NP_DIV_PDEC_SHIFT)
#  define PLL0USB_NP_DIV_PDEC(n)        ((n) << PLL0USB_NP_DIV_PDEC_SHIFT)
                                                 /* Bits 7-11:  Reserved */
#define PLL0USB_NP_DIV_NDEC_SHIFT       (12)     /* Bits 12-21: Decoded N-divider coefficient value */
#define PLL0USB_NP_DIV_NDEC_MASK        (0x3ff << PLL0USB_NP_DIV_NDEC_SHIFT)
#  define PLL0USB_NP_DIV_NDEC(n)        ((n) << PLL0USB_NP_DIV_NDEC_SHIFT)
                                                 /* Bits 22-31:  Reserved */
/* PLL0AUDIO status register */

#define PLL0AUDIO_STAT_LOCK             (1 << 0)  /* Bit 0:  PLL0 lock indicator */
#define PLL0AUDIO_STAT_FR               (1 << 1)  /* Bit 1:  PLL0 free running indicator */
                                                  /* Bits 2-31: Reserved */
/* PLL0AUDIO control register */

#define PLL0AUDIO_CTRL_PD               (1 << 0)  /* Bit 0:  PLL0 power down */
#define PLL0AUDIO_CTRL_BYPASS           (1 << 1)  /* Bit 1:  Input clock bypass control */
#define PLL0AUDIO_CTRL_DIRECTI          (1 << 2)  /* Bit 2:  PLL0 direct input */
#define PLL0AUDIO_CTRL_DIRECTO          (1 << 3)  /* Bit 3:  PLL0 direct output */
#define PLL0AUDIO_CTRL_CLKEN            (1 << 4)  /* Bit 4:  PLL0 clock enable */
                                                  /* Bit 5:  Reserved */
#define PLL0AUDIO_CTRL_FRM              (1 << 6)  /* Bit 6:  Free running mode */
                                                  /* Bits 7-10: Reserved */
#define PLL0AUDIO_CTRL_AUTOBLOCK        (1 << 11) /* Bit 11: Block clock during frequency change */

#define PLL0AUDIO_CTRL_PLLFRACTREQ      (1 << 12) /* Bit 12: Fractional PLL word write request */
#define PLL0AUDIO_CTRL_SELEXT           (1 << 13) /* Bit 13: Select fractional divider */
#define PLL0AUDIO_CTRL_MODPD            (1 << 14) /* Bit 14: Sigma-Delta modulator power-down */
                                                  /* Bits 15-23: Reserved */
#define PLL0AUDIO_CTRL_CLKSEL_SHIFT     (24)      /* Bits 24-28: Clock source selection */
#define PLL0AUDIO_CTRL_CLKSEL_MASK      (31 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)
#  define PLL0AUDIO_CLKSEL_32KHZOSC     (0 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define PLL0AUDIO_CLKSEL_IRC          (1 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)  /* IRC (default) */
#  define PLL0AUDIO_CLKSEL_ENET_RXCLK   (2 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define PLL0AUDIO_CLKSEL_ENET_TXCLK   (3 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define PLL0AUDIO_CLKSEL_GPCLKIN      (4 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define PLL0AUDIO_CLKSEL_XTAL         (6 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define PLL0AUDIO_CLKSEL_PLL1         (9 << PLL0AUDIO_CTRL_CLKSEL_SHIFT)  /* PLL1 */
#  define PLL0AUDIO_CLKSEL_IDIVA        (12 << PLL0AUDIO_CTRL_CLKSEL_SHIFT) /* IDIVA */
#  define PLL0AUDIO_CLKSEL_IDIVB        (13 << PLL0AUDIO_CTRL_CLKSEL_SHIFT) /* IDIVB */
#  define PLL0AUDIO_CLKSEL_IDIVC        (14 << PLL0AUDIO_CTRL_CLKSEL_SHIFT) /* IDIVC */
#  define PLL0AUDIO_CLKSEL_IDIVD        (15 << PLL0AUDIO_CTRL_CLKSEL_SHIFT) /* IDIVD */
#  define PLL0AUDIO_CLKSEL_IDIVE        (16 << PLL0AUDIO_CTRL_CLKSEL_SHIFT) /* IDIVE */
                                                  /* Bits 29-31:  Reserved */
/* PLL0AUDIO M-divider */

#define PLL0AUDIO_MDIV_MDEC_SHIFT       (0)       /* Bits 0-16: Decoded M-divider coefficient value (1-131071) */
#define PLL0AUDIO_MDIV_MDEC_MASK        (0x1ffff << PLL0AUDIO_MDIV_MDEC_SHIFT)
#  define PLL0AUDIO_MDIV_MDEC(n)        ((n) << PLL0AUDIO_MDIV_MDEC_SHIFT)
                                                  /* Bits 17-31:  Reserved */
/* PLL0AUDIO N/P-divider */

#define PLL0AUDIO_NP_DIV_PDEC_SHIFT     (0)      /* Bits 0-6: Decoded P-divider coefficient value */
#define PLL0AUDIO_NP_DIV_PDEC_MASK      (0x7f << PLL0AUDIO_NP_DIV_PDEC_SHIFT)
#  define PLL0AUDIO_NP_DIV_PDEC(n)      ((n) << PLL0AUDIO_NP_DIV_PDEC_SHIFT)
                                                 /* Bits 7-11:  Reserved */
#define PLL0AUDIO_NP_DIV_NDEC_SHIFT     (12)     /* Bits 12-21: Decoded N-divider coefficient value */
#define PLL0AUDIO_NP_DIV_NDEC_MASK      (0x3ff << PLL0AUDIO_NP_DIV_NDEC_SHIFT)
#  define PLL0AUDIO_NP_DIV_NDEC(n)      ((n) << PLL0AUDIO_NP_DIV_NDEC_SHIFT)
                                                 /* Bits 22-31:  Reserved */
/* PLL0AUDIO fractional */

#define PLL0AUDIO_FRAC_CTRL_SHIFT       (0)      /* Bits 0-21: Decoded P-divider coefficient value */
#define PLL0AUDIO_FRAC_CTRL_MASK        (0x3fffff << PLL0AUDIO_FRAC_CTRL_SHIFT)
#  define PLL0AUDIO_FRA_CCTRL(n)        ((n) << PLL0AUDIO_FRAC_CTRL_SHIFT)
                                                 /* Bits 22-31:  Reserved */
/* PLL1 status register */

#define PLL1_STAT_LOCK                  (1 << 0)  /* Bit 0:  PLL1 lock indicator */
                                                  /* Bits 1-31: Reserved */
/* PLL1 control register */

#define PLL1_CTRL_PD                    (1 << 0)  /* Bit 0:  PLL1 power down */
#define PLL1_CTRL_BYPASS                (1 << 1)  /* Bit 1:  Input clock bypass control */
                                                  /* Bits 2-5: Reserved */
#define PLL1_CTRL_FBSEL                 (1 << 6)  /* Bit 6:  PLL1 feedback select */
#define PLL1_CTRL_DIRECT                (1 << 7)  /* Bit 7:  PLL1 direct CCO output */

#define PLL1_CTRL_PSEL_SHIFT            (8)       /* Bits 8-9: Post-divider division ratio P */
#define PLL1_CTRL_PSEL_MASK             (3 << PLL1_CTRL_PSEL_SHIFT)
#  define PLL1_CTRL_PSEL_DIV1           (0 << PLL1_CTRL_PSEL_SHIFT)
#  define PLL1_CTRL_PSEL_DIV2           (1 << PLL1_CTRL_PSEL_SHIFT)
#  define PLL1_CTRL_PSEL_DIV4           (2 << PLL1_CTRL_PSEL_SHIFT)
#  define PLL1_CTRL_PSEL_DIV8           (3 << PLL1_CTRL_PSEL_SHIFT)
                                                  /* Bit 10: Reserved */
#define PLL1_CTRL_AUTOBLOCK             (1 << 11) /* Bit 11: Block clock during frequency change */
#define PLL1_CTRL_NSEL_SHIFT            (12)      /* Bits 12-13: Pre-divider division ratio N */
#define PLL1_CTRL_NSEL_MASK             (3 << PLL1_CTRL_NSEL_SHIFT)
#  define PLL1_CTRL_NSEL_DIV1           (0 << PLL1_CTRL_NSEL_SHIFT)
#  define PLL1_CTRL_NSEL_DIV2           (1 << PLL1_CTRL_NSEL_SHIFT)
#  define PLL1_CTRL_NSEL_DIV3           (2 << PLL1_CTRL_NSEL_SHIFT)
#  define PLL1_CTRL_NSEL_DIV4           (3 << PLL1_CTRL_NSEL_SHIFT)
                                                  /* Bits 14-15: Reserved */
#define PLL1_CTRL_MSEL_SHIFT            (16)      /* Bits 16-23: Feedback-divider division ratio M */
#define PLL1_CTRL_MSEL_MASK             (0xff << PLL1_CTRL_MSEL_SHIFT)
#  define PLL1_CTRL_MSEL(n)             (((n)-1) << PLL1_CTRL_MSEL_SHIFT) /* n=1..256 */
#define PLL1_CTRL_CLKSEL_SHIFT          (24)      /* Bits 24-28: Clock source selection */
#define PLL1_CTRL_CLKSEL_MASK           (31 << PLL1_CTRL_CLKSEL_SHIFT)
#  define PLL1_CLKSEL_32KHZOSC          (0 << PLL1_CTRL_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define PLL1_CLKSEL_IRC               (1 << PLL1_CTRL_CLKSEL_SHIFT)  /* IRC (default) */
#  define PLL1_CLKSEL_ENET_RXCLK        (2 << PLL1_CTRL_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define PLL1_CLKSEL_ENET_TXCLK        (3 << PLL1_CTRL_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define PLL1_CLKSEL_GPCLKIN           (4 << PLL1_CTRL_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define PLL1_CLKSEL_XTAL              (6 << PLL1_CTRL_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define PLL1_CLKSEL_PLL0USB           (7 << PLL1_CTRL_CLKSEL_SHIFT)  /* PLL0USB */
#  define PLL1_CLKSEL_PLL0AUDIO         (8 << PLL1_CTRL_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define PLL1_CLKSEL_IDIVA             (12 << PLL1_CTRL_CLKSEL_SHIFT) /* IDIVA */
#  define PLL1_CLKSEL_IDIVB             (13 << PLL1_CTRL_CLKSEL_SHIFT) /* IDIVB */
#  define PLL1_CLKSEL_IDIVC             (14 << PLL1_CTRL_CLKSEL_SHIFT) /* IDIVC */
#  define PLL1_CLKSEL_IDIVD             (15 << PLL1_CTRL_CLKSEL_SHIFT) /* IDIVD */
#  define PLL1_CLKSEL_IDIVE             (16 << PLL1_CTRL_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Integer divider A control register */

#define IDIVA_CTRL_PD                   (1 << 0)  /* Bit 0:  Integer divider A power down */
                                                  /* Bit 1:  Reserved */
#define IDIVA_CTRL_IDIV_SHIFT           (2)       /* Bits 2-3: Integer divider A divider values (1/(IDIV + 1)) */
#define IDIVA_CTRL_IDIV_MASK            (3 << IDIVA_CTRL_IDIV_SHIFT)
#  define IDIVA_CTRL_IDIV(n)            (((n)-1) << IDIVA_CTRL_IDIV_SHIFT) /* n=1..4 */
                                                  /* Bits 4-10: Reserved */
#define IDIVA_CTRL_AUTOBLOCK            (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define IDIVA_CTRL_CLKSEL_SHIFT         (24)      /* Bits 24-28: Clock source selection */
#define IDIVA_CTRL_CLKSEL_MASK          (31 << IDIVA_CTRL_CLKSEL_SHIFT)
#  define IDIVA_CLKSEL_32KHZOSC         (0 << IDIVA_CTRL_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define IDIVA_CLKSEL_IRC              (1 << IDIVA_CTRL_CLKSEL_SHIFT)  /* IRC (default) */
#  define IDIVA_CLKSEL_ENET_RXCLK       (2 << IDIVA_CTRL_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define IDIVA_CLKSEL_ENET_TXCLK       (3 << IDIVA_CTRL_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define IDIVA_CLKSEL_GPCLKIN          (4 << IDIVA_CTRL_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define IDIVA_CLKSEL_XTAL             (6 << IDIVA_CTRL_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define IDIVA_CLKSEL_PLL0USB          (7 << IDIVA_CTRL_CLKSEL_SHIFT)  /* PLL0USB */
#  define IDIVA_CLKSEL_PLL0AUDIO        (8 << IDIVA_CTRL_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define IDIVA_CLKSEL_PLL1             (9 << IDIVA_CTRL_CLKSEL_SHIFT)  /* PLL1 */
                                                 /* Bits 29-31:  Reserved */
/* Integer divider B/C/D control register */

#define IDIVBCD_CTRL_PD                 (1 << 0)  /* Bit 0:  Integer divider power down */
                                                  /* Bit 1: Reserved */
#define IDIVBCD_CTRL_IDIV_SHIFT         (2)       /* Bits 2-5: Integer divider A divider values (1/(IDIV + 1)) */
#define IDIVBCD_CTRL_IDIV_MASK          (15 << IDIVBCD_CTRL_IDIV_SHIFT)
#  define IDIVBCD_CTRL_IDIV(n)          (((n)-1) << IDIVBCD_CTRL_IDIV_SHIFT) /* n=1..16 */
                                                  /* Bits 6-10: Reserved */
#define IDIVBCD_CTRL_AUTOBLOCK          (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define IDIVBCD_CTRL_CLKSEL_SHIFT       (24)      /* Bits 24-28: Clock source selection */
#define IDIVBCD_CTRL_CLKSEL_MASK        (31 << IDIVBCD_CTRL_CLKSEL_SHIFT)
#  define IDIVBCD_CLKSEL_32KHZOSC       (0 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define IDIVBCD_CLKSEL_IRC            (1 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* IRC (default) */
#  define IDIVBCD_CLKSEL_ENET_RXCLK     (2 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define IDIVBCD_CLKSEL_ENET_TXCLK     (3 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define IDIVBCD_CLKSEL_GPCLKIN        (4 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define IDIVBCD_CLKSEL_XTAL           (6 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define IDIVBCD_CLKSEL_PLL0AUDIO      (8 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define IDIVBCD_CLKSEL_PLL1           (9 << IDIVBCD_CTRL_CLKSEL_SHIFT)  /* PLL1 */
#  define IDIVBCD_CLKSEL_IDIVA          (12 << IDIVBCD_CTRL_CLKSEL_SHIFT) /* IDIVA */
                                                 /* Bits 29-31:  Reserved */
/* Integer divider E control register */

#define IDIVE_CTRL_PD                   (1 << 0)  /* Bit 0:  Integer divider E power down */
                                                  /* Bit 1: Reserved */
#define IDIVE_CTRL_IDIV_SHIFT           (2)       /* Bits 2-9: Integer divider A divider values (1/(IDIV + 1)) */
#define IDIVE_CTRL_IDIV_MASK            (0xff << IDIVE_CTRL_IDIV_SHIFT)
#  define IDIVE_CTRL_IDIV(n)            (((n)-1) << IDIVE_CTRL_IDIV_SHIFT) /* n=1..256 */
                                                  /* Bit 10: Reserved */
#define IDIVE_CTRL_AUTOBLOCK            (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define IDIVE_CTRL_CLKSEL_SHIFT         (24)      /* Bits 24-28: Clock source selection */
#define IDIVE_CTRL_CLKSEL_MASK          (31 << IDIVE_CTRL_CLKSEL_SHIFT)
#  define IDIVE_CLKSEL_32KHZOSC         (0 << IDIVE_CTRL_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define IDIVE_CLKSEL_IRC              (1 << IDIVE_CTRL_CLKSEL_SHIFT)  /* IRC (default) */
#  define IDIVE_CLKSEL_ENET_RXCLK       (2 << IDIVE_CTRL_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define IDIVE_CLKSEL_ENET_TXCLK       (3 << IDIVE_CTRL_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define IDIVE_CLKSEL_GPCLKIN          (4 << IDIVE_CTRL_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define IDIVE_CLKSEL_XTAL             (6 << IDIVE_CTRL_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define IDIVE_CLKSEL_PLL0AUDIO        (8 << IDIVE_CTRL_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define IDIVE_CLKSEL_PLL1             (9 << IDIVE_CTRL_CLKSEL_SHIFT)  /* PLL1 */
#  define IDIVE_CLKSEL_IDIVA            (12 << IDIVE_CTRL_CLKSEL_SHIFT) /* IDIVA */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 0 control register (BASE_SAFE_CLK) */

#define BASE_SAFE_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_SAFE_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_SAFE_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_SAFE_CLK_CLKSEL_MASK       (31 << BASE_SAFE_CLK_CLKSEL_SHIFT)
#  define BASE_SAFE_CLKSEL_IRC          (1 << BASE_SAFE_CLK_CLKSEL_SHIFT)  /* IRC (default) */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 1 control register (BASE_USB0_CLK) */

#define BASE_USB0_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_USB0_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_USB0_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_USB0_CLK_CLKSEL_MASK       (31 << BASE_USB0_CLK_CLKSEL_SHIFT)
#  define BASE_USB0_CLKSEL_PLL0USB      (7 << BASE_USB0_CLK_CLKSEL_SHIFT)  /* PLL0USB (default) */
                                                  /* Bits 29-31:  Reserved */
/* Output stage 2 control register (BASE_PERIPH_CLK) */

#define BASE_PERIPH_CLK_PD              (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_PERIPH_CLK_AUTOBLOCK       (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_PERIPH_CLK_CLKSEL_SHIFT    (24)      /* Bits 24-28: Clock source selection */
#define BASE_PERIPH_CLK_CLKSEL_MASK     (31 << BASE_PERIPH_CLK_CLKSEL_SHIFT)
#  define BASE_PERIPH_CLKSEL_32KHZOSC   (0 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_PERIPH_CLKSEL_IRC        (1 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_PERIPH_CLKSEL_ENET_RXCLK (2 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_PERIPH_CLKSEL_ENET_TXCLK (3 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_PERIPH_CLKSEL_GPCLKIN    (4 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_PERIPH_CLKSEL_XTAL       (6 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_PERIPH_CLKSEL_PLL0AUDIO  (8 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_PERIPH_CLKSEL_PLL1       (9 << BASE_PERIPH_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_PERIPH_CLKSEL_IDIVA      (12 << BASE_PERIPH_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_PERIPH_CLKSEL_IDIVB      (13 << BASE_PERIPH_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_PERIPH_CLKSEL_IDIVC      (14 << BASE_PERIPH_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_PERIPH_CLKSEL_IDIVD      (15 << BASE_PERIPH_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_PERIPH_CLKSEL_IDIVE      (16 << BASE_PERIPH_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                  /* Bits 29-31:  Reserved */
/* Output stage 3 control register (BASE_USB1_CLK) */

#define BASE_USB1_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_USB1_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_USB1_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_USB1_CLK_CLKSEL_MASK       (31 << BASE_USB1_CLK_CLKSEL_SHIFT)
#  define BASE_USB1_CLKSEL_32KHZOSC     (0 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_USB1_CLKSEL_IRC          (1 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_USB1_CLKSEL_ENET_RXCLK   (2 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_USB1_CLKSEL_ENET_TXCLK   (3 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_USB1_CLKSEL_GPCLKIN      (4 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_USB1_CLKSEL_XTAL         (6 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_USB1_CLKSEL_PLL0USB      (7 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* PLL0USB */
#  define BASE_USB1_CLKSEL_PLL0AUDIO    (8 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_USB1_CLKSEL_PLL1         (9 << BASE_USB1_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_USB1_CLKSEL_IDIVA        (12 << BASE_USB1_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_USB1_CLKSEL_IDIVB        (13 << BASE_USB1_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_USB1_CLKSEL_IDIVC        (14 << BASE_USB1_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_USB1_CLKSEL_IDIVD        (15 << BASE_USB1_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_USB1_CLKSEL_IDIVE        (16 << BASE_USB1_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 4 control register (BASE_M4_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_M4_CLK_PD                  (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_M4_CLK_AUTOBLOCK           (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_M4_CLK_CLKSEL_SHIFT        (24)      /* Bits 24-28: Clock source selection */
#define BASE_M4_CLK_CLKSEL_MASK         (31 << BASE_M4_CLK_CLKSEL_SHIFT)
#  define BASE_M4_CLKSEL_32KHZOSC       (0 << BASE_M4_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_M4_CLKSEL_IRC            (1 << BASE_M4_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_M4_CLKSEL_ENET_RXCLK     (2 << BASE_M4_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_M4_CLKSEL_ENET_TXCLK     (3 << BASE_M4_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_M4_CLKSEL_GPCLKIN        (4 << BASE_M4_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_M4_CLKSEL_XTAL           (6 << BASE_M4_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_M4_CLKSEL_PLL0AUDIO      (8 << BASE_M4_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_M4_CLKSEL_PLL1           (9 << BASE_M4_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_M4_CLKSEL_IDIVA          (12 << BASE_M4_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_M4_CLKSEL_IDIVB          (13 << BASE_M4_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_M4_CLKSEL_IDIVC          (14 << BASE_M4_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_M4_CLKSEL_IDIVD          (15 << BASE_M4_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_M4_CLKSEL_IDIVE          (16 << BASE_M4_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 5 control register (BASE_SPIFI_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_SPIFI_CLK_PD               (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_SPIFI_CLK_AUTOBLOCK        (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_SPIFI_CLK_CLKSEL_SHIFT     (24)      /* Bits 24-28: Clock source selection */
#define BASE_SPIFI_CLK_CLKSEL_MASK      (31 << BASE_SPIFI_CLK_CLKSEL_SHIFT)
#  define BASE_SPIFI_CLKSEL_32KHZOSC    (0 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_SPIFI_CLKSEL_IRC         (1 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_SPIFI_CLKSEL_ENET_RXCLK  (2 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_SPIFI_CLKSEL_ENET_TXCLK  (3 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_SPIFI_CLKSEL_GPCLKIN     (4 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_SPIFI_CLKSEL_XTAL        (6 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_SPIFI_CLKSEL_PLL0AUDIO   (8 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_SPIFI_CLKSEL_PLL1        (9 << BASE_SPIFI_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_SPIFI_CLKSEL_IDIVA       (12 << BASE_SPIFI_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_SPIFI_CLKSEL_IDIVB       (13 << BASE_SPIFI_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_SPIFI_CLKSEL_IDIVC       (14 << BASE_SPIFI_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_SPIFI_CLKSEL_IDIVD       (15 << BASE_SPIFI_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_SPIFI_CLKSEL_IDIVE       (16 << BASE_SPIFI_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 6 control register (BASE_SPI_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_SPI_CLK_PD                 (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_SPI_CLK_AUTOBLOCK          (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_SPI_CLK_CLKSEL_SHIFT       (24)      /* Bits 24-28: Clock source selection */
#define BASE_SPI_CLK_CLKSEL_MASK        (31 << BASE_SPI_CLK_CLKSEL_SHIFT)
#  define BASE_SPI_CLKSEL_32KHZOSC      (0 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_SPI_CLKSEL_IRC           (1 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_SPI_CLKSEL_ENET_RXCLK    (2 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_SPI_CLKSEL_ENET_TXCLK    (3 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_SPI_CLKSEL_GPCLKIN       (4 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_SPI_CLKSEL_XTAL          (6 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_SPI_CLKSEL_PLL0AUDIO     (8 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_SPI_CLKSEL_PLL1          (9 << BASE_SPI_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_SPI_CLKSEL_IDIVA         (12 << BASE_SPI_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_SPI_CLKSEL_IDIVB         (13 << BASE_SPI_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_SPI_CLKSEL_IDIVC         (14 << BASE_SPI_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_SPI_CLKSEL_IDIVD         (15 << BASE_SPI_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_SPI_CLKSEL_IDIVE         (16 << BASE_SPI_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 7 control register (BASE_PHY_RX_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_PHYRX_CLK_PD               (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_PHYRX_CLK_AUTOBLOCK        (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_PHYRX_CLK_CLKSEL_SHIFT     (24)      /* Bits 24-28: Clock source selection */
#define BASE_PHYRX_CLK_CLKSEL_MASK      (31 << BASE_PHYRX_CLK_CLKSEL_SHIFT)
#  define BASE_PHYRX_CLKSEL_32KHZOSC    (0 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_PHYRX_CLKSEL_IRC         (1 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_PHYRX_CLKSEL_ENET_RXCLK  (2 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_PHYRX_CLKSEL_ENET_TXCLK  (3 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_PHYRX_CLKSEL_GPCLKIN     (4 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_PHYRX_CLKSEL_XTAL        (6 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_PHYRX_CLKSEL_PLL0AUDIO   (8 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_PHYRX_CLKSEL_PLL1        (9 << BASE_PHYRX_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_PHYRX_CLKSEL_IDIVA       (12 << BASE_PHYRX_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_PHYRX_CLKSEL_IDIVB       (13 << BASE_PHYRX_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_PHYRX_CLKSEL_IDIVC       (14 << BASE_PHYRX_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_PHYRX_CLKSEL_IDIVD       (15 << BASE_PHYRX_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_PHYRX_CLKSEL_IDIVE       (16 << BASE_PHYRX_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 8 control register (BASE_PHY_TX_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_PHYTX_CLK_PD               (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_PHYTX_CLK_AUTOBLOCK        (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_PHYTX_CLK_CLKSEL_SHIFT     (24)      /* Bits 24-28: Clock source selection */
#define BASE_PHYTX_CLK_CLKSEL_MASK      (31 << BASE_PHYTX_CLK_CLKSEL_SHIFT)
#  define BASE_PHYTX_CLKSEL_32KHZOSC    (0 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_PHYTX_CLKSEL_IRC         (1 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_PHYTX_CLKSEL_ENET_RXCLK  (2 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_PHYTX_CLKSEL_ENET_TXCLK  (3 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_PHYTX_CLKSEL_GPCLKIN     (4 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_PHYTX_CLKSEL_XTAL        (6 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_PHYTX_CLKSEL_PLL0AUDIO   (8 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_PHYTX_CLKSEL_PLL1        (9 << BASE_PHYTX_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_PHYTX_CLKSEL_IDIVA       (12 << BASE_PHYTX_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_PHYTX_CLKSEL_IDIVB       (13 << BASE_PHYTX_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_PHYTX_CLKSEL_IDIVC       (14 << BASE_PHYTX_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_PHYTX_CLKSEL_IDIVD       (15 << BASE_PHYTX_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_PHYTX_CLKSEL_IDIVE       (16 << BASE_PHYTX_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 9 control register (BASE_APB1_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_APB1_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_APB1_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_APB1_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_APB1_CLK_CLKSEL_MASK       (31 << BASE_APB1_CLK_CLKSEL_SHIFT)
#  define BASE_APB1_CLKSEL_32KHZOSC     (0 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_APB1_CLKSEL_IRC          (1 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_APB1_CLKSEL_ENET_RXCLK   (2 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_APB1_CLKSEL_ENET_TXCLK   (3 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_APB1_CLKSEL_GPCLKIN      (4 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_APB1_CLKSEL_XTAL         (6 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_APB1_CLKSEL_PLL0AUDIO    (8 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_APB1_CLKSEL_PLL1         (9 << BASE_APB1_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_APB1_CLKSEL_IDIVA        (12 << BASE_APB1_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_APB1_CLKSEL_IDIVB        (13 << BASE_APB1_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_APB1_CLKSEL_IDIVC        (14 << BASE_APB1_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_APB1_CLKSEL_IDIVD        (15 << BASE_APB1_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_APB1_CLKSEL_IDIVE        (16 << BASE_APB1_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 11 control register (BASE_LCD_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_LCD_CLK_PD                 (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_LCD_CLK_AUTOBLOCK          (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_LCD_CLK_CLKSEL_SHIFT       (24)      /* Bits 24-28: Clock source selection */
#define BASE_LCD_CLK_CLKSEL_MASK        (31 << BASE_LCD_CLK_CLKSEL_SHIFT)
#  define BASE_LCD_CLKSEL_32KHZOSC      (0 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_LCD_CLKSEL_IRC           (1 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_LCD_CLKSEL_ENET_RXCLK    (2 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_LCD_CLKSEL_ENET_TXCLK    (3 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_LCD_CLKSEL_GPCLKIN       (4 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_LCD_CLKSEL_XTAL          (6 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_LCD_CLKSEL_PLL0AUDIO     (8 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_LCD_CLKSEL_PLL1          (9 << BASE_LCD_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_LCD_CLKSEL_IDIVA         (12 << BASE_LCD_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_LCD_CLKSEL_IDIVB         (13 << BASE_LCD_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_LCD_CLKSEL_IDIVC         (14 << BASE_LCD_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_LCD_CLKSEL_IDIVD         (15 << BASE_LCD_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_LCD_CLKSEL_IDIVE         (16 << BASE_LCD_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                /* Bits 29-31:  Reserved */
/* Output stage 12 control register (BASE_VADC_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_VADC_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_VADC_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_VADC_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_VADC_CLK_CLKSEL_MASK       (31 << BASE_VADC_CLK_CLKSEL_SHIFT)
#  define BASE_VADC_CLKSEL_32KHZOSC     (0 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_VADC_CLKSEL_IRC          (1 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_VADC_CLKSEL_ENET_RXCLK   (2 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_VADC_CLKSEL_ENET_TXCLK   (3 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_VADC_CLKSEL_GPCLKIN      (4 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_VADC_CLKSEL_XTAL         (6 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_VADC_CLKSEL_PLL0AUDIO    (8 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_VADC_CLKSEL_PLL1         (9 << BASE_VADC_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_VADC_CLKSEL_IDIVA        (12 << BASE_VADC_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_VADC_CLKSEL_IDIVB        (13 << BASE_VADC_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_VADC_CLKSEL_IDIVC        (14 << BASE_VADC_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_VADC_CLKSEL_IDIVD        (15 << BASE_VADC_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_VADC_CLKSEL_IDIVE        (16 << BASE_VADC_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 14 control register (BASE_SSP0_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_SSP0_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_SSP0_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_SSP0_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_SSP0_CLK_CLKSEL_MASK       (31 << BASE_SSP0_CLK_CLKSEL_SHIFT)
#  define BASE_SSP0_CLKSEL_32KHZOSC     (0 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_SSP0_CLKSEL_IRC          (1 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_SSP0_CLKSEL_ENET_RXCLK   (2 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_SSP0_CLKSEL_ENET_TXCLK   (3 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_SSP0_CLKSEL_GPCLKIN      (4 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_SSP0_CLKSEL_XTAL         (6 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_SSP0_CLKSEL_PLL0AUDIO    (8 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_SSP0_CLKSEL_PLL1         (9 << BASE_SSP0_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_SSP0_CLKSEL_IDIVA        (12 << BASE_SSP0_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_SSP0_CLKSEL_IDIVB        (13 << BASE_SSP0_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_SSP0_CLKSEL_IDIVC        (14 << BASE_SSP0_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_SSP0_CLKSEL_IDIVD        (15 << BASE_SSP0_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_SSP0_CLKSEL_IDIVE        (16 << BASE_SSP0_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 15 control register (BASE_SSP1_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_SSP1_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_SSP1_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_SSP1_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_SSP1_CLK_CLKSEL_MASK       (31 << BASE_SSP1_CLK_CLKSEL_SHIFT)
#  define BASE_SSP1_CLKSEL_32KHZOSC     (0 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_SSP1_CLKSEL_IRC          (1 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_SSP1_CLKSEL_ENET_RXCLK   (2 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_SSP1_CLKSEL_ENET_TXCLK   (3 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_SSP1_CLKSEL_GPCLKIN      (4 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_SSP1_CLKSEL_XTAL         (6 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_SSP1_CLKSEL_PLL0AUDIO    (8 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_SSP1_CLKSEL_PLL1         (9 << BASE_SSP1_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_SSP1_CLKSEL_IDIVA        (12 << BASE_SSP1_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_SSP1_CLKSEL_IDIVB        (13 << BASE_SSP1_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_SSP1_CLKSEL_IDIVC        (14 << BASE_SSP1_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_SSP1_CLKSEL_IDIVD        (15 << BASE_SSP1_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_SSP1_CLKSEL_IDIVE        (16 << BASE_SSP1_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                /* Bits 29-31:  Reserved */
/* Output stage 16 control register (BASE_USART0_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_USART0_CLK_PD              (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_USART0_CLK_AUTOBLOCK       (1 << 11) /* Bit 11: Block clock during frequency change */
                                                 /* Bits 12-23: Reserved */
#define BASE_USART0_CLK_CLKSEL_SHIFT    (24)      /* Bits 24-28: Clock source selection */
#define BASE_USART0_CLK_CLKSEL_MASK     (31 << BASE_USART0_CLK_CLKSEL_SHIFT)
#  define BASE_USART0_CLKSEL_32KHZOSC   (0 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_USART0_CLKSEL_IRC        (1 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_USART0_CLKSEL_ENET_RXCLK (2 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_USART0_CLKSEL_ENET_TXCLK (3 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_USART0_CLKSEL_GPCLKIN    (4 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_USART0_CLKSEL_XTAL       (6 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_USART0_CLKSEL_PLL0AUDIO  (8 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_USART0_CLKSEL_PLL1       (9 << BASE_USART0_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_USART0_CLKSEL_IDIVA      (12 << BASE_USART0_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_USART0_CLKSEL_IDIVB      (13 << BASE_USART0_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_USART0_CLKSEL_IDIVC      (14 << BASE_USART0_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_USART0_CLKSEL_IDIVD      (15 << BASE_USART0_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_USART0_CLKSEL_IDIVE      (16 << BASE_USART0_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                /* Bits 29-31:  Reserved */
/* Output stage 17 control register (BASE_UART1_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_UART1_CLK_PD               (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_UART1_CLK_AUTOBLOCK        (1 << 11) /* Bit 11: Block clock during frequency change */
                                                 /* Bits 12-23: Reserved */
#define BASE_UART1_CLK_CLKSEL_SHIFT     (24)      /* Bits 24-28: Clock source selection */
#define BASE_UART1_CLK_CLKSEL_MASK      (31 << BASE_UART1_CLK_CLKSEL_SHIFT)
#  define BASE_UART1_CLKSEL_32KHZOSC    (0 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_UART1_CLKSEL_IRC         (1 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_UART1_CLKSEL_ENET_RXCLK  (2 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_UART1_CLKSEL_ENET_TXCLK  (3 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_UART1_CLKSEL_GPCLKIN     (4 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_UART1_CLKSEL_XTAL        (6 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_UART1_CLKSEL_PLL0AUDIO   (8 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_UART1_CLKSEL_PLL1        (9 << BASE_UART1_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_UART1_CLKSEL_IDIVA       (12 << BASE_UART1_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_UART1_CLKSEL_IDIVB       (13 << BASE_UART1_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_UART1_CLKSEL_IDIVC       (14 << BASE_UART1_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_UART1_CLKSEL_IDIVD       (15 << BASE_UART1_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_UART1_CLKSEL_IDIVE       (16 << BASE_UART1_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                /* Bits 29-31:  Reserved */
/* Output stage 18 control register (BASE_USART2_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_USART2_CLK_PD              (1 << 0)  /* Bit 0:  Output stage power down */
                                                 /* Bits 1-10: Reserved */
#define BASE_USART2_CLK_AUTOBLOCK       (1 << 11) /* Bit 11: Block clock during frequency change */
                                                 /* Bits 12-23: Reserved */
#define BASE_USART2_CLK_CLKSEL_SHIFT    (24)      /* Bits 24-28: Clock source selection */
#define BASE_USART2_CLK_CLKSEL_MASK     (31 << BASE_USART2_CLK_CLKSEL_SHIFT)
#  define BASE_USART2_CLKSEL_32KHZOSC   (0 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_USART2_CLKSEL_IRC        (1 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_USART2_CLKSEL_ENET_RXCLK (2 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_USART2_CLKSEL_ENET_TXCLK (3 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_USART2_CLKSEL_GPCLKIN    (4 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_USART2_CLKSEL_XTAL       (6 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_USART2_CLKSEL_PLL0AUDIO  (8 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_USART2_CLKSEL_PLL1       (9 << BASE_USART2_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_USART2_CLKSEL_IDIVA      (12 << BASE_USART2_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_USART2_CLKSEL_IDIVB      (13 << BASE_USART2_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_USART2_CLKSEL_IDIVC      (14 << BASE_USART2_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_USART2_CLKSEL_IDIVD      (15 << BASE_USART2_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_USART2_CLKSEL_IDIVE      (16 << BASE_USART2_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                /* Bits 29-31:  Reserved */
/* Output stage 19 control register (BASE_USART3_CLK) */
/* NOTE: Clocks 4-19 are identical */

#define BASE_USART3_CLK_PD              (1 << 0)  /* Bit 0:  Output stage power down */
                                                 /* Bits 1-10: Reserved */
#define BASE_USART3_CLK_AUTOBLOCK       (1 << 11) /* Bit 11: Block clock during frequency change */
                                                 /* Bits 12-23: Reserved */
#define BASE_USART3_CLK_CLKSEL_SHIFT    (24)      /* Bits 24-28: Clock source selection */
#define BASE_USART3_CLK_CLKSEL_MASK     (31 << BASE_USART3_CLK_CLKSEL_SHIFT)
#  define BASE_USART3_CLKSEL_32KHZOSC   (0 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_USART3_CLKSEL_IRC        (1 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_USART3_CLKSEL_ENET_RXCLK (2 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_USART3_CLKSEL_ENET_TXCLK (3 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_USART3_CLKSEL_GPCLKIN    (4 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_USART3_CLKSEL_XTAL       (6 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_USART3_CLKSEL_PLL0AUDIO  (8 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_USART3_CLKSEL_PLL1       (9 << BASE_USART3_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_USART3_CLKSEL_IDIVA      (12 << BASE_USART3_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_USART3_CLKSEL_IDIVB      (13 << BASE_USART3_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_USART3_CLKSEL_IDIVC      (14 << BASE_USART3_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_USART3_CLKSEL_IDIVD      (15 << BASE_USART3_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_USART3_CLKSEL_IDIVE      (16 << BASE_USART3_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                /* Bits 29-31:  Reserved */
/* Output stage 20 control register (BASE_OUT_CLK) */

#define BASE_OUT_CLK_PD                 (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_OUT_CLK_AUTOBLOCK          (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_OUT_CLK_CLKSEL_SHIFT       (24)      /* Bits 24-28: Clock source selection */
#define BASE_OUT_CLK_CLKSEL_MASK        (31 << BASE_OUT_CLK_CLKSEL_SHIFT)
#  define BASE_OUT_CLKSEL_32KHZOSC      (0 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_OUT_CLKSEL_IRC           (1 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_OUT_CLKSEL_ENET_RXCLK    (2 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_OUT_CLKSEL_ENET_TXCLK    (3 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_OUT_CLKSEL_GPCLKIN       (4 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_OUT_CLKSEL_XTAL          (6 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_OUT_CLKSEL_PLL0USB       (7 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* PLL0USB */
#  define BASE_OUT_CLKSEL_PLL0AUDIO     (8 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_OUT_CLKSEL_PLL1          (9 << BASE_OUT_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_OUT_CLKSEL_IDIVA         (12 << BASE_OUT_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_OUT_CLKSEL_IDIVB         (13 << BASE_OUT_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_OUT_CLKSEL_IDIVC         (14 << BASE_OUT_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_OUT_CLKSEL_IDIVD         (15 << BASE_OUT_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_OUT_CLKSEL_IDIVE         (16 << BASE_OUT_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 25 control register (BASE_APLL_CLK) */

#define BASE_APLL_CLK_PD                (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_APLL_CLK_AUTOBLOCK         (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_APLL_CLK_CLKSEL_SHIFT      (24)      /* Bits 24-28: Clock source selection */
#define BASE_APLL_CLK_CLKSEL_MASK       (31 << BASE_APLL_CLK_CLKSEL_SHIFT)
#  define BASE_APLL_CLKSEL_32KHZOSC     (0 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_APLL_CLKSEL_IRC          (1 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_APLL_CLKSEL_ENET_RXCLK   (2 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_APLL_CLKSEL_ENET_TXCLK   (3 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_APLL_CLKSEL_GPCLKIN      (4 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_APLL_CLKSEL_XTAL         (6 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_APLL_CLKSEL_PLL0AUDIO    (8 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_APLL_CLKSEL_PLL1         (9 << BASE_APLL_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_APLL_CLKSEL_IDIVA        (12 << BASE_APLL_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_APLL_CLKSEL_IDIVB        (13 << BASE_APLL_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_APLL_CLKSEL_IDIVC        (14 << BASE_APLL_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_APLL_CLKSEL_IDIVD        (15 << BASE_APLL_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_APLL_CLKSEL_IDIVE        (16 << BASE_APLL_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */
/* Output stage 26/27 control register (BASE_CGU_OUT0/1_CLK) */
/* NOTE: Clocks 26-27 are identical */

#define BASE_CGU_CLK_PD                 (1 << 0)  /* Bit 0:  Output stage power down */
                                                  /* Bits 1-10: Reserved */
#define BASE_CGU_CLK_AUTOBLOCK          (1 << 11) /* Bit 11: Block clock during frequency change */
                                                  /* Bits 12-23: Reserved */
#define BASE_CGU_CLK_CLKSEL_SHIFT       (24)      /* Bits 24-28: Clock source selection */
#define BASE_CGU_CLK_CLKSEL_MASK        (31 << BASE_CGU_CLK_CLKSEL_SHIFT)
#  define BASE_CGU_CLKSEL_32KHZOSC      (0 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* 32 kHz oscillator */
#  define BASE_CGU_CLKSEL_IRC           (1 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* IRC (default) */
#  define BASE_CGU_CLKSEL_ENET_RXCLK    (2 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* ENET_RX_CLK */
#  define BASE_CGU_CLKSEL_ENET_TXCLK    (3 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* ENET_TX_CLK */
#  define BASE_CGU_CLKSEL_GPCLKIN       (4 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* GP_CLKIN */
#  define BASE_CGU_CLKSEL_XTAL          (6 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* Crystal oscillator */
#  define BASE_CGU_CLKSEL_PLL0USB       (7 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* PLL0USB */
#  define BASE_CGU_CLKSEL_PLL0AUDIO     (8 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* PLL0AUDIO */
#  define BASE_CGU_CLKSEL_PLL1          (9 << BASE_CGU_CLK_CLKSEL_SHIFT)  /* PLL1 */
#  define BASE_CGU_CLKSEL_IDIVA         (12 << BASE_CGU_CLK_CLKSEL_SHIFT) /* IDIVA */
#  define BASE_CGU_CLKSEL_IDIVB         (13 << BASE_CGU_CLK_CLKSEL_SHIFT) /* IDIVB */
#  define BASE_CGU_CLKSEL_IDIVC         (14 << BASE_CGU_CLK_CLKSEL_SHIFT) /* IDIVC */
#  define BASE_CGU_CLKSEL_IDIVD         (15 << BASE_CGU_CLK_CLKSEL_SHIFT) /* IDIVD */
#  define BASE_CGU_CLKSEL_IDIVE         (16 << BASE_CGU_CLK_CLKSEL_SHIFT) /* IDIVE */
                                                 /* Bits 29-31:  Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CGU_H */
