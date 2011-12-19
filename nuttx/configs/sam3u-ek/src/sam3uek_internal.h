/************************************************************************************
 * configs/sam3uek_eval/src/sam3uek_internal.h
 * arch/arm/src/board/sam3uek_internal.n
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SAM3U_EK_SRC_SAM3UEK_INTERNAL_H
#define __CONFIGS_SAM3U_EK_SRC_SAM3UEK_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* External Memory Usage ************************************************************/
/* LCD on CS2 */

#define LCD_BASE     SAM3U_EXTCS2_BASE

/* Touchscreen controller (TSC) */

#define CONFIG_TSC_ADS7843    1   /* ADS7843 present on board */
#define CONFIG_TSC_SPI        0   /* On SPI0 */

/* SAM3U-EK GPIO Pin Definitions ****************************************************/

/* LCD:
 *   LCD Module Pin Out:                         AT91SAM3U PIO:
 *  -------------------------------------------- --------------------------------------
 *   Pin Symbol Function                         LCD            PeriphA  PeriphB Extra
 *  ---- ------ -------------------------------- -------------- -------- ------- ------
 *   1   GND    Ground                           N/A            ---      ---     ---
 *   2   CS     Chip Select                      PC16           NCS2     PWML3   AD12BAD5
 *   3   RS     Register select signal           PB8 (see A1)   CTS0     A1      AD3
 *   4   WR     Write operation signal           PB23 (NWE)     NWR0/NEW PCK1    ---
 *   5   RD     Read operation signal            PB19 (NRD)     NRD      PWML2   ---
 *   6   DB0    Data bus                         PB9            D0       DTR0    ---
 *   7   DB1    Data bus                         PB10           D1       DSR0    ---
 *   8   DB2    Data bus                         PB11           D2       DCD0    ---
 *   9   DB3    Data bus                         PB12           D3       RI0     ---
 *   10  DB4    Data bus                         PB13           D4       PWMH0   ---
 *   11  DB5    Data bus                         PB14           D5       PWMH1   ---
 *   12  DB6    Data bus                         PB15           D6       PWMH2   ---
 *   13  DB7    Data bus                         PB16           D7       PMWH3   ---
 *   14  DB8    Data bus                         PB25           D8       PWML0   ---
 *   15  DB9    Data bus                         PB26           D9       PWML1   ---
 *   16  DB10   Data bus                         PB27           D10      PWML2   ---
 *   17  DB11   Data bus                         PB28           D11      PWML3   ---
 *   18  DB12   Data bus                         PB29           D12      ---     ---
 *   19  DB13   Data bus                         PB30           D13      ---     ---
 *   20  DB14   Data bus                         PB31           D14      ---     ---
 *   21  DB15   Data bus                         PB6            TIOA1    D15     AD1
 *   22  NC     No connection                    N/A            ---      ---     ---
 *   23  NC     No connection                    N/A            ---      ---     ---
 *   24  RESET  Reset signal                     N/A            ---      ---     ---
 *   25  GND    Ground                           N/A            ---      ---     ---
 *   26  X+     Touch panel X_RIGHT              PA15           SPCK     PWMH2   ---
 *   27  Y+     Touch panel Y_UP                 PA14           MOSI     ---     ---
 *   28  X-     Touch panel X_LEFT               PA13           MISO     ---     ---
 *   29  Y-     Touch panel Y_DOWN               PC14           A3       NPCS2   ---
 *   30  GND    Ground                           N/A            ---      ---     ---
 *   31  VDD1   Power supply for digital IO Pad  N/A            ---      ---     ---
 *   32  VDD2   Power supply for analog circuit  N/A            ---      ---     ---
 *   33  A1     Power supply for backlight       PB8 (see RS)   CTS0     A1      AD3
 *   34  A2     Power supply for backlight       N/A            ---      ---     ---
 *   35  A3     Power supply for backlight       N/A            ---      ---     ---
 *   36  A4     Power supply for backlight       N/A            ---      ---     ---
 *   37  NC     No connection                    N/A            ---      ---     ---
 *   38  NC     No connection                    N/A            ---      ---     ---
 *   39  K      Backlight ground                 N/A            ---      ---     ---
 */

#define GPIO_LCD_NCS2 (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|GPIO_PIN16)
#define GPIO_LCD_RS   (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN8)
#define GPIO_LCD_NWE  (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN23)
#define GPIO_LCD_NRD  (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN19)

#define GPIO_LCD_D0   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN9)
#define GPIO_LCD_D1   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN10)
#define GPIO_LCD_D2   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN11)
#define GPIO_LCD_D3   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN12)
#define GPIO_LCD_D4   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN13)
#define GPIO_LCD_D5   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN14)
#define GPIO_LCD_D6   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN15)
#define GPIO_LCD_D7   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN16)
#define GPIO_LCD_D8   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN25)
#define GPIO_LCD_D9   (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN26)
#define GPIO_LCD_D10  (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN27)
#define GPIO_LCD_D11  (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN28)
#define GPIO_LCD_D12  (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN29)
#define GPIO_LCD_D13  (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN30)
#define GPIO_LCD_D14  (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN31)
#define GPIO_LCD_D15  (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN6)

/* LCD Backlight pin definition. */

#define GPIO_LCD_BKL  (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_OUTPUT_CLEAR|GPIO_PORT_PIOC|GPIO_PIN19)

/* Touchscreen controller (TSC) */

#define GPIO_TCS_IRQ  (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN24)
#define GPIO_TCS_BUSY (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN2)

#define SAM3U_TCS_IRQ  SAM3U_IRQ_PA24

/* LEDs */

#define GPIO_LED0     (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_CLEAR|GPIO_PIN0)
#define GPIO_LED1     (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_SET|GPIO_PIN1)
#define GPIO_LED2     (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_SET|GPIO_PIN2)

/* BUTTONS */

#define GPIO_BUTTON1  (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_CFG_DEGLITCH|GPIO_PORT_PIOA|GPIO_PIN18)
#define GPIO_BUTTON2  (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_CFG_DEGLITCH|GPIO_PORT_PIOA|GPIO_PIN19)

#define IRQ_BUTTON1   SAM3U_IRQ_PA18
#define IRQ_BUTTON2   SAM3U_IRQ_PA19

/* SD Card Detect */

#define GPIO_MCI_CD   (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN25)

/* SPI Chip Selects */

/* Chip select pin connected to the touchscreen controller and to the ZigBee module
 * connector.  Notice that the touchscreen chip select is implemented as a GPIO
 * OUTPUT that must be controlled by board-specific.  This is because the ADS7843E
 * driver must be able to sample the device BUSY GPIO input between SPI transfers.
 * However, the AD7843E will tri-state the BUSY input whenever the chip select is
 * de-asserted.  So the only option is to control the chip select manually and hold
 * it low throughout the SPI transfer.
 */

#define GPIO_TSC_NPCS2 (GPIO_OUTPUT|GPIO_CFG_PULLUP|GPIO_OUTPUT_SET|\
                        GPIO_PORT_PIOC|GPIO_PIN14)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam3u_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM3U-EK board.
 *
 ************************************************************************************/

extern void weak_function sam3u_spiinitialize(void);

/************************************************************************************
 * Name: sam3u_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the SAM3U-EK board.
 *
 ************************************************************************************/

extern void weak_function sam3u_usbinitialize(void);

/****************************************************************************
 * Name: sam3u_hsmciinit
 *
 * Description:
 *   Initialize HSMCI support 
 *
 ****************************************************************************/

#ifdef CONFIG_SAM3U_HSMCI
extern int weak_function sam3u_hsmciinit(void);
#else
# define sam3u_hsmciinit()
#endif

/****************************************************************************
 * Name: sam3u_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#ifdef CONFIG_SAM3U_HSMCI
extern bool sam3u_cardinserted(unsigned char slot);
#else
#  define sam3u_cardinserted(slot) (false)
#endif

/****************************************************************************
 * Name: sam3u_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#ifdef CONFIG_SAM3U_HSMCI
extern bool sam3u_writeprotected(unsigned char slot);
#else
#  define sam3u_writeprotected(slot) (false)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAM3U_EK_SRC_SAM3UEK_INTERNAL_H */

