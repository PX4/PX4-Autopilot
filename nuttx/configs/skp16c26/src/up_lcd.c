/************************************************************************************
 * configs/scp16c26/src/up_lcd.c
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

#include "up_arch.h"
#include "up_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_LCD

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* LCD dimensions *******************************************************************/

#define LCD_NLINES       2          /* Two lines */
#define LCD_NCHARS       8          /* Eight characters per line */

/* LCD commands *********************************************************************/

#define LCD_CLEAR        0x01       /* Clear LCD display and home cursor */
#define CURSOR_MODE_DEC  0x04       /* Cursor auto decrement after R/W */
#define CURSOR_MODE_INC  0x06       /* Cursor auto increment after R/W */
#define LCD_CURSOR_ON    0x0e       /* Display ON with Cursor */
#define LCD_CURSOR_OFF   0x0c       /* Display ON with Cursor off */
#define LCD_CURSOR_BLINK 0x0d       /* Display on with blinking cursor */
#define LCD_CURSOR_LEFT  0x10       /* Move Cursor Left One Position */
#define LCD_CURSOR_RIGHT 0x14       /* Move Cursor Right One Position */
#define FUNCTION_SET     0x28       /* Setup, 4 bits,2 lines, 5X7 */
#define LCD_CGRAM        0x40       /* Map characters to CG RAM */
#define LCD_POS_L1(p)    (0x80 | p) /* Move cursor to line 1, character p+1 */
#define LCD_POS_L2(p)    (0xc0 | p) /* Move cursor to line 2, character p+1 */
#define LCD_HOME_L1      0x80       /* Move cursor to line 1 */
#define LCD_HOME_L2      0xc0       /* Move cursor to line 2 */

/************************************************************************************
 * Private Data
 ************************************************************************************/

static uint8_t g_nchars;              /* Number of characters in lines 2 */
static uint8_t g_line[LCD_NCHARS];    /* The content of lines 2 */

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_lcddelay
 ************************************************************************************/

static void up_lcddelay(uint16_t count)
{
  uint32_t counter = (uint16_t)count << 8;
  while(counter--)
    {
      asm("\tnop\n\tnop\n\tnop\n" : :); /* 3 NOPs */
    }
}

/************************************************************************************
 * Name: up_setrs
 ************************************************************************************/

static inline void up_setrs(bool data)
{
  /* Set/clear bit 1 of port 6 */

  register uint8_t regval = getreg8(M16C_P6);
  if (data)
    {
      regval |= (1 << 0);  /* High = data */
    }
  else
    {
      regval &= ~(1 << 0); /* Low = control */
    }
  putreg8(regval, M16C_P6);
}

/************************************************************************************
 * Name: up_seten
 ************************************************************************************/

static inline void up_seten(void)
{
  /* Set bit 1 of port 6 */

  register uint8_t regval = getreg8(M16C_P6);
  regval = (1 << 1);
  putreg8(regval, M16C_P6);
}

/************************************************************************************
 * Name: up_clren
 ************************************************************************************/

static inline void up_clren(void)
{
  /* Clear bit 1 of port 6 */

  register uint8_t regval = getreg8(M16C_P6);
  regval &= ~(1 << 1);
  putreg8(regval, M16C_P6);
}

/************************************************************************************
 * Name: up_enpluse
 ************************************************************************************/

static inline void up_enpulse(bool data)
{
  up_seten();                 /* EN enable chip (HIGH) */
  up_lcddelay(0);             /* Short delay */
  up_clren();                 /* Latch data by setting EN low */
  up_lcddelay(0);             /* Short delay for data writes */
  if (!data) up_lcddelay(0);  /* Longer delay for control writes */
}

/************************************************************************************
 * Name: up_lcdwrite
 ************************************************************************************/

void up_lcdwrite(bool data, uint8_t ch)
{
  up_setrs(data);				/* Set RS appropriately */

  /* Write upper nibble first.  Only the lower 4 bits of P9 are valid.  The upper four
   * bits are reserved and must be zero.
   */

  putreg8(ch >> 4, M16C_P9);
  up_enpulse(data);

  /* Write lower nibble second */

  putreg8(ch & 0x0f, M16C_P9);
  up_enpulse(data);
}

/************************************************************************************
 * Name: up_scroll
 ************************************************************************************/

static void up_scroll(void)
{
  int i;

  /* Clear the display and position the cursor at the beginning of line 1 */

  up_lcdwrite(false, LCD_CLEAR);
  up_lcdwrite(false, LCD_HOME_L1);

  /* Copy line 2 to line 1 */

  for (i = 0; i < g_nchars; i++)
    {
      up_lcdwrite(true, g_line[i]);
    }

  /* Position the cursor at the beginning of line 2 */

  up_lcdwrite(false, LCD_HOME_L2);
  g_nchars = 0;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_lcdinit
 ************************************************************************************/

void up_lcdinit(void)
{
  uint8_t regval;

  /* Enable writing to PD9 by selecting bit 2 in the protection register */

  regval = getreg8(M16C_PRCR);
  regval |= (1 << 2);
  putreg8(regval, M16C_PRCR);

  /* We can't read PD9, so we can't OR the values in */

  putreg8(0x0f, M16C_PD9);

  /* Set EN (port 6 bit 1) and Set RS (port 6 bit 0) as outputs */

  regval = getreg8(M16C_P6);
  regval |= (1 << 1) | (1 << 0);
  putreg8(regval, M16C_P6);

  regval = getreg8(M16C_PD6);
  regval |= (1 << 1) | (1 << 0);
  putreg8(regval, M16C_PD6);

  /* Set EN low */

  up_clren();

  /* Write the reset sequence */

  up_lcdwrite(false, 0x33);
  up_lcddelay(20);
  up_lcdwrite(false, 0x32);
  up_lcddelay(20);
  up_lcdwrite(false, FUNCTION_SET);	/* reset sequence */
  up_lcdwrite(false, FUNCTION_SET);
  up_lcdwrite(false, LCD_CURSOR_OFF);
  up_lcdwrite(false, LCD_CLEAR);
  up_lcdwrite(false, LCD_HOME_L1);
}

/************************************************************************************
 * Name: up_lcdputc
 ************************************************************************************/

void up_lcdputc(char ch)
{
  /* Check for new line */

  if (ch == '\n')
    {
      up_scroll();
    }

  /* Should we wrap to truncate at the end of line???  Let's truncate.  In either
   * case, let's ignore all other non-printable characters.
   */

  else if (g_nchars < LCD_NCHARS && isprint(ch))
    {
      up_lcdwrite(true, ch);
      g_line[g_nchars] = ch;
      g_nchars++;
    }
}

#endif /* CONFIG_ARCH_LCD */
