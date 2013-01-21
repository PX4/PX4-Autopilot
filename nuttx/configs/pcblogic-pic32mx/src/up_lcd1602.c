/****************************************************************************
 * configs/pcblocic-pic32mx/src/up_lcd1602.c
 *
 * This logic supports the connection of an LCD1602 LCD to the
 * STM32F4Discovery board.  The LCD1602 is based on the Hitachi HD44780U LCD
 * controller
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

 /* LCD pin mapping (see configs/pcblogic-pic32mx/README.txt)
 *
 *  ----------------------------------- ---------- ----------------------------------
 *  PIC32                               LCD1602    UBW32 PIN
 *  PIN  SIGNAL NAME                    PIN NAME(s)
 *  ----------------------------------- ---------- ----------------------------------
 *                                      1.  Vss    GND
 *                                      2.  Vdd    Vcc (5V)
 *                                      3.  Vee    To ground via 10K potentiometer
 *    4  AN15/OCFB/PMALL/PMA0/CN12/RB15 4.  RS     PMA0, Selects registers
 *   82  PMRD/CN14/RD5                  5.  RW     PMRD/PMWR, Selects read or write
 *   81  OC5/PMWR/CN13/RD4              6.  E      PMENB, Starts data read/write
 *   93  PMD0/RE0                       7.  D0     PMD0
 *   94  PMD1/RE1                       8.  D1     PMD1
 *   98  PMD2/RE2                       9.  D2     PMD2
 *   99  PMD3/RE3                       10. D3     PMD3
 *  100  PMD4/RE4                       11. D4     PMD4
 *    3  PMD5/RE5                       12. D5     PMD5
 *    4  PMD6/RE6                       13. D6     PMD6
 *    5  PMD7/RE7                       14. D7     PMD7
 *                                      15. A      To Vcc (5V) via 10K potentiometer
 *                                      16. K      GND
 *  ----------------------------------- ---------- ----------------------------------
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/lcd/hd4478ou.h>

#include "up_arch.h"
#include "pic32mx-pmp.h"
#include "pic32mx-int.h"
#include "pic32mx-internal.h"
#include "pcblogic-internal.h"

#ifdef CONFIG_LCD_LCD1602

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_PIC32MX_PMP
#  error "CONFIG_PIC32MX_PMP is required to use the LCD"
#endif

/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug must
 * also be enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

/* Pin configuratin *********************************************************/
/* RB15, RS -- High values selects data */

#define GPIO_LCD_RS   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTB|GPIO_PIN15)

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg         dbg
#  define lcdvdbg        vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

struct lpc1620_s
{
  bool initialized; /* True: Completed initialization sequence */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static ssize_t lcd_read(FAR struct file *, FAR char *, size_t);
static ssize_t lcd_write(FAR struct file *, FAR const char *, size_t);
#ifndef CONFIG_DISABLE_POLL
static int     lcd_poll(FAR struct file *filp, FAR struct pollfd *fds,
                            bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the driver state structure (there is no retained state information) */

static const struct file_operations g_lcd1602 =
{
  0,             /* open */
  0,             /* close */
  lcd_read,      /* read */
  lcd_write,     /* write */
  0,             /* seek */
  0              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , lcd_poll     /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcd_wrcommand
 *
 * Description:
 *   Configure to write an LCD command
 *
 ****************************************************************************/

static void lcd_wrcommand(uint8_t cmd)
{
  /* Address bit A0 is RS.  Set the address latch to A0=0 */

  putreg32(1, PIC32MX_PMP_ADDRCLR);

  /* And write the command to the data out register */

  putreg32((uint32_t)cmd, PIC32MX_PMP_DOUT);
}

/****************************************************************************
 * Name: lcd_wrdata
 *
 * Description:
 *   Configure to read or write LCD data
 *
 ****************************************************************************/

static void lcd_wrdata(uint8_t data)
{
  /* Address bit A0 is RS.  Set the address latch to A0=1 */

  putreg32(1, PIC32MX_PMP_ADDRSET);

  /* And write the data to the data out register */

  putreg32((uint32_t)data, PIC32MX_PMP_DOUT);
}

/****************************************************************************
 * Name: lcd_rddata
 *
 * Description:
 *   Configure to read or write LCD data
 *
 ****************************************************************************/

static uint8_t lcd_rddata(void)
{
  /* Address bit A0 is RS.  Set the address latch to A0=1 */

  putreg32(1, PIC32MX_PMP_ADDRSET);

  /* And read the data to the data in register */

  return (uint8_t)getreg32(PIC32MX_PMP_DIN);
}

/****************************************************************************
 * Name: lcd_read
 ****************************************************************************/

static ssize_t lcd_read(FAR struct file *filp, FAR char *buffer, size_t len)
{
  int i;

  for (i = 0; i < len; i++)
    {
     *buffer++ = lcd_rddata();
    }

  return len;
}

/****************************************************************************
 * Name: lcd_write
 ****************************************************************************/

static ssize_t lcd_write(FAR struct file *filp, FAR const char *buffer, size_t len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      uint8_t data = *buffer++;
      lcd_wrdata(data);
    }

  return len;
}

/****************************************************************************
 * Name: lcd_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int lcd_poll(FAR struct file *filp, FAR struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      /* Data is always avaialble to be read */

      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  up_lcd1602_initialize
 *
 * Description:
 *   Initialize the LCD1602 hardware and register the character driver.
 *
 ****************************************************************************/

int up_lcd1602_initialize(void)
{
  uint32_t regval;
  int ret = OK;

  /* Only initialize the driver once. */

  if (!g_lcd1602.initialized)
    {
      lcdvdbg("Initializing\n");

      /* PMP Master mode configuration */
      /* Make sure that interrupts are disabled */

      putreg32(INT_PMP, PIC32MX_INT_IEC1CLR);

      /* Stop and reset the PMP module and clear the mode and control registers. */

      putreg32(0, PIC32MX_PMP_MODE);
      putreg32(0, PIC32MX_PMP_AEN);
      putreg32(0, PIC32MX_PMP_CON);
      putreg32(0, PIC32MX_PMP_ADDR);

      /* Set LCD timing values, PMP master mode 3, 8-bit mode, no address
       * increment, and no interrupts.
       */

      regval = (PMP_MODE_WAITE_RD(0) | PMP_MODE_WAITM(3) | PMP_MODE_WAITB_1TPB |
                PMP_MODE_MODE_MODE1 | PMP_MODE_MODE8 | PMP_MODE_INCM_NONE |
                PMP_MODE_IRQM_NONE);
      putreg32(regval, PIC32MX_PMP_MODE);

      /* Enable the PMP for reading and writing
       *   PMRD/PMWR is active high (1=RD; 0=WR)
       *   PMENB is active high.
       *   No chip selects
       *   Address latch is active high
       *   Enable PMRD/PMWR, PMENB, and the PMP.
       */

      
      regval = (PMP_CON_RDSP | PMP_CON_WRSP | PMP_CON_ALP |
                PMP_CON_CSF_ADDR1415 | PMP_CON_PTRDEN | PMP_CON_PTWREN |
                PMP_CON_ADRMUX_NONE | PMP_CON_ON);
      putreg32(regval, PIC32MX_PMP_CON);

      /* Configure and enable the LCD */
      /* Wait > 15 milliseconds afer Vdd > 4.5V */

      up_mdelay(100);

      /* Select the 8-bit interface. BF cannot be checked before this command.
       * This needs to be done a few times with some magic delays.
       */

      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);
      up_mdelay(50);
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);
      up_udelay(50);
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);
      lcd_wrcommand(HD4478OU_FUNC | HD4478OU_FUNC_DL8D | HD4478OU_FUNC_N1);

      /* Configure the display */

      lcd_wrcommand(HD4478OU_DISPLAY);                       /* Display, cursor, and blink off */
      lcd_wrcommand(HD4478OU_CLEAR);                         /* Clear the display */
      lcd_wrcommand(HD4478OU_INPUT | HD4478OU_INPUT_INCR);   /* Increment mode */
      lcd_wrcommand(HD4478OU_DISPLAY | HD4478OU_DISPLAY_ON); /* Display on, cursor and blink off */
      lcd_wrcommand(HD4478OU_DDRAM_AD(0));                   /* Select DDRAM RAM AD=0 */

      /* Register the LCD device driver */

      ret = register_driver("/dev/lcd1602", &g_lcd1602, 0644, &g_lcd1602);
      g_lcd1602.initialized = true;
    }

  return ret;
}

#endif /* CONFIG_LCD_LCD1602 */
