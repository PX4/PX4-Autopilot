/****************************************************************************
 * Driver for Calypso keypad hardware
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
 *   Author: Stefan Richter <ichgeh@l--putt.de>
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

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>

#include <stdint.h>
#include <semaphore.h>
#include <errno.h>
#include <unistd.h>
#include <sched.h>

#include <arch/calypso/defines.h>
#include <arch/calypso/memory.h>
#include <arch/calypso/timer.h>

/****************************************************************************
 * HW access
 ****************************************************************************/

#define BASE_ADDR_ARMIO 0xfffe4800
#define ARMIO_REG(x)    ((void *)BASE_ADDR_ARMIO + (x))

enum armio_reg
{
  LATCH_IN = 0x00,
  LATCH_OUT = 0x02,
  IO_CNTL = 0x04,
  CNTL_REG = 0x06,
  LOAD_TIM = 0x08,
  KBR_LATCH_REG = 0x0a,
  KBC_REG = 0x0c,
  BUZZ_LIGHT_REG = 0x0e,
  LIGHT_LEVEL = 0x10,
  BUZZER_LEVEL = 0x12,
  GPIO_EVENT_MODE = 0x14,
  KBD_GPIO_INT = 0x16,
  KBD_GPIO_MASKIT = 0x18,
  GPIO_DEBOUNCING = 0x1a,
  GPIO_LATCH = 0x1c,
};

#define KBD_INT     (1 << 0)
#define GPIO_INT    (1 << 1)

/****************************************************************************
 * Decoder functions for matrix and power button
 ****************************************************************************/

static int btn_dec(uint32_t * btn_state, uint8_t col, uint8_t reg,
                   char *buf, size_t buflen, size_t * len)
{
  uint8_t diff = (*btn_state ^ reg) & 0x1f;

  while (diff)
    {
      uint8_t val = diff & ~(diff - 1);
      uint8_t sc = val >> 1;
      sc |= sc << 2;
      sc += col;
      sc += (sc & 0x20) ? 0x26 : 0x3f;

      if (reg & val)
        {
          sc |= 0x20;
        }

      /* Check for space in buffer and dispatch */

      if (*len < buflen)
        {
          buf[(*len)++] = sc;
        }
      else
        {
          break;
        }

      /* Only change diff if dispatched/buffer not full */

      diff ^= val;
    }

  /* Store new state of the buttons (but only if they where dispatch) */

  *btn_state >>= 5;
#ifdef INCLUDE_ALL_COLS
  *btn_state |= (reg ^ diff) << 20;
#else
  *btn_state |= (reg ^ diff) << 15;
#endif
  return diff;
}

static int pwr_btn_dec(uint32_t * state, uint8_t reg, char *buf, size_t * len)
{
  if (reg)
    {
      /* Check for pressed power button. If pressed, ignore other
       * buttons since it collides with an entire row.
       */

      if (~*state & 0x80000000)
        {
          buf[0] = 'z';
          *len = 1;
          *state |= 0x80000000;
        }

        return 1; // break loop in caller
    }
  else
    {
      /* Check for released power button. */

      if (*state & 0x80000000)
        {
          buf[0] = 'Z';
          *len = 1;

          *state &= 0x7fffffff;

          /* Don't scan others when released; might trigger
           * false keystrokes otherwise
           */

           return 1;
        }
    }

  return 0; // continue with other columns
}

/****************************************************************************
 * Keypad: Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int keypad_open(file_t * filep);
static int keypad_close(file_t * filep);
static ssize_t keypad_read(file_t * filep, FAR char *buffer, size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int keypad_poll(file_t * filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations keypad_ops =
{
  keypad_open,                  /* open */
  keypad_close,                 /* close */
  keypad_read,                  /* read */
  0,                            /* write */
  0,                            /* seek */
  0,                            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  keypad_poll                   /* poll */
#endif
};

static sem_t kbdsem;

/****************************************************************************
 * Keypad: Fileops
 ****************************************************************************/

static int keypad_open(file_t * filep)
{
  register uint16_t reg;

  /* Unmask keypad interrupt */

  reg = readw(ARMIO_REG(KBD_GPIO_MASKIT));
  writew(reg & ~KBD_INT, ARMIO_REG(KBD_GPIO_MASKIT));

  return OK;
}

static int keypad_close(file_t * filep)
{
  register uint16_t reg;

  /* Mask keypad interrupt */

  reg = readw(ARMIO_REG(KBD_GPIO_MASKIT));
  writew(reg | KBD_INT, ARMIO_REG(KBD_GPIO_MASKIT));

  return OK;
}

static ssize_t keypad_read(file_t * filep, FAR char *buf, size_t buflen)
{
  static uint32_t btn_state = 0;
  register uint16_t reg;
  uint16_t col, col_mask;
  size_t len = 0;

  if (buf == NULL || buflen < 1)
    {
      /* Well... nothing to do */

      return -EINVAL;
    }

retry:
  col = 1;
  col_mask = 0x1e;

  if (!btn_state)
    {
      /* Drive all cols low such that all buttons cause events */

      writew(0, ARMIO_REG(KBC_REG));

      /* No button currently pressed, use IRQ */

      reg = readw(ARMIO_REG(KBD_GPIO_MASKIT));
      writew(reg & ~KBD_INT, ARMIO_REG(KBD_GPIO_MASKIT));
      sem_wait(&kbdsem);
    }
  else
    {
      writew(0x1f, ARMIO_REG(KBC_REG));
      usleep(80000);
    }

  /* Scan columns */

#ifdef INCLUDE_ALL_COLS
  while (col <= 6)
    {
#else
  while (col <= 5)
    {
#endif
      /* Read keypad latch and immediately set new column since
       * synchronization takes about 5usec. For the 1st round, the
       * interrupt has prepared this and the context switch takes
       * long enough to serve as a delay.
       */

      reg = readw(ARMIO_REG(KBR_LATCH_REG));
      writew(col_mask, ARMIO_REG(KBC_REG));

      /* Turn pressed buttons into 1s */

      reg = 0x1f & ~reg;

      if (col == 1)
        {
          /* Power/End switch */

          if (pwr_btn_dec(&btn_state, reg, buf, &len))
            {
              break;
            }
        }
      else
        {
          /* Non-power switches */

          if (btn_dec(&btn_state, col, reg, buf, buflen, &len))
            {
              break;
            }
        }

      /* Select next column and respective mask */

      col_mask = 0x1f & ~(1 << col++);

      /* We have to wait for synchronization of the inputs. The
       * processing is too fast if no/few buttons are processed.
       */

      usleep(5);

      /* XXX: usleep seems to suffer hugh overhead. Better this!?
       * If nothing else can be done, it's overhead still wastes
       * time 'usefully'.
       */
      /* sched_yield(); up_udelay(2); */
    }

  /* If we don't have anything to return, retry to avoid EOF */

  if (!len)
    {
      goto retry;
    }

  return len;
}

/****************************************************************************
 * Keypad interrupt handler
 *   mask interrupts
 *   prepare column drivers for scan
 *   posts keypad semaphore
 ****************************************************************************/

inline int calypso_kbd_irq(int irq, uint32_t * regs)
{
  register uint16_t reg;

  /* Mask keypad interrupt */

  reg = readw(ARMIO_REG(KBD_GPIO_MASKIT));
  writew(reg | KBD_INT, ARMIO_REG(KBD_GPIO_MASKIT));

  /* Turn off column drivers */

  writew(0x1f, ARMIO_REG(KBC_REG));

  /* Let the userspace know */

  sem_post(&kbdsem);

  return 0;
}

/****************************************************************************
 * Initialize device, add /dev/... nodes
 ****************************************************************************/

void up_keypad(void)
{
  /* Semaphore; helps leaving IRQ ctx as soon as possible */

  sem_init(&kbdsem, 0, 0);

  /* Drive cols low in idle state such that all buttons cause events */

  writew(0, ARMIO_REG(KBC_REG));

  (void)register_driver("/dev/keypad", &keypad_ops, 0444, NULL);
}

int keypad_kbdinit(void)
{
  calypso_armio();
  up_keypad();

  return OK;
}
