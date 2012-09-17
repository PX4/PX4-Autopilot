/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_driver.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "nxcon_internal.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     nxcon_open(FAR struct file *filep);
static ssize_t nxcon_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* This is the common NX driver file operations */

#ifdef CONFIG_NXCONSOLE_NXKBDIN

const struct file_operations g_nxcon_drvrops =
{
  nxcon_open,  /* open */
  0,           /* close */
  nxcon_read,  /* read */
  nxcon_write, /* write */
  0,           /* seek */
  0            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  ,
  nxcon_poll   /* poll */
#endif
};

#else /* CONFIG_NXCONSOLE_NXKBDIN */

const struct file_operations g_nxcon_drvrops =
{
  nxcon_open,  /* open */
  0,           /* close */
  0,           /* read */
  nxcon_write, /* write */
  0,           /* seek */
  0            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  ,
  0            /* poll */
#endif
};

#endif /* CONFIG_NXCONSOLE_NXKBDIN */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_open
 ****************************************************************************/

static int nxcon_open(FAR struct file *filep)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct nxcon_state_s *priv  = inode->i_private;

  DEBUGASSERT(filep && filep->f_inode);

  /* Get the driver structure from the inode */

  inode = filep->f_inode;
  priv  = (FAR struct nxcon_state_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Verify that the driver is opened for write-only access */

#ifndef CONFIG_NXCONSOLE_NXKBDIN
  if ((filep->f_oflags & O_RDOK) != 0)
    {
      gdbg("ERROR: Attempted open with read access\n");
      return -EACCES;
    }
#endif

  /* Assign the driver structure to the file */

  filep->f_priv = priv;
  return OK;
}

/****************************************************************************
 * Name: nxcon_write
 ****************************************************************************/

static ssize_t nxcon_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct nxcon_state_s *priv;
  enum nxcon_vt100state_e state;
  ssize_t remaining;
  char ch;
  int ret;

  /* Recover our private state structure */

  DEBUGASSERT(filep && filep->f_priv);
  priv = (FAR struct nxcon_state_s *)filep->f_priv;

  /* Get exclusive access */

  ret = nxcon_semwait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Hide the cursor while we update the display */

  nxcon_hidecursor(priv);

  /* Loop writing each character to the display */

  for (remaining = (ssize_t)buflen; remaining > 0; remaining--)
    {
      /* Get the next character from the user buffer */

      ch = *buffer++;

      /* Check if this character is part of a VT100 escape sequence */

      do
        {
          /* Is the character part of a VT100 escape sequnce? */

          state = nxcon_vt100(priv, ch);
          switch (state)
            {
              /* Character is not part of a VT100 escape sequence (and no
               * characters are buffer.
               */

              default:
              case VT100_NOT_CONSUMED:
                {
                  /* We can output the character to the window */

                  nxcon_putc(priv, (uint8_t)ch);
                }
              break;

            /* The full VT100 escape sequence was processed (and the new
             * character was consumed)
             */

            case VT100_PROCESSED:

            /* Character was consumed as part of the VT100 escape processing
             * (but the escape sequence is still incomplete.
             */

            case VT100_CONSUMED: 
              {
                /* Do nothing... the VT100 logic owns the character */
              }
              break;

            /* Invalid/unsupported character in escape sequence */

            case VT100_ABORT:
              {
                int i;

                /* Add the first unhandled character to the window */

                nxcon_putc(priv, (uint8_t)priv->seq[0]);

                /* Move all buffer characters down one */

                for (i = 1; i < priv->nseq; i++)
                  {
                    priv->seq[i-1] = priv->seq[i];
                  }
                priv->nseq--;

                /* Then loop again and check if what remains is part of a
                 * VT100 escape sequence.  We could speed this up by
                 * checking if priv->seq[0] == ASCII_ESC.
                 */
              }
              break;
            }
        }
      while (state == VT100_ABORT);
    }

  /* Show the cursor at its new position */

  nxcon_showcursor(priv);
  nxcon_sempost(priv);
  return (ssize_t)buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

