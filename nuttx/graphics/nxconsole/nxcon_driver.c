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

const struct file_operations g_nxcon_drvrops =
{
  nxcon_open,  /* open */
  0,           /* close */
  0,           /* read */
  nxcon_write, /* write */
  0,           /* seek */
  0            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0          /* poll */
#endif
};

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

  if ((filep->f_oflags & O_WROK) != 0)
    {
      gdbg("Attempted open with write access\n");
      return -EACCES;
    }

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
  char ch;
  int lineheight;

  /* Recover our private state structure */

  DEBUGASSERT(filep && filep->f_priv);
  priv = (FAR struct nxcon_state_s *)filep->f_priv;

  /* Loop writing each character to the display */

  lineheight = (priv->fheight + CONFIG_NXCONSOLE_LINESEPARATION);

  while (buflen-- > 0)
    {
      /* Will another character fit on this line? */

      if (priv->fpos.x + priv->fwidth > priv->wndo.wsize.w)
        {
          /* No.. move to the next line */

          nxcon_newline(priv);

          /* If we were about to output a newline character, then don't */

          if (*buffer == '\n')
            {
              buffer++;
              continue;
            }
        }

      /* Check if we need to scroll up (handling a corner case where
       * there may be more than one newline).
       */

      while (priv->fpos.y >= priv->wndo.wsize.h - lineheight)
        {
          nxcon_scroll(priv, lineheight);
        }

      /* Ignore carriage returns */

      ch = *buffer++;
      if (ch != '\r')
        {
          /* Finally, we can output the character */

          nxcon_putc(priv, (uint8_t)ch);
        }
    }

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

