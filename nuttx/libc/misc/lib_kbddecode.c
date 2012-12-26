/********************************************************************************************
 * libc/msic/lib_kbddecode.c
 * Decoding side of the Keyboard CODEC
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/streams.h>
#include <nuttx/ascii.h>
#include <nuttx/input/kbd_codec.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

#define NDX_ESC       0
#define NDX_BRACKET   1
#define NDX_CODE      2
#define NCX_SEMICOLON 3

#define NCH_ESC       1
#define NCH_BRACKET   2
#define NCH_CODE      3
#define NCH_SEMICOLON 4

/********************************************************************************************
 * Private Functions
 ********************************************************************************************/

/****************************************************************************
 * Name: kbd_reget
 *
 * Description:
 *   We have unused characters from the last, unsuccessful.  Return one of
 *   these instead of the .
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   pch - The location character to save the returned value.  This may be
 *     either a normal, "in-band" ASCII characer or a special, "out-of-band"
 *     command.
 *   state - A user provided buffer to support parsing.  This structure
 *     should be cleared the first time that kbd_get is called.
 *
 * Returned Value:
 *   2  - Indicates the successful receipt of a special, "out-of-band" command
 *   1  - Indicates the successful receipt of normal, "in-band" ASCII data.
 *   0  - Indicates end-of-file or that the stream has been closed
 *  EOF - An error has getting the next character (reported by the stream).
 *
 ****************************************************************************/

static int kbd_reget(FAR struct kbd_getstate_s *state, FAR uint8_t *pch)
{
  /* Return the next character */

  *pch = state->buf[state->ndx];
   state->ndx++;
   state->nch--;
   return KBD_NORMAL;
}

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

/****************************************************************************
 * Name: kbd_get
 *
 * Description:
 *   Get one byte of data or special command from the driver provided input
 *   buffer.
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   pch - The location character to save the returned value.  This may be
 *     either a normal, "in-band" ASCII characer or a special, "out-of-band"
 *     command.
 *   state - A user provided buffer to support parsing.  This structure
 *     should be cleared the first time that kbd_get is called.
 *
 * Returned Value:
 *   1  - Indicates the successful receipt of a special, "out-of-band" command.
 *        The returned value in pch is a value from enum kbd_getstate_s.
 *   0  - Indicates the successful receipt of normal, "in-band" ASCII data.
 *        The returned value in pch is a simple byte of text or control data.
 *  EOF - An error has getting the next character (reported by the stream).
 *        Normally indicates the end of file.
 *
 ****************************************************************************/

int kbd_get(FAR struct lib_instream_s *stream,
            FAR struct kbd_getstate_s *state, FAR uint8_t *pch)
{
  int ch;

  DEBUGASSERT(stream && state && pch);

  /* Are their ungotten characters from the last, failed parse? */

  if (state->nch > 0)
    {
      /* Yes, return the next ungotten character */

      return kbd_reget(state, pch);
    }

  state->ndx = 0;

  /* No, ungotten characters.  Check for the beginning of an esc sequence. */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream */

      return KBD_ERROR;
    }
  else
    {
      state->buf[NDX_ESC] = (uint8_t)ch;
      state->nch = NCH_ESC;

      if (ch != ASCII_ESC)
        {
          /* Not the beginning of an escape sequence.  Return the character. */

          return kbd_reget(state, pch);
        }
    }

  /* Check for ESC-[ */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream.  Return the escape character now.  We will
       * return the EOF indication next time.
       */

      return kbd_reget(state, pch);
    }
  else
    {
      state->buf[NDX_BRACKET] = ch;
      state->nch = NCH_BRACKET;

      if (ch != '[')
        {
          /* Not the beginning of an escape sequence.  Return the ESC now,
           * return the following character later.
           */

          return kbd_reget(state, pch);
        }
    }

  /* Get and verify the special, "out-of-band" command code */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream.  Unget everything and return the ESC character.
       */

      return kbd_reget(state, pch);
    }
  else
    {
      state->buf[NDX_CODE] = (uint8_t)ch;
      state->nch = NCH_CODE;

      /* Check for a valid special command code */

      if (ch < FIRST_KEYCODE || ch > LAST_KEYCODE)
        {
          /* Not a special command code, return the ESC now and the next two
           * characters later.
           */

          return kbd_reget(state, pch);
        }
    }

  /* Check for the final semicolon */

  ch = stream->get(stream);
  if (ch == EOF)
    {
      /* End of file/stream.  Unget everything and return the ESC character.
       */

      return kbd_reget(state, pch);
    }
  else
    {
      state->buf[NCX_SEMICOLON] = (uint8_t)ch;
      state->nch = NCH_SEMICOLON;

      /* Check for a valid special command code */

      if (ch != ';')
        {
          /* Not a special command code, return the ESC now and the next two
           * characters later.
           */

          return kbd_reget(state, pch);
        }
    }

  /* We have successfully parsed the the entire escape sequence.  Return the
   * special code in pch and the value 2.
   */

  *pch = state->buf[NDX_CODE];
  state->nch = 0;
  return KBD_SPECIAL;
}

