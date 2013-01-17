/****************************************************************************
 * libc/msic/lib_kbdencode.c
 * Encoding side of the Keyboard CODEC
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/streams.h>
#include <nuttx/ascii.h>
#include <nuttx/input/kbd_codec.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_encode
 *
 * Description:
 *   Encode one special special sequence command into the output stream.
 *
 * Input Parameters:
 *   keycode - The command to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *   terminator - Escape sequence terminating character.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void kbd_encode(uint8_t keycode, FAR struct lib_outstream_s *stream,
                       uint8_t terminator)
{
  stream->put(stream, ASCII_ESC);
  stream->put(stream, '[');
  stream->put(stream, (int)keycode);
  stream->put(stream, terminator);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_release
 *
 * Description:
 *   Encode the release of a normal key.
 *
 * Input Parameters:
 *   ch - The character associated with the key that was releared.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kbd_release(uint8_t ch, FAR struct lib_outstream_s *stream)
{
  kbd_encode(ch, stream, ('a' + KBD_RELEASE)); 
}

/****************************************************************************
 * Name: kbd_specpress
 *
 * Description:
 *   Denotes a special key press event.  Put one special keyboard command
 *   into the output stream.
 *
 * Input Parameters:
 *   keycode - The command to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kbd_specpress(enum kbd_keycode_e keycode,
                   FAR struct lib_outstream_s *stream)
{
  DEBUGASSERT(stream && keycode >= KEYCODE_FWDDEL && keycode <= LAST_KEYCODE);
  kbd_encode((uint8_t)keycode, stream, ('a' + KBD_SPECPRESS)); 
}

/****************************************************************************
 * Name: kbd_specrel
 *
 * Description:
 *   Denotes a special key release event.  Put one special keyboard
 *   command into the output stream.
 *
 * Input Parameters:
 *   keycode - The command to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kbd_specrel(enum kbd_keycode_e keycode,
                 FAR struct lib_outstream_s *stream)
{
  DEBUGASSERT(stream && keycode >= KEYCODE_FWDDEL && keycode <= LAST_KEYCODE);
  kbd_encode((uint8_t)keycode, stream, ('a' + KBD_SPECREL)); 
}
