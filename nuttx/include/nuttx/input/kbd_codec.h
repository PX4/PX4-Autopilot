/************************************************************************************
 * include/nuttx/input/kbd_codec.h
 * Serialize and marshaling keyboard data and events
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_KBD_CODEC_H
#define __INCLUDE_NUTTX_INPUT_KBD_CODEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/streams.h>

#ifdef CONFIG_LIB_KBDCODEC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These are the special keyboard commands recognized by the CODEC. */

enum kbd_keycode_e
{
  KEYCODE_NORMAL = 0,          /* Not a special keycode */

  /* Delete and Backspace keycodes (in case they may be different than the
   * ASCII BKSP and DEL values.
   */

  KEYCODE_FWDDEL,              /* DELete (forward delete) */
  KEYCODE_BACKDEL,             /* Backspace (backward delete) */

  /* Cursor movement */

  KEYCODE_HOME,                /* Home */
  KEYCODE_END,                 /* End */
  KEYCODE_LEFT,                /* Left arrow */
  KEYCODE_RIGHT,               /* Right arrow */
  KEYCODE_UP,                  /* Up arrow */
  KEYCODE_DOWN,                /* Down arrow */
  KEYCODE_PAGEUP,              /* Page up */
  KEYCODE_PAGEDOWN,            /* Page down */

  /* Edit commands */

  KEYCODE_INSERT,              /* Insert */
  KEYCODE_AGAIN,               /* Again */
  KEYCODE_UNDO,                /* Undo */
  KEYCODE_REDO,                /* Redo */
  KEYCODE_CUT,                 /* Cut */
  KEYCODE_COPY,                /* Copy */
  KEYCODE_PASTE,               /* Paste */
  KEYCODE_FIND,                /* Find */

  /* Selection codes */

  KEYCODE_ENTER,               /* Enter */
  KEYCODE_SELECT,              /* Select */
  KEYCODE_EXECUTE,             /* Execute */

  /* Keyboard modes */

  KEYCODE_CAPSLOCK,            /* Caps Lock */
  KEYCODE_SCROLLLOCK,          /* Scroll Lock */
  KEYCODE_NUMLOCK,             /* Keypad Num Lock and Clear */
  KEYCODE_LCAPSLOCK,           /* Locking Caps Lock */
  KEYCODE_LNUMLOCK,            /* Locking Num Lock */
  KEYCODE_LSCROLLLOCK,         /* Locking Scroll Lock */

  /* Misc control codes */

  KEYCODE_POWER,               /* Power */
  KEYCODE_HELP,                /* Help */
  KEYCODE_MENU,                /* Menu */
  KEYCODE_STOP,                /* Stop */
  KEYCODE_PAUSE,               /* Pause */
  KEYCODE_BREAK,               /* Break */
  KEYCODE_CANCEL,              /* Cancel */
  KEYCODE_PRTSCRN,             /* PrintScreen */
  KEYCODE_SYSREQ,              /* SysReq/Attention */

  /* Audio */

  KEYCODE_MUTE,                /* Mute */
  KEYCODE_VOLUP,               /* Volume Up */
  KEYCODE_VOLDOWN,             /* Volume Down */

  /* Telephone */

  KEYCODE_ANSWER,              /* Answer (phone) */
  KEYCODE_HANGUP,              /* Hang-up (phone) */

  /* Calculator */

  KEYCODE_CLEAR,               /* Clear */
  KEYCODE_CLEARENTRY,          /* Clear entry */
  KEYCODE_NEGATE,              /* +/- */

  KEYCODE_MEMSTORE,            /* Memory store */
  KEYCODE_MEMCLEAR,            /* Memory clear */
  KEYCODE_MEMRECALL,           /* Memory recall */
  KEYCODE_MEMADD,              /* Memory add */
  KEYCODE_MEMSUB,              /* Memory substract */
  KEYCODE_MEMMUL,              /* Memory multiply */
  KEYCODE_MEMDIV,              /* Memory divide */

  KEYCODE_BINARY,              /* Binary mode */
  KEYCODE_OCTAL,               /* Octal mode */
  KEYCODE_DECIMAL,             /* Decimal mode */
  KEYCODE_HEXADECIMAL,         /* Hexadecimal mode */

  /* Languages */

  KEYCODE_LANG1,               /* LANG1 */
  KEYCODE_LANG2,               /* LANG2 */
  KEYCODE_LANG3,               /* LANG3 */
  KEYCODE_LANG4,               /* LANG4 */
  KEYCODE_LANG5,               /* LANG5 */
  KEYCODE_LANG6,               /* LANG6 */
  KEYCODE_LANG7,               /* LANG7 */
  KEYCODE_LANG8,               /* LANG8 */

  /* Context-specific function keys */

  KEYCODE_F1,                  /* Function key 1 */
  KEYCODE_F2,                  /* Function key 2 */
  KEYCODE_F3,                  /* Function key 3 */
  KEYCODE_F4,                  /* Function key 4 */
  KEYCODE_F5,                  /* Function key 5 */
  KEYCODE_F6,                  /* Function key 6 */
  KEYCODE_F7,                  /* Function key 7 */
  KEYCODE_F8,                  /* Function key 8 */
  KEYCODE_F9,                  /* Function key 9 */
  KEYCODE_F10,                 /* Function key 10 */
  KEYCODE_F11,                 /* Function key 11 */
  KEYCODE_F12,                 /* Function key 12 */
  KEYCODE_F13,                 /* Function key 13 */
  KEYCODE_F14,                 /* Function key 14 */
  KEYCODE_F15,                 /* Function key 15 */
  KEYCODE_F16,                 /* Function key 16 */
  KEYCODE_F17,                 /* Function key 17 */
  KEYCODE_F18,                 /* Function key 18 */
  KEYCODE_F19,                 /* Function key 19 */
  KEYCODE_F20,                 /* Function key 20 */
  KEYCODE_F21,                 /* Function key 21 */
  KEYCODE_F22,                 /* Function key 22 */
  KEYCODE_F23,                 /* Function key 23 */
  KEYCODE_F24                  /* Function key 24 */
};

#define FIRST_KEYCODE KEYCODE_FWDDEL
#define LAST_KEYCODE  KEYCODE_F24

/* kbd_decode() return values */

#define KBD_PRESS     0    /* Key press event */
#define KBD_RELEASE   1    /* Key release event */
#define KBD_SPECPRESS 2    /* Special key press event */
#define KBD_SPECREL   3    /* Special key release event */
#define KBD_ERROR     EOF  /* Error or end-of-file */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct kbd_getstate_s
{
  uint8_t nch;                 /* Number of characters in the buffer */
  uint8_t ndx;                 /* Index to next character in the buffer */
  uint8_t buf[4];              /* Buffer of ungotten data */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * The following functions are intended for use by "producer", keyboard
 * or keypad drivers to encode information into driver buffers.
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_press
 *
 * Description:
 *   Indicates a normal key press event.  Put one byte of normal keyboard
 *   data into the output stream.
 *
 * Input Parameters:
 *   ch - The character to be added to the output stream.
 *   stream - An instance of lib_outstream_s to do the low-level put
 *     operation.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define kbd_press(ch, stream) (stream)->put((stream), (int)(ch))

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

void kbd_release(uint8_t ch, FAR struct lib_outstream_s *stream);

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
                   FAR struct lib_outstream_s *stream);

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
                 FAR struct lib_outstream_s *stream);

/****************************************************************************
 * The following functions are intended for use by "consumer" applications
 * to remove and decode information from the driver provided buffer.
 ****************************************************************************/

/****************************************************************************
 * Name: kbd_decode
 *
 * Description:
 *   Get one byte of data or special command from the driver provided input
 *   buffer.
 *
 * Input Parameters:
 *   stream - An instance of lib_instream_s to do the low-level get
 *     operation.
 *   pch - The location to save the returned value.  This may be
 *     either a normal, character code or a special command from enum
 *     kbd_keycode_e
 *   state - A user provided buffer to support parsing.  This structure
 *     should be cleared the first time that kbd_decode is called.
 *
 * Returned Value:
 *
 *  KBD_PRESS  - Indicates the successful receipt of normal, keyboard data.
 *    This corresponds to a keypress event.  The returned value in pch is a
 *    simple byte of text or control data corresponding to the pressed key.
 *  KBD_RELEASE - Indicates a key release event.  The returned value in pch
 *    is the byte of text or control data corresponding to the released key.
 *  KBD_SPECPRESS - Indicates the successful receipt of a special keyboard
 *    command. The returned value in pch is a value from enum kbd_getstate_s.
 *  KBD_SPECREL - Indicates a special key release event.  The returned value
 *    in pch is a value from enum kbd_getstate_s.
 *  EOF - An error has getting the next character (reported by the stream).
 *    Normally indicates the end of file.
 *
 ****************************************************************************/

int kbd_decode(FAR struct lib_instream_s *stream,
               FAR struct kbd_getstate_s *state, FAR uint8_t *pch);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LIB_KBDCODEC */
#endif /* __INCLUDE_NUTTX_INPUT_KBD_CODEC_H */

