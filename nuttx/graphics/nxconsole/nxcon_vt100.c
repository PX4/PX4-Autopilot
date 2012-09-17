/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_vt100.c
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

#include <string.h>
#include <assert.h>

#include <nuttx/vt100.h>

#include "nxcon_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*seqhandler_t)(FAR struct nxcon_state_s *priv);

struct vt100_sequence_s
{
  FAR const char *seq;
  seqhandler_t handler;
  uint8_t size;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nxcon_erasetoeol(FAR struct nxcon_state_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* All recognized VT100 escape sequences.  Very little as present, this is
 * a placeholder for a future, more complete VT100 emulation.
 */

/* <esc>[K is the VT100 command erases to the end of the line. */

static const char g_erasetoeol[] = VT100_CLEAREOL;

/* The list of all VT100 sequences supported by the emulation */

static const struct vt100_sequence_s g_vt100sequences[] =
{
  {g_erasetoeol, nxcon_erasetoeol, sizeof(g_erasetoeol)},
  {NULL, NULL, 0}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_erasetoeol
 *
 * Description:
 *   Handle the erase-to-eol VT100 escapte sequence
 *
 * Input Parameters:
 *   priv - Driver data structure
 *
 * Returned Value:
 *   The index of the match in g_vt100sequences[]
 *
 ****************************************************************************/

static int nxcon_erasetoeol(FAR struct nxcon_state_s *priv)
{
  /* Does nothing yet (other than consume the sequence) */

  return OK;
}

/****************************************************************************
 * Name: nxcon_vt100part
 *
 * Description:
 *   Return the next entry that is a partial match to the sequence.
 *
 * Input Parameters:
 *   priv - Driver data structure
 *   seqsize - The number of bytes in the sequence
 *   startndx - The index to start searching
 *
 * Returned Value:
 *   A pointer to the matching sequence in g_vt100sequences[]
 *
 ****************************************************************************/

FAR const struct vt100_sequence_s *
nxcon_vt100part(FAR struct nxcon_state_s *priv, int seqsize)
{
  FAR const struct vt100_sequence_s *seq;
  int ndx;

  /* Search from the beginning of the sequence table */

  for (ndx = 0; g_vt100sequences[ndx].seq; ndx++)
    {
      /* Is this sequence big enough? */

      seq = &g_vt100sequences[ndx];
      if (seq->size >= seqsize)
        {
          /* Yes... are the first 'seqsize' bytes the same */

          if (memcmp(seq->seq, priv->seq, seqsize) == 0)
            {
              /* Yes.. return the match */

              return seq;
            }
        }
    }
  return NULL;
}

/****************************************************************************
 * Name: nxcon_vt100seq
 *
 * Description:
 *   Determine if the new sequence is a part of a supported VT100 escape
 *   sequence.
 *
 * Input Parameters:
 *   priv - Driver data structure
 *   seqsize - The number of bytes in the sequence
 *
 * Returned Value:
 *   state - See enum nxcon_vt100state_e;
 *
 ****************************************************************************/

static enum nxcon_vt100state_e nxcon_vt100seq(FAR struct nxcon_state_s *priv,
                                              int seqsize)
{
  FAR const struct vt100_sequence_s *seq;
  enum nxcon_vt100state_e ret;

  /* Is there any VT100 escape sequence that matches what we have
   * buffered so far?
   */

  seq = nxcon_vt100part(priv, seqsize);
  if (seq)
    {
      /* Yes.. if the size of that escape sequence is the same as what we
       * have buffered, then we have an exact match.
       */

      if (seq->size == seqsize)
        {
          /* Process the VT100 sequence */

          seq->handler(priv);
          priv->nseq = 0;
          return VT100_PROCESSED;
        }

      /* The 'seqsize' is still smaller than the potential match(es).  We
       * will need to collect more characters before we can make a decision.
       * Retun an indication that we have consumed the character.
       */

      return VT100_CONSUMED;
    }

  /* We get here on a failure.  The buffer sequence is not part of any
   * supported VT100 escape sequence.  If seqsize > 1 then we need to
   * return a special value because we have to re-process the buffered
   * data.
   */

  ret = seqsize > 1 ? VT100_ABORT : VT100_NOT_CONSUMED;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_vt100
 *
 * Description:
 *   Test if the newly received byte is part of a VT100 escape sequence
 *
 * Input Parameters:
 *   priv - Driver data structure
 *   ch - The newly received character
 *
 * Returned Value:
 *   state - See enum nxcon_vt100state_e;
 *
 ****************************************************************************/

enum nxcon_vt100state_e nxcon_vt100(FAR struct nxcon_state_s *priv, char ch)
{
  enum nxcon_vt100state_e ret;
  int seqsize;

  DEBUGASSERT(priv && priv->nseq < VT100_MAX_SEQUENCE);

  /* If we have no buffered characters, then 'ch' must be the first character
   * of an escape sequence.
   */

  if (priv->nseq < 1)
    {
      /* The first character of an escape sequence must be an an escape
       * character (duh).
       */

      if (ch != ASCII_ESC)
        {
          return VT100_NOT_CONSUMED;
        }

      /* Add the escape character to the buffer but don't bother with any
       * further checking.
       */

      priv->seq[0] = ASCII_ESC;
      priv->nseq   = 1;
      return VT100_CONSUMED;
    }

  /* Temporarily add the next character to the buffer */

  seqsize = priv->nseq;
  priv->seq[seqsize] = ch;

  /* Then check if this sequence is part of an a valid escape sequence */

  seqsize++;
  ret = nxcon_vt100seq(priv, seqsize);
  if (ret == VT100_CONSUMED)
    {
      /* The newly added character is indeed part of a VT100 escape sequence
       * (which is still incomplete).  Keep it in the buffer.
       */

      priv->nseq = seqsize;
    }
  return ret;
}
