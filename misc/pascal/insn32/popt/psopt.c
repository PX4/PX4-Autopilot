/**********************************************************************
 * psopt.c
 * String Stack Optimizaitons
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 **********************************************************************/

/* The statement generation logic generates a PUSHS and POPS around
 * every statement.  These instructions save and restore the string
 * stack pointer registers.  However, only some statements actually
 * modify the string stack.  So the first major step in the optimatization
 * process is to retain only PUSHS and POPS statements that are
 * actually required.
 */

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "keywords.h"
#include "pdefs.h"
#include "pedefs.h"
#include "pinsn32.h"
#include "pxdefs.h"

#include "popt.h"
#include "psopt.h"

/**********************************************************************
 * Definitions
 **********************************************************************/

#define PBUFFER_SIZE 1024
#define NPBUFFERS       8

/**********************************************************************
 * Private Data
 **********************************************************************/

static uint8_t *pbuffer[NPBUFFERS];
static int      nbytes_in_pbuffer[NPBUFFERS];
static int      current_level = -1;
static int      inch;

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static inline void putbuf(int c, poffProgHandle_t poffProgHandle);
static inline void flushc(int c, poffProgHandle_t poffProgHandle);
static inline void flushbuf(poffProgHandle_t poffProgHandle);
static void dopush(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle);
static void dopop(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle);

/**********************************************************************
 * Private Inline Functions
 **********************************************************************/

static inline void putbuf(int c, poffProgHandle_t poffProgHandle)
{
  int dlvl = current_level;

  if (dlvl < 0)
    {
      /* No PUSHS encountered.  Write byte directly to output */

      poffAddTmpProgByte(poffProgHandle, (uint8_t)c);
    }
  else
    {
      /* PUSHS encountered.  Write byte into buffer associated with
       * nesting level.
       */

      int idx                 = nbytes_in_pbuffer[dlvl];
      uint8_t *dest             = pbuffer[dlvl] + idx;
      *dest                   = c;
      nbytes_in_pbuffer[dlvl] = idx + 1;
    }
}

static inline void flushc(int c, poffProgHandle_t poffProgHandle)
{
  if (current_level > 0)
    {
      /* Nested PUSHS encountered. Write byte into buffer associated
       * with the previous nesting level.
       */

      int dlvl                = current_level - 1;
      int idx                 = nbytes_in_pbuffer[dlvl];
      uint8_t *dest             = pbuffer[dlvl] + idx;
      *dest                   = c;
      nbytes_in_pbuffer[dlvl] = idx + 1;
    }
  else
    {
      /* Only one PUSHS encountered.  Write directly to the output
       * buffer
       */

      poffAddTmpProgByte(poffProgHandle, (uint8_t)c);
    }
}

static inline void flushbuf(poffProgHandle_t poffProgHandle)
{
  int errCode;
  int slvl = current_level;

  if (nbytes_in_pbuffer[slvl] > 0)
    {
      if (current_level > 0)
        {
          /* Nested PUSHS encountered. Flush buffer into buffer associated
           * with the previous nesting level.
           */

          int dlvl    = slvl - 1;
          uint8_t *src  = pbuffer[slvl];
          uint8_t *dest = pbuffer[dlvl] + nbytes_in_pbuffer[dlvl];

          memcpy(dest, src, nbytes_in_pbuffer[slvl]);
          nbytes_in_pbuffer[dlvl] += nbytes_in_pbuffer[slvl];
        }
      else
        {
          /* Only one PUSHS encountered.  Flush directly to the output
           * buffer
           */

          errCode = poffWriteTmpProgBytes(pbuffer[0], nbytes_in_pbuffer[0],
                                          poffProgHandle);

          if (errCode != eNOERROR)
            {
              printf("Error writing to file: %d\n", errCode);
              exit(1);
            }
        }
    }
  nbytes_in_pbuffer[slvl] = 0;
}

/**********************************************************************
 * Private Functions
 **********************************************************************/

static void dopush(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle)
{
  int opcode;

  while (inch != EOF)
    {
      /* Search for a PUSHS opcode */

      if (inch != oPUSHS)
        {
          /* Its not PUSHS, just echo to the output file/buffer */

          putbuf(inch, poffProgHandle);

          /* Get the next byte from the input stream */

          opcode = inch;
          inch = poffGetProgByte(poffHandle);

          /* Check for a 32-bit argument */

          if ((opcode & o32) != 0)
            {
              /* Echo the 32-bits of the argument */

              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
            }
        }
      else
        {
          /* We have found PUSHS.  No search for the next occurrence
           * of either and instruction that increments the string
           * stack or for the matching POPS
           */

          current_level++;
          dopop(poffHandle, poffProgHandle);
          current_level--;
        }
    }
}

static void dopop(poffHandle_t poffHandle, poffProgHandle_t poffProgHandle)
{
  /* We have found PUSHS.  No search for the next occurrence
   * of either and instruction that increments the string
   * stack or for the matching POPS
   */

  /* Skip over the PUSHS for now */

  inch = poffGetProgByte(poffHandle);

  while (inch != EOF)
    {
      /* Did we encounter another PUSHS? */

      if (inch == oPUSHS)
        {
          /* Yes... recurse to handle it */

          current_level++;
          dopop(poffHandle, poffProgHandle);
          current_level--;
        }

      else if (inch == oPOPS)
        {
          /* Flush the buffered data without the PUSHS */

          flushbuf(poffProgHandle);

          /* And discard the matching POPS */

          inch = poffGetProgByte(poffHandle);
          break;
        }
      else if (inch == oLIB)
        {
          uint32_t arg32;
          unsigned int tmp;

          /* Get the 32-bit argument from the big endian data stream */

          tmp = poffGetProgByte(poffHandle);
          arg32 = tmp << 24;
          putbuf(tmp, poffProgHandle);

          tmp = poffGetProgByte(poffHandle);
          arg32 |= tmp << 16;
          putbuf(tmp, poffProgHandle);

          tmp = poffGetProgByte(poffHandle);
          arg32 |= tmp << 8;
          putbuf(tmp, poffProgHandle);

          tmp = poffGetProgByte(poffHandle);
          arg32 |= tmp;
          putbuf(tmp, poffProgHandle);

          inch   = poffGetProgByte(poffHandle);

          /* Is it LIB MKSTK? MKSTKSTR? or MKSTKC? */

          if ((arg32 == lbMKSTK) ||
              (arg32 == lbMKSTKSTR) ||
              (arg32 == lbMKSTKC))
            {
              /* Flush the buffered data with the PUSHS */

              flushc(oPUSHS, poffProgHandle);
              flushbuf(poffProgHandle);

              /* And break out of the loop to search for
               * the next PUSHS
               */

              break;
            }
        }
      else
        {
          int opcode;

          /* Something else.  Put it in the buffer */

          putbuf(inch, poffProgHandle);

          /* Get the next byte from the input stream */

          opcode = inch;
          inch = poffGetProgByte(poffHandle);

          /* Check for a 32-bit argument */

          if (opcode & o32 != 0)
            {
              /* Buffer the remaining 24-bits of the argument */

              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
              putbuf(inch, poffProgHandle);
              inch = poffGetProgByte(poffHandle);
            }
        }
    }
}

/**********************************************************************
 * Global Functions
 **********************************************************************/

void stringStackOptimize(poffHandle_t poffHandle,
                         poffProgHandle_t poffProgHandle)
{
  int i;

  /* Allocate an array of buffers to hold pcode data */

  for (i = 0; i < NPBUFFERS; i++)
    {
      pbuffer[i] = (uint8_t*)malloc(PBUFFER_SIZE);
      if (pbuffer[i] == NULL)
        {
          printf("Failed to allocate pcode buffer\n");
          exit(1);
        }
      nbytes_in_pbuffer[i] = 0;
    }

  /* Prime the search logic */

  inch = poffGetProgByte(poffHandle);
  current_level = -1;

  /* And parse the input file to the output file, removing unnecessary string
   * stack operations.
   */

  dopush(poffHandle, poffProgHandle);

  /* Release the buffers */

  for (i = 0; i < NPBUFFERS; i++)
    {
      free(pbuffer[i]);
      pbuffer[i] = NULL;
      nbytes_in_pbuffer[i] = 0;
    }
}

/**********************************************************************/
