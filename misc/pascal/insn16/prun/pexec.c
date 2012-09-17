/****************************************************************************
 * pexec.c
 *
 *   Copyright (C) 200-2009 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "keywords.h"
#include "pdefs.h"
#include "pinsn16.h"
#include "pfdefs.h"
#include "pxdefs.h"
#include "pedefs.h"

#include "paslib.h"
#include "pexec.h"

#ifdef CONFIG_HAVE_LIBM
#include <math.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define PTRUE   ((ustack_t)-1)
#define PFALSE  ((ustack_t) 0)

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Remove the value from the top of the stack */

#define POP(st, dest) \
  do { \
    dest = (st)->dstack.i[BTOISTACK((st)->sp)]; \
    (st)->sp -= BPERI; \
  } while (0)

/* Add the value to top of the stack */

#define PUSH(st, src) \
  do { \
    (st)->sp += BPERI; \
    (st)->dstack.i[BTOISTACK((st)->sp)] = src; \
  } while (0)

/* Return an rvalue for the (word) offset from the top of the stack */

#define TOS(st, off) \
  (st)->dstack.i[BTOISTACK((st)->sp)-(off)]

/* Save the src (word) at the dest (word) stack position */

#define PUTSTACK(st, src, dest)  \
  do { \
    (st)->dstack.i[BTOISTACK(dest)] = src; \
  } while (0)

/* Return an rvalue for the (word) from the absolute stack position */

#define GETSTACK(st, src) \
  (st)->dstack.i[BTOISTACK(src)]

/* Store a byte to an absolute (byte) stack position */

#define PUTBSTACK(st, src,dest) \
  do { \
    (st)->dstack.b[dest] = dest; \
  } while (0)

/* Return an rvalue for the absolute (byte) stack position */

#define GETBSTACK(st, src) \
  (st)->dstack.b[src]

/* Return the address for an absolute (byte) stack position. */

#define ATSTACK(st, src) \
  &(st)->dstack.b[src]

/* Discard n words from the top of the stack */

#define DISCARD(st, n) \
  do { \
    (st)->sp -= BPERI*(n); \
  } while (0)

/* Release a C string */

#define free_cstring(a) \
  free(a)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

union fparg_u
{
  double   f;
  uint16_t hw[4];
};

typedef union fparg_u fparg_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint16_t pexec_sysio(struct pexec_s *st, uint8_t fno, uint16_t subfunc);
static uint16_t pexec_libcall(struct pexec_s *st, uint16_t subfunc);
static uint16_t pexec_execfp(struct pexec_s *st, uint8_t fpop);
static void     pexec_getfparguments(struct pexec_s *st, uint8_t fpop, fparg_t *arg1, fparg_t *arg2);
static ustack_t pexec_readinteger(uint8_t *ioptr);
static void     pexec_readreal(uint16_t *dest, uint8_t *ioptr);
static ustack_t pexec_getbaseaddress(struct pexec_s *st, level_t leveloffset);
static uint8_t *pexec_mkcstring(uint8_t *buffer, int buflen);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static uint8_t ioline[LINE_SIZE+1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pexec_sysio
 *
 * Description:
 *   This function process a system I/O operation.
 *
 ****************************************************************************/

static uint16_t pexec_sysio(struct pexec_s *st, uint8_t fno, uint16_t subfunc)
{
  ustack_t uparm1;
  fparg_t  fp;

  uint8_t *ptr;

  switch (subfunc)
    {
    case xEOF :
/* FINISH ME -- > */
      break;
    case xEOLN :
/* FINISH ME -- > */
      break;
    case xRESET :
/* FINISH ME -- > */
      break;
    case xREWRITE :
/* FINISH ME -- > */
      break;

    case xREADLN :
/* FINISH ME -- > */
      break;
    case xREAD_BINARY :
/* FINISH ME -- > */
      break;

      /* xREAD_INT:
       * STACK INPUTS: TOS(st, 0) = address to store integer */
    case xREAD_INT :
      (void)fgets((char*)ioline, LINE_SIZE, stdin);
      PUTSTACK(st, pexec_readinteger(ioline),TOS(st, 0));
      break;

      /* xREAD_CHAR:
       * STACK INPUTS: TOS(st, 0) = address to store integer */

    case xREAD_CHAR:
      (void)fgets((char*)ioline, LINE_SIZE, stdin);
      PUTBSTACK(st, ioline[0],TOS(st, 0));
      break;

      /* XREAD_STRING:

       * STACK INPUTS:
       *   TOS = Number of bytes to read
       *   TOS-1 = Address to store byte(s) */
    case xREAD_STRING :
      (void)fgets((char*)ATSTACK(st, TOS(st, 1)), TOS(st, 0), stdin);
      break;

      /* xREAD_REAL:
       * STACK INPUTS: TOS = address to store REAL */

    case xREAD_REAL :
      (void)fgets((char*)ioline, LINE_SIZE, stdin);
      pexec_readreal((uint16_t*)ATSTACK(st, TOS(st, 0)), ioline);
      break;

    case xWRITELN :
      putchar('\n');
      break;
    case xWRITE_PAGE :
      putchar('\f');
      break;
    case xWRITE_BINARY :
/* FINISH ME -- > */
      break;

      /* xWRITE_INT:
       * STACK INPUTS: TOS = integer value to write. */

    case xWRITE_INT :
      printf("%ld", signExtend16(TOS(st, 0)));
      break;

      /* xWRITE_CHAR:
       * STACK INPUTS: TOS = char value to write. */

    case xWRITE_CHAR :
      putchar(TOS(st, 0));
      break;

      /* xWRITE_STRING:
       * STACK INPUTS:
       *   TOS = Number of bytes to write
       *   TOS-1 = Address of src data */

    case xWRITE_STRING :
      uparm1 = TOS(st, 0);
      for (ptr = (uint8_t*)ATSTACK(st, TOS(st, 1)); uparm1; uparm1--, ptr++)
        putchar(*ptr);
      break;

      /* xWRITE_REAL:
       * STACK INPUTS: TOS = value of double */

    case xWRITE_REAL :
      fp.hw[0] = TOS(st, 3);
      fp.hw[1] = TOS(st, 2);
      fp.hw[2] = TOS(st, 1);
      fp.hw[3] = TOS(st, 0);;
      printf("%f", fp.f);
      break;

    default :
      return eBADSYSIOFUNC;

    }

  return eNOERROR;

} /* end pexec_sysio */

/****************************************************************************
 * Name: pexec_libcall
 *
 * Description:
 *   This function process a system I/O operation 
 *
 ****************************************************************************/

static uint16_t pexec_libcall(struct pexec_s *st, uint16_t subfunc)
{
  ustack_t  uparm1;
  ustack_t  uparm2;
  paddr_t   addr1;
  paddr_t   addr2;
  uint16_t *tmp;
  uint16_t *ref;
  uint8_t  *src;
  uint8_t  *dest;
  uint8_t  *name;
  int       len;
  int32_t   value;

  switch (subfunc)
    {
      /* Get the value of an environment string
       *
       * ON INPUT:
       *   TOS(st, 0) = Number of bytes in environment identifier string
       *   TOS(st, 1) = Address environment identifier string
       * ON RETURN (above replaced with):
       *   TOS(st, 0) = MS 16-bits of 32-bit C string pointer
       *   TOS(st, 1) = LS 16-bits of 32-bit C string pointer
       */

    case lbGETENV :
      len = TOS(st, 0);                    /* Number of bytes in string */
      src = (uint8_t*)&GETSTACK(st, TOS(st, 1));  /* Pointer to string */
                       
      /* Make a C string out of the pascal string */

      name = pexec_mkcstring(src, len);
      if (name == NULL)
        {
          return eNOMEMORY;
        }

      /* Make the C-library call and free the string copy */

      src = (uint8_t*)getenv((char*)name);
      free_cstring(name);

      /* Save the returned pointer in the stack */

      TOS(st, 0) = (ustack_t)((uint32_t)src >> 16);
      TOS(st, 1) = (ustack_t)((uint32_t)src & 0x0000ffff);
      break;

      /* Copy pascal string to a pascal string
       * 
       * ON INPUT:
       *   TOS(st, 0) = address of dest string hdr
       *   TOS(st, 1) = length of source string
       *   TOS(st, 2) = pointer to source string
       * ON RETURN (input consumed):
       */

    case lbSTR2STR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string header */
      POP(st, uparm1); /* length of source data */
      POP(st, addr2);  /* addr of source string data */

      /* Do nothing if the source and destinations are the same
       * string.  This happens normally on cases like:
       *   string name;
       *   char   c;
       *   name := name + c;
       */

      if (addr1 != addr2)
        {
          /* The source and destination strings are different.
           * Make sure that the string length will fit into the destination.
           */

          if (uparm1 >= sSTRING_MAX_SIZE)
            {
              /* Clip to the maximum size */

              uparm1 = sSTRING_MAX_SIZE;
              len    = sSTRING_MAX_SIZE;
            }
          else
            {
              /* We have space */

              len = (int)uparm1;
            }

          /* Get proper string pointers */

          dest = ATSTACK(st, addr1);
          src  = ATSTACK(st, addr2);

          /* Transfer the (16-bit) string length (must be aligned!) */

          tmp    = (uint16_t*)dest;
          *tmp++ = uparm1;
          dest   = (uint8_t*)tmp;

          /* Then transfer the string contents */

          memcpy(dest, src, len);
        }
      break;

      /* Copy C string to a pascal string
       * 
       * ON INPUT:
       *   TOS(st, 0) = address of dest hdr
       *   TOS(st, 1) = MS 16-bits of 32-bit C string pointer
       *   TOS(st, 2) = LS 16-bits of 32-bit C string pointer
       * ON RETURN (input consumed):
       */
    case lbCSTR2STR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string header */
      POP(st, uparm1); /* MS 16-bits of 32-bit C string pointer */
      POP(st, uparm2); /* LS 16-bits of 32-bit C string pointer */

      /* Get proper string pointers */

      dest = ATSTACK(st, addr1);
      src  = (uint8_t*)((unsigned long)uparm1 << 16 | (unsigned long)uparm2);

      /* Handle null src pointer */

      if (src == NULL)
        {
          *dest = 0;
        }
      else
        {
          /* Get the length of the string */

          uparm1 = strlen((char*)src);

          /* Make sure that the string length will fit into the
           * destination. */

          if (uparm1 >= sSTRING_MAX_SIZE)
            {
              /* Clip to the maximum size */

              uparm1 = sSTRING_MAX_SIZE;
              len    = sSTRING_MAX_SIZE;
            }
          else
            {
              /* We have space */

              len = (int)uparm1;
            }

          /* Transfer the (16-bit) string length (must be aligned!) */

          tmp    = (uint16_t*)dest;
          *tmp++ = uparm1;
          dest   = (uint8_t*)tmp;

          /* Then transfer the string contents */

          memcpy(dest, src, len);
        }
      break;

      /* Copy pascal string to a pascal string reference
       *   procedure str2rstr(src : string; var dest : rstring)
       * ON INPUT:
       *   TOS(st, 0)=address of dest string reference
       *   TOS(st, 1)=length of source string
       *   TOS(st, 2)=pointer to source string
       * ON RETURN: actual parameters released.
       */

    case lbSTR2RSTR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string reference */
      POP(st, uparm1); /* length of source data */
      POP(st, addr2);  /* addr of source string data */

      /* Make sure that the string length will fit into the destination. */

      if (uparm1 >= sSTRING_MAX_SIZE)
        {
          return eSTRSTKOVERFLOW;
        }

      /* Get a pointer to the destination reference */

      ref = (uint16_t*)ATSTACK(st, addr1);

      /* Get proper string pointers */

      dest = ATSTACK(st, ref[0] - 2);
      src  = ATSTACK(st, addr2);

      /* Transfer the (16-bit) string length (must be aligned!) */

      tmp    = (uint16_t*)dest;
      *tmp++ = uparm1;
      dest   = (uint8_t*)tmp;

      /* Then transfer the string contents and save the new size */

      memcpy(dest, src, uparm1);
      ref[1] = uparm1;
      break;

      /* Copy C string to a pascal string reference
       *   procedure cstr2str(src : cstring; var dest : string)
       * ON INPUT:
       *   TOS(st, 0)=address of dest string reference
       *   TOS(st, 0)=MS 16-bits of 32-bit C source string pointer
       *   TOS(st, 1)=LS 16-bits of 32-bit C source string pointer
       * ON RETURN: actual parameters released
       */

    case lbCSTR2RSTR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string reference */
      POP(st, uparm1); /* MS 16-bits of 32-bit C string pointer */
      POP(st, uparm2); /* LS 16-bits of 32-bit C string pointer */

      /* Get a pointer to the destination reference */

      ref = (uint16_t*)ATSTACK(st, addr1);

      /* Get proper string pointers */

      dest = ATSTACK(st, ref[0] - 2);
      src  = (uint8_t*)((unsigned long)uparm1 << 16 | (unsigned long)uparm2);

      /* Handle null src pointer */

      if (src == NULL)
        {
          *dest = 0;
        }
      else
        {
          /* Get the length of the string */

          uparm1 = strlen((char*)src);

          /* Make sure that the string length will fit into the
           * destination. */

          if (uparm1 >= sSTRING_MAX_SIZE)
            {
              return eSTRSTKOVERFLOW;
            }

          /* Transfer the (16-bit) string length (must be aligned!) */

          tmp    = (uint16_t*)dest;
          *tmp++ = uparm1;
          dest   = (uint8_t*)tmp;

          /* Then transfer the string contents */

          memcpy(dest, src, uparm1);
          ref[1] = uparm1;
        }
      break;

      /* Convert a string to a numeric value
       *   procedure val(const s : string; var v; var code : word); 
       *
       * Description:
       * val() converts the value represented in the string S to a numerical
       * value, and stores this value in the variable V, which can be of type
       * Longint, Real and Byte. If the conversion isn't succesfull, then the
       * parameter Code contains the index of the character in S which
       * prevented the conversion. The string S is allowed to contain spaces
       * in the beginning.
       *
       * The string S can contain a number in decimal, hexadecimal, binary or
       * octal format, as described in the language reference.
       *
       * Errors:
       * If the conversion doesn¡Çt succeed, the value of Code indicates the
       * position where the conversion went wrong.
       *
       * ON INPUT
       *   TOS(st, 0)=address of code
       *   TOS(st, 1)=address of value
       *   TOS(st, 2)=length of source string
       *   TOS(st, 3)=pointer to source string
       * ON RETURN: actual parameters released
       */

    case lbVAL :
      /* Get the string information */

      len = TOS(st, 2);                    /* Number of bytes in string */
      src = (uint8_t*)&GETSTACK(st, TOS(st, 3));  /* Pointer to string */

      /* Make a C string out of the pascal string */

      name = pexec_mkcstring(src, len);
      if (name == NULL)
        {
          return eNOMEMORY;
        }

      /* Convert the string to an integer */

      value = atoi((char*)name);
      if ((value < MININT) || (value > MAXINT))
        {
          return eINTEGEROVERFLOW;
        }
      PUTSTACK(st, TOS(st, 0), 0);
      PUTSTACK(st, TOS(st, 1), value);
      DISCARD(st, 4);
      break;

      /* Create an empty string
       *   function mkstk : string;
       * ON INPUT
       * ON RETURN
       *   TOS(st, 0)=length of new string
       *   TOS(st, 1)=pointer to new string
       */

    case lbMKSTK :
      /* Allocate space on the string stack for the new string
       * FIXME:  This logic does not handle strings with other than the
       * default size!
       */

      addr1             = ((st->csp + 1) & ~1);
      st->csp += sSTRING_SIZE;    /* Allocate max size */

      /* Save the length at the beginning of the copy */

      tmp    = (uint16_t*)&GETSTACK(st, addr1);  /* Pointer to new string */
      *tmp++ = 0;                          /* Save current size */

      /* Update the stack content */

      PUSH(st, addr1 + sSTRING_HDR_SIZE);      /* Pointer to new string */
      PUSH(st, 0);                             /* Current size */
      break;

      /* Replace a string with a duplicate string residing in allocated
       * string stack.
       *   function mkstkstr(name : string) : string;
       * ON INPUT
       *   TOS(st, 0)=length of original string
       *   TOS(st, 1)=pointer to original string data
       * ON RETURN
       *   TOS(st, 0)=length of new string (unchanged)
       *   TOS(st, 1)=pointer to new string data
       */

    case lbMKSTKSTR :
      /* Get the parameters from the stack (leaving the string reference
       * in place.
       */

      uparm1 = TOS(st, 0);     /* Original string size */
      addr1  = TOS(st, 1);     /* Original string data pointer */
 
      /* Check if there is space on the string stack for the new string
       * FIXME:  This logic does not handle strings with other than the
       * default size!
       */

      if (st->csp + sSTRING_SIZE >= st->spb)
        {
          return eSTRSTKOVERFLOW;
        }

      /* Allocate space on the string stack for the new string */

      addr2             = ((st->csp + 1) & ~1);
      st->csp += sSTRING_SIZE;    /* Allocate max size */

      /* Save the length at the beginning of the copy */

      tmp    = (uint16_t*)&GETSTACK(st, addr2);  /* Pointer to new string */
      *tmp++ = uparm1;                     /* Save current size */
      dest   = (uint8_t*)tmp;                 /* Pointer to string data */

      /* Copy the string into the string stack */

      src  = (uint8_t*)&GETSTACK(st, addr1);  /* Pointer to original string */
      memcpy(dest, src, uparm1);

      /* Update the stack content */

      TOS(st, 1) = addr2 + sSTRING_HDR_SIZE;
      break;

      /* Replace a character with a string residing in allocated string stack.
       *   function mkstkc(c : char) : string;
       * ON INPUT
       *   TOS(st, 0)=Character value
       * ON RETURN
       *   TOS(st, 0)=length of new string
       *   TOS(st, 1)=pointer to new string
       */

    case lbMKSTKC :
      /* Check if there is space on the string stack for the new string
       * FIXME:  This logic does not handle strings with other than the
       * default size!
       */

      if (st->csp + sSTRING_SIZE >= st->spb)
        {
          return eSTRSTKOVERFLOW;
        }

      /* Allocate space on the string stack for the new string */

      addr2             = ((st->csp + 1) & ~1);
      st->csp += sSTRING_SIZE;    /* Allocate max size */

      /* Save the length at the beginning of the copy */

      tmp    = (uint16_t*)&GETSTACK(st, addr2);  /* Pointer to new string */
      *tmp++ = 1;                          /* Save initial size */
      dest   = (uint8_t*)tmp;                 /* Pointer to string data */

      /* Copy the character into the string stack */

      *dest++ = TOS(st, 0);                    /* Save character as string */

      /* Update the stack content */

      TOS(st, 0) = addr2 + sSTRING_HDR_SIZE;   /* String address */
      PUSH(st, 1);                             /* String length */
      break;

      /* Concatenate a string to the end of a string.
       *   function strcat(name : string, c : char) : string;
       *
       * ON INPUT
       *   TOS(st, 0)=length of string1
       *   TOS(st, 1)=pointer to string1 data
       *   TOS(st, 2)=length of string2
       *   TOS(st, 3)=pointer to string2 data
       * ON OUTPUT
       *   TOS(st, 1)=new length of string2
       *   TOS(st, 2)=pointer to string2
       */

    case lbSTRCAT :
      /* Get the parameters from the stack (leaving the string reference
       * in place.
       */

      POP(st, uparm1);      /* string1 size */
      POP(st, addr1);       /* string1 data stack addr */
      uparm2 = TOS(st, 0);  /* string2 size */

      /* Check for string overflow.  FIXME:  This logic does not handle
       * strings with other than the default size!
       */

      if (uparm1 + uparm2 > sSTRING_MAX_SIZE)
          return eSTRSTKOVERFLOW;
      else
        {
          /* Get a pointer to string1 data */

          src    = ATSTACK(st, addr1);

          /* Get a pointer to string2 header, set new size then, get
           * a pointer to string2 data.
           */

          tmp    = ((uint16_t*)&GETSTACK(st, TOS(st, 1))) - 1;
          *tmp++ = uparm1 + uparm2;
          dest   = (uint8_t*)tmp;

          memcpy(&dest[uparm2], src, uparm1); /* cat strings */
          TOS(st, 0) = uparm1 + uparm2;           /* Save new size */
        }
      break;

      /* Concatenate a character  to the end of a string.
       *   function strcatc(name : string, c : char) : string;
       *
       * ON INPUT
       *   TOS(st, 0)=character to concatenate
       *   TOS(st, 1)=length of string
       *   TOS(st, 2)=pointer to string
       * ON OUTPUT
       *   TOS(st, 1)=new length of string
       *   TOS(st, 2)=pointer to string
       */

    case lbSTRCATC :
      /* Get the parameters from the stack (leaving the string reference
       * in place.
       */

      POP(st, uparm1);      /* Character to concatenate */
      uparm2 = TOS(st, 0);  /* Current length of string */

      /* Check for string overflow.  FIXME:  This logic does not handle
       * strings with other than the default size!
       */

      if (uparm2 >= sSTRING_MAX_SIZE)
          return eSTRSTKOVERFLOW;
      else
        {
          /* Get a pointer to string header, set size new size then, get
           * a pointer to string data.
           */

          tmp          = ((uint16_t*)&GETSTACK(st, TOS(st, 1))) - 1;
          *tmp++       = uparm2 + 1;
          dest         = (uint8_t*)tmp;

          /* Add the new charcter */

          dest[uparm2] = (uint8_t)uparm1;

          /* Save the new string size */

          TOS(st, 0)       = uparm2 + 1;
        }
      break;

      /* Compare two pascal strings
       *   function strcmp(name1 : string, name2 : string) : integer;
       * ON INPUT
       *   TOS(st, 1)=length of string2
       *   TOS(st, 2)=address of string2 data
       *   TOS(st, 3)=length of string1
       *   TOS(st, 4)=address of string1 data
       * ON OUTPUT
       *   TOS(st, 0)=(-1=less than, 0=equal, 1=greater than} 
       */

    case lbSTRCMP :
      {
        int result;

        /* Get the parameters from the stack (leaving space for the
         * return value);
         */

        POP(st, uparm2);     /* length of string2 */
        POP(st, addr2);      /* address of string2 data */
        POP(st, uparm1);     /* length of string1 */
        addr1 = TOS(st, 0);  /* address of string1 data */

        /* Get full address */

        dest   = ATSTACK(st, addr1);
        src    = ATSTACK(st, addr2);

        /* If name1 is shorter than name2, then we can only return
         * -1 (less than) or +1 greater than.  If the substrings
         * of length of name1 are equal, then we return less than.
         */

        if (uparm1 < uparm2)
          {
            result = memcmp(dest, src, uparm1);
            if (result == 0) result = -1;
          }

        /* If name1 is longer than name2, then we can only return
         * -1 (less than) or +1 greater than.  If the substrings
         * of length of name2 are equal, then we return greater than.
         */

        else if (uparm1 > uparm2)
          {
            result = memcmp(dest, src, uparm2);
            if (result == 0) result = 1;
          }

        /* The strings are of equal length. Return the result of
         * the comparison.
         */

        else
          {
            result = memcmp(dest, src, uparm1);
          }
        TOS(st, 0) = result;
      }
      break;

    default :
      return eBADSYSLIBCALL;

    }

  return eNOERROR;

} /* end pexec_libcall */

/****************************************************************************
 * Name: pexec_execfp
 *
 * Description:
 *   This function processes a floating point operation.
 *
 ****************************************************************************/

static uint16_t pexec_execfp(struct pexec_s *st, uint8_t fpop)
{
  int16_t intValue;
  fparg_t arg1;
  fparg_t arg2;
  fparg_t result;

  switch (fpop & fpMASK)
    {
      /* Floating Pointer Conversions (On stack argument:  FP or Integer) */

    case fpFLOAT :
      POP(st, intValue);
      result.f = (double)intValue;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;

    case fpTRUNC :
    case fpROUND :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      intValue = (int16_t)arg1.f;
      PUSH(st, intValue);
      break;

      /* Floating Point arithmetic instructions (Two FP stack arguments) */

    case fpADD :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f + arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpSUB :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f - arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpMUL :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f * arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpDIV :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f / arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpMOD :
      return eBADFPOPCODE;
#if 0 /* Not yet */
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f % arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#endif

      /* Floating Point Comparisons (Two FP stack arguments) */

    case fpEQU :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f == arg2.f)
        intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpNEQ :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f != arg2.f)
        intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpLT :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f < arg2.f)
        intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpGTE :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f >= arg2.f)
        intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpGT :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f > arg2.f)
        intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpLTE :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f <= arg2.f)
        intValue = PTRUE;
      PUSH(st, intValue);
      break;

      /* Floating Point arithmetic instructions (One FP stack arguments) */

    case fpNEG :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = -arg1.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#ifdef CONFIG_HAVE_LIBM
    case fpABS :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = fabs(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#endif
    case fpSQR :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = arg1.f * arg1.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#ifdef CONFIG_HAVE_LIBM
    case fpSQRT :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = sqrt(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpSIN :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = sin(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpCOS :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = cos(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpATAN :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = atan(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpLN :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = log(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpEXP :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = exp(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#endif

    default :
      return eBADFPOPCODE;

    }
  return eNOERROR;

} /* end pexec_execfp */

/****************************************************************************
 * Name: pexec_getfparguments
 *
 * Description:
 *   This function retrieves the floating point arguments and performs
 *   integer to REAL conversions as necessary
 *
 ****************************************************************************/

static void pexec_getfparguments(struct pexec_s *st, uint8_t fpop, fparg_t *arg1, fparg_t *arg2)
{
  int16_t sparm;

  /* Extract arg2 from the stack */

  if (arg2)
    {
      /* Convert an integer argument to type REAL */

      if ((fpop & fpARG2) != 0)
        {
          POP(st, sparm);
          arg2->f = (double)sparm;
        }
      else
        {
          POP(st, arg2->hw[3]);
          POP(st, arg2->hw[2]);
          POP(st, arg2->hw[1]);
          POP(st, arg2->hw[0]);
        }
    }

  /* Extract arg1 from the stack */

  if (arg1)
    {
      /* Convert an integer argument to type REAL */

      if ((fpop & fpARG1) != 0)
        {
          POP(st, sparm);
          arg1->f = (double)sparm;
        }
      else
        {
          POP(st, arg1->hw[3]);
          POP(st, arg1->hw[2]);
          POP(st, arg1->hw[1]);
          POP(st, arg1->hw[0]);
        }
    }

} /* end pexec_getfparguments */

/****************************************************************************
 * Name: pexec_readinteger
 *
 * Description:
 *   This function parses a decimal integer from ioptr
 ****************************************************************************/

static ustack_t pexec_readinteger(uint8_t *ioptr)
{
  sstack_t value = 0;

  while (isspace(*ioptr)) ioptr++;
  while ((*ioptr >= '0') && (*ioptr <= '9'))
    {
      value = 10*value
        + (sstack_t)(*ioptr)
        - (sstack_t)'0';
      ioptr++;
    }

  return (ustack_t)value;

} /* end pexec_readinteger */

/****************************************************************************
 * Name: pexec_readreal
 *
 * Description:
 *    This function parses a decimal integer from ioptr.
 *
 ****************************************************************************/

static void pexec_readreal(uint16_t *dest, uint8_t *inPtr)
{
  int32_t  intpart;
  fparg_t  result;
  double   fraction;
  uint8_t  unaryop;

  intpart = 0;
  unaryop = '+';

  /* Check for a leading unary - */

  if ((*inPtr == '-') || (*inPtr == '+'))
    unaryop = *inPtr++;

  /* Get the integer part of the real */

  while ((*inPtr >= '0') && (*inPtr <= '9'))
    intpart = 10*intpart + ((int32_t)*inPtr++) - ((int32_t)'0');

  result.f = ((double)intpart);

  /* Check for the a fractional part */

  if (*inPtr == '.')
    {
      inPtr++;
      fraction = 0.1;
      while ((*inPtr >= '0') && (*inPtr <= '9'))
        {
          result.f += fraction * (double)(((int32_t)*inPtr++) - ((int32_t)'0'));
          fraction /= 10.0;
        }
    }

  /* Correct the sign of the result */

  if (unaryop == '-')
    result.f = -result.f;

  /* Return the value into the P-Machine stack */

  *dest++ = result.hw[0];
  *dest++ = result.hw[1];
  *dest++ = result.hw[2];
  *dest   = result.hw[3];

} /* end pexec_readreal */

/****************************************************************************
 * Name: pexec_getbaseaddress
 *
 * Description:
 *   This function binds the base address corresponding to a given level
 *   offset.
 *
 ****************************************************************************/

static ustack_t pexec_getbaseaddress(struct pexec_s *st, level_t leveloffset)
{
  /* Start with the base register of the current frame */

  ustack_t baseAddress = st->fp;

  /* Search backware "leveloffset" frames until the correct frame is
   * found
   */

   while (leveloffset > 0)
     {
       baseAddress = st->dstack.i[BTOISTACK(baseAddress)];
       leveloffset--;
     }

   /* Offset that value by two words (one for the st->fp and one for the
    * return value
    */

   return baseAddress + 2*BPERI;

} /* end pexec_getbaseaddress */

/****************************************************************************
 * Name: pexec_mkcstring
 ****************************************************************************/

static uint8_t *pexec_mkcstring(uint8_t *buffer, int buflen)
{
  uint8_t *string;

  string = malloc(buflen + 1);
  if (string != NULL)
    {
      memcpy(string, buffer, buflen);
      string[buflen] = '\0';
    }
  return string;
}

/****************************************************************************
 * Name: pexec8
 *
 * Descripton:
 *   Handle 8-bit instructions with no immediate data
 *
 ****************************************************************************/

static inline int pexec8(FAR struct pexec_s *st, uint8_t opcode)
{
  sstack_t sparm;
  ustack_t uparm1;
  ustack_t uparm2;
  ustack_t uparm3;

  switch (opcode)
    {
      /* Arithmetic & logical & and integer conversions (One stack argument) */
    case oNEG  :
      TOS(st, 0) = (ustack_t)(-(sstack_t)TOS(st, 0));
      break;
    case oABS  :
      if (signExtend16(TOS(st, 0)) < 0)
        {
          TOS(st, 0) = (ustack_t)(-signExtend16(TOS(st, 0)));
        }
      break;
    case oINC  :
      TOS(st, 0)++;
      break;
    case oDEC  :
      TOS(st, 0)--;
      break;
    case oNOT  :
      TOS(st, 0) = ~TOS(st, 0);
      break;

      /* Arithmetic & logical (Two stack arguments) */

    case oADD :
      POP(st, sparm);
      TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) + sparm);
      break;
    case oSUB :
      POP(st, sparm);
      TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) - sparm);
      break;
    case oMUL :
      POP(st, sparm);
      TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) * sparm);
      break;
    case oDIV :
      POP(st, sparm);
      TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) / sparm);
      break;
    case oMOD :
      POP(st, sparm);
      TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) % sparm);
      break;
    case oSLL :
      POP(st, sparm);
      TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) << sparm);
      break;
    case oSRL :
      POP(st, sparm);
      TOS(st, 0) = (TOS(st, 0) >> sparm);
      break;
    case oSRA :
      POP(st, sparm);
      TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) >> sparm);
      break;
    case oOR  :
      POP(st, uparm1);
      TOS(st, 0) = (TOS(st, 0) | uparm1);
      break;
    case oAND :
      POP(st, uparm1);
      TOS(st, 0) = (TOS(st, 0) & uparm1);
      break;
    case oBIT :
      POP(st, uparm1);
      uparm2 = TOS(st, 0);
      if ((uparm1 & (1 << uparm2)) != 0)
        {
          TOS(st, 0) = PTRUE;
        }
      else
        {
          TOS(st, 0) = PFALSE;
        }
      break;

      /* Comparisons (One stack argument) */

     case oEQUZ :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm == 0)
        {
          uparm1 = PTRUE;
        }
      PUSH(st, uparm1);
      break;
    case oNEQZ :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm != 0)
        {
          uparm1 = PTRUE;
        }
      PUSH(st, uparm1);
      break;
    case oLTZ  :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm < 0)
        {
          uparm1 = PTRUE;
        }
      PUSH(st, uparm1);
      break;
    case oGTEZ :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm >= 0)
        {
          uparm1 = PTRUE;
        }
      PUSH(st, uparm1);
      break;
    case oGTZ  :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm > 0)
        {
          uparm1 = PTRUE;
        }
      PUSH(st, uparm1);
      break;
    case oLTEZ :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm <= 0)
        {
          uparm1 = PTRUE;
        }
      PUSH(st, uparm1);
      break;

      /* Comparisons (Two stack arguments) */

    case oEQU  :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm == (sstack_t)TOS(st, 0))
        {
          uparm1 = PTRUE;
        }
      TOS(st, 0) = uparm1;
      break;
    case oNEQ  :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm != (sstack_t)TOS(st, 0))
        {
          uparm1 = PTRUE;
        }
      TOS(st, 0) = uparm1;
      break;
    case oLT   :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm < (sstack_t)TOS(st, 0))
        {
          uparm1 = PTRUE;
        }
      TOS(st, 0) = uparm1;
      break;
    case oGTE  :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm >= (sstack_t)TOS(st, 0))
        {
          uparm1 = PTRUE;
        }
      TOS(st, 0) = uparm1;
      break;
    case oGT   :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm > (sstack_t)TOS(st, 0))
        {
          uparm1 = PTRUE;
        }
      TOS(st, 0) = uparm1;
      break;
    case oLTE  :
      POP(st, sparm);
      uparm1 = PFALSE;
      if (sparm <= (sstack_t)TOS(st, 0))
        {
          uparm1 = PTRUE;
        }
      TOS(st, 0) = uparm1;
      break;

      /* Load (One stack argument) */

    case oLDI  :
      POP(st, uparm1);                   /* Address */
      PUSH(st, GETSTACK(st, uparm1));
      PUSH(st, GETSTACK(st, uparm1 + BPERI));
      break;
    case oLDIH  :
      TOS(st, 0) = GETSTACK(st, TOS(st, 0));
      break;
    case oLDIB :
      TOS(st, 0) = GETBSTACK(st, TOS(st, 0));
      break;
    case oLDIM :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1); /* Size */
      POP(st, uparm2); /* Stack offset */
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUSH(st, GETSTACK(st, uparm2));
              uparm2 += BPERI;
              uparm1 -= BPERI;
            }
          else
            {
              PUSH(st, GETBSTACK(st, uparm2));
              uparm2++;
              uparm1--;
            }
        }
      break;
    case oDUP :
      uparm1 = TOS(st, 0);
      uparm2 = TOS(st, 1);
      PUSH(st, uparm2);
      PUSH(st, uparm1);
      break;
    case oDUPH :
      uparm1 = TOS(st, 0);
      PUSH(st, uparm1);
      break;
    case oPUSHS :
      PUSH(st, st->csp);
      break;
    case oPOPS :
      POP(st, st->csp);
      break;

      /* Store (Two stack arguments) */

    case oSTIH  :
      POP(st, uparm1);
      POP(st, uparm2);
      PUTSTACK(st, uparm1,uparm2);
      break;
    case oSTIB :
      POP(st, uparm1);
      POP(st, uparm2);
      PUTBSTACK(st, uparm1, uparm2);
      break;
    case oSTIM :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);                /* Size in bytes */
      uparm3 = uparm1;            /* Save for stack discard */
      sparm = ROUNDBTOI(uparm1); /* Size in words */
      uparm2 = TOS(st, sparm);       /* Stack offset */
      sparm--;
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUTSTACK(st, TOS(st, sparm), uparm2);
              uparm2 += BPERI;
              uparm1 -= BPERI;
              sparm--;
            }
          else
            {
              PUTBSTACK(st, TOS(st, sparm), uparm2);
              uparm2++;
              uparm1--;
            }
        }

      /* Discard the stored data + the stack offset */

      DISCARD(st, (ROUNDBTOI(uparm3) + 1));
      break;

      /* Program control (No stack arguments) */

    case oNOP   :
      break;
    case oRET   :
      POP(st, st->pc);
      POP(st, st->fp);
      DISCARD(st, 1);
      return eNOERROR;

      /* System Functions (No stack arguments) */

    case oEND   :
      return eEXIT;

    default :
      return eILLEGALOPCODE;
    }

  st->pc += 1;
  return eNOERROR;
}

/****************************************************************************
 * Name: pexec16
 *
 * Descripton:
 *   Handle 16-bit instructions with 8-bits of immediate data (imm8)
 *
 ****************************************************************************/

static inline int pexec16(FAR struct pexec_s *st, uint8_t opcode, uint8_t imm8)
{
  int ret = eNOERROR;

  st->pc += 2;
  switch (opcode)
    {
      /* Data stack:  imm8 = 8 bit unsigned data (no stack arguments) */

    case oPUSHB  :
      PUSH(st, imm8);
      break;

      /* Floating Point:  imm8 = FP op-code (varying number of stack arguments) */
    case oFLOAT  :
      ret = pexec_execfp(st, imm8);
      break;

    default :
      ret = eILLEGALOPCODE;
      break;
    }
  return ret;
}

/****************************************************************************
 * Name: pexec24
 *
 * Descripton:
 *   Handle 24-bit instructions with 16-bits of immediate data (imm16)
 *
 ****************************************************************************/

static inline int pexec24(FAR struct pexec_s *st, uint8_t opcode, uint16_t imm16)
{
  sstack_t sparm1;
  sstack_t sparm2;
  ustack_t uparm1;
  ustack_t uparm2;
  ustack_t uparm3;
  int ret = eNOERROR;

  switch (opcode)
    {
      /* Program control:  imm16 = unsigned label (no stack arguments) */

    case oJMP   :
      goto branch_out;

      /* Program control:  imm16 = unsigned label (One stack argument) */

    case oJEQUZ :
      POP(st, sparm1);
      if (sparm1 == 0)
        {
          goto branch_out;
        }
      break;
    case oJNEQZ :
      POP(st, sparm1);
      if (sparm1 != 0)
        {
          goto branch_out;
        }
      break;
    case oJLTZ  :
      POP(st, sparm1);
      if (sparm1 < 0)
        {
          goto branch_out;
        }
      break;
    case oJGTEZ :
      POP(st, sparm1);
      if (sparm1 >= 0)
        {
          goto branch_out;
        }
      break;
    case oJGTZ  :
      POP(st, sparm1);
      if (sparm1 > 0)
        {
          goto branch_out;
        }
      break;
    case oJLTEZ :
      POP(st, sparm1);
      if (sparm1 <= 0)
        {
          goto branch_out;
        }
      break;

      /* Program control:  imm16 = unsigned label (Two stack arguments) */

    case oJEQU :
      POP(st, sparm1);
      POP(st, sparm2);
      if (sparm2 == sparm1)
        {
          goto branch_out;
        }
      break;
    case oJNEQ :
      POP(st, sparm1);
      POP(st, sparm2);
      if (sparm2 != sparm1)
        {
          goto branch_out;
        }
      break;
    case oJLT  :
      POP(st, sparm1);
      POP(st, sparm2);
      if (sparm2 < sparm1)
        {
          goto branch_out;
        }
      break;
    case oJGTE :
      POP(st, sparm1);
      POP(st, sparm2);
      if (sparm2 >= sparm1)
        {
          goto branch_out;
        }
      break;
    case oJGT  :
      POP(st, sparm1);
      POP(st, sparm2);
      if (sparm2 > sparm1)
        {
          goto branch_out;
        }
      break;
    case oJLTE :
      POP(st, sparm1);
      POP(st, sparm2);
      if (sparm2 <= sparm1)
        {
          goto branch_out;
        }
      break;

      /* Load:  imm16 = usigned offset (no stack arguments) */

    case oLD :
      uparm1 = st->spb + imm16;
      PUSH(st, GETSTACK(st, uparm1));
      PUSH(st, GETSTACK(st, uparm1 + BPERI));
      break;
    case oLDH :
      uparm1 = st->spb + imm16;
      PUSH(st, GETSTACK(st, uparm1));
      break;
    case oLDB :
      uparm1 = st->spb + imm16;
      PUSH(st, GETBSTACK(st, uparm1));
      break;
    case oLDM :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);
      uparm2 = st->spb + imm16;
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUSH(st, GETSTACK(st, uparm2));
              uparm2 += BPERI;
              uparm1 -= BPERI;
            }
          else
            {
              PUSH(st, GETBSTACK(st, uparm2));
              uparm2++;
              uparm1--;
            }
        }
      break;

      /* Load & store: imm16 = unsigned base offset (One stack argument) */

    case oST :
      uparm1 = st->spb + imm16;
      POP(st, uparm2);
      PUTSTACK(st, uparm2, uparm1 + BPERI);
      POP(st, uparm2);
      PUTSTACK(st, uparm2, uparm1);
      break;
    case oSTH   :
      uparm1  = st->spb + imm16;
      POP(st, uparm2);
      PUTSTACK(st, uparm2, uparm1);
      break;
    case oSTB  :
      uparm1  = st->spb + imm16;
      POP(st, uparm2);
      PUTBSTACK(st, uparm2, uparm1);
      break;
    case oSTM :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);                /* Size */
      uparm3 = uparm1;            /* Save for stack discard */
      uparm2 = st->spb + imm16;
      sparm1 = ROUNDBTOI(uparm1) - 1;
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUTSTACK(st, TOS(st, sparm1), uparm2);
              uparm2 += BPERI;
              uparm1 -= BPERI;
              sparm1--;
            }
          else
            {
              PUTBSTACK(st, TOS(st, sparm1), uparm2);
              uparm2++;
              uparm1--;
            }
        }

      /* Discard the stored data */

      DISCARD(st, ROUNDBTOI(uparm3));
      break;
    case oLDX  :
      uparm1 = st->spb + imm16 + TOS(st, 0);
      TOS(st, 0) = GETSTACK(st, uparm1);
      PUSH(st, GETSTACK(st, uparm1 + BPERI));
      break;
    case oLDXH  :
      uparm1 = st->spb + imm16 + TOS(st, 0);
      TOS(st, 0) = GETSTACK(st, uparm1);
      break;
    case oLDXB :
      uparm1 = st->spb + imm16 + TOS(st, 0);
      TOS(st, 0) = GETBSTACK(st, uparm1);
      break;
    case oLDXM  :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);
      POP(st, uparm2);
      uparm2 += st->spb + imm16;
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUSH(st, GETSTACK(st, uparm2));
              uparm2 += BPERI;
              uparm1 -= BPERI;
            }
          else
            {
              PUSH(st, GETBSTACK(st, uparm2));
              uparm2++;
              uparm1--;
            }
        }
      break;

      /* Store: imm16 = unsigned base offset (Two stack arguments) */

    case oSTXH  :
      POP(st, uparm1);
      POP(st, uparm2);
      uparm2 += st->spb + imm16;
      PUTSTACK(st, uparm1,uparm2);
      break;
    case oSTXB :
      POP(st, uparm1);
      POP(st, uparm2);
      uparm2 += st->spb + imm16;
      PUTBSTACK(st, uparm1, uparm2);
      break;
    case oSTXM :
/* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);                /* Size */
      uparm3 = uparm1;            /* Save for stack discard */
      sparm1 = ROUNDBTOI(uparm1); /* Size in 16-bit words */
      uparm2 = TOS(st, sparm1);       /* index */
      sparm1--;
      uparm2 += st->spb + imm16;
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUTSTACK(st, TOS(st, sparm1), uparm2);
              uparm2 += BPERI;
              uparm1 -= BPERI;
              sparm1--;
            }
          else
            {
              PUTBSTACK(st, TOS(st, sparm1), uparm2);
              uparm2++;
              uparm1--;
            }
        }

      /* Discard the stored data + the index */

      DISCARD(st, (ROUNDBTOI(uparm3) + 1));
      break;

    case oLA  :
      uparm1 = st->spb + imm16;
      PUSH(st, uparm1);
      break;
    case oLAX :
      TOS(st, 0) = st->spb + imm16 + TOS(st, 0);
      break;

      /* Data stack:  imm16 = 16 bit signed data (no stack arguments) */

    case oPUSH  :
      PUSH(st, imm16);
      break;
    case oINDS  :
      st->sp += signExtend16(imm16);
      break;

      /* System Functions:
       * For LIB:        imm16 = sub-function code
       */

    case oLIB  :
      ret = pexec_libcall(st, imm16);
      break;

      /* Program control:  imm16 = unsigned label (no stack arguments) */

    case oLAC :
      uparm1 = imm16 + st->rop;
      PUSH(st, uparm1);
      break;

    case oLABEL :
    default:
      ret = eILLEGALOPCODE;
      break;
    }

  st->pc += 3;
  return ret;

branch_out:
  st->pc = (paddr_t)imm16;
  return ret;
}

/****************************************************************************
 * Name: pexec32
 *
 * Descripton:
 *   Handle 32-bit instructions with 24-bits of immediate data (imm8+imm16)
 *
 ****************************************************************************/

static int pexec32(FAR struct pexec_s *st, uint8_t opcode, uint8_t imm8, uint16_t imm16)
{
  sstack_t sparm;
  ustack_t uparm1;
  ustack_t uparm2;
  ustack_t uparm3;
  int ret = eNOERROR;

  switch (opcode)
    {
      /* Load:  imm8 = level; imm16 = signed frame offset (no stack arguments) */
    case oLDS :
      uparm1 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      PUSH(st, GETSTACK(st, uparm1));
      PUSH(st, GETSTACK(st, uparm1 + BPERI));
      break;
    case oLDSH :
      uparm1 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      PUSH(st, GETSTACK(st, uparm1));
      break;
    case oLDSB :
      uparm1 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      PUSH(st, GETBSTACK(st, uparm1));
      break;
    case oLDSM :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);
      uparm2 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUSH(st, GETSTACK(st, uparm2));
              uparm2 += BPERI;
              uparm1 -= BPERI;
            }
          else
            {
              PUSH(st, GETBSTACK(st, uparm2));
              uparm2++;
              uparm1--;
            }
        }
      break;

      /* Load & store: imm8 = level; imm16 = signed frame offset (One stack argument) */

    case oSTSH   :
      uparm1  = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      POP(st, uparm2);
      PUTSTACK(st, uparm2, uparm1);
      break;
    case oSTSB  :
      uparm1  = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      POP(st, uparm2);
      PUTBSTACK(st, uparm2, uparm1);
      break;
    case oSTSM :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);            /* Size */
      uparm3 = uparm1;            /* Save for stack discard */
      uparm2 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      sparm = ROUNDBTOI(uparm1) - 1;
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUTSTACK(st, TOS(st, sparm), uparm2);
              uparm2 += BPERI;
              uparm1 -= BPERI;
              sparm--;
            }
          else
            {
              PUTBSTACK(st, TOS(st, sparm), uparm2);
              uparm2++;
              uparm1--;
            }
        }

      /* Discard the stored data */

      DISCARD(st, ROUNDBTOI(uparm3));
      break;
    case oLDSX  :
      uparm1 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16) + TOS(st, 0);
      TOS(st, 0) = GETSTACK(st, uparm1);
      PUSH(st, GETSTACK(st, uparm1 + BPERI));
      break;
    case oLDSXH  :
      uparm1 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16) + TOS(st, 0);
      TOS(st, 0) = GETSTACK(st, uparm1);
      break;
    case oLDSXB :
      uparm1 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16) + TOS(st, 0);
      TOS(st, 0) = GETBSTACK(st, uparm1);
      break;
    case oLDSXM  :
 /* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);
      POP(st, uparm2);
      uparm2 += pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUSH(st, GETSTACK(st, uparm2));
              uparm2 += BPERI;
              uparm1 -= BPERI;
            }
          else
            {
              PUSH(st, GETBSTACK(st, uparm2));
              uparm2++;
              uparm1--;
            }
        }
      break;

      /* Store: imm8 = level; imm16 = signed frame offset (Two stack arguments) */

    case oSTSXH  :
      POP(st, uparm1);
      POP(st, uparm2);
      uparm2 += pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      PUTSTACK(st, uparm1,uparm2);
      break;
    case oSTSXB :
      POP(st, uparm1);
      POP(st, uparm2);
      uparm2 += pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      PUTBSTACK(st, uparm1, uparm2);
      break;
    case oSTSXM :
/* FIX ME --> Need to handle the unaligned case */
      POP(st, uparm1);                /* Size */
      uparm3 = uparm1;            /* Save for stack discard */
      sparm = ROUNDBTOI(uparm1); /* Size in 16-bit words */
      uparm2 = TOS(st, sparm);       /* index */
      sparm--;
      uparm2 += pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      while (uparm1 > 0)
        {
          if (uparm1 >= BPERI)
            {
              PUTSTACK(st, TOS(st, sparm), uparm2);
              uparm2 += BPERI;
              uparm1 -= BPERI;
              sparm--;
            }
          else
            {
              PUTBSTACK(st, TOS(st, sparm), uparm2);
              uparm2++;
              uparm1--;
            }
        }

      /* Discard the stored data + the index */

      DISCARD(st, (ROUNDBTOI(uparm3) + 1));
      break;

    case oLAS  :
      uparm1 = pexec_getbaseaddress(st, imm8) + signExtend16(imm16);
      PUSH(st, uparm1);
      break;
    case oLASX :
      TOS(st, 0) = pexec_getbaseaddress(st, imm8) + signExtend16(imm16) + TOS(st, 0);
      break;

      /* Program Control:  imm8 = level; imm16 = unsigned label (No
       * stack arguments)
       */

    case oPCAL  :
      PUSH(st, pexec_getbaseaddress(st, imm8));
      PUSH(st, st->fp);
      uparm1 = st->sp;
      PUSH(st, st->pc + 4);
      st->fp = uparm1;
      st->pc = (paddr_t)imm16;
      return eNOERROR;

      /* System Functions:
       * For SYSIO:   imm8 = file number; imm16 = sub-function code
       */

    case oSYSIO  :
      ret = pexec_sysio(st, imm8, imm16);
      break;

      /* Pseudo-operations:  (No stack arguments)
       * For LINE:    imm8 = file number; imm16 = line number
       */

    case oLINE   :
    default :
      ret = eILLEGALOPCODE;
      break;
    }

  st->pc += 4;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pexec_init
 ****************************************************************************/

FAR struct pexec_s *pexec_init(struct pexec_attr_s *attr)
{
  struct pexec_s *st;
  paddr_t stacksize;
  paddr_t adjusted_rosize;

  /* Allocate the p-machine state stucture */

  st = (struct pexec_s *)malloc(sizeof(struct pexec_s));
  if (!st)
    {
      return NULL;
    }

  /* Set up I-Space */

  st->ispace = attr->ispace;
  st->maxpc  = attr->maxpc;

  /* Align size of read-only data to 16-bit boundary. */

  adjusted_rosize = (attr->rosize + 1) & ~1;

  /* Allocate the pascal stack.  Organization is string stack, then
   * constant data, then "normal" pascal stack.
   */

  stacksize = attr->varsize + adjusted_rosize + attr->strsize;
  st->dstack.b = (uint8_t*)malloc(stacksize);
  if (!st->dstack.b)
    {
      free(st);
      return NULL;
    }

  /* Copy the rodata into the stack */

  if (attr->rodata && attr->rosize)
    {
      memcpy(&st->dstack.b[attr->strsize], attr->rodata, attr->rosize);
    }

  /* Set up info needed to perform a simulated reset */

  st->strsize   = attr->strsize;
  st->rosize    = adjusted_rosize;
  st->entry     = attr->entry;
  st->stacksize = stacksize;

  /* Then perform a simulated reset */

  pexec_reset(st);
  return st;
}

/****************************************************************************
 * Name: pexec
 ****************************************************************************/

int pexec(FAR struct pexec_s *st)
{
  uint8_t opcode;
  int ret;

  /* Make sure that the program counter is within range */

  if (st->pc >= st->maxpc)
    {
      ret = eBADPC;
    }
  else
    {
      /* Get the instruction to execute */

      opcode = st->ispace[st->pc];
      if ((opcode & o8) != 0)
        {
          /* Get the immediate, 8-bit value */

          uint8_t imm8 = st->ispace[st->pc + 1];
          if ((opcode & o16) != 0)
            {
              /* Get the immediate, big-endian 16-bit value */

              uint16_t imm16  = ((st->ispace[st->pc + 2]) << 8) | st->ispace[st->pc + 3];

              /* Handle 32 bit instructions */

              ret = pexec32(st, opcode, imm8, imm16);
            }
          else
            {
              /* Handle 16-bit instructions */

              ret = pexec16(st, opcode, imm8);
            }
        }
      else if ((opcode & o16) != 0)
        {
          /* Get the immediate, big-endian 16-bit value */

          uint16_t imm16  = ((st->ispace[st->pc + 1]) << 8) | st->ispace[st->pc + 2];

          /* Handle 24-bit instructions */

          ret = pexec24(st, opcode, imm16);
        }
      else
        {
          /* Handle 8-bit instructions */

          ret = pexec8(st, opcode);
        }
    }
  return ret;
}

/****************************************************************************
 * Name: pexec_reset
 ****************************************************************************/

void pexec_reset(struct pexec_s *st)
{
  int dndx;

  /* Setup the bottom of the "normal" pascal stack */

  st->rop   = st->strsize;
  st->spb   = st->strsize + st->rosize;

  /* Initialize the emulated P-Machine registers */

  st->csp   = 0;
  st->sp    = st->spb + 2*BPERI;
  st->fp    = st->spb + BPERI;
  st->pc    = st->entry;

  /* Initialize the P-Machine stack */

  dndx                 = BTOISTACK(st->spb);
  st->dstack.i[dndx]   =  0;
  st->dstack.i[dndx+1] =  0;
  st->dstack.i[dndx+2] = -1;
}

/****************************************************************************
 * Name: pexec_release
 ****************************************************************************/

void pexec_release(struct pexec_s *st)
{
  if (st)
    {
      if (st->dstack.i)
        {
          free(st->dstack.i);
        }

      if (st->ispace)
        {
          free(st->ispace);
        }

      free(st);
    }
}
