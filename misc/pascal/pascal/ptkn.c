/***************************************************************
 * ptkn.c
 * Tokenization Package
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
 ***************************************************************/

/***************************************************************
 * Included Functions
 ***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "keywords.h"
#include "pasdefs.h"
#include "ptdefs.h"
#include "pedefs.h"

#include "pas.h"
#include "ptkn.h"
#include "ptbl.h"
#include "perr.h"

/***************************************************************
 * Private Function Prototypes
 ***************************************************************/

static void getCharacter        (void);
static void skipLine            (void);
static bool getLine             (void);
static void identifier          (void);
static void string              (void);
static void unsignedNumber      (void);
static void unsignedRealNumber  (void);
static void unsignedExponent    (void);
static void unsignedHexadecimal (void);
static void unsignedBinary      (void);

/***************************************************************
 * Private Variables
 ***************************************************************/

static char    *strStack;      /* String Stack */
static uint16_t inChar;        /* last gotten character */

/***************************************************************
 * Public Variables
 ***************************************************************/

char *tkn_strt;               /* Start of token in string stack */
char *stringSP;               /* Top of string stack */

/***************************************************************
 * Public Functions
 ***************************************************************/

int16_t primeTokenizer(unsigned long stringStackSize)
{
  TRACE(lstFile,"[primeTokenizer]");

  /* Allocate and initialize the string stack and stack pointers */

  strStack = malloc(stringStackSize);
  if (!strStack)
    {
      fatal(eNOMEMORY);
    }

  /* Initially, everything points to the bottom of the
   * string stack.
   */

  tkn_strt = strStack;
  stringSP = strStack;

  /* Set up for input at the initial level of file parsing */

  rePrimeTokenizer();
  return 0;
}

/***************************************************************/

int16_t rePrimeTokenizer(void)
{
  TRACE(lstFile,"[rePrimeTokenizer]");

  /* (Re-)set the char pointer to the beginning of the line */

  FP->cp = FP->buffer;

  /* Read the next line from the input stream */

  if (!fgets(FP->cp, LINE_SIZE, FP->stream))
    {
      /* EOF.. close file */

      return 1;
    }

  /* Initialize the line nubmer */

  FP->line = 1;

  /* Get the first character from the new file */

  getCharacter();
  return 0;
}

/***************************************************************/
/* Tell 'em what what the next character will be (if they should
 * choose to get it).  This is similar to getCharacter(), except that
 * the character pointer is not incremented past the character.  The
 * next time that getCharacter() is called, it will get the character
 * again.
 */

char getNextCharacter(bool skipWhiteSpace)
{
  /* Get the next character from the line buffer. */

  inChar = *(FP->cp);

  /* If it is the EOL then read the next line from the input file */

  if (!inChar)
    {
      /* We have used all of the characters on this line.  Read the next
       * line of data
       */

      if (getLine())
        {
          /* Uh-oh, we are out of data!  Just return some bogus value. */
          inChar = '?';

        } /* end if */
      else
        {
          /* Otherwise, recurse to try again. */

          return getNextCharacter(skipWhiteSpace);

        } /* end else */
    } /* end if */

  /* If it is a space and we have been told to skip spaces then consume
   * the input line until a non-space or the EOL is encountered.
   */ 

  else if (skipWhiteSpace)
    {
      while ((isspace(inChar)) && (inChar))
        {
          /* Skip over the space */

          (FP->cp)++;

          /* A get the character after the space */

          inChar = *(FP->cp);

        } /* end while */

      /* If we hit the EOL while searching for the next non-space, then
       * recurse to try again on the next line
       */

      if (!inChar)
        {
          return getNextCharacter(skipWhiteSpace);
        }
    } /* end else if */

  return inChar;

} /* end getNextCharacter */

/***************************************************************/

void getToken(void)
{
  /* Skip over leading spaces and comments */

  while (isspace(inChar)) getCharacter();

  /* Point to the beginning of the next token */

  tkn_strt = stringSP;

  /* Process Identifier, Symbol, or Reserved Word */

  if ((isalpha(inChar)) || (inChar == '_'))
    identifier();

  /* Process Numeric */

  else if (isdigit(inChar))
    unsignedNumber();

  /* Process string */

  else if (inChar == SQUOTE)
    string();                         /* process string type */

  /* Process ':' or assignment */

  else if (inChar == ':')
    {
      getCharacter();
      if (inChar == '=') {token = tASSIGN; getCharacter();}
      else token = ':';
    } /* end else if */

  /* Process '.' or subrange or real-number */

  else if (inChar == '.')
    {
      /* Get the character after the '.' */

      getCharacter();

      /* ".." indicates a subrange */

      if (inChar == '.')
        {
          token = tSUBRANGE;
          getCharacter();
        }

      /* '.' digit is a real number */

      else if (isdigit(inChar))
        unsignedRealNumber();

      /* Otherwise, it is just a '.' */

      else token = '.';
    } /* end else if */

  /* Process '<' or '<=' or '<>' or '<<' */

  else if (inChar == '<')
    {
      getCharacter();
      if (inChar == '>') {token = tNE; getCharacter();}
      else if (inChar == '=') {token = tLE; getCharacter();}
      else if (inChar == '<') {token = tSHL; getCharacter();}
      else token = tLT;
    } /* end else if */

  /* Process '>' or '>=' or '><' or '>>' */

  else if (inChar == '>')
    {
      getCharacter();
      if (inChar == '<') {token = tNE; getCharacter();}
      else if (inChar == '=') {token = tGE; getCharacter();}
      else if (inChar == '>') {token = tSHR; getCharacter();}
      else token = tGT;
    } /* end else if */

  /* Get Comment -- form { .. } */

  else if (inChar == '{')
    {
      do getCharacter();                 /* get the next character */
      while (inChar != '}');             /* loop until end of comment */
      getCharacter();                    /* skip over end of comment */
      getToken();                        /* get the next real token */
    } /* end else if */

  /* Get comment -- form (* .. *) */

  else if (inChar == '(')
    {
      getCharacter();                    /* skip over comment character */
      if (inChar != '*')                 /* is this a comment? */
        {
          token = '(';                   /* No return '(' leaving the
                                          * unprocessed char in inChar */
        }
      else
        {
          uint16_t lastChar = ' ';         /* YES... prime the look behind */
          for (;;)                       /* look for end of comment */
            {
              getCharacter();            /* get the next character */
              if ((lastChar == '*') &&   /* Is it '*)' ?  */
                  (inChar == ')'))
                {
                  break;                 /* Yes... break out */
                }
              lastChar = inChar;         /* save the last character */
            } /* end for */

          getCharacter();                /* skip over the comment end char */
          getToken();                    /* and get the next real token */
      } /* end else */
    } /* end else if */

  /* NONSTANDARD:  All C/C++-style comments */

  else if (inChar == '/')
    {
      getCharacter();                    /* skip over comment character */
      if (inChar == '/')                 /* C++ style comment? */
        {
          skipLine();                    /* Yes, skip rest of line */
          getToken();                    /* and get the next real token */
        }
      else if (inChar != '*')            /* is this a C-style comment? */
        {
          token = '/';                   /* No return '/' leaving the
                                          * unprocessed char in inChar */
        }
      else
        {
          uint16_t lastChar = ' ';         /* YES... prime the look behind */
          for (;;)                       /* look for end of comment */
            {
              getCharacter();            /* get the next character */
              if ((lastChar == '*') &&   /* Is it '*)' ?  */
                  (inChar == '/'))
                {
                  break;                 /* Yes... break out */
                }
              lastChar = inChar;         /* save the last character */
            } /* end for */

          getCharacter();                /* skip over the comment end char */
          getToken();                    /* and get the next real token */
      } /* end else */
    } /* end else if */

  /* Check for $XXXX (hex) */

  else if (inChar == '%')
    unsignedHexadecimal();

  /* Check for $BBBB (binary) */

  else if (inChar == '%')
    unsignedBinary();

  /* if inChar is an ASCII character then return token = character */

  else if (isascii(inChar))
    {
      token = inChar;
      getCharacter();
    } /* end else if */

  /* Otherwise, discard the character and try again */

  else
    {
      getCharacter();
      getToken();
    } /* end else */

  DEBUG(lstFile,"[%02x]", token);

} /* End getToken */  

/***************************************************************
 * Private Functions
 ***************************************************************/

static void identifier(void)
{
  const  RTYPE *rptr;                         /* Pointer to reserved word */

  tknSubType = txNONE;                        /* Initialize */

  /* Concatenate identifier */

  do
    {
      *stringSP++ = toupper(inChar);          /* concatenate char */
      getCharacter();                         /* get next character */
    }
  while ((isalnum(inChar)) || (inChar == '_'));
  *stringSP++ = '\0';                         /* make ASCIIZ string */

  /* Check if the identifier is a reserved word */

  rptr = findReservedWord(tkn_strt);
  if (rptr)
    {
      token      = rptr->rtype;               /* get type from rsw table */
      tknSubType = rptr->subtype;             /* get subtype from rsw table */
      stringSP      = tkn_strt;               /* pop token from stack */
    } /* End if */

  /* Check if the identifier is a symbol */

  else
    {
      tknPtr = findSymbol(tkn_strt);
      if (tknPtr)
        {
          token = tknPtr->sKind;              /* get type from symbol table */
          stringSP = tkn_strt;                /* pop token from stack */

          /* The following assignments only apply to constants.  However it
           * is simpler just to make the assignments than it is to determine
           * if is appropriate to do so
           */

          if (token == tREAL_CONST)
            tknReal = tknPtr->sParm.c.val.f;
          else
            tknInt  = tknPtr->sParm.c.val.i;
        } /* End if */

      /* Otherwise, the token is an identifier */
      else
        token = tIDENT;

    } /* end else */

} /* End identifier */  

/***************************************************************/
/* Process string */

static void string(void)
{
  register int16_t count = 0;         /* # chars in string */

  token = tSTRING_CONST;             /* indicate string constant type */
  getCharacter();                    /* skip over 1st single quote */

  while (inChar != SQUOTE)           /* loop until next single quote */
    {
      if (inChar == '\n')            /* check for EOL in string */
        {
          error(eNOSQUOTE);          /* ERROR, terminate string */
          break;
        } /* end if */
      else
        {
          *stringSP++ = inChar;      /* concatenate character */
          count++;                   /* bump count of chars */
        } /* end else */
      getCharacter();                /* get the next character */
    } /* end while */
  *stringSP++ = '\0';                /* terminate ASCIIZ string */

  getCharacter();                    /* skip over last single quote */
  if (count == 1)                    /* Check for char constant */
    {
      token = tCHAR_CONST;           /* indicate char constant type */
      tknInt = *tkn_strt;            /* (integer) value = single char */
      stringSP = tkn_strt;           /* "pop" from string stack */
    } /* end if */
} /* end string */

/***************************************************************/

static void getCharacter(void)
{
  /* Get the next character from the line buffer.  If EOL, get next line */

  inChar = *(FP->cp)++;
  if (!inChar)
    {
      /* We have used all of the characters on this line.  Read the next
       * line of data
       */

      skipLine();
    }
}

/***************************************************************/

static void skipLine(void)
{
  if (getLine())
    {
      /* Uh-oh, we are out of data!  Just return some bogus value. */

      inChar = '?';
    } /* end if */
  else
    {
      /* Otherwise, get the first character from the line */

      getCharacter();
    }
}

/***************************************************************/

static bool getLine(void)
{
  bool endOfFile = false;

  /* Reset the character pointer to the start of the new line */

  FP->cp = FP->buffer;

  /* Read the next line from the currently active file */

  if (!fgets(FP->cp, LINE_SIZE, FP->stream))
    {
      /* We are at an EOF for this file.  Check if we are processing an
       * included file
       */

      if (includeIndex > 0)
        {
          /* Yes.  Close the file */

          closeNestedFile();

          /* Indicate that there is no data on the input line. NOTE:
           * that FP now refers to the previous file at the next lower
           * level of nesting.
           */

          FP->buffer[0] = '\0';
        } /* end if */
       else
         {
           /* No.  We are completely out of data.  Return true in this case. */

           endOfFile = true;
         } /* end else */
     } /* end if */
   else
     {
       /* We have a new line of data.  Increment the line number, then echo
        * the new line to the list file.
        */

       (FP->line)++;
       fprintf(lstFile, "%d:%04ld %s", FP->include, FP->line, FP->buffer);
     } /* end else */

   return endOfFile;

} /* end getLine */

/***************************************************************/

static void unsignedNumber(void)
{
  /* This logic (along with with unsignedRealNumber, and
   * unsignedRealExponent) handles:
   *
   * FORM: integer-number = decimal-integer | hexadecimal-integer |
   *       binary-integer
   * FORM: decimal-integer = digit-sequence
   * FORM: real-number =
   *       digit-sequence '.' [digit-sequence] [ exponent scale-factor ] |
   *       '.' digit-sequence [ exponent scale-factor ] |
   *       digit-sequence exponent scale-factor
   * FORM: exponent = 'e' | 'E'
   *
   * When called, inChar is equal to the leading digit of a
   * digit-sequence. NOTE that the real-number form beginning with
   * '.' does not use this logic.
   */

  /* Assume an integer type (might be real) */

  token = tINT_CONST;

  /* Concatenate all digits until an non-digit is found */

  do
    {
      *stringSP++ = inChar;
      getCharacter();
    }
  while (isdigit(inChar));

  /* If it is a digit-sequence followed by 'e' (or 'E'), then
   * continue processing this token as a real number.
   */

  if ((inChar == 'e') || (inChar == 'E'))
    {
      unsignedExponent();
    }

  /* If the digit-sequence is followed by '.' but not by ".." (i.e.,
   * this is not a subrange), then switch we are parsing a real time.
   * Otherwise, convert the integer string to binary.
   */

  else if ((inChar != '.') || (getNextCharacter(false) == '.'))
    {
      /* Terminate the integer string and convert it using sscanf */

      *stringSP++ = '\0';
      (void)sscanf(tkn_strt, "%ld", &tknInt);

      /* Remove the integer string from the character identifer stack */

      stringSP = tkn_strt;
    } /* end if */
  else
    {
      /* Its a real value!  Now really get the next character and
       * after the decimal point (this will work whether or not
       * getNextCharacter() was called). Then process the real number.
       */

      getCharacter();
      unsignedRealNumber();
    } /* end if */
}

/***************************************************************/

static void unsignedRealNumber(void)
{
  /* This logic (along with with unsignedNumber and unsignedExponent)
   * handles:
   *
   * FORM: real-number =
   *       digit-sequence '.' [digit-sequence] [ exponent scale-factor ] |
   *       '.' digit-sequence [ exponent scale-factor ] |
   *       digit-sequence exponent scale-factor
   * FORM: exponent = 'e' | 'E'
   *
   * When called:
   * - inChar is the character AFTER the '.'.
   * - Any leading digit-sequence is already in the character stack
   * - the '.' is not in the character stack.
   */

  /* Its a real constant */

  token = tREAL_CONST;

  /* Save the decimal point (inChar points to the character after
   * the decimal point).
   */

  *stringSP++ = '.';

  /* Now, loop to process the optional digit-sequence after the
   * decimal point.
   */

  while (isdigit(inChar))
    {
      *stringSP++ = inChar;
      getCharacter();
    }

  /* If it is a digit-sequence followed by 'e' (or 'E'), then
   * continue processing this token as a real number.
   */

  if ((inChar == 'e') || (inChar == 'E'))
    {
      unsignedExponent();
    }
  else
    {
      /* There is no exponent...
       * Terminate the real number string  and convert it to binay
       * using sscanf.
       */

      *stringSP++ = '\0';
      (void) sscanf(tkn_strt, "%lf", &tknReal);
    } /* end if */

  /* Remove the number string from the character identifer stack */

  stringSP = tkn_strt;
}

/***************************************************************/

static void unsignedExponent(void)
{
  /* This logic (along with with unsignedNumber and unsignedRealNumber)
   * handles:
   *
   * FORM: real-number =
   *       digit-sequence '.' [digit-sequence] [ exponent scale-factor ] |
   *       '.' digit-sequence [ exponent scale-factor ] |
   *       digit-sequence exponent scale-factor
   * FORM: exponent = 'e'
   * FORM: scale-factor = [ sign ] digit-sequence
   *
   * When called:
   * - inChar holds the 'E' (or 'e') exponent
   * - Any leading digit-sequences or decimal points are already in the
   *   character stack
   * - the 'E' (or 'e') is not in the character stack.
   */

  /* Its a real constant */

  token = tREAL_CONST;

  /* Save the decimal point (inChar points to the character after
   * the decimal point).
   */

  *stringSP++ = inChar;
  getCharacter();

  /* Check for an optional sign before the exponent value */

  if ((inChar == '-') || (inChar == '+'))
    {
      /* Add the sign to the stack */

      *stringSP++ = inChar;
      getCharacter();
    }
  else
    {
      /* Add a '+' sign to the stack */

      *stringSP++ = '+';
    }

  /* A digit sequence must appear after the exponent and optional
   * sign.
   */

  if (!isdigit(inChar))
    {
      error(eEXPONENT);
      tknReal = 0.0;
    }
  else
    {
      /* Now, loop to process the required digit-sequence */

      do
        {
          *stringSP++ = inChar;
          getCharacter();
        }
      while (isdigit(inChar));

      /* Terminate the real number string  and convert it to binay
       * using sscanf.
       */

      *stringSP++ = '\0';
      (void) sscanf(tkn_strt, "%lf", &tknReal);
    }

  /* Remove the number string from the character identifer stack */

  stringSP = tkn_strt;
}

/***************************************************************/

static void unsignedHexadecimal(void)
{
  /* FORM: integer-number = decimal-integer | hexadecimal-integer |
   *       binary-integer
   * FORM: hexadecimal-integer = '$' hex-digit-sequence
   * FORM: hex-digit-sequence = hex-digit { hex-digit }
   * FORM: hex-digit = digit | 'a' | 'b' | 'c' | 'd' | 'e' | 'f'
   *
   * On entry, inChar is '$'
   */

  /* This is another representation for an integer */

  token = tINT_CONST;

  /* Loop to process each hex 'digit' */

  for (;;)
    {
      /* Get the next character */

      getCharacter();

      /* Is it a decimal digit? */

      if (isdigit(inChar))
        *stringSP++ = inChar;

      /* Is it a hex 'digit'? */

      else if ((inChar >= 'A') && (inChar <= 'F'))
        *stringSP++ = inChar;

      else if ((inChar >= 'a') && (inChar <= 'f'))
        *stringSP++ = _toupper(inChar);

      /* Otherwise, that must be the end of the hex value */

      else break;
    }

  /* Terminate the hex string and convert to binary using sscanf */

  *stringSP++ = '\0';
  (void)sscanf(tkn_strt, "%lx", &tknInt);

  /* Remove the hex string from the character identifer stack */

  stringSP = tkn_strt;
}

/***************************************************************/

static void unsignedBinary(void)
{
  uint32_t value;

  /* FORM: integer-number = decimal-integer | hexadecimal-integer |
   *       binary-integer
   * FORM: binary-integer = '%' binary-digit-sequence
   * FORM: binary-digit-sequence = binary-digit { binary-digit }
   * FORM: binary-digit = '0' | '1'
   *
   * On entry, inChar is '%'
   */

  /* This is another representation for an integer */

  token = tINT_CONST;

  /* Loop to process each hex 'digit' */

  value = 0;

  for (;;)
    {
      /* Get the next character */

      getCharacter();

      /* Is it a binary 'digit'? */

      if (inChar == '0')
        value <<= 1;

      else if (inChar == '1')
        {
          value <<= 1;
          value  |= 1;
        }

      /* Otherwise, that must be the end of the binary value */

      else break;
    }

  /* I don't there there is an sscanf conversion for binary, that's
   * why we did it above.
   */

  tknInt = (int32_t)value;
}

/***************************************************************/
