/****************************************************************************
 * apps/include/netutils/base64.h
 *
 * This file is part of the NuttX RTOS:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Darcy Gong
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <apps/netutils/urldecode.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE
#  define IS_HEX_CHAR(ch) \
  ((ch >= '0' && ch <= '9') || \
   (ch >= 'a' && ch <= 'f') || \
   (ch >= 'A' && ch <= 'F'))

#  define HEX_VALUE(ch, value) \
  if (ch >= '0' && ch <= '9') \
    { \
      value = ch - '0'; \
    } \
  else if (ch >= 'a' && ch <= 'f') \
    { \
      value = ch - 'a' + 10; \
    } \
  else \
    { \
      value = ch - 'A' + 10; \
    }
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: from_hex
 *
 * Description:
 *   Converts a hex character to its integer value,
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE_NEWMEMORY
static char from_hex(char ch)
{
  return isdigit(ch) ? ch - '0' : tolower(ch) - 'a' + 10;
}
#endif

/****************************************************************************
 * Name:
 *
 * Description:
 *   Converts an integer value to its hex character,
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE_NEWMEMORY
static char to_hex(char code)
{
  static const char hex[] = "0123456789abcdef";
  return hex[code & 15];
}
#endif

/****************************************************************************
 * Name: int2h
 *
 * Description:
 *   Convert a single character to a 2 digit hex str a terminating '\0' is
 *   added
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_AVR_URLCODE
static void int2h(char c, char *hstr)
{
  hstr[1] = (c & 0xf) + '0';
  if ((c & 0xf) > 9)
    {
      hstr[1] = (c & 0xf) - 10 + 'a';
    }

  c = (c >> 4) & 0xf;
  hstr[0] = c + '0';
  if (c > 9)
    {
      hstr[0] = c - 10 + 'a';
    }

  hstr[2] = '\0';
}
#endif

/****************************************************************************
 * Name: h2int
 *
 * Description:
 *   Convert a single hex digit character to its integer value.
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_AVR_URLCODE
static unsigned char h2int(char c)
{
  if (c >= '0' && c <= '9')
    {
      return ((unsigned char)c - '0');
    }

  if (c >= 'a' && c <= 'f')
    {
      return ((unsigned char)c - 'a' + 10);
    }

  if (c >= 'A' && c <= 'F')
    {
      return ((unsigned char)c - 'A' + 10);
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:
 *
 * Description:
 *   Returns a url-encoded version of str.
 *
 *   IMPORTANT: be sure to free() the returned string after use.
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE_NEWMEMORY
char *url_encode(char *str)
{
  char *pstr = str;
  char *buf = malloc(strlen(str) * 3 + 1);
  char *pbuf = buf;

  while (*pstr)
    {
      if (isalnum(*pstr) || *pstr == '-' || *pstr == '_' || *pstr == '.' ||  *pstr == '~')
        {
          *pbuf++ = *pstr;
        }
      else if (*pstr == ' ')
        {
          *pbuf++ = '+';
        }
      else
        {
          *pbuf++ = '%';
          *pbuf++ = to_hex(*pstr >> 4);
          *pbuf++ = to_hex(*pstr & 15);
        }

      pstr++;
    }

  *pbuf = '\0';
  return buf;
}
#endif

/****************************************************************************
 * Name: url_decode
 *
 * Description:
 *   Returns a url-decoded version of str.
 *
 *   IMPORTANT: be sure to free() the returned string after use.
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE_NEWMEMORY
char *url_decode(char *str)
{
  char *pstr = str;
  char *buf = malloc(strlen(str) + 1);
  char *pbuf = buf;

  while (*pstr)
    {
      if (*pstr == '%')
        {
          if (pstr[1] && pstr[2])
            {
              *pbuf++ = from_hex(pstr[1]) << 4 | from_hex(pstr[2]);
              pstr += 2;
            }
        }
      else if (*pstr == '+')
        {
          *pbuf++ = ' ';
        }
      else
        {
          *pbuf++ = *pstr;
        }

      pstr++;
    }

  *pbuf = '\0';
  return buf;
}
#endif

/****************************************************************************
 * Name: urlencode
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE
char *urlencode(const char *src, const int src_len, char *dest, int *dest_len)
{
  static const unsigned char hex_chars[] = "0123456789ABCDEF";
  const unsigned char *pSrc;
  const unsigned char *pEnd;
  char *pDest;

  pDest = dest;
  pEnd = (unsigned char *)src + src_len;
  for (pSrc = (unsigned char *)src; pSrc < pEnd; pSrc++)
    {
      if ((*pSrc >= '0' && *pSrc <= '9') ||
          (*pSrc >= 'a' && *pSrc <= 'z') ||
          (*pSrc >= 'A' && *pSrc <= 'Z') ||
          (*pSrc == '_' || *pSrc == '-' || *pSrc == '.' || *pSrc == '~'))
        {
          *pDest++ = *pSrc;
        }
      else if (*pSrc == ' ')
        {
          *pDest++ = '+';
        }
      else
        {
          *pDest++ = '%';
          *pDest++ = hex_chars[(*pSrc) >> 4];
          *pDest++ = hex_chars[(*pSrc) & 0x0F];
        }
    }

  *pDest = '\0';
  *dest_len = pDest - dest;
  return dest;
}
#endif

/****************************************************************************
 * Name: urldecode
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE
char *urldecode(const char *src, const int src_len, char *dest, int *dest_len)
{
  const unsigned char *pSrc;
  const unsigned char *pEnd;
  char *pDest;
  unsigned char cHigh;
  unsigned char cLow;
  int valHigh;
  int valLow;

  pDest = dest;
  pSrc = (unsigned char *)src;
  pEnd = (unsigned char *)src + src_len;
  while (pSrc < pEnd)
    {
      if (*pSrc == '%' && pSrc + 2 < pEnd)
        {
          cHigh = *(pSrc + 1);
          cLow = *(pSrc + 2);

          if (IS_HEX_CHAR(cHigh) && IS_HEX_CHAR(cLow))
            {
              HEX_VALUE(cHigh, valHigh)
              HEX_VALUE(cLow, valLow)
              *pDest++ = (valHigh << 4) | valLow;
              pSrc += 3;
            }
          else
            {
              *pDest++ = *pSrc;
              pSrc++;
            }
        }
      else if (*pSrc == '+')
        {
          *pDest++ = ' ';
          pSrc++;
        }
      else
        {
          *pDest++ = *pSrc;
          pSrc++;
        }
    }

  *pDest = '\0';
  *dest_len = pDest - dest;
  return dest;
}
#endif

/****************************************************************************
 * Name: urlencode_len
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE
int urlencode_len(const char *src, const int src_len)
{
  const unsigned char *pSrc;
  const unsigned char *pEnd;
  int len = 0;

  pEnd = (unsigned char *)src + src_len;
  for (pSrc = (unsigned char *)src; pSrc < pEnd; pSrc++)
    {
      if ((*pSrc >= '0' && *pSrc <= '9') ||
          (*pSrc >= 'a' && *pSrc <= 'z') ||
          (*pSrc >= 'A' && *pSrc <= 'Z') ||
          (*pSrc == '_' || *pSrc == '-' || *pSrc == '.' || *pSrc == '~' || *pSrc == ' '))
        {
          len++;
        }
      else
        {
          len+=3;
        }
    }

  return len;
}
#endif

/****************************************************************************
 * Name: urldecode_len
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE
int urldecode_len(const char *src, const int src_len)
{
  const unsigned char *pSrc;
  const unsigned char *pEnd;
  int len = 0;
  unsigned char cHigh;
  unsigned char cLow;

  pSrc = (unsigned char *)src;
  pEnd = (unsigned char *)src + src_len;
  while (pSrc < pEnd)
    {
      if (*pSrc == '%' && pSrc + 2 < pEnd)
        {
          cHigh = *(pSrc + 1);
          cLow = *(pSrc + 2);

          if (IS_HEX_CHAR(cHigh) && IS_HEX_CHAR(cLow))
            {
              pSrc += 2;
            }
        }

      len++;
      pSrc++;
    }

  return len;
}
#endif

/****************************************************************************
 * Name: urlrawdecode
 *
 * Description:
 * decode a url string e.g "hello%20joe" or "hello+joe" becomes "hello joe"
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_AVR_URLCODE
void urlrawdecode(char *urlbuf)
{
  char c;
  char *dst;
  dst = urlbuf;
  while ((c = *urlbuf))
    {
      if (c == '+')
        {
          c = ' ';
        }

      if (c == '%')
        {
          urlbuf++;
          c = *urlbuf;
          urlbuf++;
          c = (h2int(c) << 4) | h2int(*urlbuf);
        }

      *dst = c;
      dst++;
      urlbuf++;
    }

  *dst = '\0';
}
#endif

/****************************************************************************
 * Name: urlrawencode
 *
 * Description:
 *   There must be enoug space in urlbuf. In the worst case that is 3 times
 *   the length of str
 *
 ****************************************************************************/

#ifdef CONFIG_CODECS_AVR_URLCODE
void urlrawencode(char *str, char *urlbuf)
{
  char c;
  while ((c = *str))
    {
      if (c == ' ' || isalnum(c) || c == '_' || c == '-' || c == '.' || c == '~')
        {
          if (c == ' ')
            {
              c = '+';
            }

          *urlbuf = c;
          str++;
          urlbuf++;
          continue;
        }

      *urlbuf = '%';
      urlbuf++;
      int2h(c, urlbuf);
      urlbuf++;
      urlbuf++;
      str++;
    }

  *urlbuf = '\0';
}
#endif
