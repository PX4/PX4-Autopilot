/****************************************************************************
 * apps/nshlib/nsh_codeccmd.c
 *
 * This file is part of NuttX, contributed by Darcy Gong
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Darcy Gong 2012-10-30
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
#ifdef CONFIG_NETUTILS_CODECS

#include <sys/stat.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <fcntl.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#if defined(CONFIG_NSH_DISABLE_URLENCODE) && defined(CONFIG_NSH_DISABLE_URLDECODE)
#  undef CONFIG_CODECS_URLCODE
#endif

#ifdef CONFIG_CODECS_URLCODE
#include <apps/netutils/urldecode.h>
#endif

#if defined(CONFIG_NSH_DISABLE_BASE64ENC) && defined(CONFIG_NSH_DISABLE_BASE64ENC)
#  undef CONFIG_CODECS_BASE64
#endif

#ifdef CONFIG_CODECS_BASE64
#include <apps/netutils/base64.h>
#endif

#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
#include <apps/netutils/md5.h>
#endif

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NSH_CODECS_BUFSIZE
#  define CONFIG_NSH_CODECS_BUFSIZE    128
#endif

#define CODEC_MODE_URLENCODE  1
#define CODEC_MODE_URLDECODE  2
#define CODEC_MODE_BASE64ENC  3
#define CODEC_MODE_BASE64DEC  4
#define CODEC_MODE_HASH_MD5   5

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*codec_callback_t)(FAR char *src_buff, int src_buff_len,
                                 FAR char *dst_buff, FAR int *dst_buff_len,
                                 int mode);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: urlencode_cb
 ****************************************************************************/

#if defined(CONFIG_CODECS_URLCODE) && !defined(CONFIG_NSH_DISABLE_URLENCODE)
static void urlencode_cb(FAR char *src_buff, int src_buff_len,
                         FAR char *dst_buff, FAR int *dst_buff_len, int mode)
{
  urlencode(src_buff,src_buff_len,dst_buff,dst_buff_len);
}
#endif

/****************************************************************************
 * Name: urldecode_cb
 ****************************************************************************/

#if defined(CONFIG_CODECS_URLCODE) && !defined(CONFIG_NSH_DISABLE_URLDECODE)
static void urldecode_cb(FAR char *src_buff, int src_buff_len, FAR char *dst_buff,
                         FAR int *dst_buff_len, int mode)
{
  urldecode(src_buff,src_buff_len,dst_buff,dst_buff_len);
}
#endif

/****************************************************************************
 * Name: b64enc_cb
 ****************************************************************************/

#if defined(CONFIG_CODECS_BASE64) && !defined(CONFIG_NSH_DISABLE_BASE64ENC)
static void b64enc_cb(FAR char *src_buff, int src_buff_len, FAR char *dst_buff,
                      FAR int *dst_buff_len, int mode)
{
  if (mode == 0)
    {
      //dst_buff =
      base64_encode((unsigned char *)src_buff, src_buff_len,
                    (unsigned char *)dst_buff, (size_t *)dst_buff_len);
    }
  else
    {
      //dst_buff =
      base64w_encode((unsigned char *)src_buff, src_buff_len,
                     (unsigned char *)dst_buff, (size_t *)dst_buff_len);
    }
}
#endif

/****************************************************************************
 * Name: b64dec_cb
 ****************************************************************************/

#if defined(CONFIG_CODECS_BASE64) && !defined(CONFIG_NSH_DISABLE_BASE64DEC)
static void b64dec_cb(FAR char *src_buff, int src_buff_len, FAR char *dst_buff,
                      FAR int *dst_buff_len, int mode)
{
  if (mode == 0)
    {
      //dst_buff =
      base64_decode((unsigned char *)src_buff, src_buff_len,
                    (unsigned char *)dst_buff, (size_t *)dst_buff_len);
    }
  else
    {
      //dst_buff =
      base64w_decode((unsigned char *)src_buff, src_buff_len,
                     (unsigned char *)dst_buff,(size_t *)dst_buff_len);
    }
}
#endif

/****************************************************************************
 * Name: md5_cb
 ****************************************************************************/

#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
static void md5_cb(FAR char *src_buff, int src_buff_len, FAR char *dst_buff,
                   FAR int *dst_buff_len, int mode)
{
  MD5Update((MD5_CTX *)dst_buff, (unsigned char *)src_buff, src_buff_len);
}
#endif

/****************************************************************************
 * Name: calc_codec_buffsize
 ****************************************************************************/

static int calc_codec_buffsize(int src_buffsize, uint8_t mode)
{
  switch (mode)
  {
  case CODEC_MODE_URLENCODE:
    return src_buffsize*3+1;
  case CODEC_MODE_URLDECODE:
    return src_buffsize+1;
  case CODEC_MODE_BASE64ENC:
    return ((src_buffsize + 2)/ 3 * 4)+1;
  case CODEC_MODE_BASE64DEC:
    return (src_buffsize / 4 * 3 + 2)+1;
  case CODEC_MODE_HASH_MD5:
    return 32+1;
  default:
    return src_buffsize+1;
  }
}

/****************************************************************************
 * Name: cmd_codecs_proc
 ****************************************************************************/

static int cmd_codecs_proc(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv,
                           uint8_t mode, codec_callback_t func)
{
#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
  static const unsigned char hex_chars[] = "0123456789abcdef";
  MD5_CTX ctx;
  unsigned char mac[16];
  char *pSrc;
  char *pDest;
#endif

  char *localfile = NULL;
  char *src_buffer = NULL;
  char *buffer = NULL;
  char *fullpath = NULL;
  const char *fmt;
  char *s_data;
  bool badarg = false;
  bool is_file = false;
  bool is_websafe=false;
  int option;
  int fd = -1;
  int buff_len = 0;
  int src_buff_len = 0;
  int i = 0;
  int ret = OK;

  /* Get the command options */

  while ((option = getopt(argc, argv, ":fw")) != ERROR)
    {
      switch (option)
        {
          case 'f':
            is_file = true;
            break;

#ifdef CONFIG_CODECS_BASE64
          case 'w':
            is_websafe = true;

            if (!(mode == CODEC_MODE_BASE64ENC || mode == CODEC_MODE_BASE64DEC))
              {
                badarg = true;
              }
            break;
#endif
          case ':':
            nsh_output(vtbl, g_fmtargrequired, argv[0]);
            badarg = true;
            break;

          case '?':
          default:
            nsh_output(vtbl, g_fmtarginvalid, argv[0]);
            badarg = true;
            break;
        }
    }

  /* If a bad argument was encountered, then return without processing the command */

  if (badarg)
    {
      return ERROR;
    }

  /* There should be exactly on parameter left on the command-line */

  if (optind == argc-1)
    {
      s_data = argv[optind];
    }
  else if (optind >= argc)
    {
      fmt = g_fmttoomanyargs;
      goto errout;
    }
  else
    {
      fmt = g_fmtargrequired;
      goto errout;
    }

#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
  if (mode == CODEC_MODE_HASH_MD5)
    {
      MD5Init(&ctx);
    }
#endif

  if (is_file)
    {
      /* Get the local file name */

      localfile = s_data;

      /* Get the full path to the local file */

      fullpath = nsh_getfullpath(vtbl, localfile);

      /* Open the local file for writing */

      fd = open(fullpath, O_RDONLY|O_TRUNC, 0644);
      if (fd < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "open", NSH_ERRNO);
          ret = ERROR;
          goto exit;
        }

      src_buffer = malloc(CONFIG_NSH_CODECS_BUFSIZE+2);
#if defined(CONFIG_CODECS_BASE64) && !defined(CONFIG_NSH_DISABLE_BASE64ENC)
      if (mode == CODEC_MODE_BASE64ENC)
        {
          src_buff_len = CONFIG_NSH_CODECS_BUFSIZE / 3 * 3;
        }
      else
#endif
        {
          src_buff_len = CONFIG_NSH_CODECS_BUFSIZE;
        }

      buff_len = calc_codec_buffsize(src_buff_len+2, mode);
      buffer = malloc(buff_len);
      while(true)
        {
          memset(src_buffer, 0, src_buff_len+2);
          ret=read(fd, src_buffer, src_buff_len);
          if (ret < 0)
            {
              nsh_output(vtbl, g_fmtcmdfailed, argv[0], "read", NSH_ERRNO);
              ret = ERROR;
              goto exit;
            }
          else if(ret==0)
            {
              break;
            }

#if defined(CONFIG_CODECS_URLCODE) && !defined(CONFIG_NSH_DISABLE_URLDECODE)
          if (mode == CODEC_MODE_URLDECODE)
            {
              if (src_buffer[src_buff_len-1]=='%')
                {
                  ret += read(fd,&src_buffer[src_buff_len],2);
                }
              else if (src_buffer[src_buff_len-2]=='%')
                {
                  ret += read(fd,&src_buffer[src_buff_len],1);
                }
            }
#endif
          memset(buffer, 0, buff_len);
          if (func)
            {
#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
              if (mode == CODEC_MODE_HASH_MD5)
                {
                  func(src_buffer, ret, (char *)&ctx, &buff_len,0);
                }
              else
#endif
                {
                  func(src_buffer, ret, buffer, &buff_len,(is_websafe)?1:0);
                  nsh_output(vtbl, "%s", buffer);
                }
            }

          buff_len = calc_codec_buffsize(src_buff_len+2, mode);
        }

#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
      if (mode == CODEC_MODE_HASH_MD5)
        {
          MD5Final(mac, &ctx);
          pSrc = (char *)&mac;
          pDest = buffer;
          for(i=0;i<16;i++,pSrc++)
            {
              *pDest++ = hex_chars[(*pSrc) >> 4];
              *pDest++ = hex_chars[(*pSrc) & 0x0f];
            }

          *pDest='\0';
          nsh_output(vtbl, "%s\n", buffer);
        }
#endif
      ret = OK;
      goto exit;
    }
  else
    {
      src_buffer = s_data;
      src_buff_len = strlen(s_data);
      buff_len = calc_codec_buffsize(src_buff_len, mode);
      buffer = malloc(buff_len);
      buffer[0]=0;
      if (!buffer)
        {
          fmt = g_fmtcmdoutofmemory;
          goto errout;
        }

      memset(buffer, 0, buff_len);
      if (func)
        {
#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
          if (mode == CODEC_MODE_HASH_MD5)
            {
              func(src_buffer, src_buff_len, (char *)&ctx, &buff_len, 0);
              MD5Final(mac, &ctx);
              pSrc = (char *)&mac;
              pDest = buffer;
              for(i=0;i<16;i++,pSrc++)
                {
                  *pDest++ = hex_chars[(*pSrc) >> 4];
                  *pDest++ = hex_chars[(*pSrc) & 0x0f];
                }

              *pDest='\0';
            }
          else
#endif
            {
              func(src_buffer, src_buff_len, buffer, &buff_len,(is_websafe)?1:0);
            }
        }

      nsh_output(vtbl, "%s\n",buffer);
      src_buffer = NULL;
      goto exit;
    }

exit:
  if (fd >= 0)
    {
      close(fd);
    }

  if (fullpath)
    {
      free(fullpath);
    }

  if (src_buffer)
    {
      free(src_buffer);
    }

  if (buffer)
    {
      free(buffer);
    }

  return ret;

errout:
  nsh_output(vtbl, fmt, argv[0]);
  ret = ERROR;
  goto exit;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_urlencode
 ****************************************************************************/

#if defined(CONFIG_CODECS_URLCODE) && !defined(CONFIG_NSH_DISABLE_URLENCODE)
int cmd_urlencode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return cmd_codecs_proc(vtbl, argc, argv, CODEC_MODE_URLENCODE, urlencode_cb);
}
#endif

/****************************************************************************
 * Name: cmd_urldecode
 ****************************************************************************/

#if defined(CONFIG_CODECS_URLCODE) && !defined(CONFIG_NSH_DISABLE_URLDECODE)
int cmd_urldecode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return cmd_codecs_proc(vtbl, argc, argv, CODEC_MODE_URLDECODE, urldecode_cb);
}
#endif

/****************************************************************************
 * Name: cmd_base64encode
 ****************************************************************************/

#if defined(CONFIG_CODECS_BASE64) && !defined(CONFIG_NSH_DISABLE_BASE64ENC)
int cmd_base64encode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return cmd_codecs_proc(vtbl, argc, argv, CODEC_MODE_BASE64ENC, b64enc_cb);
}
#endif

/****************************************************************************
 * Name: cmd_base64decode
 ****************************************************************************/

#if defined(CONFIG_CODECS_BASE64) && !defined(CONFIG_NSH_DISABLE_BASE64DEC)
int cmd_base64decode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return cmd_codecs_proc(vtbl, argc, argv, CODEC_MODE_BASE64DEC, b64dec_cb);
}
#endif

/****************************************************************************
 * Name: cmd_md5
 ****************************************************************************/

#if defined(CONFIG_CODECS_HASH_MD5) && !defined(CONFIG_NSH_DISABLE_MD5)
int cmd_md5(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return cmd_codecs_proc(vtbl,argc,argv,CODEC_MODE_HASH_MD5,md5_cb);
}
#endif

#endif /* CONFIG_NETUTILS_CODECS */
