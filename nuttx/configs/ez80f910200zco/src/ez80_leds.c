/****************************************************************************
 * configs/ez80f910200zco/src/ez80_leds.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>
#include "ez80f910200zco.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 5x7 LED matrix character glyphs.  Each glyph consists of 7 bytes, one
 * each row and each containing 5 bits of data, one for each column
 */

#if 0 /* Not used */
static const uint8_t g_chblock[7]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* block */
#endif

static const uint8_t g_chspace[7]  = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};  /* space */

#if 0 /* Not used */
static const uint8_t g_chexclam[7] = {0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1f, 0x1b};  /* ! */
static const uint8_t g_chquote[7]  = {0x15, 0x15, 0x15, 0x1f, 0x1f, 0x1f, 0x1f};  /* " */
static const uint8_t g_chnum[7]    = {0x1f, 0x15, 0x00, 0x15, 0x00, 0x15, 0x1f};  /* # */
static const uint8_t g_chdollar[7] = {0x1b, 0x11, 0x0a, 0x11, 0x0a, 0x11, 0x1b};  /* $ */
static const uint8_t g_chpct[7]    = {0x1f, 0x1e, 0x15, 0x1b, 0x15, 0x0f, 0x1f};  /* % */
static const uint8_t g_champ[7]    = {0x11, 0x0e, 0x0e, 0x11, 0x15, 0x0e, 0x10};  /* & */
static const uint8_t g_chsquote[7] = {0x1b, 0x1b, 0x1b, 0x1f, 0x1f, 0x1f, 0x1f};  /* ' */
static const uint8_t g_chlparen[7] = {0x1d, 0x1b, 0x17, 0x17, 0x17, 0x1b, 0x1d};  /* ( */
static const uint8_t g_chrparen[7] = {0x17, 0x1b, 0x1d, 0x1d, 0x1d, 0x1b, 0x17};  /* ) */
#endif

static const uint8_t g_chast[7]    = {0x1f, 0x0a, 0x11, 0x00, 0x11, 0x0a, 0x1f};  /* * */

#if 0 /* Not used */
static const uint8_t g_chplus[7]   = {0x1f, 0x1b, 0x1b, 0x00, 0x1b, 0x1b, 0x1f};  /* + */
static const uint8_t g_chcomma[7]  = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1b, 0x17};  /* , */
static const uint8_t g_chhyphen[7] = {0x1f, 0x1f, 0x1f, 0x00, 0x1f, 0x1f, 0x1f};  /* - */
static const uint8_t g_chperiod[7] = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1b};  /* . */
static const uint8_t g_chslash[7]  = {0x1f, 0x1e, 0x1d, 0x1b, 0x17, 0x0f, 0x1f};  /* / */
#endif

static const uint8_t g_ch0[7]      = {0x11, 0x0e, 0x0c, 0x0a, 0x06, 0x0e, 0x11};  /* 0 */

#if 0 /* Not used */
static const uint8_t g_ch1[7]      = {0x1b, 0x13, 0x1b, 0x1b, 0x1b, 0x1b, 0x11};  /* 1 */
static const uint8_t g_ch2[7]      = {0x11, 0x0e, 0x1d, 0x1b, 0x17, 0x0f, 0x00};  /* 2 */
static const uint8_t g_ch3[7]      = {0x11, 0x0e, 0x1e, 0x19, 0x1e, 0x0e, 0x11};  /* 3 */
static const uint8_t g_ch4[7]      = {0x0e, 0x0e, 0x0e, 0x10, 0x1e, 0x1e, 0x1e};  /* 4 */
static const uint8_t g_ch5[7]      = {0x00, 0x0f, 0x0f, 0x01, 0x1e, 0x0e, 0x11};  /* 5 */
static const uint8_t g_ch6[7]      = {0x11, 0x0f, 0x0f, 0x01, 0x0e, 0x0e, 0x11};  /* 6 */
static const uint8_t g_ch7[7]      = {0x00, 0x1e, 0x1e, 0x1d, 0x1b, 0x1b, 0x1b};  /* 7 */
static const uint8_t g_ch8[7]      = {0x11, 0x0e, 0x0e, 0x11, 0x0e, 0x0e, 0x11};  /* 8 */
static const uint8_t g_ch9[7]      = {0x11, 0x0e, 0x0e, 0x10, 0x1e, 0x1d, 0x1b};  /* 9 */
static const uint8_t g_chcolon[7]  = {0x1f, 0x1f, 0x1b, 0x1f, 0x1b, 0x1f, 0x1f};  /* : */
static const uint8_t g_shsemi[7]   = {0x1f, 0x1f, 0x1b, 0x1f, 0x1b, 0x17, 0x1f};  /* ; */
static const uint8_t g_chlt[7]     = {0x1d, 0x1b, 0x17, 0x0f, 0x17, 0x1b, 0x1d};  /* < */
static const uint8_t g_cheq[7]     = {0x1f, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x1f};  /* = */
static const uint8_t g_chgt[7]     = {0x17, 0x1b, 0x1d, 0x1e, 0x1d, 0x1b, 0x17};  /* > */
static const uint8_t g_chquest[7]  = {0x11, 0x0e, 0x0d, 0x1b, 0x1b, 0x1f, 0x1b};  /* ? */
static const uint8_t g_chat[7]     = {0x11, 0x0a, 0x04, 0x04, 0x05, 0x0a, 0x11};  /* @ */
#endif

static const uint8_t g_chA[7]      = {0x11, 0x0e, 0x0e, 0x0e, 0x00, 0x0e, 0x0e};  /* A */

#if 0 /* Not used */
static const uint8_t g_chB[7]      = {0x01, 0x0e, 0x0e, 0x01, 0x0e, 0x0e, 0x01};  /* B */
#endif

static const uint8_t g_chC[7]      = {0x11, 0x0e, 0x0f, 0x0f, 0x0f, 0x0e, 0x11};  /* C */

#if 0 /* Not used */
static const uint8_t g_chD[7]       = {0x01, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x01};  /* D */
#endif

static const uint8_t g_chE[7]       = {0x00, 0x0f, 0x0f, 0x01, 0x0f, 0x0f, 0x00};  /* E */

#if 0 /* Not used */
static const uint8_t g_chF[7]      = {0x00, 0x0f, 0x0f, 0x01, 0x0f, 0x0f, 0x0f};  /* F */
static const uint8_t g_chG[7]      = {0x11, 0x0e, 0x0f, 0x08, 0x0e, 0x0e, 0x11};  /* G */
#endif

static const uint8_t g_chH[7]      = {0x0e, 0x0e, 0x0e, 0x00, 0x0e, 0x0e, 0x0e};  /* H */
static const uint8_t g_chI[7]      = {0x00, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x00};  /* I */

#if 0 /* Not used */
static const uint8_t g_chJ[7]      = {0x00, 0x1d, 0x1d, 0x1d, 0x0d, 0x0d, 0x13};  /* J */
static const uint8_t g_chK[7]      = {0x0e, 0x0d, 0x0b, 0x07, 0x0b, 0x0d, 0x0e};  /* K */
static const uint8_t g_chL[7]      = {0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x00};  /* L */
static const uint8_t g_chM[7]      = {0x0e, 0x04, 0x0a, 0x0a, 0x0e, 0x0e, 0x0e};  /* M */
static const uint8_t g_chN[7]      = {0x0e, 0x0e, 0x06, 0x0a, 0x0c, 0x0e, 0x0e};  /* N */
static const uint8_t g_chO[7]      = {0x11, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x11};  /* O */
static const uint8_t g_chP[7]      = {0x01, 0x0e, 0x0e, 0x01, 0x0f, 0x0f, 0x0f};  /* P */
static const uint8_t g_chQ[7]      = {0x11, 0x0e, 0x0e, 0x0e, 0x0a, 0x0c, 0x10};  /* Q */
#endif

static const uint8_t g_chR[7]      = {0x01, 0x0e, 0x0e, 0x01, 0x0b, 0x0d, 0x0e};  /* R */
static const uint8_t g_chS[7]      = {0x11, 0x0e, 0x0f, 0x11, 0x1e, 0x0e, 0x11};  /* S */

#if 0 /* Not used */
static const uint8_t g_chT[7]      = {0x00, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b};  /* T */
static const uint8_t g_chU[7]      = {0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x11};  /* U */
static const uint8_t g_chV[7]      = {0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x15, 0x1b};  /* V */
static const uint8_t g_chW[7]      = {0x0e, 0x0e, 0x0a, 0x0a, 0x0a, 0x0a, 0x15};  /* W */
static const uint8_t g_chX[7]      = {0x0e, 0x0e, 0x15, 0x1b, 0x15, 0x0e, 0x0e};  /* X */
static const uint8_t g_chY[7]      = {0x0e, 0x0e, 0x15, 0x1b, 0x1b, 0x1b, 0x1b};  /* Y */
static const uint8_t g_chZ[7]      = {0x00, 0x1e, 0x1d, 0x1b, 0x17, 0x0f, 0x00};  /* Z */
static const uint8_t g_chlbrack[7] = {0x03, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x03};  /* [ */
static const uint8_t g_chbslash[7] = {0x1f, 0x0f, 0x17, 0x1b, 0x1d, 0x1e, 0x1f};  /* backslash */
static const uint8_t g_chrbrack[7] = {0x1c, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1c};  /* ] */
static const uint8_t g_chcaret[7]  = {0x1b, 0x15, 0x0e, 0x1f, 0x1f, 0x1f, 0x1f};  /* ^ */
static const uint8_t g_chunder[7]  = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x00};  /* _ */
static const uint8_t g_chgrave[7]  = {0x1b, 0x1b, 0x1b, 0x1f, 0x1f, 0x1f, 0x1f};  /* ' */
static const uint8_t g_cha[7]      = {0x1f, 0x1f, 0x19, 0x16, 0x16, 0x16, 0x18};  /* a */
static const uint8_t g_chb[7]      = {0x17, 0x17, 0x11, 0x16, 0x16, 0x16, 0x11};  /* b */
static const uint8_t g_chc[7]      = {0x1f, 0x1f, 0x19, 0x16, 0x17, 0x16, 0x19};  /* c */
static const uint8_t g_chd[7]      = {0x1e, 0x1e, 0x18, 0x16, 0x16, 0x16, 0x18};  /* d */
static const uint8_t g_che[7]      = {0x1f, 0x1f, 0x19, 0x10, 0x17, 0x16, 0x19};  /* e */
static const uint8_t g_chf[7]      = {0x1d, 0x1a, 0x1b, 0x11, 0x1b, 0x1b, 0x1b};  /* f */
static const uint8_t g_chg[7]      = {0x1f, 0x19, 0x16, 0x16, 0x18, 0x16, 0x19};  /* g */
static const uint8_t g_chh[7]      = {0x17, 0x17, 0x11, 0x16, 0x16, 0x16, 0x16};  /* h */
static const uint8_t g_chi[7]      = {0x1f, 0x1f, 0x1b, 0x1f, 0x1b, 0x1b, 0x1b};  /* i */
static const uint8_t g_chj[7]      = {0x1f, 0x1d, 0x1f, 0x1d, 0x1d, 0x1d, 0x13};  /* j */
static const uint8_t g_chk[7]      = {0x17, 0x17, 0x15, 0x13, 0x13, 0x15, 0x16};  /* k */
static const uint8_t g_chl[7]      = {0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b};  /* l */
static const uint8_t g_chm[7]      = {0x1f, 0x1f, 0x05, 0x0a, 0x0a, 0x0a, 0x0a};  /* m */
static const uint8_t g_chn[7]      = {0x1f, 0x1f, 0x11, 0x16, 0x16, 0x16, 0x16};  /* n */
static const uint8_t g_cho[7]      = {0x1f, 0x1f, 0x19, 0x16, 0x16, 0x16, 0x19};  /* o */
static const uint8_t g_chp[7]      = {0x1f, 0x11, 0x16, 0x16, 0x11, 0x17, 0x17};  /* p */
static const uint8_t g_chq[7]      = {0x1f, 0x18, 0x16, 0x16, 0x18, 0x1e, 0x1e};  /* q */
static const uint8_t g_chr[7]      = {0x1f, 0x1f, 0x11, 0x16, 0x17, 0x17, 0x17};  /* r */
static const uint8_t g_chs[7]      = {0x1f, 0x1f, 0x18, 0x17, 0x19, 0x1e, 0x11};  /* s */
static const uint8_t g_cht[7]      = {0x1f, 0x1f, 0x1b, 0x11, 0x1b, 0x1b, 0x1b};  /* t */
static const uint8_t g_chu[7]      = {0x1f, 0x1f, 0x16, 0x16, 0x16, 0x16, 0x18};  /* u */
static const uint8_t g_chv[7]      = {0x1f, 0x1f, 0x16, 0x16, 0x16, 0x16, 0x19};  /* v */
static const uint8_t g_chw[7]      = {0x1f, 0x1f, 0x0a, 0x0a, 0x0a, 0x0a, 0x15};  /* w */
static const uint8_t g_chx[7]      = {0x1f, 0x1f, 0x0e, 0x15, 0x1b, 0x15, 0x0e};  /* x */
static const uint8_t g_chy[7]      = {0x1f, 0x1a, 0x1a, 0x1a, 0x1d, 0x1b, 0x17};  /* y */
static const uint8_t g_cha[7]      = {0x1f, 0x1f, 0x10, 0x1d, 0x1b, 0x17, 0x10};  /* z */
static const uint8_t g_chlbrace[7] = {0x1d, 0x1b, 0x1b, 0x17, 0x1b, 0x1b, 0x1d};  /* { */
static const uint8_t g_chvbar[7]   = {0x1b, 0x1b, 0x1b, 0x1f, 0x1b, 0x1b, 0x1b};  /* | */
static const uint8_t g_chrbrace[7] = {0x17, 0x1b, 0x1b, 0x1d, 0x1b, 0x1b, 0x17};  /* } */
static const uint8_t g_chtilde[7]  = {0x1f, 0x1a, 0x15, 0x1f, 0x1f, 0x1f, 0x1f};  /* ~ */
#endif

/* The current and previously selected glyph */

static const uint8_t *g_currglyph = g_chspace;
static const uint8_t *g_prevglyph = g_chspace;

/* Current row and column */

static uint8_t g_anodecol         = 1;
static uint8_t g_cathoderow       = 0;
static int8_t  g_intcount         = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
  g_currglyph  = g_chspace;
  g_prevglyph  = g_chspace;
  g_anodecol   = 1;
  g_cathoderow = 0;
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  FAR const uint8_t *tmp = g_currglyph;
  switch (led)
    {
    case LED_STARTED:
      g_currglyph = g_ch0;
      break;

    case LED_HEAPALLOCATE:
      g_currglyph = g_chH;
      break;

    case LED_IRQSENABLED:
      g_currglyph = g_chE;
      break;

    case LED_STACKCREATED:
      g_currglyph = g_chC;
      break;

    case LED_IDLE:
      g_currglyph = g_chR;
      break;

    case LED_INIRQ:
      g_intcount++;
      return;

    case LED_ASSERTION:
      g_currglyph = g_chA;
      break;

    case LED_SIGNAL:
      g_currglyph = g_chS;
      break;

    case LED_PANIC:
      g_currglyph = g_chast;
      break;

    default:
      return;    
    }

  g_prevglyph = tmp;
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  if (led == LED_INIRQ)
    {
      g_intcount--;
    }
  else if (led == LED_PANIC && g_intcount > 0)
    {
      g_currglyph = g_chI;
    }
  else
    {
      g_currglyph = g_prevglyph;
    }
}

/****************************************************************************
 * Name: up_timerhook
 ****************************************************************************/

 void up_timerhook(void)
{
  if (g_cathoderow > 6)
    {
      g_anodecol   = 1;
      g_cathoderow = 0;
    }

  ez80_putmmreg8(g_anodecol, EZ80_LEDANODE);
  ez80_putmmreg8(g_currglyph[g_cathoderow], EZ80_LEDCATHODE);

  g_cathoderow++;
  g_anodecol = g_anodecol << 1;
}

#endif /* CONFIG_ARCH_LEDS */
