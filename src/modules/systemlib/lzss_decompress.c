/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 ****************************************************************************
 *
 * Portions of this module are derived from code attributed:
 * **************************************************************
 *        LZSS.C -- A Data Compression Program
 * ***************************************************************
 *        4/6/1989 Haruhiko Okumura
 *        Use, distribute, and modify this program freely.
 *        Please send me your improved versions.
 *                PC-VAN          SCIENCE
 *                NIFTY-Serve     PAF01022
 *                CompuServe      74050,1022
 * **************************************************************
 *
 ****************************************************************************/

/**
 * @file	lzss_decompress.c
 *
 * Simple and slightly enhanced LZSS decompression engine.
 */

#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "lzss.h"

static int	lz_getc(struct lzss_decomp *s);
static void	lz_putc(struct lzss_decomp *s, int c);

__EXPORT int
lzss_decompress(struct lzss_decomp *s)
{
	int  i, j, k, r, c;
	unsigned int  flags;

	if (s->N == 0)
		s->N = LZSS_DEFAULT_N;
	if (s->F == 0)
		s->F = LZSS_DEFAULT_F;
	if (s->THRESHOLD == 0)
		s->THRESHOLD = LZSS_DEFAULT_THRESHOLD;

	uint8_t *text_buf = (uint8_t *)malloc(s->N + s->F - 1);
	if (text_buf == NULL)
		return ENOMEM;

	r = s->N - s->F;
	memset(text_buf, ' ', r);
	flags = 0;

	while (s->resid > 0) {
		if (((flags >>= 1) & 256) == 0) {
			c = lz_getc(s);
			flags = c | 0xff00;             /* uses higher byte cleverly */
		}                                       /* to count eight */

		if (flags & 1) {
			c = lz_getc(s);
			lz_putc(s, c);
			text_buf[r++] = c;
			r &= (s->N - 1);

		} else {
			i = lz_getc(s);
			j = lz_getc(s);

			i |= ((j & 0xf0) << 4);
			j = (j & 0x0f) + s->THRESHOLD;

			for (k = 0; k <= j; k++) {
				c = text_buf[(i + k) & (s->N - 1)];

				lz_putc(s, c);
				text_buf[r++] = c;
				r &= (s->N - 1);
			}
		}
	}

	free(text_buf);
	return 0;
}

int
lz_getc(struct lzss_decomp *s)
{
	return *(s->src++);
}

void
lz_putc(struct lzss_decomp *s, int c)
{
	if (s->discard > 0) {
		s->discard--;
	} else if (s->resid > 0) {
		*(s->dst++) = c;
		s->resid--;
	}
}
