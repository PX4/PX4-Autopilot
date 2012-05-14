/****************************************************************************
 * arch/arm/src/calypso/calypso_uwire.c
 * Driver for Calypso uWire Master Controller
 *
 * (C) 2010 by Sylvain Munaut <tnt@246tNt.com>
 *
 * This source code is derivated from Osmocom-BB project and was
 * relicensed as BSD with permission from original authors.
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
 **************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <debug.h>

#include "up_arch.h"

#define BASE_ADDR_UWIRE	0xfffe4000
#define UWIRE_REG(n)	(BASE_ADDR_UWIRE+(n))

enum uwire_regs {
	REG_DATA	= 0x00,
	REG_CSR		= 0x02,
	REG_SR1		= 0x04,
	REG_SR2		= 0x06,
	REG_SR3		= 0x08,
};

#define UWIRE_CSR_BITS_RD(n)	(((n) & 0x1f) << 0)
#define UWIRE_CSR_BITS_WR(n)	(((n) & 0x1f) << 5)
#define UWIRE_CSR_IDX(n)	(((n) & 3) << 10)
#define UWIRE_CSR_CS_CMD	(1 << 12)
#define UWIRE_CSR_START		(1 << 13)
#define UWIRE_CSR_CSRB		(1 << 14)
#define UWIRE_CSR_RDRB		(1 << 15)

#define UWIRE_CSn_EDGE_RD	(1 << 0)	/* 1=falling 0=rising */
#define UWIRE_CSn_EDGE_WR	(1 << 1)	/* 1=falling 0=rising */
#define UWIRE_CSn_CS_LVL	(1 << 2)
#define UWIRE_CSn_FRQ_DIV2	(0 << 3)
#define UWIRE_CSn_FRQ_DIV4	(1 << 3)
#define UWIRE_CSn_FRQ_DIV8	(2 << 3)
#define UWIRE_CSn_CKH

#define UWIRE_CSn_SHIFT(n)	(((n) & 1) ? 6 : 0)
#define UWIRE_CSn_REG(n)	(((n) & 2) ? REG_SR2 : REG_SR1)

#define UWIRE_SR3_CLK_EN	(1 << 0)
#define UWIRE_SR3_CLK_DIV2	(0 << 1)
#define UWIRE_SR3_CLK_DIV4	(1 << 1)
#define UWIRE_SR3_CLK_DIV7	(2 << 1)
#define UWIRE_SR3_CLK_DIV10	(3 << 1)

static inline void _uwire_wait(int mask, int val)
{
	while ((getreg16(UWIRE_REG(REG_CSR)) & mask) != val);
}

void uwire_init(void)
{
	putreg16(UWIRE_SR3_CLK_EN | UWIRE_SR3_CLK_DIV2, UWIRE_REG(REG_SR3));
	/* FIXME only init CS0 for now */
	putreg16(((UWIRE_CSn_CS_LVL | UWIRE_CSn_FRQ_DIV2) << UWIRE_CSn_SHIFT(0)),
		UWIRE_REG(UWIRE_CSn_REG(0)));
	putreg16(UWIRE_CSR_IDX(0) | UWIRE_CSR_CS_CMD, UWIRE_REG(REG_CSR));
	_uwire_wait(UWIRE_CSR_CSRB, 0);
}

int uwire_xfer(int cs, int bitlen, const void *dout, void *din)
{
	uint16_t tmp = 0;

	if (bitlen <= 0 || bitlen > 16)
		return -1;
	if (cs < 0 || cs > 4)
		return -1;

	/* FIXME uwire_init always selects CS0 for now */

	dbg("uwire_xfer(dev_idx=%u, bitlen=%u\n", cs, bitlen);

	/* select the chip */
	putreg16(UWIRE_CSR_IDX(0) | UWIRE_CSR_CS_CMD, UWIRE_REG(REG_CSR));
	_uwire_wait(UWIRE_CSR_CSRB, 0);

	if (dout) {
		if (bitlen <= 8)
			tmp = *(uint8_t *)dout;
		else if (bitlen <= 16)
			tmp = *(uint16_t *)dout;
		tmp <<= 16 - bitlen; /* align to MSB */
		putreg16(tmp, UWIRE_REG(REG_DATA));
		dbg(", data_out=0x%04hx", tmp);
	}

	tmp =	(dout ? UWIRE_CSR_BITS_WR(bitlen) : 0) |
		(din  ? UWIRE_CSR_BITS_RD(bitlen) : 0) |
		UWIRE_CSR_START;
	putreg16(tmp, UWIRE_REG(REG_CSR));

	_uwire_wait(UWIRE_CSR_CSRB, 0);

	if (din) {
		_uwire_wait(UWIRE_CSR_RDRB, UWIRE_CSR_RDRB);

		tmp = getreg16(UWIRE_REG(REG_DATA));
		dbg(", data_in=0x%08x", tmp);

		if (bitlen <= 8)
			*(uint8_t *)din = tmp & 0xff;
		else if (bitlen <= 16)
			*(uint16_t *)din = tmp & 0xffff;
	}
	/* unselect the chip */
	putreg16(UWIRE_CSR_IDX(0) | 0, UWIRE_REG(REG_CSR));
	_uwire_wait(UWIRE_CSR_CSRB, 0);

	dbg(")\n");

	return 0;
}
