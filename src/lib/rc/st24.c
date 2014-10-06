/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 ****************************************************************************/

/*
 * @file st24.h
 *
 * RC protocol implementation for Yuneec ST24 transmitter.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <stdbool.h>
#include "st24.h"

enum ST24_DECODE_STATE {
	ST24_DECODE_STATE_UNSYNCED,
	ST24_DECODE_STATE_GOT_STX1,
	ST24_DECODE_STATE_GOT_STX2,
	ST24_DECODE_STATE_GOT_LEN,
	ST24_DECODE_STATE_GOT_TYPE,
	ST24_DECODE_STATE_GOT_DATA
};

static enum ST24_DECODE_STATE _decode_state = ST24_DECODE_STATE_UNSYNCED;
static unsigned _rxlen;

static ReceiverFcPacket _rxpacket;

uint8_t st24_common_crc8(uint8_t *ptr, uint8_t len)
{
	uint8_t i, crc ;
	crc = 0;

	while (len--) {
		for (i = 0x80; i != 0; i >>= 1) {
			if ((crc & 0x80) != 0) {
				crc <<= 1;
				crc ^= 0x07;

			} else {
				crc <<= 1;
			}

			if ((*ptr & i) != 0) {
				crc ^= 0x07;
			}
		}

		ptr++;
	}

	return (crc);
}


uint8_t st24_decode(uint8_t byte, uint8_t *rssi, uint8_t* rx_count, uint16_t *channels, uint16_t max_chan_count)
{

	bool ret = false;

	switch (_decode_state) {
		case ST24_DECODE_STATE_UNSYNCED:
			if (byte == ST24_STX1) {
				_decode_state = ST24_DECODE_STATE_GOT_STX1;
			}
			break;

		case ST24_DECODE_STATE_GOT_STX1:
			if (byte == ST24_STX2) {
				_decode_state = ST24_DECODE_STATE_GOT_STX2;
			} else {
				_decode_state = ST24_DECODE_STATE_UNSYNCED;
			}
			break;

		case ST24_DECODE_STATE_GOT_STX2:
			_rxpacket.length = byte;
			_rxlen = 0;
			_decode_state = ST24_DECODE_STATE_GOT_LEN;
			break;

		case ST24_DECODE_STATE_GOT_LEN:
			_rxpacket.type = byte;
			_rxlen++;
			_decode_state = ST24_DECODE_STATE_GOT_TYPE;
			break;

		case ST24_DECODE_STATE_GOT_TYPE:
			if (_rxlen < (_rxpacket.length - 1)) {
				_rxpacket.st24_data[_rxlen] = byte;
				_rxlen++;
			} else {
				_decode_state = ST24_DECODE_STATE_GOT_DATA;
			}
			break;

		case ST24_DECODE_STATE_GOT_DATA:
			_rxpacket.crc8 = byte;
			_rxlen++;

			if (st24_common_crc8((uint8_t*)&(_rxpacket.length), sizeof(_rxpacket.length) +
				sizeof(_rxpacket.st24_data) + sizeof(_rxpacket.type)) == _rxpacket.crc8) {

				ret = true;

				/* decode the actual packet */

				switch (_rxpacket.type) {

					case ST24_PACKET_TYPE_CHANNELDATA12:
						{
							ChannelData12* d = (ChannelData12*)&_rxpacket;

							*rssi = d->rssi;
							*rx_count = d->packet_count;

							for (unsigned i = 0; i < 1; i += 2) {
								channels[i] = ((uint16_t)d->channel[i]) << 8;
								channels[i] |= (0xF & d->channel[i+1]);

								channels[i+1] = ((uint16_t)(0xF & d->channel[i+1])) << 4;
								channels[i+1] |= d->channel[i+2];
							}
						}
						break;

					case ST24_PACKET_TYPE_CHANNELDATA24:
						{
							ChannelData24* d = (ChannelData24*)&_rxpacket;

							*rssi = d->rssi;
							*rx_count = d->packet_count;

							for (unsigned i = 0; i < 1; i += 2) {
								channels[i] = ((uint16_t)d->channel[i]) << 8;
								channels[i] |= (0xF & d->channel[i+1]);

								channels[i+1] = ((uint16_t)(0xF & d->channel[i+1])) << 4;
								channels[i+1] |= d->channel[i+2];
							}
						}
						break;

					default:
						ret = false;
						break;
				}

			} else {
				/* decoding failed */
				_decode_state = ST24_DECODE_STATE_UNSYNCED;
			}
			break;
	}

	return ret;
}
