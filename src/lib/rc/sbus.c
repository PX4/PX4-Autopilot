/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

/**
 * @file sbus.c
 *
 * Serial protocol decoder for the Futaba S.bus protocol.
 */

#include <px4_config.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "sbus.h"
#include <drivers/drv_hrt.h>

#define SBUS_START_SYMBOL	0x0f

#define SBUS_FRAME_SIZE		25
#define SBUS_MAX_BUF_SIZE	(SBUS_FRAME_SIZE + 10)
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2
#define SBUS1_FRAME_DELAY	14000

#define SBUS_SINGLE_CHAR_LEN_US		(1/((100000/10)) * 1000 * 1000)

#define SBUS_FRAME_INTERVAL_US	2500
#define SBUS_MIN_CALL_INTERVAL_US	(SBUS_FRAME_GAP_US / 3)
#define SBUS_EPSILON_US	2500

/*
  Measured values with Futaba FX-30/R6108SB:
    -+100% on TX:  PCM 1.100/1.520/1.950ms -> SBus raw values: 350/1024/1700  (100% ATV)
    -+140% on TX:  PCM 0.930/1.520/2.112ms -> SBus raw values:  78/1024/1964  (140% ATV)
    -+152% on TX:  PCM 0.884/1.520/2.160ms -> SBus raw values:   1/1024/2047  (140% ATV plus dirty tricks)
*/

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

#define SBUS_SYNC_PHASE0	0
#define SBUS_SYNC_PHASE1	1

#define SBUS_DEBUG

#ifdef SBUS_DEBUG
#include <stdio.h>
#endif

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

static hrt_abstime last_rx_time;
static hrt_abstime last_frame_time;
static hrt_abstime last_txframe_time = 0;

#define SBUS2_FRAME_SIZE_RX_VOLTAGE	3
#define SBUS2_FRAME_SIZE_GPS_DIGIT	3

static enum SBUS2_DECODE_STATE {
	SBUS2_DECODE_STATE_DESYNC = 0xFFF,
	SBUS2_DECODE_STATE_SBUS_START = 0x2FF,
	SBUS2_DECODE_STATE_SBUS1_SYNC = 0x00,
	SBUS2_DECODE_STATE_SBUS2_SYNC = 0x1FF,
	SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE = 0x04,
	SBUS2_DECODE_STATE_SBUS2_GPS = 0x14,
	SBUS2_DECODE_STATE_SBUS2_DATA1 = 0x24,
	SBUS2_DECODE_STATE_SBUS2_DATA2 = 0x34
} sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;

static uint8_t	sbus_frame[SBUS_FRAME_SIZE + (SBUS_FRAME_SIZE / 2)];

static unsigned partial_frame_count;

static unsigned sbus_frame_drops;
static unsigned sbus_sync = SBUS_SYNC_PHASE0;

unsigned
sbus_dropped_frames()
{
	return sbus_frame_drops;
}

static bool sbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe,
			bool *sbus_frame_drop, uint16_t max_channels);

int
sbus_init(const char *device, bool singlewire)
{
	int sbus_fd = open(device, O_RDWR | O_NONBLOCK);

	if (sbus_fd >= 0) {
		struct termios t;

		/* 100000bps, even parity, two stop bits */
		tcgetattr(sbus_fd, &t);
		cfsetspeed(&t, 100000);
		t.c_cflag |= (CSTOPB | PARENB);
		tcsetattr(sbus_fd, TCSANOW, &t);

		if (singlewire) {
			/* only defined in configs capable of IOCTL */
#ifdef SBUS_SERIAL_PORT
			//ioctl(uart, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
#endif
		}

		/* initialise the decoder */
		partial_frame_count = 0;
		last_rx_time = hrt_absolute_time();
		last_frame_time = last_rx_time;
		sbus_frame_drops = 0;
	}

	return sbus_fd;
}

void
sbus1_output(int sbus_fd, uint16_t *values, uint16_t num_values)
{
	uint8_t byteindex = 1; /*Data starts one byte into the sbus frame. */
	uint8_t offset = 0;
	uint16_t value;
	hrt_abstime now;

	now = hrt_absolute_time();

	if ((now - last_txframe_time) > SBUS1_FRAME_DELAY) {
		last_txframe_time = now;
		uint8_t	oframe[SBUS_FRAME_SIZE] = { 0x0f };

		/* 16 is sbus number of servos/channels minus 2 single bit channels.
		* currently ignoring single bit channels.  */

		for (unsigned i = 0; (i < num_values) && (i < 16); ++i) {
			value = (uint16_t)(((values[i] - SBUS_SCALE_OFFSET) / SBUS_SCALE_FACTOR) + .5f);

			/*protect from out of bounds values and limit to 11 bits*/
			if (value > 0x07ff) {
				value = 0x07ff;
			}

			while (offset >= 8) {
				++byteindex;
				offset -= 8;
			}

			oframe[byteindex] |= (value << (offset)) & 0xff;
			oframe[byteindex + 1] |= (value >> (8 - offset)) & 0xff;
			oframe[byteindex + 2] |= (value >> (16 - offset)) & 0xff;
			offset += 11;
		}

		write(sbus_fd, oframe, SBUS_FRAME_SIZE);
	}
}
void
sbus2_output(int sbus_fd, uint16_t *values, uint16_t num_values)
{
	char b = 'B';
	write(sbus_fd, &b, 1);
}

bool
sbus_input(uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop,
	   uint16_t max_channels)
{
	ssize_t		ret;
	hrt_abstime	now;

	/*
	 * The S.BUS protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
	 * frame transmission time is ~2ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time();

	if ((now - last_rx_time) > SBUS_INTER_FRAME_TIMEOUT) {
		if (partial_frame_count > 0) {
			sbus_frame_drops++;
			partial_frame_count = 0;
		}
	}

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * a complete frame.
	 */
	ret = read(sbus_fd, &sbus_frame[partial_frame_count], SBUS_FRAME_SIZE - partial_frame_count);

	/* if the read failed for any reason, just give up here */
	if (ret < 1) {
		return false;
	}

	/*
	 * Add bytes to the current frame
	 */
	partial_frame_count += ret;

	/*
	 * Try to decode something with what we got
	 */
	return sbus_parse(now, sbus_frame, &partial_frame_count, values, num_values, sbus_failsafe, sbus_frame_drop, &sbus_frame_drops, max_channels);
}

bool
sbus_parse(hrt_abstime now, uint8_t *frame, unsigned *partial_count, uint16_t *values,
	uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t *frame_drops, uint16_t max_channels)
{

	last_rx_time = now;

	/* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = true;

	/* keep decoding until we have consumed the buffer or have given up */
	for (unsigned d = 0; ((d < 10) && (*partial_count > 0) && (decode_ret)); d++) {
		unsigned n_consumed = 0;

		switch (sbus_decode_state) {
			case SBUS2_DECODE_STATE_DESYNC:
				/* we are de-synced and only interested in the frame marker */
				{
					unsigned i = 0;
					while (i < *partial_count) {
						
						/* for the case where frame[i] is the start sign, the index
						 * is exactly the number of bytes to consume
						 */
						n_consumed = i;

						if (frame[i] == 0x0F) {
							sbus_decode_state = SBUS2_DECODE_STATE_SBUS_START;
							break;
						}

						/* increment index to next position */
						i++;
					}
				}
				break;
			/* fall through */
			case SBUS2_DECODE_STATE_SBUS_START:
			case SBUS2_DECODE_STATE_SBUS1_SYNC:
			/* fall through */
			case SBUS2_DECODE_STATE_SBUS2_SYNC:
				{
					/* decode whatever we got and expect */
					if (*partial_count < SBUS_FRAME_SIZE) {
						decode_ret = false;
						break;
					}

					/*
					 * Great, it looks like we might have a frame.  Go ahead and
					 * decode it.
					 */
					decode_ret = sbus_decode(now, frame, values, num_values, sbus_failsafe, sbus_frame_drop, max_channels);

					if (decode_ret) {
						n_consumed = SBUS_FRAME_SIZE;
					}
				}
				break;
			case SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE:
				{
					if (*partial_count < SBUS2_FRAME_SIZE_RX_VOLTAGE) {
						decode_ret = false;
						break;
					}

					/* find out which payload we're dealing with in this slot */
					switch(frame[0]) {
						case 0x03:
							{
								uint16_t rx_voltage = (frame[1] << 8) | frame[2];
								isr_debug(30, "rx_voltage %d", (int)rx_voltage);
								n_consumed = 3;
							}
							break;
						case 0x0F:
							/* the battery slot is unused and followed by a normal frame */
							sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
							break;
						default:
							/* this is not what we expect it to be, go back to sync */
							sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
							sbus_frame_drops++;
							/* throw unknown bytes away */
							n_consumed = 3;
					}
				}
				break;
			case SBUS2_DECODE_STATE_SBUS2_GPS:
				{
					if (*partial_count < 24) {
						decode_ret = false;
						break;
					}

					/* find out which payload we're dealing with in this slot */
					switch(frame[0]) {
						case 0x13:
							{
								uint16_t gps_something = (frame[1] << 8) | frame[2];
								isr_debug(30, "gps_something %d", (int)gps_something);
								n_consumed = 24;
							}
							break;
						case 0x0F:
							/* the GPS slot is unused and followed by a normal frame */
							sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
							break;
						default:
							/* this is not what we expect it to be, go back to sync */
							sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
							sbus_frame_drops++;
							/* throw unknown bytes away */
							n_consumed = 3;
					}
				}
				break;
			default:
				decode_ret = false;
		}

		/* move buffer to start after this step, keep decoding if any bytes remain */

		/* XXX should be still more efficient than single-byte ringbuffer accesses */
		uint8_t frame_buf[SBUS_FRAME_SIZE + 10]; 
		memcpy(&frame_buf[0], &frame[n_consumed], *partial_count - n_consumed);
		memcpy(&frame[0], &frame_buf[0], *partial_count - n_consumed);
		*partial_count = *partial_count - n_consumed;
	}

	*frame_drops = sbus_frame_drops;

	/* return false as default */
	return decode_ret;
}
