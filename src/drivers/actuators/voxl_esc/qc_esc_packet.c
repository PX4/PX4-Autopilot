/****************************************************************************
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
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
 * 3. Neither the name The Linux Foundation nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE.
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
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 *
 ****************************************************************************/

#include "crc16.h"
#include "qc_esc_packet.h"
#include "qc_esc_packet_types.h"

#include <string.h>

int32_t qc_esc_create_version_request_packet(uint8_t id, uint8_t *out, uint16_t out_size)
{
	return qc_esc_create_packet(ESC_PACKET_TYPE_VERSION_REQUEST, &id, 1, out, out_size);
}

int32_t qc_esc_create_extended_version_request_packet(uint8_t id, uint8_t *out, uint16_t out_size)
{
	return qc_esc_create_packet(ESC_PACKET_TYPE_VERSION_EXT_REQUEST, &id, 1, out, out_size);
}

int32_t qc_esc_create_reset_packet(uint8_t id, uint8_t *out, uint16_t out_size)
{
	char payload[]  = "RESET0";
	payload[5]      += id;

	return qc_esc_create_packet(ESC_PACKET_TYPE_RESET_CMD, (uint8_t *)payload, 6 /*sizeof(payload)*/, out, out_size);
}


int32_t qc_esc_create_sound_packet(uint8_t frequency, uint8_t duration, uint8_t power, uint8_t mask, uint8_t *out,
				   uint16_t out_size)
{
	uint8_t data[4] = {frequency, duration, power, mask};
	return qc_esc_create_packet(ESC_PACKET_TYPE_SOUND_CMD, (uint8_t *) & (data[0]), 4, out, out_size);
}

int32_t qc_esc_create_led_control_packet(uint8_t led_byte_1, uint8_t led_byte_2, uint8_t *out, uint16_t out_size)
{
	uint8_t data[2] = {led_byte_1, led_byte_2};
	return qc_esc_create_packet(ESC_PACKET_TYPE_LED_CMD, (uint8_t *) & (data[0]), 2, out, out_size);
}

int32_t qc_esc_create_set_id_packet(uint8_t id, uint8_t *out, uint16_t out_size)
{
	return qc_esc_create_packet(ESC_PACKET_TYPE_SET_ID_CMD, (uint8_t *)&id, 1, out, out_size);
}

int32_t qc_esc_create_pwm_packet4(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
				  uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				  uint8_t *out, uint16_t out_size)
{
	return qc_esc_create_pwm_packet4_fb(pwm0, pwm1, pwm2, pwm3, led0, led1, led2, led3, -1, out, out_size);
}

int32_t qc_esc_create_pwm_packet4_fb(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
				     uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				     int32_t fb_id, uint8_t *out, uint16_t out_size)
{
	uint16_t data[5];
	uint16_t leds = 0;

	if (fb_id != -1) { fb_id = fb_id % 4; }

	//limit the pwm commands

	if (pwm0 > 800) { pwm0 = 800; } if (pwm0 < -800) { pwm0 = -800; }

	if (pwm1 > 800) { pwm1 = 800; } if (pwm1 < -800) { pwm1 = -800; }

	if (pwm2 > 800) { pwm2 = 800; } if (pwm2 < -800) { pwm2 = -800; }

	if (pwm3 > 800) { pwm3 = 800; } if (pwm3 < -800) { pwm3 = -800; }

	//least significant bit is used for feedback request
	pwm0 &= ~(0x0001); pwm1 &= ~(0x0001); pwm2 &= ~(0x0001); pwm3 &= ~(0x0001);

	if (fb_id == 0) { pwm0 |= 0x0001; } if (fb_id == 1) { pwm1 |= 0x0001; }

	if (fb_id == 2) { pwm2 |= 0x0001; } if (fb_id == 3) { pwm3 |= 0x0001; }

	leds |=             led0 & 0b00000111;
	leds |= (led1 & 0b00000111)  << 3;
	leds |= ((uint16_t)(led2 & 0b00000111)) << 6;
	leds |= ((uint16_t)(led3 & 0b00000111)) << 9;

	data[0] = pwm0; data[1] = pwm1; data[2] = pwm2; data[3] = pwm3; data[4] = leds;
	return qc_esc_create_packet(ESC_PACKET_TYPE_PWM_CMD, (uint8_t *) & (data[0]), 10, out, out_size);
}


int32_t qc_esc_create_rpm_packet4(int32_t rpm0, int32_t rpm1, int32_t rpm2, int32_t rpm3,
				  uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				  uint8_t *out, uint16_t out_size, uint8_t ext_rpm)
{
	return qc_esc_create_rpm_packet4_fb(rpm0, rpm1, rpm2, rpm3, led0, led1, led2, led3, -1, out, out_size, ext_rpm);
}

int32_t qc_esc_create_rpm_packet4_fb(int32_t rpm0, int32_t rpm1, int32_t rpm2, int32_t rpm3,
				     uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				     int32_t fb_id, uint8_t *out, uint16_t out_size, uint8_t ext_rpm)
{
	int16_t data[5];
	uint16_t leds = 0;
	uint8_t cmd = ESC_PACKET_TYPE_RPM_CMD;
	int32_t max = ext_rpm > 0 ? ESC_RPM_MAX_EXT : ESC_RPM_MAX;
	int32_t min = ext_rpm > 0 ? ESC_RPM_MIN_EXT : ESC_RPM_MIN;

	// Limit RPMs to prevent overflow when converting to int16_t

	if (rpm0 > max) { rpm0 = max; } if (rpm0 < min) { rpm0 = min; }

	if (rpm1 > max) { rpm1 = max; } if (rpm1 < min) { rpm1 = min; }

	if (rpm2 > max) { rpm2 = max; } if (rpm2 < min) { rpm2 = min; }

	if (rpm3 > max) { rpm3 = max; } if (rpm3 < min) { rpm3 = min; }

	if (fb_id != -1) { fb_id = fb_id % 4; }

	leds |= led0 & 0b00000111;
	leds |= (led1 & 0b00000111)  << 3;
	leds |= ((uint16_t)(led2 & 0b00000111)) << 6;
	leds |= ((uint16_t)(led3 & 0b00000111)) << 9;

	if (ext_rpm > 0) {
		cmd = ESC_PACKET_TYPE_RPM_DIV2_CMD;
		data[0] = ((rpm0 / 4) * 2);
		data[1] = ((rpm1 / 4) * 2);
		data[2] = ((rpm2 / 4) * 2);
		data[3] = ((rpm3 / 4) * 2);
		data[4] = leds;

	} else {
		data[0] = rpm0; data[1] = rpm1; data[2] = rpm2; data[3] = rpm3; data[4] = leds;
	}

	//least significant bit is used for feedback request
	data[0] &= ~(0x0001); data[1] &= ~(0x0001); data[2] &= ~(0x0001); data[3] &= ~(0x0001);

	if (fb_id == 0) { data[0] |= 0x0001; } if (fb_id == 1) { data[1] |= 0x0001; }

	if (fb_id == 2) { data[2] |= 0x0001; } if (fb_id == 3) { data[3] |= 0x0001; }

	return qc_esc_create_packet(cmd, (uint8_t *) & (data[0]), 10, out, out_size);
}

int32_t qc_esc_create_packet(uint8_t type, uint8_t *data, uint16_t size, uint8_t *out, uint16_t out_size)
{
	uint16_t packet_size = size + 5;

	if (packet_size > 255) { return -1; }

	if (out_size < packet_size) { return -2; }

	out[0] = 0xAF;
	out[1] = packet_size;
	out[2] = type;

	memcpy(&(out[3]), data, size);

	uint16_t crc = crc16_init();
	crc = crc16(crc, &(out[1]), packet_size - 3);

	memcpy(&(out[packet_size - 2]), &crc, sizeof(uint16_t));

	return packet_size;
}




//feed in a character and see if we got a complete packet
int16_t   qc_esc_packet_process_char(uint8_t c, EscPacket *packet)
{
	int16_t ret = ESC_NO_PACKET;

	uint16_t chk_comp;
	uint16_t chk_rcvd;

	if (packet->len_received >= (sizeof(packet->buffer) - 1)) {
		packet->len_received = 0;
	}

	//reset the packet and start parsing from beginning if length byte == header
	//this can only happen if the packet is de-synced and last char of checksum
	//ends up being equal to the header, in that case we can end up in endless loop
	//unable to re-sync with the packet
	if (packet->len_received == 1 && c == ESC_PACKET_HEADER) {
		packet->len_received = 0;
	}

	switch (packet->len_received) {
	case 0:  //header
		packet->bp = packet->buffer;           //reset the pointer for storing data
		qc_esc_packet_checksum_reset(packet);  //reset the checksum to starting value

		if (c != ESC_PACKET_HEADER) {          //check the packet header
			packet->len_received = 0;
			ret = ESC_ERROR_BAD_HEADER;
			break;
		}

		packet->len_received++;
		*(packet->bp)++ = c;
		break;

	case 1:  //length
		packet->len_received++;
		*(packet->bp)++      = c;
		packet->len_expected = c;

		if (packet->len_expected >= (sizeof(packet->buffer) - 1)) {
			packet->len_received = 0;
			ret = ESC_ERROR_BAD_LENGTH;
			break;
		}

		qc_esc_packet_checksum_process_char(packet, c);
		break;

	default: //rest of the packet
		packet->len_received++;
		*(packet->bp)++ = c;

		if (packet->len_received < (packet->len_expected - 1)) { //do not compute checksum of checksum (last 2 bytes)
			qc_esc_packet_checksum_process_char(packet, c);
		}

		if (packet->len_received < packet->len_expected) {   //waiting for more bytes
			break;
		}

		//grab the computed checksum and compare against the received value
		chk_comp = qc_esc_packet_checksum_get(packet);

		memcpy(&chk_rcvd, packet->bp - 2, sizeof(uint16_t));

		if (chk_comp == chk_rcvd) { ret  = packet->len_received; }

		else { ret  = ESC_ERROR_BAD_CHECKSUM; }

		packet->len_received = 0;
		break;
	}

	return ret;
}
