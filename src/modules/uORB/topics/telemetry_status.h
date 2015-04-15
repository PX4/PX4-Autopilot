/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file telemetry_status.h
 *
 * Telemetry status topics - radio status outputs
 */

#ifndef TOPIC_TELEMETRY_STATUS_H
#define TOPIC_TELEMETRY_STATUS_H

#include <stdint.h>
#include "../uORB.h"

enum TELEMETRY_STATUS_RADIO_TYPE {
	TELEMETRY_STATUS_RADIO_TYPE_GENERIC = 0,
	TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO,
	TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET,
	TELEMETRY_STATUS_RADIO_TYPE_WIRE
};

/**
 * @addtogroup topics
 * @{
 */

struct telemetry_status_s {
	uint64_t timestamp;
	uint64_t heartbeat_time;		/**< Time of last received heartbeat from remote system */
	uint64_t telem_time;			/**< Time of last received telemetry status packet, 0 for none */
	enum TELEMETRY_STATUS_RADIO_TYPE type;	/**< type of the radio hardware     */
	uint8_t rssi;				/**< local signal strength                      */
	uint8_t remote_rssi;			/**< remote signal strength                     */
	uint16_t rxerrors;			/**< receive errors                             */
	uint16_t fixed;				/**< count of error corrected packets           */
	uint8_t noise;				/**< background noise level                     */
	uint8_t remote_noise;			/**< remote background noise level              */
	uint8_t txbuf;				/**< how full the tx buffer is as a percentage  */
	uint8_t system_id;			/**< system id of the remote system */
	uint8_t component_id;			/**< component id of the remote system */
};

/**
 * @}
 */

ORB_DECLARE(telemetry_status_0);
ORB_DECLARE(telemetry_status_1);
ORB_DECLARE(telemetry_status_2);
ORB_DECLARE(telemetry_status_3);

#define TELEMETRY_STATUS_ORB_ID_NUM	4

static const struct orb_metadata *telemetry_status_orb_id[TELEMETRY_STATUS_ORB_ID_NUM] = {
	ORB_ID(telemetry_status_0),
	ORB_ID(telemetry_status_1),
	ORB_ID(telemetry_status_2),
	ORB_ID(telemetry_status_3),
};

// This is a hack to quiet an unused-variable warning for when telemetry_status.h is
// included but telemetry_status_orb_id is not referenced. The inline works if you
// choose to use it, but you can continue to just directly index into the array as well.
// If you don't use the inline this ends up being a no-op with no additional code emitted.
extern inline const struct orb_metadata *telemetry_status_orb_id_lookup(size_t index);
extern inline const struct orb_metadata *telemetry_status_orb_id_lookup(size_t index)
{
	return telemetry_status_orb_id[index];
}

#endif /* TOPIC_TELEMETRY_STATUS_H */
