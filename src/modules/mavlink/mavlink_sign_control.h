/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file mavlink_sign_control.h
 * Mavlink messages signing control helpers.
 *
 * @author Yulian oifa <yulian.oifa@mobius-software.com>
 */

#ifndef MAVLINK_SIGN_CONTROL_H_
#define MAVLINK_SIGN_CONTROL_H_

#define MAVLINK_SD_ROOT_PATH    CONFIG_BOARD_ROOT_PATH "/"
#define MAVLINK_FOLDER_PATH MAVLINK_SD_ROOT_PATH"/mavlink"
#define MAVLINK_SECRET_FILE MAVLINK_FOLDER_PATH"/mavlink-signing-key.bin"

#define MAVLINK_SECRET_KEY_TIMESTAMP_LENGTH 8 ///< size of timestamp in bytes
#define MAVLINK_SECRET_KEY_LENGTH 32 ///< size of key in bytes

#include "mavlink_receiver.h"

class Mavlink;

class MavlinkSignControl
{

public:
	MavlinkSignControl();
	~MavlinkSignControl();

	/**
	 * Initialize signing state and read key from file.
	 * Only enables signing if a valid key exists on the SD card.
	 */
	void start(int instance_id, mavlink_status_t *mavlink_status,
		   mavlink_accept_unsigned_t accept_unsigned_callback);

	enum SetupSigningResult {
		NOT_SETUP_SIGNING = 0,  ///< Message was not SETUP_SIGNING
		KEY_ACCEPTED,           ///< New key provisioned successfully
		SIGNING_DISABLED,       ///< Signing disabled via signed blank key
		BLANK_KEY_REJECTED      ///< Blank key rejected (unsigned or signing not active)
	};

	/**
	 * Checks whether the message is SETUP_SIGNING, and if yes, updates local key.
	 * Enables or disables signing based on whether the new key is valid.
	 */
	SetupSigningResult check_for_signing(const mavlink_message_t *msg);

	/**
	 * Reload key from SD card file. Called on the instance's own receiver thread
	 * when the signing key dirty flag is set by another instance.
	 */
	void reload_key();

	/**
	 * Stores the key and timestamp from memory to file
	 */
	void write_key_and_timestamp();

	/**
	 * Checks whether an unsigned message should be accepted
	 */
	bool accept_unsigned(uint32_t message_id);

	bool is_signing_active() const { return _is_signing_initialized; }

	static bool is_array_all_zeros(uint8_t arr[], size_t size);

private:
	mavlink_signing_t _mavlink_signing {};
	mavlink_status_t *_mavlink_status{nullptr};

	bool _is_signing_initialized{false};

	/**
	 * Wire or unwire the signing struct into the mavlink status based on key state.
	 */
	void _update_signing_state();
};


#endif /* MAVLINK_SIGN_CONTROL_H_ */
