/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
#define MAVLINK_SECRET_FILE MAVLINK_FOLDER_PATH"/.secret"

#include "mavlink_receiver.h"

class Mavlink;

class MavlinkSignControl
{

public:
	MavlinkSignControl();
	~MavlinkSignControl();

	enum PROTO_SIGN {
		PROTO_SIGN_OPTIONAL = 0,
		PROTO_SIGN_NON_USB,
		PROTO_SIGN_ALWAYS
	};

	/**
	 * Initialize signing and read configuration from file
	 */
	void start(int _instance_id, mavlink_status_t *_mavlink_status, mavlink_accept_unsigned_t accept_unsigned_callback);

	/**
	 * Checks whether the message is SETUP_SIGNING, and if yes , updates local key
	 */
	bool check_for_signing(const mavlink_message_t *msg);

	/**
	 * Checks whether should accept unsigned message for specific sign mode
	 */
	static bool accept_unsigned(int32_t sign_mode, bool is_usb_uart, uint32_t message_id);
private:
	mavlink_signing_t _mavlink_signing {};
};


#endif /* MAVLINK_SIGN_CONTROL_H_ */
