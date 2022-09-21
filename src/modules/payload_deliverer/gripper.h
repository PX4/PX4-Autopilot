/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/gripper.h>

using namespace time_literals;

/**
 * Griper class to handle functionality of using a Gripper
 */

typedef struct GripperConfig {
	enum class GripperType {
		UNDEFINED = -1,
		SERVO = 0,
	};

	GripperType type{GripperType::UNDEFINED};

	// Gripper state feedback sensor type
	enum class GripperSensorType {
		NONE = -1,
		ENCODER,
	};

	GripperSensorType sensor{GripperSensorType::NONE};

	hrt_abstime timeout_us{0};

} GripperConfig;

enum class GripperState {
	IDLE = 0,
	GRABBING,
	GRABBED,
	RELEASING,
	RELEASED
};

class Gripper
{
public:
	Gripper() = default;

	// Initialize the gripper
	void init(const GripperConfig &config);

	// Actuate the gripper to the grab position
	void grab();

	// Actuate the gripper to release position
	void release();

	// Update gripper state
	void update();

	// Return griper's state in string format
	const char *get_state_str() const;

	// Returns true if in grabbed position either sensed or timeout based
	bool grabbed() { return _state == GripperState::GRABBED; }

	// Returns true if in grabbing position either sensed or timeout based
	bool grabbing() { return _state == GripperState::GRABBING; }

	// Returns true if in released position either sensed or timeout based
	bool released() { return _state == GripperState::RELEASED; }

	// Returns true only once after the state transitioned into released
	// Used for sending vehicle command ack only when the gripper actually releases
	// in payload deliverer
	bool released_read_once()
	{
		if (_released_state_cache) {
			_released_state_cache = false;
			return true;

		} else {
			return false;
		}
	}

	// Returns true if Gripper output function is assigned properly
	bool is_valid() const { return _valid; }

private:
	// Internal function to send gripper command via uORB
	void publish_gripper_command(const int8_t gripper_command);

	// Internal settings
	bool _valid{false};
	bool _has_feedback_sensor{false};
	hrt_abstime _timeout_us{0};

	// Internal state
	GripperState _state{GripperState::IDLE};
	hrt_abstime _last_command_time{0};
	bool _released_state_cache{false}; // Cached flag that is set once gripper release was successful, gets reset once read

	uORB::Publication<gripper_s> _gripper_pub{ORB_ID(gripper)};
};
