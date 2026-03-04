/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file LaunchDetector.h
 * Auto launch detection for catapult/hand-launch vehicles
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef LAUNCHDETECTOR_H
#define LAUNCHDETECTOR_H

#include <px4_platform_common/module_params.h>
#include <uORB/topics/launch_detection_status.h>

namespace launchdetection
{

// Info delay threshold (to publish info every kInfoDelay seconds)
static constexpr float kInfoDelay = 4.f;

class __EXPORT LaunchDetector : public ModuleParams
{
public:
	LaunchDetector(ModuleParams *parent) : ModuleParams(parent) {}
	~LaunchDetector() = default;

	LaunchDetector(const LaunchDetector &) = delete;
	LaunchDetector operator=(const LaunchDetector &) = delete;

	/**
	 * @brief Reset launch detection state machine.
	 */
	void reset();

	/**
	 * @brief Updates the state machine based on the current vehicle condition.
	 *
	 * @param dt Time step [us]
	 * @param accel_x Measured acceleration in body x [m/s/s]
	 */
	void update(const float dt, const float accel_x);

	/**
	 * @brief Get the Launch Detected state
	 *
	 * @return uint (aligned with launch_detection_status_s::launch_detection_state)
	 */
	uint getLaunchDetected() const;

	/**
	 * @brief Forces state of launch detection state machine to STATE_FLYING.
	 */
	void forceSetFlyState() { state_ = launch_detection_status_s::STATE_FLYING; }

private:
	/**
	 * Motor delay counter [s]
	 */
	float motor_delay_counter_{0.f};

	/**
	 * Info delay counter (to publish info every kInfoDelay seconds) [s]
	 */
	float info_delay_counter_s_{kInfoDelay};

	/**
	 *  Counter for how long the measured acceleration is above the defined threshold [s]
	 */
	float acceleration_detected_counter_{0.f};

	/**
	 * Current state of the launch detection state machine [launch_detection_status_s::launch_detection_state]
	 */
	uint state_{launch_detection_status_s::STATE_WAITING_FOR_LAUNCH};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_LAUN_AC_THLD>) param_fw_laun_ac_thld_,
		(ParamFloat<px4::params::FW_LAUN_AC_T>) param_fw_laun_ac_t_,
		(ParamFloat<px4::params::FW_LAUN_MOT_DEL>) param_fw_laun_mot_del_
	)
};

} // namespace launchdetection

#endif // LAUNCHDETECTOR_H
