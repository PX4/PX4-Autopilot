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

class __EXPORT LaunchDetector : public ModuleParams
{
public:
	LaunchDetector(ModuleParams *parent) : ModuleParams(parent) {}
	~LaunchDetector() = default;

	LaunchDetector(const LaunchDetector &) = delete;
	LaunchDetector operator=(const LaunchDetector &) = delete;

	void reset();

	/**
	 * @brief Updates the state machine based on the current vehicle condition.
	 *
	 * @param dt Time step [us]
	 * @param accel_x Measured acceleration in body x [m/s/s]
	 * @param mavlink_log_pub
	 */
	void update(const float dt, float accel_x,  orb_advert_t *mavlink_log_pub);

	/**
	 * @brief Get the Launch Detected state
	 *
	 * @return uint (aligned with launch_detection_status_s::launch_detection_state)
	 */
	uint getLaunchDetected() const;

	/**
	 * @return Launch detection is enabled
	 */
	bool launchDetectionEnabled() { return _param_laun_all_on.get(); }

	void forceSetFlyState() { _state = launch_detection_status_s::STATE_FLYING; }

private:
	/**
	 * Integrator [s]
	 */
	float _launchDetectionDelayCounter{0.f};

	/**
	 * Motor delay counter [s]
	 */
	float _motorDelayCounter{0.f};

	float _launchDetectionRunningInfoDelay{4.f};

	/**
	 * Current state of the launch detection state machine [launch_detection_status_s::launch_detection_state]
	 */
	uint _state{launch_detection_status_s::STATE_WAITING_FOR_LAUNCH};

	// [us] logs the last time the launch detection notification was sent (used not to spam notifications during launch detection)
	hrt_abstime _last_time_launch_detection_notified{0};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::LAUN_ALL_ON>) _param_laun_all_on,
		(ParamFloat<px4::params::LAUN_CAT_A>) _param_laun_cat_a,
		(ParamFloat<px4::params::LAUN_CAT_T>) _param_laun_cat_t,
		(ParamFloat<px4::params::LAUN_CAT_MDEL>) _param_laun_cat_mdel
	)
};

} // namespace launchdetection

#endif // LAUNCHDETECTOR_H
