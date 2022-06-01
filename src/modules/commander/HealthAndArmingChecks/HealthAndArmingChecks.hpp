/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include "Common.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/health_report.h>

class HealthAndArmingChecks : public ModuleParams
{
public:
	HealthAndArmingChecks(ModuleParams *parent, vehicle_status_flags_s &status_flags, vehicle_status_s &status);
	~HealthAndArmingChecks() = default;

	/**
	 * Run arming checks and report if necessary.
	 * This should be called regularly (e.g. 1Hz).
	 * @param force_reporting if true, force reporting even if nothing changed
	 * @return true if there was a report (also when force_reporting=true)
	 */
	bool update(bool force_reporting = false);

	/**
	 * Whether arming is possible for a given navigation mode
	 */
	bool canArm(uint8_t nav_state) const { return _reporter.canArm(nav_state); }

protected:
	void updateParams() override;
private:
	Context _context;
	Report _reporter;
	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<health_report_s> _health_report_pub{ORB_ID(health_report)};

	// all checks
	HealthAndArmingCheckBase *_checks[10] = {
	};
};

