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

// Functional test for FailureInjectionManager::evaluateRcInjection(): an
// unmapped aux channel now publishes NaN (see rc_update.cpp / mavlink_receiver.cpp)
// instead of 0, so the RC-triggered failure injection source must explicitly
// treat a non-finite aux value as "not triggered" rather than relying on
// `NaN > threshold` evaluating false. This needs the uORB runtime to publish
// manual_control_setpoint samples, so it runs as a functional gtest.

#include <gtest/gtest.h>

#include "FailureInjectionManager.hpp"

#include <uORB/Publication.hpp>

namespace
{

class TestFailureInjectionManager : public FailureInjectionManager
{
public:
	// Sets the cached param value directly (no param_set()/commit): evaluateRcInjection()
	// only ever reads the cached value via _param_sys_fail_rc_src.get(), so committing to
	// the global param store (which would also trigger autosave scheduling on the work
	// queue) is unnecessary here.
	void configureRcSource(int32_t src) { _param_sys_fail_rc_src.set(src); }

	void evaluate() { evaluateRcInjection(); }

	bool rcActive() const { return _rc_active; }
};

void publishManualControlSetpoint(uORB::Publication<manual_control_setpoint_s> &pub, float aux1)
{
	manual_control_setpoint_s mc{};
	mc.timestamp = mc.timestamp_sample = 1;
	mc.valid = true;
	mc.aux1 = aux1;
	pub.publish(mc);
}

} // namespace

TEST(FailureInjectionManagerRcInjection, UnmappedNaNAuxDoesNotTrigger)
{
	uORB::Publication<manual_control_setpoint_s> pub{ORB_ID(manual_control_setpoint)};
	pub.advertise();

	TestFailureInjectionManager manager;
	manager.configureRcSource(1); // aux1

	// Unmapped aux channel: producers (rc_update.cpp, mavlink_receiver.cpp) now
	// publish NaN instead of 0 for this case.
	publishManualControlSetpoint(pub, NAN);

	manager.evaluate();
	EXPECT_FALSE(manager.rcActive());
}

TEST(FailureInjectionManagerRcInjection, RealAuxAboveThresholdTriggers)
{
	uORB::Publication<manual_control_setpoint_s> pub{ORB_ID(manual_control_setpoint)};
	pub.advertise();

	TestFailureInjectionManager manager;
	manager.configureRcSource(1); // aux1

	publishManualControlSetpoint(pub, 0.8f);

	manager.evaluate();
	EXPECT_TRUE(manager.rcActive());
}

TEST(FailureInjectionManagerRcInjection, RealAuxBelowThresholdDoesNotTrigger)
{
	uORB::Publication<manual_control_setpoint_s> pub{ORB_ID(manual_control_setpoint)};
	pub.advertise();

	TestFailureInjectionManager manager;
	manager.configureRcSource(1); // aux1

	publishManualControlSetpoint(pub, 0.1f);

	manager.evaluate();
	EXPECT_FALSE(manager.rcActive());
}
