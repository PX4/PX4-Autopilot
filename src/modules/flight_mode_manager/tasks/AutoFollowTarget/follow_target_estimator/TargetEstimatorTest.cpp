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

#include <gtest/gtest.h>

#include "TargetEstimator.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/follow_target_estimator.h>
#include <uORB/topics/vehicle_local_position.h>

// to run: make tests TESTFILTER=TargetEstimator

/* EVENT
 * @skip-file
 */

using namespace time_literals;

namespace
{

constexpr double kLat = 47.397742;
constexpr double kLon = 8.545594;
constexpr float kAlt = 500.f;
constexpr double kMetresPerDegreeLat = 111320.;

} // namespace

class TargetEstimatorTestPeer : public TargetEstimator
{
public:
	using TargetEstimator::update;
	const filter_states_s &states() const { return _filter_states; }
	matrix::Vector3<double> latLonAlt() const { return get_lat_lon_alt_est(); }
};

// stop the work queue manager at the end so the process exits
class TargetEstimatorTestEnvironment : public ::testing::Environment
{
public:
	void TearDown() override { px4::WorkQueueManagerStop(); }
};
static const auto *global_env = ::testing::AddGlobalTestEnvironment(new TargetEstimatorTestEnvironment());

class TargetEstimatorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// the estimator attaches to a work queue at construction, the manager is started once per process
		static bool wq_manager_started = false;

		if (!wq_manager_started) {
			hrt_init();
			hrt_work_queue_init();
			ASSERT_EQ(px4::WorkQueueManagerStart(), 0);
			wq_manager_started = true;
		}

		const hrt_abstime wq_wait_start = hrt_absolute_time();

		while (px4::WorkQueueFindOrCreate(px4::wq_configurations::nav_and_controllers) == nullptr) {
			ASSERT_LT(hrt_elapsed_time(&wq_wait_start), 10_s) << "work queue manager did not start";
			px4_usleep(100_ms);
		}
	}

	void publishReference(double ref_lat, double ref_lon, float ref_alt)
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_global = true;
		local_position.z_global = true;
		local_position.ref_timestamp = local_position.timestamp;
		local_position.ref_lat = ref_lat;
		local_position.ref_lon = ref_lon;
		local_position.ref_alt = ref_alt;
		_local_position_pub.publish(local_position);
	}

	void publishTarget(double lat, double lon, float alt, hrt_abstime timestamp)
	{
		follow_target_s follow_target{};
		follow_target.timestamp = timestamp;
		follow_target.lat = lat;
		follow_target.lon = lon;
		follow_target.alt = alt;
		follow_target.vx = 0.f;
		follow_target.vy = 0.f;
		follow_target.vz = 0.f;
		_follow_target_pub.publish(follow_target);
	}

	follow_target_estimator_s published()
	{
		follow_target_estimator_s follow_target_estimator{};
		EXPECT_TRUE(_follow_target_estimator_sub.copy(&follow_target_estimator));
		return follow_target_estimator;
	}

	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<follow_target_s> _follow_target_pub{ORB_ID(follow_target)};
	uORB::Subscription _follow_target_estimator_sub{ORB_ID(follow_target_estimator)};
};

// The target stands still while the local position reference moves, as it does on an estimator
// switch or an origin change. The estimate must restart in the new frame instead of mixing the two
TEST_F(TargetEstimatorTest, EstimateRestartsInTheNewFrameWhenTheReferenceMoves)
{
	TargetEstimatorTestPeer estimator;

	// the first call only records the time, every later one runs the filter
	estimator.update();

	const double target_lat = kLat + 100. / kMetresPerDegreeLat;
	const double target_lon = kLon;
	const float target_alt = kAlt + 20.f;

	// GIVEN: a target 100 m north of the reference and 20 m above it, measured a second ago
	const hrt_abstime now = hrt_absolute_time();
	publishReference(kLat, kLon, kAlt);
	publishTarget(target_lat, target_lon, target_alt, now - 1_s);
	estimator.update();

	const follow_target_estimator_s before = published();
	EXPECT_TRUE(before.valid);
	EXPECT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(1), 0.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), -20.f, 0.1f);
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.1);

	// WHEN: the reference moves 200 m north and 50 m up
	publishReference(kLat + 200. / kMetresPerDegreeLat, kLon, kAlt + 50.f);
	estimator.update();

	// THEN: the estimate is dropped rather than carried into the new frame, and the restart is announced
	const follow_target_estimator_s after_move = published();
	EXPECT_FALSE(after_move.valid);
	EXPECT_FALSE(estimator.states().pos_ned_est.isAllFinite());
	EXPECT_NE(after_move.last_filter_reset_timestamp, before.last_filter_reset_timestamp);

	// AND WHEN: the next measurement of the same target comes in
	publishTarget(target_lat, target_lon, target_alt, now);
	estimator.update();

	// THEN: the estimate restarts in the new frame, on the target
	EXPECT_TRUE(published().valid);
	EXPECT_NEAR(estimator.states().pos_ned_est(0), -100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(1), 0.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), 30.f, 0.1f);
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.1);
	EXPECT_LT(estimator.states().vel_ned_est.norm(), 0.5f);
}
