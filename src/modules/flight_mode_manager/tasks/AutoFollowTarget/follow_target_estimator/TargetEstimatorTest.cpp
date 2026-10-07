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
	uint8_t posResetCounter() const { return _pos_reset_counter; }
	const matrix::Vector3f &deltaPos() const { return _delta_pos; }
};

// stop the work queue manager at the end so the process exits
class TargetEstimatorTestEnvironment : public ::testing::Environment
{
public:
	void TearDown() override { px4::WorkQueueManagerStop(); }
};
static const auto *global_env = ::testing::AddGlobalTestEnvironment(new TargetEstimatorTestEnvironment());

// one item keeps the queue alive between the estimators of the single tests
class WorkQueueKeeper : public px4::ScheduledWorkItem
{
public:
	WorkQueueKeeper() : ScheduledWorkItem("wq_keeper", px4::wq_configurations::nav_and_controllers) {}
private:
	void Run() override {}
};

class TargetEstimatorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// the estimator attaches to a work queue at construction
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

		static WorkQueueKeeper *wq_keeper = new WorkQueueKeeper();
		ASSERT_NE(wq_keeper, nullptr);
	}

	void publishReference(double ref_lat, double ref_lon, float ref_alt, bool xy_global = true, bool z_global = true)
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_global = xy_global;
		local_position.z_global = z_global;
		local_position.ref_timestamp = local_position.timestamp;
		local_position.ref_lat = ref_lat;
		local_position.ref_lon = ref_lon;
		local_position.ref_alt = ref_alt;
		_local_position_pub.publish(local_position);
	}

	follow_target_estimator_s published()
	{
		follow_target_estimator_s follow_target_estimator{};
		EXPECT_TRUE(_follow_target_estimator_sub.copy(&follow_target_estimator));
		return follow_target_estimator;
	}

	// what EKF2 publishes while it has no global origin
	void publishNoReference()
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_global = false;
		local_position.z_global = false;
		local_position.ref_lat = static_cast<double>(NAN);
		local_position.ref_lon = static_cast<double>(NAN);
		local_position.ref_alt = NAN;
		_local_position_pub.publish(local_position);
	}

	void publishTarget(double lat, double lon, float alt)
	{
		follow_target_s follow_target{};
		follow_target.timestamp = hrt_absolute_time();
		follow_target.lat = lat;
		follow_target.lon = lon;
		follow_target.alt = alt;
		follow_target.vx = 0.f;
		follow_target.vy = 0.f;
		follow_target.vz = 0.f;
		_follow_target_pub.publish(follow_target);
	}

	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<follow_target_s> _follow_target_pub{ORB_ID(follow_target)};
	uORB::Subscription _follow_target_estimator_sub{ORB_ID(follow_target_estimator)};
};

// The target stands still while the local position reference moves, as it does on an estimator
// switch or an origin change. The estimate must stay on the target instead of jumping with the frame
TEST_F(TargetEstimatorTest, EstimateStaysPutWhenTheReferenceMoves)
{
	TargetEstimatorTestPeer estimator;

	// the first call only records the time, every later one runs the filter
	const auto step = [&estimator]() {
		px4_usleep(10_ms);
		estimator.update();
	};
	estimator.update();

	const double target_lat = kLat + 100. / kMetresPerDegreeLat;
	const double target_lon = kLon;
	const float target_alt = kAlt + 20.f;

	publishReference(kLat, kLon, kAlt);
	publishTarget(target_lat, target_lon, target_alt);
	step();

	EXPECT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), -20.f, 0.1f);
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.1);

	// WHEN: the reference moves 200 m north and 50 m up
	publishReference(kLat + 200. / kMetresPerDegreeLat, kLon, kAlt + 50.f);
	step();

	// THEN: the states are in the new frame and the estimate still points at the target
	EXPECT_NEAR(estimator.states().pos_ned_est(0), -100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), 30.f, 0.1f);
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.1);

	// AND: the consumers are told how far the estimate moved
	const follow_target_estimator_s message = published();
	EXPECT_EQ(message.pos_reset_counter, 1);
	EXPECT_NEAR(message.delta_pos[0], -200.f, 1.f);
	EXPECT_NEAR(message.delta_pos[1], 0.f, 1.f);
	EXPECT_NEAR(message.delta_pos[2], 50.f, 0.1f);

	// AND WHEN: the same target position comes in after the fusion interval
	px4_usleep(600_ms);
	publishTarget(target_lat, target_lon, target_alt);
	step();

	// THEN: there is nothing to correct, the target didn't move
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.5);
	EXPECT_LT(estimator.states().vel_ned_est.norm(), 0.5f);
}

// Follow Me only needs a local position, so the estimator can start before EKF2 has an origin.
// It must wait for the first reference and then fuse normally
TEST_F(TargetEstimatorTest, StartsWithoutAReferenceAndTakesTheFirstOne)
{
	TargetEstimatorTestPeer estimator;

	const auto step = [&estimator]() {
		px4_usleep(10_ms);
		estimator.update();
	};
	estimator.update();

	const double target_lat = kLat + 100. / kMetresPerDegreeLat;
	const float target_alt = kAlt + 20.f;

	publishNoReference();
	publishTarget(target_lat, kLon, target_alt);
	step();
	step();

	publishReference(kLat, kLon, kAlt);
	step();
	px4_usleep(600_ms);
	publishTarget(target_lat, kLon, target_alt);
	step();

	EXPECT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), -20.f, 0.1f);
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.1);
}

// The selector switches to an instance without an origin, then that instance gets its own.
// The estimate holds in the last known frame and moves into the new one
TEST_F(TargetEstimatorTest, ReferenceLostThenRegained)
{
	TargetEstimatorTestPeer estimator;

	const auto step = [&estimator]() {
		px4_usleep(10_ms);
		estimator.update();
	};
	estimator.update();

	const double target_lat = kLat + 100. / kMetresPerDegreeLat;
	const float target_alt = kAlt + 20.f;

	publishReference(kLat, kLon, kAlt);
	publishTarget(target_lat, kLon, target_alt);
	step();
	ASSERT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);

	// WHEN: the reference is lost
	publishNoReference();
	step();

	// THEN: the estimate holds
	EXPECT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), -20.f, 0.1f);
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.1);

	// AND: the target moving 10 m north and 10 m up meanwhile is still fused, in the frame it knows
	px4_usleep(600_ms);
	publishTarget(target_lat + 10. / kMetresPerDegreeLat, kLon, target_alt + 10.f);
	step();
	EXPECT_GT(estimator.states().pos_ned_est(0), 105.f);
	EXPECT_LT(estimator.states().pos_ned_est(2), -25.f);
	const matrix::Vector3f position_before = estimator.states().pos_ned_est;
	const matrix::Vector3<double> lat_lon_alt_before = estimator.latLonAlt();

	// WHEN: a new origin 300 m south and 10 m down arrives
	publishReference(kLat - 300. / kMetresPerDegreeLat, kLon, kAlt - 10.f);
	step();

	// THEN: the states are in the new frame, the estimate still points at the target (give or take the
	// 10 ms of prediction with the velocity the fusion left) and the shift is reported
	EXPECT_NEAR(estimator.states().pos_ned_est(0), position_before(0) + 300.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), position_before(2) - 10.f, 0.1f);
	EXPECT_NEAR(estimator.latLonAlt()(0), lat_lon_alt_before(0), 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), lat_lon_alt_before(2), 0.1);
	const follow_target_estimator_s message = published();
	EXPECT_EQ(message.pos_reset_counter, 1);
	EXPECT_NEAR(message.delta_pos[0], 300.f, 1.f);
	EXPECT_NEAR(message.delta_pos[2], -10.f, 0.1f);
}

// An origin change can move the reference east only or in altitude only
TEST_F(TargetEstimatorTest, EastAndAltitudeOnlyReferenceMoves)
{
	TargetEstimatorTestPeer estimator;

	const auto step = [&estimator]() {
		px4_usleep(10_ms);
		estimator.update();
	};
	estimator.update();

	const double target_lat = kLat + 100. / kMetresPerDegreeLat;
	const float target_alt = kAlt + 20.f;

	publishReference(kLat, kLon, kAlt);
	publishTarget(target_lat, kLon, target_alt);
	step();
	ASSERT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);

	// WHEN: the reference moves 150 m east
	const double metres_per_degree_lon = kMetresPerDegreeLat * cos(math::radians(kLat));
	publishReference(kLat, kLon + 150. / metres_per_degree_lon, kAlt);
	step();

	// THEN: the states and the reported shift follow
	EXPECT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(1), -150.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), -20.f, 0.1f);
	EXPECT_NEAR(published().delta_pos[1], -150.f, 1.f);
	EXPECT_EQ(published().pos_reset_counter, 1);

	// WHEN: only the reference altitude moves 50 m up
	publishReference(kLat, kLon + 150. / metres_per_degree_lon, kAlt + 50.f);
	step();

	// THEN: only the vertical state and shift change, the altitude estimate stays
	EXPECT_NEAR(estimator.states().pos_ned_est(1), -150.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), 30.f, 0.1f);
	EXPECT_NEAR(published().delta_pos[2], 50.f, 0.1f);
	EXPECT_EQ(published().pos_reset_counter, 2);
	EXPECT_NEAR(estimator.latLonAlt()(0), target_lat, 1e-6);
	EXPECT_NEAR(estimator.latLonAlt()(2), target_alt, 0.1);
}

// A local position that is not global carries no reference, whatever its reference fields hold.
// MicroStrain publishes zeros with both flags clear, LPE sets xy_global before its origin exists
TEST_F(TargetEstimatorTest, ReferenceWithoutGlobalFlagsIsNotTaken)
{
	TargetEstimatorTestPeer estimator;

	const auto step = [&estimator]() {
		px4_usleep(10_ms);
		estimator.update();
	};
	estimator.update();

	const double target_lat = kLat + 100. / kMetresPerDegreeLat;
	const float target_alt = kAlt + 20.f;

	// WHEN: zeros come with both flags clear, then with only xy_global set
	publishReference(0., 0., 0.f, false, false);
	publishTarget(target_lat, kLon, target_alt);
	step();
	step();

	// THEN: nothing is fused
	EXPECT_EQ(published().fusion_count, 0u);

	px4_usleep(600_ms);
	publishReference(0., 0., 0.f, true, false);
	publishTarget(target_lat, kLon, target_alt);
	step();
	step();
	EXPECT_EQ(published().fusion_count, 0u);

	// AND WHEN: the real reference arrives
	publishReference(kLat, kLon, kAlt);
	step();
	px4_usleep(600_ms);
	publishTarget(target_lat, kLon, target_alt);
	step();

	// THEN: the target is fused in that frame with no shift reported
	EXPECT_NEAR(estimator.states().pos_ned_est(0), 100.f, 1.f);
	EXPECT_NEAR(estimator.states().pos_ned_est(2), -20.f, 0.1f);
	EXPECT_EQ(published().pos_reset_counter, 0);
}
