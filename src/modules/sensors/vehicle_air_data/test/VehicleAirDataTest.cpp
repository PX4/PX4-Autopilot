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
 * Test for VehicleAirData
 */

#include <gtest/gtest.h>

#include "../VehicleAirData.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_gnss.h>
#include <uORB/uORBManager.hpp>

class VehicleAirDataTest : public ::testing::Test
{
public:
	class VehicleAirDataTestable : public sensors::VehicleAirData
	{
	public:
		// Run() re-arms itself through the hrt callout worker, cleared again so the cycle stays
		// in the test's hands
		void RunPublic()
		{
			Run();
			ScheduleClear();
		}
		int selectedIndex() const { return selectedSensorIndex(); }
		uint8_t priority(int index) const { return sensorPriority(index); }
		bool callbackRegistered(int index) const { return VehicleAirData::callbackRegistered(index); }
	};

	static constexpr int kSensorCount = 1;

	// device i publishes on sensor_baro instance i and is calibrated in slot CAL_BAROi for the whole suite
	static constexpr uint32_t kDeviceIds[kSensorCount] {0x3c4d01};

	// uORB nodes outlive a manager restart, so the suite brings the manager up once and keeps its topics
	static void SetUpTestSuite()
	{
		// the gtest harness does not bring up the hrt: Run() schedules its next cycle through it,
		// and without hrt_init() the callout lock is not recursive, so the worker deadlocks on
		// its own nested lock on Linux the first time a callout is due
		hrt_init();
		hrt_work_queue_init();
		uORB::Manager::initialize();

		for (int i = 0; i < kSensorCount; i++) {
			ASSERT_TRUE(_sensor_baro_pubs[i].advertise());
			ASSERT_EQ(_sensor_baro_pubs[i].get_instance(), i);
		}
	}

	// every test starts with all devices disabled and uncalibrated, and enables the ones it uses
	void SetUp() override
	{
		for (int i = 0; i < kSensorCount; i++) {
			setPriority(i, 0);
			setParamFloat(kOffsetParams[i], 0.f);
		}
	}

	static void setParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	static void setParamFloat(const char *name, float value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	static float getParamFloat(const char *name)
	{
		float value = NAN;
		param_get(param_find(name), &value);
		return value;
	}

	static void setPriority(int device, int32_t priority)
	{
		char name[20] {};
		snprintf(name, sizeof(name), "CAL_BARO%d_ID", device);
		setParam(name, static_cast<int32_t>(kDeviceIds[device]));
		snprintf(name, sizeof(name), "CAL_BARO%d_PRIO", device);
		setParam(name, priority);
	}

	static void publishSample(int device, float pressure_pa)
	{
		sensor_baro_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.timestamp = sample.timestamp_sample;
		sample.device_id = kDeviceIds[device];
		sample.is_external = true;
		sample.pressure = pressure_pa;
		sample.temperature = 20.f;
		_sensor_baro_pubs[device].publish(sample);
	}

	// landed, with EKF2 fusing GNSS height, by default: the state the GNSS baro calibration runs in
	static void publishEstimatorStatusFlags(bool in_air = false, bool gnss_height_fused = true)
	{
		estimator_status_flags_s flags{};
		flags.timestamp = hrt_absolute_time();
		flags.cs_gps_hgt = gnss_height_fused;
		flags.cs_in_air = in_air;
		_estimator_status_flags_pub.publish(flags);
	}

	static void publishGnss(hrt_abstime timestamp, double altitude_msl, uint16_t failed_strict_checks)
	{
		vehicle_gnss_s gnss{};
		gnss.timestamp = timestamp;
		gnss.timestamp_sample = timestamp;
		gnss.receiver.timestamp = timestamp;
		gnss.receiver.timestamp_sample = timestamp;
		gnss.receiver.altitude_msl = altitude_msl;
		gnss.usable = true;
		gnss.strict = false;
		gnss.failed_strict_checks = failed_strict_checks;
		_vehicle_gnss_pub.publish(gnss);
	}

	// a run of GNSS samples 100 ms apart, all with the same solution and estimator state
	struct GnssSegment {
		int samples;
		double altitude_msl;
		uint16_t failed_strict_checks;
		bool in_air = false;
		bool gnss_height_fused = true;
	};

	// brings the module up on the given altitude, then feeds it the given GNSS segments in order, and returns
	// the baro offset it ends up with
	template<size_t N>
	static float runGnssCalibration(const GnssSegment(&segments)[N])
	{
		setPriority(0, 75);
		setParam("SENS_BAR_AUTOCAL", 1);

		VehicleAirDataTestable module;

		// a little noise, as the voter flags a long run of identical samples as stale
		int sample_count = 0;
		auto noisy_pressure = [&]() { return kPressurePa + ((sample_count++ % 2) ? 0.5f : -0.5f); };

		// selection, then the relative calibration that has to complete first
		runFor(module, 1500_ms, [&]() {
			publishSample(0, noisy_pressure());
			publishEstimatorStatusFlags();
		});

		EXPECT_EQ(module.selectedIndex(), 0);

		// the calibration times its windows on the GNSS timestamps, so the samples can come faster than real time
		hrt_abstime gnss_timestamp = hrt_absolute_time();

		for (const GnssSegment &segment : segments) {
			for (int i = 0; i < segment.samples; i++) {
				gnss_timestamp += 100_ms;
				publishSample(0, noisy_pressure());
				publishEstimatorStatusFlags(segment.in_air, segment.gnss_height_fused);
				publishGnss(gnss_timestamp, segment.altitude_msl, segment.failed_strict_checks);
				module.RunPublic();
				px4_usleep(5_ms);
			}
		}

		return getParamFloat(kOffsetParams[0]);
	}

	static void publishParameterUpdate()
	{
		parameter_update_s update{};
		update.timestamp = hrt_absolute_time();
		_parameter_update_pub.publish(update);
	}

	// run the module at its own 50 ms cadence for the given time, feeding it the given samples
	template<typename Feed>
	static void runFor(VehicleAirDataTestable &module, hrt_abstime duration, Feed feed)
	{
		const hrt_abstime start = hrt_absolute_time();

		do {
			feed();
			module.RunPublic();
			px4_usleep(50_ms);
		} while (hrt_absolute_time() < start + duration);
	}

	static uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pubs[kSensorCount];
	static uORB::Publication<parameter_update_s> _parameter_update_pub;
	static uORB::Publication<estimator_status_flags_s> _estimator_status_flags_pub;
	static uORB::Publication<vehicle_gnss_s> _vehicle_gnss_pub;

	static constexpr const char *kOffsetParams[kSensorCount] {"CAL_BARO0_OFF"};

	// standard sea level pressure at the default SENS_BARO_QNH, so the uncalibrated baro reads 0 m
	static constexpr float kPressurePa = 101325.f;
};

constexpr uint32_t VehicleAirDataTest::kDeviceIds[];
uORB::PublicationMulti<sensor_baro_s> VehicleAirDataTest::_sensor_baro_pubs[kSensorCount] {{ORB_ID(sensor_baro)}};
uORB::Publication<parameter_update_s> VehicleAirDataTest::_parameter_update_pub{ORB_ID(parameter_update)};
uORB::Publication<estimator_status_flags_s> VehicleAirDataTest::_estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
uORB::Publication<vehicle_gnss_s> VehicleAirDataTest::_vehicle_gnss_pub{ORB_ID(vehicle_gnss)};
constexpr const char *VehicleAirDataTest::kOffsetParams[];

TEST_F(VehicleAirDataTest, BarometerDisabledFromBootDoesNotDriveRun)
{
	// the only barometer is disabled before the module sees it: nothing ever gets selected, so
	// nothing ever clears the callbacks, and one registered here would wake Run() at the sample rate
	VehicleAirDataTestable module;

	runFor(module, 500_ms, [&]() { publishSample(0, kPressurePa); });

	EXPECT_EQ(module.selectedIndex(), -1);
	EXPECT_EQ(module.priority(0), 0);
	EXPECT_FALSE(module.callbackRegistered(0));

	// re-enabled, the sensor is selected and only then drives Run()
	setPriority(0, 75);
	publishParameterUpdate();

	// long enough for the module's throttled parameter subscription to pick up the change
	runFor(module, 1200_ms, [&]() { publishSample(0, kPressurePa); });

	EXPECT_EQ(module.selectedIndex(), 0);
	EXPECT_EQ(module.priority(0), 75);
	EXPECT_TRUE(module.callbackRegistered(0));
}

TEST_F(VehicleAirDataTest, GnssBaroCalibrationUsesGnssPassingStrictChecks)
{
	// a receiver passing the strict checks on a steady altitude 15 m above the baro's calibrates the baro onto it
	const float offset = runGnssCalibration({{60, 15., 0}});

	// roughly 12 Pa per metre at sea level, subtracted from the measured pressure so the baro reads higher
	EXPECT_GT(offset, 150.f);
	EXPECT_LT(offset, 210.f);
}

TEST_F(VehicleAirDataTest, GnssBaroCalibrationIgnoresGnssFailingStrictChecks)
{
	// once the strict checks have passed on the ground, the relaxed ones decide usable, so EKF2 keeps fusing
	// GNSS height through a degraded patch, here a 15 m altitude error with an EPV above GNSS_REQ_EPV, so
	// the strict checks reject the sample
	const float offset = runGnssCalibration({{60, 15., vehicle_gnss_s::CHECK_EPV}});

	EXPECT_FLOAT_EQ(offset, 0.f);
}

TEST_F(VehicleAirDataTest, GnssBaroCalibrationRestartsAfterGnssFailingStrictChecks)
{
	// the receiver reads 8 m long enough for the first half of the stability window, fails the strict checks,
	// and comes back reading 15 m. The window restarts from scratch on the 15 m solution, which is steady for
	// longer than the whole window, so the baro is calibrated onto it. A reference left over from before the
	// failure would differ by more than the 4 m tolerance, reject the window, and leave the baro uncalibrated
	const float offset = runGnssCalibration({
		{25, 8., 0},
		{5, 8., vehicle_gnss_s::CHECK_EPV},
		{45, 15., 0},
	});

	// calibrated onto 15 m, not 8 m, at roughly 12 Pa per metre
	EXPECT_GT(offset, 150.f);
	EXPECT_LT(offset, 210.f);
}

TEST_F(VehicleAirDataTest, GnssBaroCalibrationRestartsAfterFlight)
{
	// the vehicle takes off past the first half of the stability window and lands after longer than the whole
	// window. Back on the ground, the window restarts from scratch and a few samples are not enough to calibrate.
	// A window resumed from before the takeoff would already have run its course, and the first sample after
	// landing would complete it, possibly with the props still spinning
	const float offset = runGnssCalibration({
		{30, 15., 0},
		{50, 115., 0, true},
		{5, 15., 0},
	});

	EXPECT_FLOAT_EQ(offset, 0.f);
}

TEST_F(VehicleAirDataTest, GnssBaroCalibrationRestartsAfterGnssHeightFusionStops)
{
	// as above, with EKF2 dropping GNSS height on the ground instead of the vehicle taking off
	const float offset = runGnssCalibration({
		{30, 15., 0},
		{50, 15., 0, false, false},
		{5, 15., 0},
	});

	EXPECT_FLOAT_EQ(offset, 0.f);
}

TEST_F(VehicleAirDataTest, GnssBaroCalibrationCompletesAfterFlight)
{
	// interrupted by a flight, the calibration still completes once the solution is steady on the ground for the
	// whole window
	const float offset = runGnssCalibration({
		{30, 15., 0},
		{50, 115., 0, true},
		{45, 15., 0},
	});

	EXPECT_GT(offset, 150.f);
	EXPECT_LT(offset, 210.f);
}
