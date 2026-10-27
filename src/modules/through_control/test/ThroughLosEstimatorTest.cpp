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
#include <LosEstimator.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;
using through::LosEstimator;

namespace
{

constexpr uint64_t FRAME_US = 16667; // 60 Hz camera

LosEstimator makeEstimator()
{
	LosEstimator los;
	LosEstimator::Params params{};
	params.lpf_cutoff_hz = 8.f;
	los.setParams(params);
	return los;
}

/** Image angles of a target at relative position r (NED) seen by a camera with attitude q_cam. */
LosEstimator::Measurement measure(uint64_t t, const Quatf &q_cam, const Vector3f &r)
{
	const Vector3f r_c = q_cam.rotateVectorInverse(r);
	LosEstimator::Measurement m{};
	m.timestamp_sample = t;
	m.pan = atanf(r_c(1) / r_c(0));
	m.tilt = atanf(r_c(2) / r_c(0));
	m.valid = true;
	return m;
}

} // namespace

TEST(ThroughLosEstimatorTest, CameraRayFollowsImageSigns)
{
	// + pan = right (camera y), + tilt = below (camera z)
	const Vector3f ray = LosEstimator::cameraRay(math::radians(10.f), math::radians(5.f));
	EXPECT_GT(ray(1), 0.f);
	EXPECT_GT(ray(2), 0.f);
	EXPECT_NEAR(ray.norm(), 1.f, 1e-6f);
	EXPECT_NEAR(atan2f(ray(1), ray(0)), math::radians(10.f), 1e-5f);
}

TEST(ThroughLosEstimatorTest, LosDirectionUsesCameraAttitude)
{
	LosEstimator los = makeEstimator();

	// camera looking east, 30 deg down (FRD pitch -30 deg)
	const Quatf q_cam(Eulerf(0.f, math::radians(-30.f), math::radians(90.f)));
	los.addGimbalAttitude(1000000, q_cam);

	LosEstimator::Measurement m{};
	m.timestamp_sample = 1000000;
	m.valid = true;
	ASSERT_TRUE(los.update(m));

	EXPECT_NEAR(los.u()(0), 0.f, 1e-5f);
	EXPECT_NEAR(los.u()(1), cosf(math::radians(30.f)), 1e-5f);
	EXPECT_NEAR(los.u()(2), sinf(math::radians(30.f)), 1e-5f);
	EXPECT_NEAR(los.elevation(), math::radians(-30.f), 1e-5f);
	EXPECT_NEAR(los.azimuth(), math::radians(90.f), 1e-5f);
	EXPECT_EQ(los.attitudeSource(), LosEstimator::AttitudeSource::GimbalFeedback);

	// level camera looking east, target 10 deg right in the image -> azimuth 100 deg
	los.reset();
	m.timestamp_sample += FRAME_US;
	los.addGimbalAttitude(m.timestamp_sample, Quatf(Eulerf(0.f, 0.f, math::radians(90.f))));
	m.pan = math::radians(10.f);
	ASSERT_TRUE(los.update(m));
	EXPECT_NEAR(los.azimuth(), math::radians(100.f), 1e-5f);
	EXPECT_NEAR(los.elevation(), 0.f, 1e-5f);
}

TEST(ThroughLosEstimatorTest, InterpolatesAttitudeAtCaptureTime)
{
	LosEstimator los = makeEstimator();
	const uint64_t t_image = 2000000;

	// yaw 0 at 4 ms before the capture, 8 deg at 12 ms after it -> 2 deg at the capture
	los.addGimbalAttitude(t_image - 4000, Quatf(Eulerf(0.f, 0.f, 0.f)));
	los.addGimbalAttitude(t_image + 12000, Quatf(Eulerf(0.f, 0.f, math::radians(8.f))));

	LosEstimator::Measurement m{};
	m.timestamp_sample = t_image;
	m.valid = true;
	ASSERT_TRUE(los.update(m));
	EXPECT_NEAR(los.azimuth(), math::radians(2.f), math::radians(0.01f));

	// after the newest sample: hold the newest one (within max_attitude_dt)
	m.timestamp_sample = t_image + 15000;
	ASSERT_TRUE(los.update(m));
	EXPECT_NEAR(los.azimuth(), math::radians(8.f), 1e-5f);
}

TEST(ThroughLosEstimatorTest, FallsBackWithoutGimbalFeedback)
{
	LosEstimator los = makeEstimator();
	const uint64_t t = 3000000;

	LosEstimator::Measurement m{};
	m.timestamp_sample = t;
	m.valid = true;

	// no attitude at all: no LOS
	EXPECT_FALSE(los.update(m));
	EXPECT_FALSE(los.valid());

	// stale gimbal feedback (1 s old), fresh fallback
	los.addGimbalAttitude(t - 1000000, Quatf(Eulerf(0.f, 0.f, math::radians(45.f))));
	los.addFallbackAttitude(t, Quatf(Eulerf(0.f, 0.f, math::radians(-45.f))));
	ASSERT_TRUE(los.update(m));
	EXPECT_EQ(los.attitudeSource(), LosEstimator::AttitudeSource::Fallback);
	EXPECT_NEAR(los.azimuth(), math::radians(-45.f), 1e-5f);
}

TEST(ThroughLosEstimatorTest, InvalidMeasurementKeepsLos)
{
	LosEstimator los = makeEstimator();
	const Quatf q_cam{};
	los.addGimbalAttitude(1000000, q_cam);

	LosEstimator::Measurement m{};
	m.timestamp_sample = 1000000;
	m.pan = math::radians(5.f);
	m.valid = true;
	ASSERT_TRUE(los.update(m));
	const Vector3f u = los.u();

	m.timestamp_sample += FRAME_US;
	m.valid = false;
	m.pan = NAN;
	EXPECT_FALSE(los.update(m));
	EXPECT_FALSE(los.valid());
	EXPECT_TRUE(los.initialized());
	EXPECT_NEAR((los.u() - u).norm(), 0.f, 1e-6f);
}

TEST(ThroughLosEstimatorTest, StaticGeometryHasNoLosRate)
{
	LosEstimator los = makeEstimator();
	const Quatf q_cam(Eulerf(0.f, math::radians(-20.f), math::radians(30.f)));
	const Vector3f r{20.f, 10.f, 8.f};

	for (int k = 0; k < 120; ++k) {
		const uint64_t t = 1000000 + k * FRAME_US;
		los.addGimbalAttitude(t, q_cam);
		ASSERT_TRUE(los.update(measure(t, q_cam, r)));
	}

	EXPECT_NEAR((los.u() - r.normalized()).norm(), 0.f, 1e-5f);
	EXPECT_LT(los.omega().norm(), 1e-4f);
	EXPECT_NEAR(los.closingVelocity(Vector3f{0.f, 0.f, 0.f}), 0.f, 1e-6f);
}

TEST(ThroughLosEstimatorTest, ConstantVelocityMatchesAnalyticLosRate)
{
	LosEstimator los = makeEstimator();

	const Vector3f target{30.f, 5.f, -5.f};        // NED [m]
	const Vector3f p0{0.f, 0.f, -20.f};
	const Vector3f v{4.f, 1.5f, 2.f};               // [m/s]
	const Quatf q_cam(Eulerf(0.f, math::radians(-30.f), 0.f));

	Vector3f r{};

	for (int k = 0; k < 120; ++k) {  // 2 s
		const uint64_t t = 1000000 + k * FRAME_US;
		r = target - (p0 + v * (k * FRAME_US * 1e-6f));
		los.addGimbalAttitude(t, q_cam);
		ASSERT_TRUE(los.update(measure(t, q_cam, r)));
	}

	// omega_LOS = (r x r_dot) / |r|^2 with r_dot = -v (target at rest)
	const Vector3f omega_expected = r.cross(-v) / r.dot(r);
	const Vector3f omega = los.omega();

	EXPECT_NEAR((los.u() - r.normalized()).norm(), 0.f, 1e-4f);
	EXPECT_LT((omega - omega_expected).norm(), 0.03f * omega_expected.norm())
			<< "omega " << omega(0) << " " << omega(1) << " " << omega(2)
			<< ", expected " << omega_expected(0) << " " << omega_expected(1) << " " << omega_expected(2);
	EXPECT_NEAR(los.closingVelocity(v), v.dot(r.normalized()), 1e-4f);
}
