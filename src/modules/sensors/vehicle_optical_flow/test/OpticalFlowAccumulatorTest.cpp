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

#include "../OpticalFlowAccumulator.hpp"

using namespace sensors;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Vector3f;

static constexpr uint32_t kFrameUs = 8000;

static sensor_optical_flow_s makeFrame(uint64_t timestamp_sample, float flow_x, float flow_y, uint8_t quality)
{
	sensor_optical_flow_s flow{};
	flow.timestamp_sample = timestamp_sample;
	flow.integration_timespan_us = kFrameUs;
	flow.pixel_flow[0] = flow_x;
	flow.pixel_flow[1] = flow_y;
	flow.quality = quality;
	flow.delta_angle_available = false;
	return flow;
}

TEST(OpticalFlowAccumulatorTest, TrackedFramesSumWithTheirOwnGyroWindow)
{
	OpticalFlowAccumulator accumulator;
	const Vector3f rate{0.1f, -0.2f, 0.3f};

	for (uint64_t t = 1000; t <= 40000; t += 1000) {
		accumulator.addGyroSample(t, rate, 1e-3f);
	}

	// the integrator's first sample has no predecessor to average with, so start from a primed window
	accumulator.addFrame(makeFrame(10000, 0.f, 0.f, 100));
	accumulator.reset();

	EXPECT_FALSE(accumulator.addFrame(makeFrame(18000, 0.01f, 0.02f, 100)));
	EXPECT_FALSE(accumulator.addFrame(makeFrame(26000, 0.03f, -0.01f, 120)));

	const OpticalFlowAccumulator::Window window = accumulator.window();
	EXPECT_EQ(window.frame_count, 2);
	EXPECT_EQ(window.integration_timespan_us, 2 * kFrameUs);
	EXPECT_EQ(window.quality, 110);
	EXPECT_NEAR(window.pixel_flow(0), 0.04f, 1e-6f);
	EXPECT_NEAR(window.pixel_flow(1), 0.01f, 1e-6f);

	const Vector3f expected = rate * (2 * kFrameUs * 1e-6f);

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(window.delta_angle(i), expected(i), 1e-6f);
	}
}

TEST(OpticalFlowAccumulatorTest, RejectedFramesCountTowardCadenceOnly)
{
	OpticalFlowAccumulator accumulator;

	accumulator.addFrame(makeFrame(10000, 0.01f, 0.f, 100));
	accumulator.addFrame(makeFrame(18000, 5.f, 5.f, 0));

	EXPECT_EQ(accumulator.accumulatedTimespanUs(), 2 * kFrameUs);

	OpticalFlowAccumulator::Window window = accumulator.window();
	EXPECT_EQ(window.frame_count, 1);
	EXPECT_EQ(window.integration_timespan_us, kFrameUs);
	EXPECT_EQ(window.quality, 100);
	EXPECT_NEAR(window.pixel_flow(0), 0.01f, 1e-6f);
	EXPECT_NEAR(window.pixel_flow(1), 0.f, 1e-6f);

	accumulator.reset();
	accumulator.addFrame(makeFrame(26000, 5.f, 5.f, 0));

	window = accumulator.window();
	EXPECT_EQ(window.frame_count, 0);
	EXPECT_EQ(window.quality, 0);
	EXPECT_EQ(window.integration_timespan_us, kFrameUs);
	EXPECT_NEAR(window.pixel_flow(0), 0.f, 1e-6f);
}

TEST(OpticalFlowAccumulatorTest, GapDiscardsAccumulation)
{
	OpticalFlowAccumulator accumulator;

	EXPECT_FALSE(accumulator.addFrame(makeFrame(10000, 0.01f, 0.f, 100)));
	EXPECT_TRUE(accumulator.addFrame(makeFrame(10000 + 3 * kFrameUs, 0.02f, 0.f, 100)));

	const OpticalFlowAccumulator::Window window = accumulator.window();
	EXPECT_EQ(window.frame_count, 1);
	EXPECT_NEAR(window.pixel_flow(0), 0.02f, 1e-6f);
}

TEST(OpticalFlowAccumulatorTest, SensorDeltaAngleIsRotatedAndKeepsMissingZ)
{
	OpticalFlowAccumulator accumulator;
	accumulator.setSensorRotation(Dcmf{Eulerf{0.f, 0.f, M_PI_F}});

	sensor_optical_flow_s flow = makeFrame(10000, 0.01f, 0.f, 100);
	flow.delta_angle_available = true;
	flow.delta_angle[0] = 0.01f;
	flow.delta_angle[1] = 0.02f;
	flow.delta_angle[2] = NAN;

	accumulator.addFrame(flow);

	EXPECT_FALSE(accumulator.gyroRequired());

	const OpticalFlowAccumulator::Window window = accumulator.window();
	EXPECT_NEAR(window.delta_angle(0), -0.01f, 1e-6f);
	EXPECT_NEAR(window.delta_angle(1), -0.02f, 1e-6f);
	EXPECT_FALSE(PX4_ISFINITE(window.delta_angle(2)));
}
