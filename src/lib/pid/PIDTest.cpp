/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
#include <PID.hpp>

TEST(PIDTest, AllZeroCase)
{
	PID pid;
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.f, false), 0.f);
}

TEST(PIDTest, OutputLimit)
{
	PID pid;
	pid.setOutputLimit(.01f);
	pid.setGains(.1f, 0.f, 0.f);
	pid.setSetpoint(1.f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.f, false), .01f);
	EXPECT_FLOAT_EQ(pid.update(.9f, 0.f, false), .01f);
	EXPECT_NEAR(pid.update(.95f, 0.f, false), .005f, 1e-6f);
	EXPECT_FLOAT_EQ(pid.update(1.f, 0.f, false), 0.f);
	EXPECT_NEAR(pid.update(1.05f, 0.f, false), -.005f, 1e-6f);
	EXPECT_FLOAT_EQ(pid.update(1.1f, 0.f, false), -.01f);
	EXPECT_FLOAT_EQ(pid.update(1.15f, 0.f, false), -.01f);
	EXPECT_FLOAT_EQ(pid.update(2.f, 0.f, false), -.01f);
}

TEST(PIDTest, ProportinalOnly)
{
	PID pid;
	pid.setOutputLimit(1.f);
	pid.setGains(.1f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.f, false), 0.f);
	pid.setSetpoint(1.f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.f, false), .1f);
	EXPECT_FLOAT_EQ(pid.update(1.f, 0.f, false), 0.f);

	float plant = 0.f;
	float output = 10000.f;
	int i; // need function scope to check how many steps

	for (i = 1000; i > 0; i--) {
		const float output_new = pid.update(plant, 0.f, false);
		plant += output_new;

		// expect the output to get smaller with each iteration
		if (output_new >= output) {
			break;
		}

		output = output_new;
	}

	EXPECT_FLOAT_EQ(plant, 1.f);
	EXPECT_GT(i, 0); // it shouldn't have taken longer than an iteration timeout to converge
}

TEST(PIDTest, InteralOpenLoop)
{
	PID pid;
	pid.setOutputLimit(1.f);
	pid.setGains(0.f, .1f, 0.f);
	pid.setIntegralLimit(.05f);
	pid.setSetpoint(1.f);

	// Zero error
	EXPECT_FLOAT_EQ(pid.update(1.f, 0.f, true), 0.f);
	EXPECT_FLOAT_EQ(pid.update(1.f, 0.f, true), 0.f);
	EXPECT_FLOAT_EQ(pid.update(1.f, 0.f, true), 0.f);

	// Open loop ramp up
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), 0.f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .01f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .02f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .03f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .04f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .05f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .05f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .05f);

	// Open loop ramp down
	pid.setSetpoint(-1.f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .05f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .04f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .03f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), .02f);
	EXPECT_NEAR(pid.update(0.f, 0.1f, true), .01f, 1e-6f);
	EXPECT_NEAR(pid.update(0.f, 0.1f, true), 0.f, 1e-6f);
	EXPECT_NEAR(pid.update(0.f, 0.1f, true), -.01f, 1e-6f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), -.02f);
	EXPECT_NEAR(pid.update(0.f, 0.1f, true), -.03f, 1e-6f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), -.04f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), -.05f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), -.05f);
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), -.05f);
	pid.resetIntegral();
	EXPECT_FLOAT_EQ(pid.update(0.f, 0.1f, true), -.01f);
}
