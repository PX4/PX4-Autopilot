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

#include "TerrainAltitudeLatch.hpp"

static vehicle_local_position_s localPosition(bool dist_bottom_valid, float dist_bottom)
{
	vehicle_local_position_s local_pos{};
	local_pos.z_global = true;
	local_pos.ref_alt = 400.f;
	local_pos.z = -50.f; // 450 m AMSL
	local_pos.dist_bottom_valid = dist_bottom_valid;
	local_pos.dist_bottom = dist_bottom;
	return local_pos;
}

static position_setpoint_s setpoint(float alt, float agl_alt)
{
	position_setpoint_s sp{};
	sp.valid = true;
	sp.alt = alt;
	sp.agl_alt = agl_alt;
	return sp;
}

TEST(TerrainAltitudeLatchTest, NeverValid)
{
	TerrainAltitudeLatch latch;
	EXPECT_FALSE(PX4_ISFINITE(latch.get()));
	EXPECT_FALSE(latch.isValid());
	EXPECT_FALSE(latch.isLatched());

	latch.update(localPosition(false, 20.f));
	EXPECT_FALSE(PX4_ISFINITE(latch.get()));
	EXPECT_FALSE(latch.isLatched());

	// without a terrain estimate, the setpoint keeps its own AMSL altitude
	EXPECT_FLOAT_EQ(latch.setpointAltitude(setpoint(480.f, 30.f)), 480.f);
}

TEST(TerrainAltitudeLatchTest, ValidThenLatched)
{
	TerrainAltitudeLatch latch;
	latch.update(localPosition(true, 20.f));
	EXPECT_TRUE(latch.isValid());
	EXPECT_FALSE(latch.isLatched());
	EXPECT_FLOAT_EQ(latch.get(), 430.f);

	// the estimate is lost: the last valid terrain altitude is kept
	latch.update(localPosition(false, 35.f));
	EXPECT_FALSE(latch.isValid());
	EXPECT_TRUE(latch.isLatched());
	EXPECT_FLOAT_EQ(latch.get(), 430.f);

	// and updated again once valid
	latch.update(localPosition(true, 35.f));
	EXPECT_FALSE(latch.isLatched());
	EXPECT_FLOAT_EQ(latch.get(), 415.f);
}

TEST(TerrainAltitudeLatchTest, NoGlobalReference)
{
	TerrainAltitudeLatch latch;
	vehicle_local_position_s local_pos = localPosition(true, 20.f);
	local_pos.z_global = false;
	latch.update(local_pos);
	EXPECT_FALSE(latch.isValid());
	EXPECT_FALSE(PX4_ISFINITE(latch.get()));
}

TEST(TerrainAltitudeLatchTest, ConvertTriplet)
{
	TerrainAltitudeLatch latch;
	latch.update(localPosition(true, 20.f)); // terrain at 430 m

	position_setpoint_triplet_s triplet{};
	triplet.previous = setpoint(500.f, NAN);
	triplet.current = setpoint(480.f, 30.f);
	triplet.next = setpoint(NAN, 40.f);

	latch.convertTriplet(triplet);

	// only the terrain-relative setpoints change
	EXPECT_FLOAT_EQ(triplet.previous.alt, 500.f);
	EXPECT_FLOAT_EQ(triplet.current.alt, 460.f);
	EXPECT_FLOAT_EQ(triplet.next.alt, 470.f);
	EXPECT_FLOAT_EQ(triplet.current.agl_alt, 30.f);
}

TEST(TerrainAltitudeLatchTest, InvalidSetpointNotConverted)
{
	TerrainAltitudeLatch latch;
	latch.update(localPosition(true, 20.f));

	// a zero-initialized setpoint has a finite agl_alt of 0, it must not become a setpoint on the terrain
	position_setpoint_s sp{};
	sp.alt = 480.f;
	EXPECT_FLOAT_EQ(latch.setpointAltitude(sp), 480.f);
}
