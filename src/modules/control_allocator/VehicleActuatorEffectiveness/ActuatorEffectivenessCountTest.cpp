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
 * @file ActuatorEffectivenessCountTest.cpp
 *
 * The control surface and tilt counts come from parameters that are not range-checked on the way in.
 * They bound loops over fixed-size arrays, so they must be clamped to the class maximum.
 */

#include <gtest/gtest.h>
#include <parameters/param.h>

#include "ActuatorEffectivenessControlSurfaces.hpp"
#include "ActuatorEffectivenessTilts.hpp"

static void setCount(const char *name, int32_t value)
{
	param_t handle = param_find(name);
	ASSERT_NE(handle, PARAM_INVALID) << name;
	ASSERT_EQ(param_set(handle, &value), 0) << name;
}

TEST(ActuatorEffectivenessCountTest, ControlSurfacesCountIsClamped)
{
	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	setCount("CA_SV_CS_COUNT", ActuatorEffectivenessControlSurfaces::MAX_COUNT + 5);
	ActuatorEffectivenessControlSurfaces too_many(nullptr);
	EXPECT_EQ(too_many.count(), ActuatorEffectivenessControlSurfaces::MAX_COUNT);

	setCount("CA_SV_CS_COUNT", -3);
	ActuatorEffectivenessControlSurfaces negative(nullptr);
	EXPECT_EQ(negative.count(), 0);

	setCount("CA_SV_CS_COUNT", 2);
	ActuatorEffectivenessControlSurfaces two(nullptr);
	EXPECT_EQ(two.count(), 2);

	setCount("CA_SV_CS_COUNT", 0);
}

TEST(ActuatorEffectivenessCountTest, TiltsCountIsClamped)
{
	param_control_autosave(false);

	setCount("CA_SV_TL_COUNT", ActuatorEffectivenessTilts::MAX_COUNT + 5);
	ActuatorEffectivenessTilts too_many(nullptr);
	EXPECT_EQ(too_many.count(), ActuatorEffectivenessTilts::MAX_COUNT);

	setCount("CA_SV_TL_COUNT", -3);
	ActuatorEffectivenessTilts negative(nullptr);
	EXPECT_EQ(negative.count(), 0);

	setCount("CA_SV_TL_COUNT", 1);
	ActuatorEffectivenessTilts one(nullptr);
	EXPECT_EQ(one.count(), 1);

	setCount("CA_SV_TL_COUNT", 0);
}
