/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
#include "ModeManagement.hpp"

static bool modeValid(uint8_t mode)
{
	return mode >= Modes::FIRST_EXTERNAL_NAV_STATE && mode <= Modes::LAST_EXTERNAL_NAV_STATE;
}

static int32_t readHash(int idx)
{
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "COM_MODE%u_HASH", idx);
	param_t param = param_find(buffer);
	int32_t value{};
	param_get(param, &value);
	return value;
}

TEST(ModeManagementTest, Hashes)
{
	param_control_autosave(false);

	// Reset parameters
	for (int i = 0; i < Modes::MAX_NUM; ++i) {
		char buffer[20];
		snprintf(buffer, sizeof(buffer), "COM_MODE%u_HASH", i);
		param_t param = param_find(buffer);
		param_reset(param);
	}

	// Add full set of modes, which stores the hashes
	Modes modes;
	Modes::Mode mode;

	for (int i = 0; i < Modes::MAX_NUM; ++i) {
		snprintf(mode.name, sizeof(mode.name), "mode %i", i);
		EXPECT_EQ(modes.addExternalMode(mode), Modes::FIRST_EXTERNAL_NAV_STATE + i);
		EXPECT_EQ(readHash(i), events::util::hash_32_fnv1a_const(mode.name));
	}

	EXPECT_FALSE(modes.hasFreeExternalModes());

	// Remove all modes, except last
	for (int i = 0; i < Modes::MAX_NUM - 1; ++i) {
		snprintf(mode.name, sizeof(mode.name), "mode %i", i);
		EXPECT_TRUE(modes.removeExternalMode(Modes::FIRST_EXTERNAL_NAV_STATE + i, mode.name));
	}

	// Add some mode, ensure it gets the same index
	const int mode_to_add_idx = 3;
	snprintf(mode.name, sizeof(mode.name), "mode %i", mode_to_add_idx);
	EXPECT_EQ(modes.addExternalMode(mode), Modes::FIRST_EXTERNAL_NAV_STATE + mode_to_add_idx);

	// Try to add another one with the same name: should succeed, with the hash of the added index reset
	uint8_t added_mode_nav_state = modes.addExternalMode(mode);
	EXPECT_EQ(readHash(added_mode_nav_state - Modes::FIRST_EXTERNAL_NAV_STATE), 0);

	// 3 Modes are used now. Add N-3 new ones which must overwrite previous hashes
	for (int i = 0; i < Modes::MAX_NUM - 3; ++i) {
		snprintf(mode.name, sizeof(mode.name), "new mode %i", i);
		added_mode_nav_state = modes.addExternalMode(mode);
		EXPECT_TRUE(modeValid(added_mode_nav_state));
		EXPECT_EQ(readHash(added_mode_nav_state - Modes::FIRST_EXTERNAL_NAV_STATE),
			  events::util::hash_32_fnv1a_const(mode.name));
	}

	EXPECT_FALSE(modes.hasFreeExternalModes());
}
