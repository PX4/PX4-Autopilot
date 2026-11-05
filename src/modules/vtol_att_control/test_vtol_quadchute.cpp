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
 * @file test_vtol_quadchute.cpp
 *
 * A quad-chute commanded by the user only switches to multicopter. A detected one is a fixed-wing failure,
 * which triggers the quad-chute failsafe action and blocks arming.
 */

#include <gtest/gtest.h>

#include "vtol_att_control_main.h"

TEST(VtolQuadchuteTest, CommandedQuadchuteIsNoFixedWingFailure)
{
	VtolAttitudeControl vtol_att_control;

	vtol_att_control.quadchute(QuadchuteReason::ExternalCommand);
	EXPECT_TRUE(vtol_att_control.get_external_quadchute()); // forces multicopter flight
	EXPECT_FALSE(vtol_att_control.get_vtol_vehicle_status()->fixed_wing_system_failure);

	vtol_att_control.quadchute(QuadchuteReason::TransitionTimeout);
	EXPECT_TRUE(vtol_att_control.get_vtol_vehicle_status()->fixed_wing_system_failure);
}
