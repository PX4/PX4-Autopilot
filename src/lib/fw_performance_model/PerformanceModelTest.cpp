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
#include "PerformanceModel.hpp"

// to run: make tests TESTFILTER=PerformanceModel

class PerformanceModelTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}

	void setWeightParams(float base, float gross, float fuel)
	{
		param_set(param_handle(px4::params::WEIGHT_BASE), &base);
		param_set(param_handle(px4::params::WEIGHT_GROSS), &gross);
		param_set(param_handle(px4::params::WEIGHT_FUEL), &fuel);
		_model.updateParameters();
	}

	PerformanceModel _model;
};

TEST_F(PerformanceModelTest, weightRatioDefaultsToOne)
{
	// GIVEN: default parameters (weight compensation disabled)
	// THEN: the weight ratio is 1
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.0f);
}

TEST_F(PerformanceModelTest, weightRatioWithoutFuelCompensation)
{
	// GIVEN: base and gross weight set, fuel compensation disabled
	setWeightParams(10.0f, 12.0f, -1.0f);

	// THEN: the weight ratio is gross/base, independent of any fuel fraction
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.2f);

	_model.setFuelFractionRemaining(0.5f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.2f);
}

TEST_F(PerformanceModelTest, weightRatioScalesWithFuelFraction)
{
	// GIVEN: base and gross weight set, 4kg full fuel load
	setWeightParams(10.0f, 12.0f, 4.0f);

	// THEN: with unknown fuel fraction the full-tank gross weight is assumed
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.2f);

	// AND: the weight ratio scales linearly with the burned fuel mass
	_model.setFuelFractionRemaining(1.0f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.2f);

	_model.setFuelFractionRemaining(0.5f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.0f);

	_model.setFuelFractionRemaining(0.0f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 0.8f);
}

TEST_F(PerformanceModelTest, fuelFractionKeptOnNonFiniteInput)
{
	// GIVEN: base and gross weight set, 4kg full fuel load, and a known fuel fraction
	setWeightParams(10.0f, 12.0f, 4.0f);
	_model.setFuelFractionRemaining(0.5f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.0f);

	// WHEN: the fuel fraction input becomes non-finite (e.g. sensor failure)
	_model.setFuelFractionRemaining(NAN);

	// THEN: the last known fuel fraction is kept
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.0f);
}

TEST_F(PerformanceModelTest, fuelFractionIsConstrained)
{
	// GIVEN: base and gross weight set, 4kg full fuel load
	setWeightParams(10.0f, 12.0f, 4.0f);

	// THEN: out-of-range fuel fractions are constrained to [0, 1]
	_model.setFuelFractionRemaining(-0.5f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 0.8f);

	_model.setFuelFractionRemaining(1.5f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 1.2f);
}

TEST_F(PerformanceModelTest, weightRatioIsConstrained)
{
	// GIVEN: a fuel weight larger than the gross weight (misconfiguration)
	setWeightParams(10.0f, 12.0f, 12.0f);

	// THEN: the weight ratio is constrained to its lower bound
	_model.setFuelFractionRemaining(0.0f);
	EXPECT_FLOAT_EQ(_model.getWeightRatio(), 0.5f);
}

TEST_F(PerformanceModelTest, fuelFractionFromTankStatusFallbackChain)
{
	// GIVEN: a fuel tank status with no measurements
	fuel_tank_status_s status{};
	status.maximum_fuel_capacity = NAN;
	status.remaining_fuel = NAN;
	status.consumed_fuel = NAN;
	status.percent_remaining = UINT8_MAX;

	// THEN: no fuel fraction is available
	EXPECT_TRUE(std::isnan(PerformanceModel::getFuelFractionRemaining(status)));

	// AND: the consumed fuel is the last fallback, assuming a full tank at boot
	status.maximum_fuel_capacity = 2000.f;
	status.consumed_fuel = 500.f;
	EXPECT_FLOAT_EQ(PerformanceModel::getFuelFractionRemaining(status), 0.75f);

	// AND: a valid remaining percentage takes precedence over the consumed fuel
	status.percent_remaining = 40;
	EXPECT_FLOAT_EQ(PerformanceModel::getFuelFractionRemaining(status), 0.4f);

	// AND: the measured remaining fuel takes precedence over everything else
	status.remaining_fuel = 1000.f;
	EXPECT_FLOAT_EQ(PerformanceModel::getFuelFractionRemaining(status), 0.5f);
}

TEST_F(PerformanceModelTest, fuelFractionFromTankStatusIsConstrained)
{
	// GIVEN: a remaining fuel measurement exceeding the maximum capacity
	fuel_tank_status_s status{};
	status.maximum_fuel_capacity = 2000.f;
	status.remaining_fuel = 3000.f;
	status.consumed_fuel = NAN;
	status.percent_remaining = UINT8_MAX;

	// THEN: the fuel fraction is constrained to 1
	EXPECT_FLOAT_EQ(PerformanceModel::getFuelFractionRemaining(status), 1.0f);

	// AND: more consumed fuel than the maximum capacity results in a fraction of 0
	status.remaining_fuel = NAN;
	status.consumed_fuel = 3000.f;
	EXPECT_FLOAT_EQ(PerformanceModel::getFuelFractionRemaining(status), 0.0f);

	// AND: a remaining fuel measurement without a valid maximum capacity is unusable
	status.maximum_fuel_capacity = 0.f;
	status.remaining_fuel = 1000.f;
	EXPECT_TRUE(std::isnan(PerformanceModel::getFuelFractionRemaining(status)));
}
