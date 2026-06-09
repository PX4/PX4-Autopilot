/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file base_station.h
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "gps_helper.h"

/**
 * @class GPSBaseStationSupport
 * GPS driver base class with Base Station Support
 */
class GPSBaseStationSupport : public GPSHelper
{
public:
	GPSBaseStationSupport(GPSCallbackPtr callback, void *callback_user)
		: GPSHelper(callback, callback_user) {}

	virtual ~GPSBaseStationSupport() = default;

	/**
	 * set survey-in specs for RTK base station setup (for finding an accurate base station position
	 * by averaging the position measurements over time).
	 * @param survey_in_acc_limit minimum accuracy in 0.1mm
	 * @param survey_in_min_dur minimum duration in seconds
	 */
	void setSurveyInSpecs(uint32_t survey_in_acc_limit, uint32_t survey_in_min_dur)
	{
		_base_settings.type = BaseSettingsType::survey_in;
		_base_settings.settings.survey_in.acc_limit = survey_in_acc_limit;
		_base_settings.settings.survey_in.min_dur = survey_in_min_dur;
	}

	/**
	 * Set a fixed base station position. This can be used if the base position is already known to
	 * avoid doing a survey-in.
	 * @param latitude [deg]
	 * @param longitude [deg]
	 * @param altitude [m]
	 * @param position_accuracy 3D position accuracy (set to 0 if unknown) [mm]
	 */
	void setBasePosition(double latitude, double longitude, float altitude, float position_accuracy)
	{
		_base_settings.type = BaseSettingsType::fixed_position;
		_base_settings.settings.fixed_position.latitude = latitude;
		_base_settings.settings.fixed_position.longitude = longitude;
		_base_settings.settings.fixed_position.altitude = altitude;
		_base_settings.settings.fixed_position.position_accuracy = position_accuracy;
	}

protected:

	enum class BaseSettingsType : uint8_t {
		survey_in,
		fixed_position
	};
	struct SurveyInSettings {
		uint32_t acc_limit;
		uint32_t min_dur;
	};
	struct FixedPositionSettings {
		double latitude;
		double longitude;
		float altitude;
		float position_accuracy;
	};
	struct BaseSettings {
		BaseSettingsType type;
		union {
			SurveyInSettings survey_in;
			FixedPositionSettings fixed_position;
		} settings;
	};
	BaseSettings _base_settings;
};

