/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file PerformanceModel.hpp
 * Performance model.
 */

#include <px4_platform_common/module_params.h>

#ifndef PX4_SRC_MODULES_FW_POS_CONTROL_PERFORMANCEMODEL_H_
#define PX4_SRC_MODULES_FW_POS_CONTROL_PERFORMANCEMODEL_H_

class PerformanceModel : public ModuleParams
{
public:
	PerformanceModel();
	~PerformanceModel() = default;

	void updateParameters();

	/**
	 * Get the maximum climb rate (true airspeed) expected for current air density and weight.
	 * @param air_density in kg/m^3
	 * @return maximum climb rate in m/s
	 */
	float getMaximumClimbRate(float air_density) const;

	/**
	 * Get the minimum sink rate (true airspeed) expected for current air density and weight.
	 * @param air_density in kg/m^3
	 * @return minimum sink rate in m/s
	 */
	float getMinimumSinkRate(float air_density) const;

	/**
	 * Get the ration of actual weight to base weight
	 * @return weight ratio
	 */
	float getWeightRatio() const;

	/**
	 * Get the trim throttle for the current airspeed setpoint as well as air density and weight.
	 * @param throttle_min minimum throttle in range [0,1]
	 * @param throttle_max maximum throttle in range [0,1]
	 * @param airspeed_sp calibrated airspeed setpoint in m/s
	 * @param air_density air density in kg/m^3
	 * @return trim throttle in range [0,1]
	 */
	float getTrimThrottle(float throttle_min, float throttle_max, float airspeed_sp, float air_density) const;

	/**
	 * Get the throttle scale factor for the current air density.
	 * @param air_density in kg/m^3
	 * @return throttle scale factor for air density
	 */
	float getAirDensityThrottleScale(float air_density) const;

	/**
	 * Get the trim airspeed compensated for weight.
	 * @return calibrated trim airspeed in m/s
	 */
	float getCalibratedTrimAirspeed() const;

	/**
	 * Get the minimum airspeed compensated for weight and load factor due to bank angle.
	 * @param load_factor due to banking
	 * @return calibrated minimum airspeed in m/s
	 */
	float getMinimumCalibratedAirspeed(float load_factor = 1.0f) const;

	/**
	 * Get the maximum airspeed.
	 * @return calibrated maximum airspeed in m/s
	 */
	float getMaximumCalibratedAirspeed() const;

	/**
	 * get the stall airspeed compensated for load factor due to bank angle.
	 * @param load_factor load factor due to banking
	 * @return calibrated stall airspeed in m/s
	 */
	float getCalibratedStallAirspeed(float load_factor) const;

	/**
	 * Run some checks on parameters and detect unfeasible combinations.
	 * @return true if all checks pass
	 */
	bool runSanityChecks() const;

private:
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,
		(ParamFloat<px4::params::FW_T_CLMB_MAX>) _param_fw_t_clmb_max,
		(ParamFloat<px4::params::FW_T_SINK_MIN>) _param_fw_t_sink_min,
		(ParamFloat<px4::params::WEIGHT_BASE>) _param_weight_base,
		(ParamFloat<px4::params::WEIGHT_GROSS>) _param_weight_gross,
		(ParamFloat<px4::params::FW_SERVICE_CEIL>) _param_service_ceiling,
		(ParamFloat<px4::params::FW_THR_TRIM>) _param_fw_thr_trim,
		(ParamFloat<px4::params::FW_THR_IDLE>) _param_fw_thr_idle,
		(ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
		(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,
		(ParamFloat<px4::params::FW_THR_ASPD_MIN>) _param_fw_thr_aspd_min,
		(ParamFloat<px4::params::FW_THR_ASPD_MAX>) _param_fw_thr_aspd_max)

	/**
	 * Get the sea level trim throttle for a given calibrated airspeed setpoint.
	 * @param calibrated_airspeed_sp [m/s] calibrated
	 * @return trim throttle [0, 1] at sea level
	 */
	float getTrimThrottleForCalibratedAirspeed(float calibrated_airspeed_sp) const;


	static float sanitiseAirDensity(float air_density);
};

#endif //PX4_SRC_MODULES_FW_POS_CONTROL_PERFORMANCEMODEL_H_
