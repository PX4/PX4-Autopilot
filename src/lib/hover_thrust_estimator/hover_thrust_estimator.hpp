/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file hover_thrust_estimator.hpp
 * @brief Interface class for a hover thrust estimator
 * Convention is positive thrust, hover thrust and acceleration UP
 *
 * @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <drivers/drv_hrt.h>

#include "zero_order_hover_thrust_ekf.hpp"

class HoverThrustEstimator : public ModuleParams
{
public:
	HoverThrustEstimator(ModuleParams *parent) :
		ModuleParams(parent)
	{
		ZeroOrderHoverThrustEkf::status status{};
		publishStatus(status);
	}
	~HoverThrustEstimator() = default;

	void reset();

	void update(float dt);

	void setThrust(float thrust) { _thrust = thrust; };
	void setAccel(float accel) { _acc_z = accel; };

	float getHoverThrustEstimate() const { return _hover_thrust_ekf.getHoverThrustEstimate(); }

protected:
	void updateParams() override;

private:
	void publishStatus(ZeroOrderHoverThrustEkf::status &status);

	ZeroOrderHoverThrustEkf _hover_thrust_ekf{};
	float _acc_z{};
	float _thrust{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HTE_HT_NOISE>) _param_hte_ht_noise,
		(ParamFloat<px4::params::HTE_ACC_GATE>) _param_hte_acc_gate,
		(ParamFloat<px4::params::HTE_HT_ERR_INIT>) _param_hte_ht_err_init
	)

	uORB::Publication<hover_thrust_estimate_s> _hover_thrust_ekf_pub{ORB_ID(hover_thrust_estimate)};
};
