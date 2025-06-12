/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocation.hpp
 *
 * Interface for Control Allocation Algorithms
 *
 * Implementers of this interface are expected to update the members
 * of this base class in the `allocate` method.
 *
 * Example usage:
 * ```
 * [...]
 * // Initialization
 * ControlAllocationMethodImpl alloc();
 * alloc.setEffectivenessMatrix(effectiveness, actuator_trim);
 * alloc.setActuatorMin(actuator_min);
 * alloc.setActuatorMin(actuator_max);
 *
 * while (1) {
 * 	[...]
 *
 * 	// Set control setpoint, allocate actuator setpoint, retrieve actuator setpoint
 * 	alloc.setControlSetpoint(control_sp);
 * 	alloc.allocate();
 * 	actuator_sp = alloc.getActuatorSetpoint();
 *
 * 	// Check if the control setpoint was fully allocated
 *	unallocated_control = control_sp - alloc.getAllocatedControl()
 *
 *	[...]
 * }
 * ```
 *
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"

class ControlAllocation
{
public:
	ControlAllocation();
	virtual ~ControlAllocation() = default;

	static constexpr uint8_t NUM_ACTUATORS = ActuatorEffectiveness::NUM_ACTUATORS;
	static constexpr uint8_t NUM_AXES = ActuatorEffectiveness::NUM_AXES;

	typedef matrix::Vector<float, NUM_ACTUATORS> ActuatorVector;

	enum ControlAxis {
		ROLL = 0,
		PITCH,
		YAW,
		THRUST_X,
		THRUST_Y,
		THRUST_Z
	};

	/**
	 * Allocate control setpoint to actuators
	 */
	virtual void allocate() = 0;

	/**
	 * Set actuator failure flag
	 * This prevents a change of the scaling in the matrix normalization step
	 * in case of a motor failure.
	 *
	 * @param failure  Motor failure flag
	 */
	void setHadActuatorFailure(bool failure) { _had_actuator_failure = failure; }

	/**
	 * Set the control effectiveness matrix
	 *
	 * @param B Effectiveness matrix
	 */
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
					    bool update_normalization_scale);

	/**
	 * Get the allocated actuator vector
	 *
	 * @return Actuator vector
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorSetpoint() const { return _actuator_sp; }

	/**
	 * Set the desired control vector
	 *
	 * @param Control vector
	 */
	void setControlSetpoint(const matrix::Vector<float, NUM_AXES> &control) { _control_sp = control; }

	/**
	 * Get the desired control vector
	 *
	 * @return Control vector
	 */
	const matrix::Vector<float, NUM_AXES> &getControlSetpoint() const { return _control_sp; }

	/**
	 * Get the allocated control vector
	 *
	 * @return Control vector
	 */
	matrix::Vector<float, NUM_AXES> getAllocatedControl() const
	{ return (_effectiveness * (_actuator_sp - _actuator_trim)).emult(_control_allocation_scale); }

	/**
	 * Get the control effectiveness matrix
	 *
	 * @return Effectiveness matrix
	 */
	const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &getEffectivenessMatrix() const { return _effectiveness; }

	/**
	 * Set the minimum actuator values
	 *
	 * @param actuator_min Minimum actuator values
	 */
	void setActuatorMin(const matrix::Vector<float, NUM_ACTUATORS> &actuator_min) { _actuator_min = actuator_min; }

	/**
	 * Get the minimum actuator values
	 *
	 * @return Minimum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMin() const { return _actuator_min; }

	/**
	 * Set the maximum actuator values
	 *
	 * @param actuator_max Maximum actuator values
	 */
	void setActuatorMax(const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) { _actuator_max = actuator_max; }

	/**
	 * Get the maximum actuator values
	 *
	 * @return Maximum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMax() const { return _actuator_max; }

	/**
	 * Set the current actuator setpoint.
	 *
	 * Use this when a new allocation method is started to initialize it properly.
	 * In most cases, it is not needed to call this method before `allocate()`.
	 * Indeed the previous actuator setpoint is expected to be stored during calls to `allocate()`.
	 *
	 * @param actuator_sp Actuator setpoint
	 */
	void setActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp);

	void setSlewRateLimit(const matrix::Vector<float, NUM_ACTUATORS> &slew_rate_limit)
	{ _actuator_slew_rate_limit = slew_rate_limit; }

	/**
	 * Apply slew rate to current actuator setpoint
	 */
	void applySlewRateLimit(float dt);

	/**
	 * Clip the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [min; max]
	 *
	 * @param actuator Actuator vector to clip
	 */
	void clipActuatorSetpoint(matrix::Vector<float, NUM_ACTUATORS> &actuator) const;

	void clipActuatorSetpoint() { clipActuatorSetpoint(_actuator_sp); }

	/**
	 * Normalize the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [-1; +1]
	 *
	 * @param actuator Actuator vector to normalize
	 *
	 * @return Clipped actuator setpoint
	 */
	matrix::Vector<float, NUM_ACTUATORS> normalizeActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator)
	const;

	virtual void updateParameters() {}

	int numConfiguredActuators() const { return _num_actuators; }

	void setNormalizeRPY(bool normalize_rpy) { _normalize_rpy = normalize_rpy; }

protected:
	friend class ControlAllocator; // for _actuator_sp

	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness;  ///< Effectiveness matrix
	matrix::Vector<float, NUM_AXES> _control_allocation_scale;  	///< Scaling applied during allocation
	matrix::Vector<float, NUM_ACTUATORS> _actuator_trim; 	///< Neutral actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_min; 	///< Minimum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_max; 	///< Maximum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_slew_rate_limit; 	///< Slew rate limit
	matrix::Vector<float, NUM_ACTUATORS> _prev_actuator_sp;  	///< Previous actuator setpoint
	matrix::Vector<float, NUM_ACTUATORS> _actuator_sp;  	///< Actuator setpoint
	matrix::Vector<float, NUM_AXES> _control_sp;   		///< Control setpoint
	matrix::Vector<float, NUM_AXES> _control_trim; 		///< Control at trim actuator values
	int _num_actuators{0};
	bool _normalize_rpy{false};				///< if true, normalize roll, pitch and yaw columns
	bool _had_actuator_failure{false};
};
