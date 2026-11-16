// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#pragma once

#include "TurtleMixing.hpp"
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/motor_direction_status.h>
#include <uORB/topics/turtle_control.h>

/** Select Turtle output and require fresh flight setpoints after leaving it. */
class TurtleControlGate
{
public:
	static constexpr uint64_t CONTROL_TIMEOUT_US = 100000;
	static constexpr uint64_t DIRECTION_TIMEOUT_US = 300000;
	static constexpr uint16_t MOTOR_MASK = TurtleMixing::MOTOR_MASK;

	void configureGeometry(const TurtleMixing::Geometry *geometry)
	{
		TurtleMixing updated;
		updated.configure(geometry);
		// A motor remapping invalidates the current recovery until disarm.
		_geometry_changed |= _turtle_armed && !_mixing.sameMapping(updated);
		_mixing = updated;
	}

	bool geometryValid() const { return _mixing.geometryValid() && !_geometry_changed; }

	void setSelected(bool selected, uint64_t now, uint64_t armed_time)
	{
		if (armed_time != _turtle_armed_at) {
			_geometry_changed = false;
		}

		if (selected && armed_time != 0) {
			_turtle_armed_at = armed_time;
		}

		_turtle_armed = armed_time != 0 && armed_time == _turtle_armed_at;

		if (_selected != selected) {
			_selected = selected;
			_entered_at = selected ? now : 0;
			_normal_after = selected ? 0 : now;
			_torque_timestamp = 0;
			_thrust_timestamp = 0;
		}
	}

	bool selected() const { return _selected; }
	bool waitingForNormal() const { return _normal_after != 0; }
	void observeTorque(uint64_t timestamp) { _torque_timestamp = timestamp; }
	void observeThrust(uint64_t timestamp) { _thrust_timestamp = timestamp; }

	bool allowNormal(uint64_t now, const motor_direction_status_s &direction)
	{
		if (_selected) {
			return false;
		}

		if (_normal_after == 0) {
			return true;
		}

		if (_torque_timestamp < _normal_after || _thrust_timestamp < _normal_after
		    || _torque_timestamp > now || _thrust_timestamp > now
		    || direction.reversed_mask != 0
		    || direction.state == motor_direction_status_s::STATE_PENDING) {
			return false;
		}

		// The driver must have observed disarming after the Turtle arming epoch.
		if (_turtle_armed_at != 0
		    && (!fresh(direction.timestamp, now, DIRECTION_TIMEOUT_US)
			|| direction.timestamp_disarmed <= _turtle_armed_at || direction.timestamp_disarmed > direction.timestamp
			|| (direction.state != motor_direction_status_s::STATE_IDLE
			    && direction.state != motor_direction_status_s::STATE_COMPLETE))) {
			return false;
		}

		_normal_after = 0;
		_turtle_armed_at = 0;
		return true;
	}

	bool mixTurtleControls(uint64_t now, bool armed, const turtle_control_s &control,
			       const motor_direction_status_s &direction,
			       float (&output)[actuator_motors_s::NUM_CONTROLS]) const
	{
		if (!_selected || !armed || _geometry_changed || !control.enabled || !control.driving || control.disarm_requested
		    || !fresh(control.timestamp, now, CONTROL_TIMEOUT_US) || control.timestamp < _entered_at
		    || !fresh(direction.timestamp, now, DIRECTION_TIMEOUT_US)
		    || control.direction_request_id == 0 || direction.request_id != control.direction_request_id
		    || direction.state != motor_direction_status_s::STATE_COMPLETE
		    || direction.supported_motor_mask != MOTOR_MASK || direction.motor_mask != MOTOR_MASK
		    || direction.reversed_mask != MOTOR_MASK) {
			TurtleMixing::stop(output);
			return false;
		}

		return _mixing.mix(control.roll, control.pitch, control.thrust, output);
	}

private:
	static bool fresh(uint64_t timestamp, uint64_t now, uint64_t timeout)
	{
		return timestamp != 0 && timestamp <= now && now - timestamp <= timeout;
	}

	TurtleMixing _mixing;
	bool _selected{false};
	bool _turtle_armed{false};
	bool _geometry_changed{false};
	uint64_t _entered_at{0};
	uint64_t _normal_after{0};
	uint64_t _turtle_armed_at{0};
	uint64_t _torque_timestamp{0};
	uint64_t _thrust_timestamp{0};
};
