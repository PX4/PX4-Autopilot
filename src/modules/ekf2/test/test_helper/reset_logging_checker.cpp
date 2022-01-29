#include "reset_logging_checker.h"

// call immediately after state reset
void ResetLoggingChecker::capturePreResetState()
{
	float a[2];
	float b;
	uint8_t c;
	_ekf->get_velNE_reset(a, &c);
	horz_vel_reset_counter_before_reset = c;
	_ekf->get_velD_reset(&b, &c);
	vert_vel_reset_counter_before_reset = c;
	_ekf->get_posNE_reset(a, &c);
	horz_pos_reset_counter_before_reset = c;
	_ekf->get_posD_reset(&b, &c);
	vert_pos_reset_counter_before_reset = c;

	velocity_before_reset = _ekf->getVelocity();
	position_before_reset = _ekf->getPosition();
}

// call immediately after state reset
void ResetLoggingChecker::capturePostResetState()
{
	float a[2];
	float b;
	uint8_t c;
	_ekf->get_velNE_reset(a, &c);
	logged_delta_velocity(0) = a[0];
	logged_delta_velocity(1) = a[1];
	horz_vel_reset_counter_after_reset = c;
	_ekf->get_velD_reset(&b, &c);
	logged_delta_velocity(2) = b;
	vert_vel_reset_counter_after_reset = c;
	_ekf->get_posNE_reset(a, &c);
	logged_delta_position(0) = a[0];
	logged_delta_position(1) = a[1];
	horz_pos_reset_counter_after_reset = c;
	_ekf->get_posD_reset(&b, &c);
	logged_delta_position(2) = b;
	vert_pos_reset_counter_after_reset = c;

	velocity_after_reset = _ekf->getVelocity();
	position_after_reset = _ekf->getPosition();
}

bool ResetLoggingChecker::isVelocityDeltaLoggedCorrectly(float accuracy)
{
	const  Vector3f measured_delta_velocity = velocity_after_reset -
			velocity_before_reset;

	return matrix::isEqual(logged_delta_velocity,
			       measured_delta_velocity,
			       accuracy);
}

bool ResetLoggingChecker::isHorizontalVelocityResetCounterIncreasedBy(int offset)
{
	return horz_vel_reset_counter_after_reset ==
	       horz_vel_reset_counter_before_reset + offset;
}

bool ResetLoggingChecker::isVerticalVelocityResetCounterIncreasedBy(int offset)
{
	return vert_vel_reset_counter_after_reset ==
	       vert_vel_reset_counter_before_reset + offset;
}

bool ResetLoggingChecker::isPositionDeltaLoggedCorrectly(float accuracy)
{
	const  Vector3f measured_delta_position = position_after_reset -
			position_before_reset;

	return matrix::isEqual(logged_delta_position,
			       measured_delta_position,
			       accuracy);
}

bool ResetLoggingChecker::isHorizontalPositionResetCounterIncreasedBy(int offset)
{
	return horz_pos_reset_counter_after_reset ==
	       horz_pos_reset_counter_before_reset + offset;
}

bool ResetLoggingChecker::isVerticalPositionResetCounterIncreasedBy(int offset)
{
	return vert_pos_reset_counter_after_reset ==
	       vert_pos_reset_counter_before_reset + offset;
}
