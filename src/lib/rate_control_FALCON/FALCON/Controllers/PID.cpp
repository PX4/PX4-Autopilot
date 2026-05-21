#include "PID.hpp"

PID::PID(float saturation_positive, float saturation_negative, float integral_limit){
        _saturation_positive = saturation_positive;
        _saturation_negative = saturation_negative;
        _integral_limit = integral_limit;

    }


float PID::update(const float &state, const float &state_setpoint, const float &angular_accel,
			     const float dt, const bool landed)
{
    // angular rates error
	float rate_error = state - state_setpoint;

	// PID control with feed forward
	float torque = _gain_p * rate_error + _rate_int - _gain_d * angular_accel + _gain_ff * state_setpoint;

	// update integral only if we are not landed
	if (!landed) {
		update_integral(rate_error, dt);
	}

	return torque;
}

void PID::update_integral(float &rate_error, const float dt)
{
    // prevent further positive control saturation
    if (_saturation_positive < 0.001f) {
        rate_error = math::min(rate_error, 0.f);
    }

    // prevent further negative control saturation
    if (_saturation_negative < 0.001f) {
        rate_error = math::max(rate_error, 0.f);
    }

    float i_factor = rate_error / math::radians(400.f);
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

    // Perform the integration using a first order method
    float rate_i = _rate_int + i_factor * _gain_i * rate_error * dt;

    // do not propagate the result if out of range or invalid
    if (PX4_ISFINITE(rate_i)) {
        _rate_int = math::constrain(rate_i, -_integral_limit, _integral_limit);
    }
}

void PID::set_gains(float p, float i, float d){
    _gain_p = p;
    _gain_i = i;
    _gain_d = d;
}
