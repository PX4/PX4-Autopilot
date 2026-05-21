#include "RSLQR.hpp"
#include <iostream>


RSLQR::RSLQR(float saturation_positive, float saturation_negative){
        _saturation_positive = saturation_positive;
        _saturation_negative = saturation_negative;

    }
float RSLQR::update(const float &state, const float &state_setpoint, const float &angular_accel,
                const float &integral_limit, const float dt, const bool landed)
{

    // TODO: flip signs to match implementation described by orange book
    float torque = -_gain_p * state - _gain_i * _rate_int;
    // float torque = proportional_gain * rate_error + integral_value + feed_forward_gain * state_setpoint;

    // update integral only if we are not landed
    float rate_error = state - state_setpoint;
    if (!landed) {
        // prevent further positive control saturation
        if (_saturation_positive < 0.001f) {
            rate_error = math::min(rate_error, 0.f);
        }

        // prevent further negative control saturation
        if (_saturation_negative < 0.001f) {
            rate_error = math::max(rate_error, 0.f);
        }

        // Perform the integration using a first order method
        float rate_i = _rate_int + rate_error * dt;

        // do not propagate the result if out of range or invalid
        if (PX4_ISFINITE(rate_i)) {
            _rate_int = math::constrain(rate_i, -integral_limit, integral_limit);
        }
    }
    return torque;
}

void RSLQR::update_integral(float &rate_error, float dt, float integral_limit)
{
    // prevent further positive control saturation
    if (_saturation_positive < 0.001f) {
        rate_error = math::min(rate_error, 0.f);
    }

    // prevent further negative control saturation
    if (_saturation_negative < 0.001f) {
        rate_error = math::max(rate_error, 0.f);
    }

    // Perform the integration using a first order method
    float rate_i = _rate_int + rate_error * dt;

    // do not propagate the result if out of range or invalid
    if (PX4_ISFINITE(rate_i)) {

        _rate_int = math::constrain(rate_i, -integral_limit, integral_limit);
    }
}

void RSLQR::set_gains(float p, float i, float d){
    std::cout << "Setting PID: " << p << " " << i << " " << d << std::endl;
    _gain_p = p;
    _gain_i = i;
    _gain_d = d;
}
