#include "RSLQR.hpp"
#include <px4_platform_common/defines.h>

RSLQR::RSLQR(float proportional_gain, float integral_gain, float saturation_positive, float saturation_negative, float integral_limit){
        _proportional_gain = proportional_gain;
        _integral_gain = integral_gain;
        _saturation_positive = saturation_positive;
        _saturation_negative = saturation_negative;
        _integral_limit = integral_limit;
    }
float RSLQR::update(float state, float state_setpoint, const float dt, const bool landed)
{
    
    float rate_error = state_setpoint - state;

    // TODO: flip signs to match implementation described by orange book
    float torque = _proportional_gain * rate_error + _rate_int;
    // float torque = proportional_gain * rate_error + integral_value + feed_forward_gain * state_setpoint;

    // update integral only if we are not landed
    if (!landed) {
        update_integral(rate_error, dt);
    }
    return torque; // replace with actual torque calculation
}

void RSLQR::update_integral(float &rate_error, float dt)
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
    float rate_i = _rate_int + i_factor * _integral_gain * rate_error * dt;

    // do not propagate the result if out of range or invalid
    if (PX4_ISFINITE(rate_i)) {
        _rate_int = math::constrain(rate_i, -_integral_limit, _integral_limit);
    }
}
