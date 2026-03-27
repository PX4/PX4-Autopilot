#pragma once


#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class RSLQR
{
private:
    /* data */

public:
    RSLQR() = default;
    RSLQR(float proportional_gain, float integral_gain, float saturation_positive, float saturation_negative, float integral_limit);
    ~RSLQR() = default;

    float _proportional_gain;
    float _integral_gain;
    float _saturation_positive;
    float _saturation_negative;
    float _integral_limit;
    float _rate_int;
    // Update integral, calc control command, return control command (torque)
    float update(float state, float state_setpoint, const float dt, const bool landed);

    // copy from rate control but for scalar
    void update_integral(float &rate_error, float dt);
};
