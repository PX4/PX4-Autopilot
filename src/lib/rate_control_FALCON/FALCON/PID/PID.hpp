#pragma once

#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>


class RSLQR
{
private:
    /* data */
    float _proportional_gain;
    float _integral_gain;
    float _integral_value;
    float _saturation_positive;
    float _saturation_negative;
    float _integral_limit;
    float _rate_int;

    double _kr_i; // yaw rate integral gain for anti-windup
	double _kr_p; // yaw rate proportional gain for feedback control
	double _kr_aw; // yaw rate anti-windup gain
	double _kv_p; // sway feedback gain for yaw rate control

public:
    RSLQR(float p_gain, float i_gain, float sat_pos, float sat_neg, float lim_int);
    ~RSLQR();
    // Update integral, calc control command, return control command (torque)
    float update(float state, float state_setpoint, const float dt, const bool landed);

    // copy from rate control but for scalar
    void update_integral(float &rate_error, float dt);
};
