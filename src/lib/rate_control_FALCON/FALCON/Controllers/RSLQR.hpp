#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class RSLQR
{
private:
    // copy from rate control but for scalar
    void update_integral(float &rate_error, float dt, float integral_limit);

    float _gain_p{0.0f};
    float _gain_i{0.0f};
    float _gain_d{0.0f};
    float _gain_ff{0.0f};

    float _saturation_positive{};
    float _saturation_negative{};


public:
    RSLQR(
        float saturation_positive,
        float saturation_negative
    );

    void set_gains(float p, float i, float d);
    float _rate_int{0.0f};


    // Update integral, calc control command, return control command (torque)
    float update(
        const float &state,
        const float &state_setpoint,
        const float &angular_accel,
        const float &integral_limit,
		const float dt,
        const bool landed);

};
