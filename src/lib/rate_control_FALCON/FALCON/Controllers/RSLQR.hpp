#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class RSLQR
{
private:

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


    // Update integral, calc control command, return control command (torque)
    float update(
        float rate_error,
        float rate_int,
        float angular_accel);

};
