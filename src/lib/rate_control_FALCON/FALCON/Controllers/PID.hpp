#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>


class PID
{
private:
    void update_integral(float &rate_error, float dt);

    float _gain_p{0.0f};
    float _gain_i{0.0f};
    float _gain_d{0.0f};
    float _gain_ff{0.0f};


    float _saturation_positive{};
    float _saturation_negative{};
    float _integral_limit{};
    float _rate_int{0.0f};

public:

    PID(
        float saturation_positive,
        float saturation_negative,
        float integral_limit
    );

    float update(
        const float &state,
        const float &state_setpoint,
        const float &angular_accel,
		const float dt,
        const bool landed);

    void set_gains(float p, float i, float d);


};
