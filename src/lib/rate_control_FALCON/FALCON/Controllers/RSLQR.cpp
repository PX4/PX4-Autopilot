#include "RSLQR.hpp"


RSLQR::RSLQR(float saturation_positive, float saturation_negative){
        _saturation_positive = saturation_positive;
        _saturation_negative = saturation_negative;

    }
float RSLQR::update(float rate_error, float rate_int, float angular_accel)
{

    // PID control
	float torque = _gain_p*rate_error + rate_int - _gain_d*angular_accel;

    return torque;
}

void RSLQR::set_gains(float p, float i, float d){
    _gain_p = p;
    _gain_i = i;
    _gain_d = d;
}
