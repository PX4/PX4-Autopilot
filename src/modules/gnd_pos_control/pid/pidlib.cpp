#ifndef _Pid_SOURCE_
#define _Pid_SOURCE_

#include <iostream>
#include <cmath>
#include "pidlib.h"

using namespace std;


Pid::Pid() :

    _param_set(false),
    _dt(0.0f),
    _max(0.0f),
    _min(0.0f),
    _Kp(0.0f),
    _Kd(0.0f),
    _Ki(0.0f),
    _pre_error(0.0f),
    _integral(0.0f),
    _max_i(0.0f)
{
}

void Pid::update_gains (float time, float a_max, float a_min, float Kp, float Kd, float Ki, float a_max_i ) {
    _dt = time;
    _max = a_max;
    _min = a_min;
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _max_i = a_max_i;
}

void Pid::update_dt(float time) {

    _dt = time;
}

float Pid::calculate( float setpoint, float control_feedback )
{
    // Calculate error
    float error = setpoint - control_feedback;

    // Integral term
    if (_integral > _max_i)
    {
        _integral = _max_i;
    } else {
        _integral += error * _dt;
    }

    // Derivative term
    float derivative = (error - _pre_error) / _dt;

    // Calculate total output
    float output = _Kp * error + _Ki * _integral + _Kd * derivative;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

#endif